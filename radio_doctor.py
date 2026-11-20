#!/usr/bin/env python3
"""Diagnose and fix Crazyradio USB claim problems on Linux.

Built for one symptom: usb.core.USBError: [Errno 16] Resource busy, raised from
crazyradio.py -> self.dev.set_configuration(1).

READ THE ERRNO, it names the problem:
    13 EACCES  Permission denied  -> udev/group. Run the udev setup script.
    16 EBUSY   Resource busy      -> something else HOLDS the device. This tool.
    19 ENODEV  No such device     -> dongle unplugged mid-operation.

EBUSY on libusb_set_configuration means libusb opened the device and found its
interfaces already claimed. Only two things do that: (a) another userspace
process (cfclient, a crashed run, a second terminal), or (b) a kernel driver
bound to an interface -- on Ubuntu nearly always ModemManager, which probes
every new USB device for a modem and holds CDC interfaces for seconds. The
Crazyradio 2.0 does present a CDC interface, so it can be held indefinitely.

    python3 radio_doctor.py         # diagnose only, changes nothing
    python3 radio_doctor.py --fix   # detach kernel drivers + USB reset
    sudo python3 radio_doctor.py --fix   # also sees other users' processes
    python3 radio_doctor.py --scan  # also hunt for drones at unknown addresses
"""

import argparse
import glob
import os
import subprocess
import sys
import time

BITCRAZE = {
    (0x1915, 0x7777): "Crazyradio PA",
    (0x1915, 0x7778): "Crazyradio 2.0",
    (0x1915, 0x0101): "Crazyradio (bootloader)",
    (0x0483, 0x5740): "Crazyflie 2.x over USB",
}

G, R, Y, D, X = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"
if not sys.stdout.isatty():
    G = R = Y = D = X = ""


def ok(m, d=""):   print(f"  {G}OK{X}    {m} {D}{d}{X}")
def bad(m, d=""):  print(f"  {R}ISSUE{X} {m} {D}{d}{X}")
def note(m, d=""): print(f"  {Y}NOTE{X}  {m} {D}{d}{X}")
def head(t):       print(f"\n{t}\n" + "-" * len(t))


def find_devices():
    import usb.core
    out = []
    for dev in usb.core.find(find_all=True):
        key = (dev.idVendor, dev.idProduct)
        if key in BITCRAZE:
            out.append((dev, BITCRAZE[key]))
    return out


def node_path(dev):
    return f"/dev/bus/usb/{dev.bus:03d}/{dev.address:03d}"


def holders(path):
    """
    Which PIDs have this device node open?

    Implemented by walking /proc/*/fd rather than shelling out to lsof, so it
    works on a minimal box. Without root you only see your own processes --
    which is usually enough, because the stale holder is almost always your own
    earlier python run.
    """
    found = []
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = fd_dir.split("/")[2]
        try:
            for fd in os.listdir(fd_dir):
                try:
                    if os.readlink(os.path.join(fd_dir, fd)) == path:
                        try:
                            cmd = open(f"/proc/{pid}/cmdline").read()
                            cmd = cmd.replace("\0", " ").strip() or "?"
                        except Exception:
                            cmd = "?"
                        found.append((pid, cmd))
                        break
                except OSError:
                    continue
        except (PermissionError, FileNotFoundError):
            continue
    return found


def kernel_drivers(dev):
    """Interfaces of the active configuration that have a kernel driver bound."""
    bound = []
    try:
        cfg = dev.get_active_configuration()
    except Exception:
        return bound
    for intf in cfg:
        i = intf.bInterfaceNumber
        try:
            if dev.is_kernel_driver_active(i):
                bound.append(i)
        except Exception:
            pass
    return sorted(set(bound))


def sysfs_path(dev):
    """Locate THIS device's sysfs node by matching busnum/devnum."""
    for p in glob.glob("/sys/bus/usb/devices/*"):
        try:
            b = int(open(os.path.join(p, "busnum")).read())
            d = int(open(os.path.join(p, "devnum")).read())
            if b == dev.bus and d == dev.address:
                return p
        except Exception:
            continue
    return None


def sysfs_drivers(dev):
    """
    Which driver is bound to each interface of THIS device.

    'usbfs' is the interesting one: it means a *userspace* process has claimed
    the interface through libusb. That is not a kernel driver, so
    is_kernel_driver_active() reports nothing -- but it is exactly what causes
    EBUSY. This is the check that catches a running cfclient.
    """
    base = sysfs_path(dev)
    if not base:
        return []
    out = []
    for p in sorted(glob.glob(f"{base}:*")):
        drv = os.path.join(p, "driver")
        if os.path.islink(drv):
            out.append((os.path.basename(p), os.path.basename(os.readlink(drv))))
    return out


def service_state(name):
    try:
        r = subprocess.run(["systemctl", "is-active", name],
                           capture_output=True, text=True, timeout=5)
        return r.stdout.strip()
    except Exception:
        return "unknown"


FACTORY_ADDR = 0xE7E7E7E7E7
RATES = (("250K", 0), ("1M", 1), ("2M", 2))   # Crazyradio.DR_* values


def parse_ids(spec):
    """'1-10,15,0x1a' -> [1..10, 15, 26]. Each id becomes address E7E7E7E7<id>."""
    out = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = (int(x, 0) for x in part.split("-", 1))
            out.extend(range(lo, hi + 1))
        else:
            out.append(int(part, 0))
    bad_ids = [i for i in out if not 0 <= i <= 0xFF]
    if bad_ids:
        raise ValueError(f"ids must be 0-255: {bad_ids}")
    return out


def uri_address(uri):
    """Address of a radio://0/<ch>/<rate>/<ADDR> URI, or None if it has none."""
    parts = uri.split("/")
    if len(parts) >= 6 and parts[5]:
        return int(parts[5], 16)
    return None


def wide_scan(addresses):
    """Scan every channel and data rate at each address. Returns found URIs.

    Talks to the Crazyradio directly, NOT through cflib.crtp.scan_interfaces(),
    which swallows USBError and returns []: a radio that is EBUSY, or that
    drops off the bus mid-scan, then reads as "no drone", and you go hunting
    for a radio problem in the drone. Here a USB error stops the scan and is
    reported as what it is.
    """
    from cflib.drivers.crazyradio import Crazyradio

    radio = Crazyradio()
    found = []
    try:
        radio.set_arc(1)            # one retry: a scan only needs a single ACK
        for addr in addresses:
            radio.set_address(tuple(addr.to_bytes(5, "big")))
            hits = []
            for name, dr in RATES:
                radio.set_data_rate(dr)
                for ch in radio.scan_channels(0, 125, (0xFF,)):
                    hits.append(f"radio://0/{ch}/{name}/{addr:010X}")
            if hits:
                for u in hits:
                    ok("Crazyflie answered", u)
                found.extend(hits)
            else:
                print(f"  {D}..    {addr:010X}: nothing{X}")
    finally:
        try:
            radio.close()
        except Exception:
            pass
    return found


def main():
    ap = argparse.ArgumentParser(
        epilog="examples:\n"
               "  radio_doctor.py --scan              # ids 0-20 + factory + --uri\n"
               "  radio_doctor.py --scan --ids 3      # just E7E7E7E703\n"
               "  radio_doctor.py --scan --ids 1-40 --address E7E7E7E7AA",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fix", action="store_true",
                    help="detach kernel drivers and reset the device")
    ap.add_argument("--uri", default="radio://0/80/2M/E7E7E7E701",
                    help="the URI you intend to fly with; --scan says whether it answers")
    ap.add_argument("--scan", action="store_true",
                    help="scan all channels and rates across a range of addresses "
                         "to find drones whose address you do not know")
    ap.add_argument("--ids", default="0-20",
                    help="with --scan: address suffixes to try, E7E7E7E7<id> "
                         "(lab drone N is usually id N). Default 0-20")
    ap.add_argument("--address", action="append", default=[],
                    help="with --scan: extra full 5-byte hex address; repeatable")
    a = ap.parse_args()

    try:
        scan_ids = parse_ids(a.ids)
        extra = [int(x, 16) for x in a.address]
        want = uri_address(a.uri)
    except ValueError as exc:
        ap.error(str(exc))

    print("=" * 70)
    print("Crazyradio doctor -- EBUSY / resource-busy diagnosis")
    print("=" * 70)

    try:
        import usb.core  # noqa
    except Exception as exc:
        print(f"pyusb missing ({exc}). pip install --user pyusb")
        return 1

    head("1. Devices")
    devs = find_devices()
    if not devs:
        bad("No Bitcraze USB device found.",
            "Check 'lsusb | grep 1915'. Is the dongle in THIS machine?")
        return 1
    for dev, name in devs:
        ok(f"{name}", f"{dev.idVendor:04x}:{dev.idProduct:04x} @ {node_path(dev)}")

    issues = 0

    for dev, name in devs:
        if dev.idVendor != 0x1915:
            continue  # only diagnose the radio, not a plugged-in Crazyflie

        head(f"2. Who is holding {name}?")
        path = node_path(dev)

        procs = [p for p in holders(path) if p[0] != str(os.getpid())]
        if procs:
            issues += 1
            bad(f"{len(procs)} process(es) have {path} open:")
            for pid, cmd in procs:
                print(f"        pid {pid}: {cmd[:90]}")
            print(f"        {Y}kill them:  kill {' '.join(p for p, _ in procs)}{X}")
        else:
            if os.geteuid() != 0:
                ok("No process of YOURS holds the device.",
                   "re-run with sudo to also see other users' processes")
            else:
                ok("No process holds the device.")

        head("3. Kernel drivers bound to its interfaces")
        bound = kernel_drivers(dev)
        sysfs = sysfs_drivers(dev)
        if sysfs:
            print(f"  {D}sysfs: " +
                  ", ".join(f"{i} -> {d}" for i, d in sysfs) + f"{X}")
        usbfs = [i for i, d in sysfs if d == "usbfs"]
        if usbfs:
            issues += 1
            bad(f"Interface(s) {usbfs} are claimed via usbfs (libusb).",
                "A USERSPACE process holds this radio -- see section 2. "
                "This is not a kernel driver and cannot be detached; "
                "the owning process must exit.")
        if bound:
            issues += 1
            bad(f"Kernel driver(s) bound to interface(s): {bound}",
                "this is what makes set_configuration() return EBUSY")
            if a.fix:
                for i in bound:
                    try:
                        dev.detach_kernel_driver(i)
                        ok(f"detached kernel driver from interface {i}")
                    except Exception as exc:
                        bad(f"could not detach interface {i}: {exc}",
                            "try: sudo python3 radio_doctor.py --fix")
            else:
                note("re-run with --fix to detach them")
        else:
            ok("No kernel driver bound.")

        head("4. ModemManager")
        st = service_state("ModemManager")
        if st == "active":
            issues += 1
            bad("ModemManager is running.",
                "It probes every new USB device and holds CDC interfaces open. "
                "This is the #1 cause of intermittent EBUSY on Ubuntu.")
            print(f"        {Y}Test it:      sudo systemctl stop ModemManager{X}")
            print(f"        {Y}Permanent:    sudo bash setup_crazyradio_ubuntu.sh{X}")
            print(f"        {D}              (installs ID_MM_DEVICE_IGNORE rules;"
                  f" requires a replug){X}")
        else:
            ok(f"ModemManager: {st}")

        if a.fix:
            head("5. USB port reset")
            try:
                dev.reset()
                ok("device reset", "re-enumerated; state cleared")
                time.sleep(1.5)
            except Exception as exc:
                note(f"reset failed ({exc})", "physically unplug and replug instead")

    head("6. Live open test")
    # NOTE: do NOT use cflib.crtp.scan_interfaces() to test this. It catches
    # USBError internally, prints it to stdout, and returns an empty list --
    # so a device that is still EBUSY looks identical to a working radio with
    # no drone in range. Open the Crazyradio directly so the error propagates.
    try:
        from cflib.drivers.crazyradio import Crazyradio
        radio = Crazyradio()
        ok("radio opened directly -- USB is clear")
        try:
            radio.close()
        except Exception:
            pass

        # Scan at the address --uri names, not just the factory default: a
        # drone given its own address never answers a default-address scan,
        # so the old scan_interfaces() call here reported "no Crazyflie" for
        # a drone that was on and in range. --scan does the wide sweep in
        # section 7, so this stays a quick check of the two likely addresses.
        if not a.scan:
            quick = [x for x in (want, FACTORY_ADDR) if x is not None]
            quick = list(dict.fromkeys(quick))
            found = wide_scan(quick)
            hit = [u for u in found if u.upper() == a.uri.upper()]
            if hit:
                ok(f"--uri {a.uri} answers -- ready to connect")
            elif found:
                issues += 1
                bad(f"--uri {a.uri} did NOT answer, but the drone above did.",
                    "Point --uri / your config's uri at it.")
            else:
                note("No Crazyflie answered.",
                     "USB is fine, so this is the radio link: is the drone ON "
                     "and booted? If its address is unknown, re-run with --scan.")
    except Exception as exc:
        msg = str(exc)
        issues += 1
        if "16" in msg or "busy" in msg.lower():
            bad("Still EBUSY.", msg)
            print(f"        {Y}Ordered remedies:{X}")
            print("          1. Close cfclient. Only ONE process can hold a")
            print("             Crazyradio -- see section 2 for the PID.")
            print("          2. sudo systemctl stop ModemManager")
            print("          3. physically unplug and replug the dongle")
            print("          4. sudo python3 radio_doctor.py --fix")
            print("          5. reboot (clears any stuck kernel binding)")
        elif "13" in msg or "denied" in msg.lower():
            bad("EACCES -- this IS a permissions problem after all.", msg)
            print(f"        {Y}sudo bash setup_crazyradio_ubuntu.sh{X}")
        else:
            bad(f"open failed: {msg}")
        radio_ok = False
    else:
        radio_ok = True

    if a.scan:
        head("7. Wide scan")
        addrs = []
        for x in ([want] if want is not None else []) + [FACTORY_ADDR] + \
                [0xE7E7E7E700 | i for i in scan_ids] + extra:
            if x not in addrs:
                addrs.append(x)
        if not radio_ok:
            note("skipped -- the radio did not open (section 6)")
        else:
            print(f"  {D}{len(addrs)} addresses x 126 channels x 3 rates ...{X}")
            t0 = time.time()
            try:
                found = wide_scan(addrs)
            except Exception as exc:
                issues += 1
                msg = str(exc)
                if "19" in msg or "no such device" in msg.lower():
                    bad("The radio dropped off USB mid-scan.", msg)
                    print(f"        {Y}Loose dongle or a weak port: try another "
                          f"port, ideally not through a hub.{X}")
                elif "16" in msg or "busy" in msg.lower():
                    bad("Another process grabbed the radio mid-scan.", msg)
                else:
                    bad(f"scan failed: {msg}")
                found = None
            if found is not None:
                print(f"  {D}scan took {time.time() - t0:.1f} s{X}")
                if not found:
                    note("Nothing answered at any address.",
                         "Is the drone ON and booted? If so its address is outside "
                         "--ids; widen it, or read the address over USB in cfclient.")
                elif want is not None:
                    hit = [u for u in found if u.upper() == a.uri.upper()]
                    if hit:
                        ok(f"--uri {a.uri} answers")
                    else:
                        issues += 1
                        bad(f"--uri {a.uri} did NOT answer, but the drone(s) above did.",
                            "Point your config's uri at one of those.")

    print("\n" + "=" * 70)
    print(f"{G}No issues found.{X}" if issues == 0
          else f"{R}{issues} issue(s) found.{X} Work the remedies top-down.")
    print("=" * 70)
    return 1 if issues else 0


if __name__ == "__main__":
    sys.exit(main())
