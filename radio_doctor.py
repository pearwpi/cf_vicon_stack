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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fix", action="store_true",
                    help="detach kernel drivers and reset the device")
    ap.add_argument("--uri", default="radio://0/80/2M/E7E7E7E701")
    a = ap.parse_args()

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

        import cflib.crtp
        cflib.crtp.init_drivers(enable_debug_driver=False)
        found = cflib.crtp.scan_interfaces()
        if found:
            for uri, _ in found:
                ok("Crazyflie answered", uri)
        else:
            note("No Crazyflie answered the scan.",
                 "USB is fine, so this is a radio-link issue: drone powered "
                 f"on? channel/address matching {a.uri}?")
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

    print("\n" + "=" * 70)
    print(f"{G}No issues found.{X}" if issues == 0
          else f"{R}{issues} issue(s) found.{X} Work the remedies top-down.")
    print("=" * 70)
    return 1 if issues else 0


if __name__ == "__main__":
    sys.exit(main())
