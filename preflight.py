#!/usr/bin/env python3
"""preflight.py -- refuse to fly on a rig that has not been measured.

WHY THIS EXISTS
---------------
On 2026-09-07 the Vicon link spent an unknown number of days losing 200 ms of
pose data at a time. `systemctl status vicon-rto` reported active. `ip route
show | grep` reported present. Both were wrong; only the TCP socket knew.

That was the third instrument in one session found reporting health over
broken state. So this checks the THINGS, not their proxies:

    rto        read off the live socket, not systemd, not the route table
    rate       counted at the subscriber, not assumed from Tracker's config
    markers    the bridge's own visible/expected companion sample
    latency    now - header.stamp, the number a policy actually inherits
    branch     geodesic attitude jumps while the drone sits still

Every one of those corresponds to something that lied on 2026-09-07.

    python3 preflight.py --topic /vicon/crazyflie2/crazyflie2

Exit code 0 means every check passed and it is reasonable to fly. Non-zero
means at least one did not, and the reason is printed in plain English.

Run it inside the container (it needs rclpy and iproute2) with the bridge
already running and the drone sitting STILL in the volume.
"""
import argparse
import math
import re
import subprocess
import sys
import time

# Thresholds: each is what this rig measured (pear-2, 7 Sep 2026), with margin.
RTO_MAX_MS = 50.0        # measured 20 with the route, 202 without
RATE_MIN_HZ = 200.0      # 240 published, ~235 seen by a python subscriber
GAP_MAX_MS = 60.0        # 240 Hz is 4.2 ms; 17 ms holes are the RTO floor
LAT_MED_MAX_MS = 15.0    # measured 4.6 at the topic, 5.9 end to end
LAT_MAX_MS = 60.0
BRANCH_DEG = 45.0        # a still drone must not reorient at all
FAILED = []


def check(name, ok, detail=""):
    print("  %-4s %-34s %s" % ("PASS" if ok else "FAIL", name, detail))
    if not ok:
        FAILED.append(name)
    return ok


def att_change_deg(q1, q2):
    """Geodesic angle between orientations. No parameterisation singularity."""
    d = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, d))))


def socket_rto_ms(peer):
    """The only honest source for the effective TCP retransmission timeout.

    `systemctl status` reports the unit, not the route. `ip route show`
    reports the route, not the socket. This reports what the connection is
    actually using -- which on 2026-09-07 disagreed with both.
    """
    try:
        out = subprocess.run(["ss", "-ti", "dst", peer], capture_output=True,
                             text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        return None, "could not run ss (%s)" % exc
    m = re.search(r"\brto:(\d+)", out)
    if not m:
        if "ESTAB" not in out:
            return None, ("no established connection to %s -- is the bridge "
                          "running?" % peer)
        return None, "ss gave no rto: field"
    return float(m.group(1)), ""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", required=True)
    ap.add_argument("--peer", default="192.168.10.1",
                    help="Vicon DataStream server (for the socket check)")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--skip-socket", action="store_true",
                    help="skip the rto check (e.g. ss unavailable)")
    a = ap.parse_args()

    print("\n  preflight -- %s, %.0f s\n" % (a.topic, a.seconds))

    # -- 1. the socket ----------------------------------------------------
    if a.skip_socket:
        print("  SKIP rto (asked to)")
    else:
        rto, why = socket_rto_ms(a.peer)
        if rto is None:
            check("tcp rto to %s" % a.peer, False, why)
        else:
            check("tcp rto to %s" % a.peer, rto <= RTO_MAX_MS,
                  "rto:%.0f ms (limit %.0f)%s"
                  % (rto, RTO_MAX_MS,
                     "  -- the network fix is missing: course site, Installation, step 5"
                     if rto > RTO_MAX_MS else ""))

    # -- 2. the stream ----------------------------------------------------
    try:
        import rclpy
        from rclpy.node import Node
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        sys.exit("\n  cannot import rclpy (%s)\n"
                 "  Source the ROS workspace first:\n"
                 "    source /opt/ros/humble/setup.bash\n"
                 "    source <cf_vicon_stack>/install/setup.bash\n" % exc)

    rclpy.init()
    node = Node("preflight")

    mtype = qtype = None
    qtopic = a.topic.rstrip("/") + "/quality"
    for _ in range(50):
        for name, types in node.get_topic_names_and_types():
            if name == a.topic and types:
                mtype = types[0]
            if name == qtopic and types:
                qtype = types[0]
        if mtype:
            break
        time.sleep(0.1)
    if not mtype:
        rclpy.shutdown()
        sys.exit("\n  topic %s not found. Is the bridge running, and does "
                 "ROS_DOMAIN_ID match it?\n" % a.topic)

    recv, lat, quats, partial, seen = [], [], [], 0, []

    def pose_cb(msg):
        p = getattr(msg, "pose", None)
        o = p.orientation if p is not None and hasattr(p, "orientation") else \
            getattr(getattr(msg, "transform", None), "rotation", None)
        if o is None:
            return
        now = time.time()
        h = msg.header.stamp
        recv.append(now)
        lat.append(now - (h.sec + h.nanosec * 1e-9))
        quats.append((o.x, o.y, o.z, o.w))

    def qual_cb(msg):
        nonlocal partial
        v = msg.vector
        seen.append((int(v.x), int(v.y)))
        if v.y and v.x < v.y:
            partial += 1

    node.create_subscription(get_message(mtype), a.topic, pose_cb, 50)
    if qtype:
        node.create_subscription(get_message(qtype), qtopic, qual_cb, 50)

    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass

    if len(recv) < 10:
        FAILED.append("pose stream")
        print("  FAIL %-34s only %d messages in %.0f s"
              % ("pose stream", len(recv), a.seconds))
    else:
        span = recv[-1] - recv[0]
        hz = (len(recv) - 1) / span if span > 0 else 0.0
        check("publish rate", hz >= RATE_MIN_HZ,
              "%.1f Hz (min %.0f)" % (hz, RATE_MIN_HZ))

        gaps = sorted((recv[i] - recv[i - 1]) * 1000.0
                      for i in range(1, len(recv)))
        check("worst gap between poses", gaps[-1] <= GAP_MAX_MS,
              "max %.1f ms, p99 %.1f ms (limit %.0f)"
              % (gaps[-1], gaps[int(0.99 * len(gaps))], GAP_MAX_MS))

        s = sorted(lat)
        med, mx = s[len(s) // 2] * 1000.0, s[-1] * 1000.0
        check("latency now - header.stamp",
              med <= LAT_MED_MAX_MS and mx <= LAT_MAX_MS,
              "median %.2f ms, max %.2f ms" % (med, mx))
        if med < 1.0:
            print("       note: a median under 1 ms means the bridge is "
                  "stamping PULL time,\n"
                  "       not capture time -- your image predates the "
                  "capture-time fix; rebuild it.")

        worst = max((att_change_deg(quats[i], quats[i - 1])
                     for i in range(1, len(quats))), default=0.0)
        check("no solver branch jumps", worst < BRANCH_DEG,
              "worst single-frame reorientation %.1f deg%s"
              % (worst, "  -- Vicon fits the markers two ways: tell the TA"
                 if worst >= BRANCH_DEG else ""))

    # -- 3. the markers ---------------------------------------------------
    if not qtype:
        check("marker completeness", False,
              "%s not published -- rebuild the image"
              % qtopic)
    elif not seen:
        check("marker completeness", False, "no samples on %s" % qtopic)
    else:
        vis, exp = seen[-1]
        low = min(v for v, _ in seen)
        check("marker completeness", partial == 0 and low == exp,
              "%d/%d now, fewest %d, %d partial of %d samples"
              % (vis, exp, low, partial, len(seen)))

    print()
    if FAILED:
        print("  NOT CLEARED TO FLY -- %d check(s) failed: %s\n"
              % (len(FAILED), ", ".join(FAILED)))
        return 1
    print("  All checks passed. Rig measured, not assumed.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
