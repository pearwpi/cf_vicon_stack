#!/usr/bin/env python3
"""Passive Vicon probe. No radio, no motors.

Merges what used to be vicon_probe.py and vicon_pose_subscriber.py: link health
(rate, latency, dropouts) and rigid-body health (stationarity, tilt, yaw
stability, solver flips) come from one capture, because they are always
diagnosed together.

    python3 vicon_probe.py [seconds] [out.csv]   capture, analyse, write CSV
    python3 vicon_probe.py --live               streaming summary, no CSV

Yaw convention and tilt formula are cf_core's, i.e. the same ones the flight
code uses.
"""
import math
import os
import sys
import csv

from cf_core import (Pose, capture_stats, tilt_deg_of, vicon_capture,
                     wrap180, yaw_deg_of)

TOPIC = os.environ.get("VICON_TOPIC", "/vicon/crazyflie1/crazyflie1")


def rpy_deg(qx, qy, qz, qw):
    """Standard ZYX. Roll/pitch are for reference only; tilt_deg_of is the
    convention-free measure and the one the verdict uses."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    return math.degrees(roll), math.degrees(pitch), yaw_deg_of(qx, qy, qz, qw)


def stats(v):
    m = sum(v) / len(v)
    return m, math.sqrt(sum((a - m) ** 2 for a in v) / len(v)), min(v), max(v)


def trimmed_span(v):
    """A solver flip moves the reported ORIGIN as well as the axes, so raw
    min/max reports MOVING for a drone that never budged. Trim the outer 1%."""
    s = sorted(v)
    k = max(1, len(s) // 100)
    return s[-k] - s[k - 1]


def live_summary(rows, elapsed):
    s = capture_stats(rows)
    if not s:
        print(f"[{elapsed:4.0f}s] waiting for frames on {TOPIC}")
        return
    q = rows[-1][5:9]
    print(f"[{elapsed:4.0f}s] {s['hz']:6.1f} Hz  lat med {s['lat_med']:5.2f} ms "
          f"p95 {s['lat_p95']:5.2f}  drops {s['drops']:3d} "
          f"(max {s['max_drop_ms']:5.1f} ms)  tilt {tilt_deg_of(*q):6.2f} deg  "
          f"yaw {yaw_deg_of(*q):+7.2f}")


def analyse(rows, dur):
    xs, ys, zs = ([r[i] for r in rows] for i in (2, 3, 4))
    ts = [r[0] for r in rows]
    quats = [r[5:9] for r in rows]
    yaws = [yaw_deg_of(*q) for q in quats]
    tilts = [tilt_deg_of(*q) for q in quats]

    s = capture_stats(rows)
    print("\n=== VICON PASSIVE PROBE ===")
    print(f"samples {s['n']} over {s['dur']:.1f}s = {s['hz']:.1f} Hz")

    print("\n--- LINK ---")
    print(f"  latency (stamp -> receive): med {s['lat_med']:.2f} ms  "
          f"p95 {s['lat_p95']:.2f} ms")
    print("    ^ ROS transport only. vicon_receiver stamps on receive, not at")
    print("      capture, so camera-to-bridge delay is NOT in this number.")
    print(f"  dropouts > {s['drop_thresh_ms']:.1f} ms (2x measured period): "
          f"{s['drops']}, longest {s['max_drop_ms']:.1f} ms")
    print("    ^ an OCCLUDED body does not show up here: the bridge keeps")
    print("      publishing at full rate with position (0,0,0). See below.")

    print("\n--- POSITION (is the object stationary?) ---")
    for name, v in (("x", xs), ("y", ys), ("z", zs)):
        m, sd, lo, hi = stats(v)
        print(f"  {name}: mean {m:+.4f} m   sd {sd*1000:6.2f} mm   "
              f"range {(hi-lo)*1000:7.2f} mm")
    span = max(max(v) - min(v) for v in (xs, ys, zs))
    rspan = max(trimmed_span(v) for v in (xs, ys, zs))
    stationary = rspan < 0.01
    print(f"  -> {'STATIONARY' if stationary else 'MOVING'} "
          f"(robust span {rspan*1000:.1f} mm, raw {span*1000:.1f} mm)")
    if stationary and span > 0.01:
        print("     (raw span inflated by brief outliers -- almost certainly the")
        print("      origin jumping between solver solutions, not real motion)")

    at_origin = sum(1 for r in rows if abs(r[2]) < 1e-9 and abs(r[3]) < 1e-9
                    and abs(r[4]) < 1e-9)
    if at_origin:
        print(f"  *** {at_origin} frame(s) at EXACTLY (0,0,0): that is the")
        print("      SDK's occluded-segment value, not a measurement. ***")

    print("\n--- TEMPLATE Z AXIS vs VICON +Z (convention-free) ---")
    mt, sdt, lot, hit = stats(tilts)
    print(f"  tilt: mean {mt:7.2f} deg  sd {sdt:5.2f}  min {lot:7.2f}  max {hit:7.2f}")
    inverted = mt > 90.0
    if inverted:
        print("  *** THE RIGID BODY IS UPSIDE DOWN -- ITS Z AXIS POINTS DOWN ***")
        print(f"  {180.0-mt:.2f} deg off an exact half-turn. Injecting this tells")
        print("  the EKF the drone is inverted and the attitude controller drives")
        print("  toward it the moment motors spool. DO NOT FLY.")
    elif mt > 10.0:
        print(f"  Tilted {mt:.2f} deg. Cross-check against gravity: python3 imu_tilt.py")

    print("\n--- ORIENTATION (Euler, reference only) ---")
    rolls = [rpy_deg(*q)[0] for q in quats]
    pitches = [rpy_deg(*q)[1] for q in quats]
    for name, v in (("roll", rolls), ("pitch", pitches), ("yaw", yaws)):
        m, sd, lo, hi = stats(v)
        note = ("   <- wraparound at +/-180, not instability"
                if name == "roll" and
                sum(1 for a in v if abs(abs(a) - 180) < 5) > len(v) * 0.9 else "")
        print(f"  {name}: mean {m:+8.2f} deg  sd {sd:6.2f}  "
              f"min {lo:+8.2f}  max {hi:+8.2f}{note}")

    print("\n--- YAW STEP ANALYSIS (wrapped) ---")
    steps = sorted(((abs(wrap180(yaws[i] - yaws[i-1])), ts[i] - ts[i-1], i)
                    for i in range(1, len(rows))), reverse=True)
    big = [s_ for s_ in steps if s_[0] > 45.0]
    print(f"  max single-frame yaw step: {steps[0][0]:.2f} deg in "
          f"{steps[0][1]*1000:.2f} ms ({steps[0][0]/max(steps[0][1],1e-9):.0f} deg/s)")
    print(f"  steps > 45 deg (solver flips): {len(big)}")
    for s_ in big[:10]:
        i = s_[2]
        print(f"     idx {i}: {yaws[i-1]:+8.2f} -> {yaws[i]:+8.2f} "
              f"({s_[0]:.1f} deg in {s_[1]*1000:.2f} ms)")
    unw = [yaws[0]]
    for i in range(1, len(yaws)):
        unw.append(unw[-1] + wrap180(yaws[i] - yaws[i-1]))
    print(f"  unwrapped yaw drift {unw[-1]-unw[0]:+.2f} deg, "
          f"range {max(unw)-min(unw):.2f} deg")

    # Count flip EVENTS, not steps: an out-and-back excursion produces two
    # steps over 45 deg, so raw step counts read double.
    modal = sorted(yaws)[len(yaws) // 2]
    off = [i for i, v in enumerate(yaws) if abs(wrap180(v - modal)) > 20.0]
    events, cur = [], []
    for i in off:
        if cur and i == cur[-1] + 1:
            cur.append(i)
        else:
            if cur:
                events.append(cur)
            cur = [i]
    if cur:
        events.append(cur)

    print("\n--- VERDICT ---")
    if inverted:
        print("  *** RIGID BODY IS UPSIDE DOWN -- DO NOT FLY ***")
        if events:
            alt = sum(tilts[i] for e in events for i in e) / sum(len(e) for e in events)
            print(f"  The solver also visits a second solution ({len(events)}x in")
            print(f"  {dur:.0f}s) whose tilt is {alt:.2f} deg, i.e. the UPRIGHT one:")
            print("  the marker set has a 2-fold symmetry about a horizontal axis.")
            print("  Coplanar markers do this. RAISE ONE MARKER on a stalk, then")
            print("  delete and re-create the rigid body.")
        else:
            print("  Re-create the rigid body with the drone upright, flat and")
            print("  level, nose along Vicon +X. Delete it first -- editing keeps")
            print("  the old template.")
    elif not stationary:
        print("  Object MOVED during the probe -- rerun with the drone untouched.")
    elif events:
        print("  *** ROTATIONAL AMBIGUITY STILL PRESENT ***")
        print(f"  {len(events)} flip event(s) ({len(big)} steps) on a provably")
        print(f"  stationary object, longest {max(len(e) for e in events)} frame(s).")
        print("  The marker layout still maps onto itself under some rotation.")
    else:
        my, sdy, _, _ = stats(yaws)
        print(f"  No solver flips in {dur:.0f}s. Yaw stable at {my:+.2f} (sd {sdy:.2f}).")
        print("  Ambiguity appears FIXED. Remaining question is the CONSTANT offset:")
        print(f"  if the nose is physically on Vicon +X now, the offset is {my:+.2f} deg.")


def main():
    argv = [a for a in sys.argv[1:] if a != "--live"]
    if "--live" in sys.argv:
        vicon_capture(TOPIC, float(argv[0]) if argv else 1e9, live=live_summary)
        return 0
    dur = float(argv[0]) if argv else 30.0
    out = argv[1] if len(argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "vicon_probe.csv")
    rows = vicon_capture(TOPIC, dur)
    if len(rows) < 10:
        print(f"FAIL: only {len(rows)} samples in {dur}s")
        return 1
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["recv_t", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw",
                    "roll", "pitch", "yaw"])
        for r in rows:
            w.writerow(list(r) + list(rpy_deg(*r[5:9])))
    analyse(rows, rows[-1][0] - rows[0][0])
    print(f"\n[csv] {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
