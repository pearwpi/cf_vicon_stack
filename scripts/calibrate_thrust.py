#!/usr/bin/env python3
"""calibrate_thrust.py -- measure the hover thrust word, fit the N -> word map.

Flies in the driver's ordinary POSITION mode (never rate mode): takes off
through crazyflie_server, lets it hold a fixed height, records the firmware
controller's thrust output (stabilizer.thrust, the same units as the RPYT
thrust word) and battery voltage from <prefix>/log/thrust, lands, and writes a
parameter YAML for the driver:

    thrust_poly   the model you chose (default: battery-compensated linear)
    thrust_scale  so that  mass * g  maps EXACTLY to the measured hover word

    # 1. with the driver running normally (enable_rate_cmd: false):
    python3 scripts/calibrate_thrust.py --mass 0.034 --height 0.5 \\
        --out thrust_cal.yaml
    # 2. pass it to the driver after the main config:
    ros2 run crazyflie_ros crazyflie_server --ros-args \\
        --params-file crazyflie.yaml --params-file thrust_cal.yaml

No-fly paths:
    --dry-run            connect to the ROS graph, check the topics and the
                         driver's parameters, sample the log for a few seconds
                         ON THE GROUND, never call takeoff. With --hover-pwm,
                         also writes the YAML from that number; add
                         --no-ros to skip the ROS graph entirely.
    --offline CSV        recompute from a CSV this script saved earlier.

WEIGH THE DRONE as flown (battery, markers, mount). The scale is only as good
as --mass. Calibrate on a battery between ~3.9 and 3.7 V; on firmware without
battery compensation the hover word drifts upward as the battery drains, and
the script prints the voltage so you can see which regime you measured in.
"""
import argparse
import csv
import datetime
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
try:
    from crazyflie_ros import rate_cmd as R
except ImportError:                       # run from a source checkout
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "src", "crazyflie_ros"))
    from crazyflie_ros import rate_cmd as R

POLYS = {"linear": R.FW_LINEAR_POLY, "foerster": R.FOERSTER_POLY}


def summarise(rows, mass, poly):
    """rows: [(t, thrust_word, vbat)] captured while hovering."""
    us = [u for _, u, _ in rows if u > 0]
    vs = [v for _, _, v in rows if v > 0]
    hover = R.robust_hover(us)
    if hover is None:
        raise SystemExit("no non-zero thrust samples -- was the drone flying?")
    out = {
        "n": len(us), "hover_pwm": hover,
        "hover_pwm_min": min(us), "hover_pwm_max": max(us),
        "vbat_mean": (sum(vs) / len(vs)) if vs else float("nan"),
        "vbat_first": vs[0] if vs else float("nan"),
        "vbat_last": vs[-1] if vs else float("nan"),
    }
    if mass:
        out["scale"] = R.fit_thrust_scale(hover, mass, poly)
        out["model_hover_pwm"] = R.thrust_to_cmd(mass * R.G, poly, 1.0, 0, 65535)[0]
        # what the fitted map says the extremes are, to compare with training
        out["n_at_min_pwm"] = R.cmd_to_thrust(10001, poly, out["scale"])
        out["n_at_max_pwm"] = R.cmd_to_thrust(55000, poly, out["scale"])
    return out


def render_yaml(res, mass, poly_name, poly, height, node="/crazyflie_server"):
    stamp = datetime.datetime.now().isoformat(timespec="seconds")
    return (
        f"# Written by scripts/calibrate_thrust.py at {stamp}\n"
        f"# mass {mass:.4f} kg, hover at {height:.2f} m, {res['n']} samples\n"
        f"# hover thrust word {res['hover_pwm']:.0f} "
        f"(range {res['hover_pwm_min']:.0f}..{res['hover_pwm_max']:.0f}); "
        f"uncalibrated model predicted {res['model_hover_pwm']}\n"
        f"# battery {res['vbat_first']:.2f} -> {res['vbat_last']:.2f} V "
        f"(mean {res['vbat_mean']:.2f})\n"
        f"# fitted map: 10001 -> {res['n_at_min_pwm']:.3f} N, "
        f"55000 -> {res['n_at_max_pwm']:.3f} N collective\n"
        f"{node}:\n"
        f"  ros__parameters:\n"
        f"    thrust_poly: [{float(poly[0])!r}, {float(poly[1])!r}, {float(poly[2])!r}]"
        f"   # {poly_name}\n"
        f"    thrust_scale: {res['scale']:.6f}\n")


def report(res):
    print(f"hover thrust word : {res['hover_pwm']:.0f}  "
          f"({res['hover_pwm_min']:.0f}..{res['hover_pwm_max']:.0f}, n={res['n']})")
    print(f"battery           : {res['vbat_first']:.2f} -> {res['vbat_last']:.2f} V")
    if "scale" in res:
        print(f"model predicted   : {res['model_hover_pwm']}  ->  thrust_scale "
              f"{res['scale']:.4f}")
        if not 0.7 < res["scale"] < 1.4:
            print("WARNING: scale is far from 1. Check --mass and --poly before "
                  "trusting it (wrong firmware thrust regime?).")
        print(f"fitted collective : {res['n_at_min_pwm']:.3f} N at 10001, "
              f"{res['n_at_max_pwm']:.3f} N at 55000")


def write_outputs(args, rows, res, poly):
    if args.out:
        with open(args.out, "w") as fh:
            fh.write(render_yaml(res, args.mass, args.poly, poly, args.height))
        print(f"wrote {args.out}")
        csv_path = os.path.splitext(args.out)[0] + "_samples.csv"
        if rows and not args.offline:
            with open(csv_path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t", "thrust_word", "vbat"])
                w.writerows(rows)
            print(f"wrote {csv_path}")


# ==============================================================================
# ROS side. rclpy is imported here so --offline and the tests need no ROS.
# ==============================================================================
def run_ros(args, poly):
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from crazyflie_interfaces.msg import LogDataGeneric
    from crazyflie_interfaces.srv import Land, Takeoff

    rclpy.init()
    node = Node("calibrate_thrust")
    p = args.prefix
    rows = []
    node.create_subscription(
        LogDataGeneric, f"{p}/log/thrust",
        lambda m: rows.append((time.time(), float(m.values[0]),
                               float(m.values[1]))) if len(m.values) >= 2 else None,
        10)
    cli_takeoff = node.create_client(Takeoff, f"{p}/takeoff")
    cli_land = node.create_client(Land, f"{p}/land")
    cli_par = node.create_client(GetParameters, f"{args.server}/get_parameters")

    def spin_for(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(node, timeout_sec=0.05)

    def call(cli, req, what):
        if not cli.wait_for_service(timeout_sec=3.0):
            raise SystemExit(f"{what}: service {cli.srv_name} not available")
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        return fut.result()

    landed = True
    try:
        # Refuse to run with the rate path armed: calibration must be a plain
        # position-mode hover, and a stray policy on cmd_rate must not be able
        # to take over mid-capture.
        req = GetParameters.Request()
        req.names = ["enable_rate_cmd", "rate_mode_setup", "no_fly"]
        res = call(cli_par, req, "driver parameters")
        vals = [v.bool_value for v in res.values] if res else []
        if len(vals) == 3:
            print(f"driver: enable_rate_cmd={vals[0]} rate_mode_setup={vals[1]} "
                  f"no_fly={vals[2]}")
            if vals[0] or vals[1]:
                raise SystemExit("REFUSED: restart the driver with "
                                 "enable_rate_cmd:=false rate_mode_setup:=false")
            if vals[2] and not args.dry_run:
                raise SystemExit("REFUSED: driver is in no_fly; use --dry-run")
        else:
            raise SystemExit(f"could not read {args.server} parameters")

        spin_for(2.0)
        print(f"{p}/log/thrust: {len(rows)} samples in 2 s "
              f"(expect ~20){'' if rows else '  -- NONE: is the driver new enough?'}")
        if args.dry_run:
            if rows:
                print(f"on the ground: thrust word {rows[-1][1]:.0f}, "
                      f"vbat {rows[-1][2]:.2f} V")
            return []

        if not rows:
            raise SystemExit("no thrust log; not flying blind")
        rows.clear()
        treq = Takeoff.Request()
        treq.height = float(args.height)
        call(cli_takeoff, treq, "takeoff")
        landed = False
        climb = args.height / 0.35 + 1.0            # cf_core.TAKEOFF_CLIMB_MS
        print(f"climbing ({climb:.1f} s), settling {args.settle:.1f} s ...")
        spin_for(climb + args.settle)
        rows.clear()
        print(f"recording {args.duration:.0f} s of hover ...")
        spin_for(args.duration)
        captured = list(rows)
        return captured
    finally:
        if not landed:
            print("landing")
            try:
                call(cli_land, Land.Request(), "land")
                spin_for(args.height / 0.30 + 1.5)
            except BaseException as exc:
                print(f"LAND CALL FAILED ({exc}) -- use the teleop or /emergency")
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mass", type=float, help="flying mass in kg (weigh it)")
    ap.add_argument("--height", type=float, default=0.5)
    ap.add_argument("--settle", type=float, default=4.0)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--poly", choices=sorted(POLYS), default="linear",
                    help="linear = battery-compensated firmware (default); "
                         "foerster = raw-PWM firmware")
    ap.add_argument("--prefix", default="cf1")
    ap.add_argument("--server", default="/crazyflie_server")
    ap.add_argument("--out", help="parameter YAML to write")
    ap.add_argument("--dry-run", action="store_true",
                    help="never take off; check the graph and sample on the ground")
    ap.add_argument("--hover-pwm", type=float,
                    help="with --dry-run: fit from this hover word instead")
    ap.add_argument("--no-ros", action="store_true",
                    help="with --dry-run --hover-pwm: pure arithmetic, no ROS graph")
    ap.add_argument("--offline", help="recompute from a saved *_samples.csv")
    args = ap.parse_args(argv)
    poly = POLYS[args.poly]

    if not args.dry_run and not args.offline and not args.mass:
        ap.error("--mass is required to fly")

    if args.offline:
        with open(args.offline) as fh:
            rows = [(float(r["t"]), float(r["thrust_word"]), float(r["vbat"]))
                    for r in csv.DictReader(fh)]
    elif args.dry_run and args.hover_pwm and args.no_ros:
        rows = []
    else:
        rows = run_ros(args, poly)

    if args.dry_run and args.hover_pwm:
        rows = [(0.0, args.hover_pwm, float("nan"))]
    if not rows:
        print("dry run complete; nothing to fit")
        return 0
    if not args.mass:
        ap.error("--mass is required to fit")
    res = summarise(rows, args.mass, poly)
    report(res)
    write_outputs(args, rows, res, poly)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
