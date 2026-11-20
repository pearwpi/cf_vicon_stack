#!/usr/bin/env python3
"""Tests for scripts/calibrate_thrust.py: sample hygiene, capture checks and the
landing cleanup, with rclpy and the message packages stubbed (no ROS needed).

    python3 tests/test_calibrate_thrust.py
"""
import importlib.util
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "crazyflie_ros"))

spec = importlib.util.spec_from_file_location(
    "calibrate_thrust", os.path.join(ROOT, "scripts", "calibrate_thrust.py"))
K = importlib.util.module_from_spec(spec)
spec.loader.exec_module(K)

NAN = float("nan")
GOOD = [(float(i), 38000.0 + (i % 5), 3.8) for i in range(60)]


def _refused(fn, *a, **k):
    try:
        fn(*a, **k)
    except SystemExit as exc:
        return "REFUSED" in str(exc) or "no usable" in str(exc)
    return False


# --- sample hygiene -------------------------------------------------------------
def test_clean_samples_drops_nonfinite_and_out_of_range():
    rows = GOOD + [(1.0, NAN, 3.8), (1.0, 38000.0, NAN), (1.0, -5.0, 3.8),
                   (1.0, 70000.0, 3.8), (1.0, 38000.0, 0.0), (1.0, 38000.0, 9.9),
                   (NAN, 38000.0, 3.8), (1.0, float("inf"), 3.8)]
    good, bad = K.clean_samples(rows)
    assert len(good) == len(GOOD) and bad == 8


def test_summarise_refuses_mostly_bad_or_tiny_captures():
    poly = K.R.FW_LINEAR_POLY
    assert K.summarise(GOOD, 0.034, poly)["n"] == len(GOOD)
    assert _refused(K.summarise, GOOD[:5], 0.034, poly)
    assert _refused(K.summarise, GOOD[:30] + [(1.0, NAN, 3.8)] * 30, 0.034, poly)
    assert _refused(K.summarise, [(float(i), 3000.0, 3.8) for i in range(60)],
                    0.034, poly)          # hover word below the driver's range


def test_hover_pwm_dry_run_row_still_fits():
    res = K.summarise([(0.0, 38000.0, NAN)], 0.034, K.R.FW_LINEAR_POLY,
                      vbat_required=False)
    assert res["hover_pwm"] == 38000.0


# --- capture checks -------------------------------------------------------------
def test_capture_problems():
    ok_z, ok_s = [0.5] * 20, [(True, False)] * 5
    assert K.capture_problems(ok_z, ok_s, 0.5) == []
    assert K.capture_problems([0.5] * 3, ok_s, 0.5)                  # too few
    assert K.capture_problems([0.5] * 10 + [0.2] * 10, ok_s, 0.5)    # sank
    assert K.capture_problems([0.5] * 19 + [NAN], ok_s, 0.5)
    assert K.capture_problems(ok_z, [(True, False)] * 4 + [(False, False)], 0.5)
    assert K.capture_problems(ok_z, [(True, True)], 0.5)
    assert K.capture_problems(ok_z, [], 0.5)


# --- landing cleanup, with a stubbed ROS graph -----------------------------------
def _stub_ros(clock, z_of, takeoff_raises=None):
    calls = {"takeoff": 0, "land": 0}
    subs = []

    class Fut:
        def __init__(s, r): s.r = r
        def done(s): return True
        def result(s): return s.r

    class Cli:
        def __init__(s, name): s.srv_name = name
        def wait_for_service(s, timeout_sec=0): return True

        def call_async(s, req):
            if s.srv_name.endswith("takeoff"):
                calls["takeoff"] += 1
                if takeoff_raises:
                    raise takeoff_raises
            if s.srv_name.endswith("land"):
                calls["land"] += 1
            if s.srv_name.endswith("get_parameters"):
                v = types.SimpleNamespace(bool_value=False)
                return Fut(types.SimpleNamespace(values=[v, v, v]))
            return Fut(object())

    class Node:
        def __init__(s, name): pass
        def create_subscription(s, t, topic, cb, qos): subs.append((topic, cb))
        def create_client(s, t, name): return Cli(name)
        def destroy_node(s): pass

    def spin_once(node, timeout_sec=0.0):
        clock[0] += max(timeout_sec, 0.01)
        for topic, cb in subs:
            if topic.endswith("log/thrust"):
                cb(types.SimpleNamespace(values=[38000.0, 3.8]))
            elif topic.endswith("/pose"):
                cb(types.SimpleNamespace(pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(z=z_of(clock[0])))))
            elif topic.endswith("/status"):
                cb(types.SimpleNamespace(supervisor_info=16))

    class Req:
        pass

    mods = {
        "rclpy": types.SimpleNamespace(init=lambda: None, shutdown=lambda: None,
                                       spin_once=spin_once),
        "rclpy.node": types.SimpleNamespace(Node=Node),
        "rcl_interfaces": types.ModuleType("rcl_interfaces"),
        "rcl_interfaces.srv": types.SimpleNamespace(
            GetParameters=type("GP", (), {"Request": Req})),
        "crazyflie_interfaces": types.ModuleType("crazyflie_interfaces"),
        "crazyflie_interfaces.msg": types.SimpleNamespace(
            LogDataGeneric=object,
            Status=type("St", (), {"SUPERVISOR_INFO_IS_FLYING": 16,
                                   "SUPERVISOR_INFO_IS_TUMBLED": 32})),
        "crazyflie_interfaces.srv": types.SimpleNamespace(
            Land=type("L", (), {"Request": Req}),
            Takeoff=type("T", (), {"Request": Req})),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.SimpleNamespace(PoseStamped=object),
    }
    return mods, calls


def _run(z_of=lambda t: 0.5, takeoff_raises=None):
    clock = [1000.0]
    mods, calls = _stub_ros(clock, z_of, takeoff_raises)
    saved = {k: sys.modules.get(k) for k in mods}
    real_time = K.time
    K.time = types.SimpleNamespace(time=lambda: clock[0], sleep=lambda s: None)
    sys.modules.update(mods)
    args = types.SimpleNamespace(prefix="cf1", server="/crazyflie_server",
                                 height=0.5, settle=1.0, duration=3.0,
                                 dry_run=False)
    try:
        try:
            K.run_ros(args, K.R.FW_LINEAR_POLY)
            outcome = "returned"
        except BaseException as exc:          # noqa: BLE001
            outcome = type(exc).__name__ + ": " + str(exc)
    finally:
        K.time = real_time
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return outcome, calls


def test_land_runs_when_the_takeoff_request_itself_raises():
    out, calls = _run(takeoff_raises=RuntimeError("send failed"))
    assert "RuntimeError" in out and calls["takeoff"] == 1 and calls["land"] == 1, \
        (out, calls)


def test_land_runs_on_ctrl_c_during_takeoff_request():
    out, calls = _run(takeoff_raises=KeyboardInterrupt())
    assert "KeyboardInterrupt" in out and calls["land"] == 1, (out, calls)


def test_unstable_altitude_discards_capture_and_lands():
    out, calls = _run(z_of=lambda t: 0.5 if t < 1006.5 else 0.2)
    assert "REFUSED" in out and calls["land"] == 1, (out, calls)


def test_steady_hover_captures_and_lands():
    out, calls = _run()
    assert out == "returned" and calls["land"] == 1, (out, calls)


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"  PASS  {name}")
            except AssertionError as exc:
                fails += 1
                print(f"  FAIL  {name}  {exc}")
    print("ALL CHECKS PASSED" if not fails else f"{fails} FAILED")
    sys.exit(1 if fails else 0)
