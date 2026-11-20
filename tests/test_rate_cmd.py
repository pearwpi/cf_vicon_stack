#!/usr/bin/env python3
"""Pure tests for crazyflie_ros/rate_cmd.py (no ROS, no cflib).

    python3 -m pytest tests/test_rate_cmd.py -q
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "src", "crazyflie_ros"))

from crazyflie_ros import rate_cmd as R   # noqa: E402

LO, HI = 10001, 55000


# --- thrust map ---------------------------------------------------------------
def test_linear_map_matches_firmware_rescale():
    # motors.c: f = u/65535 * THRUST_MAX, THRUST_MAX = 0.12 N per motor
    u, c = R.thrust_to_cmd(0.24, lo=0, hi=65535)
    assert u == round(0.06 / 0.12 * 65535) and not c


def test_linear_map_reproduces_hand_tuned_hover():
    # cf_core.ATT_THRUST_HOVER = 38000 was tuned by hand on a ~29 g drone
    u, _ = R.thrust_to_cmd(0.029 * R.G)
    assert abs(u - 38000) < 1500


def test_foerster_map_round_trips():
    for t in (0.1, 0.25, 0.4):
        u, c = R.thrust_to_cmd(t, R.FOERSTER_POLY, lo=0, hi=65535)
        assert not c
        assert abs(R.cmd_to_thrust(u, R.FOERSTER_POLY) - t) < 1e-4


def test_foerster_hover_is_plausible():
    u, _ = R.thrust_to_cmd(0.027 * R.G, R.FOERSTER_POLY)
    assert 35000 < u < 50000


def test_clamps_to_pwm_range():
    assert R.thrust_to_cmd(0.0) == (LO, True)
    assert R.thrust_to_cmd(-5.0)[0] == LO
    assert R.thrust_to_cmd(10.0) == (HI, True)


def test_monotonic():
    us = [R.thrust_to_cmd(t / 100.0, R.FOERSTER_POLY, lo=0, hi=65535)[0]
          for t in range(5, 45)]
    assert us == sorted(us)


def test_scale_from_calibration_hits_measured_hover():
    for poly in (R.FW_LINEAR_POLY, R.FOERSTER_POLY):
        s = R.fit_thrust_scale(43000, 0.034, poly)
        u, _ = R.thrust_to_cmd(0.034 * R.G, poly, s)
        assert abs(u - 43000) <= 1


def test_check_poly():
    assert R.check_poly(R.FW_LINEAR_POLY)[0]
    assert R.check_poly(R.FOERSTER_POLY)[0]
    assert not R.check_poly((0.0, 0.0, 0.0))[0]
    assert not R.check_poly((0.0, -1e-6, 0.0))[0]
    assert not R.check_poly((0.0, 1e-6))[0]
    assert not R.check_poly((0.0, float("nan"), 0.0))[0]


def test_robust_hover_trims_spikes():
    xs = [40000] * 90 + [60000] * 5 + [10000] * 5
    assert abs(R.robust_hover(xs) - 40000) < 1e-6
    assert R.robust_hover([]) is None


# --- command conversion -------------------------------------------------------
def test_sign_and_units():
    ok, _, sp, c = R.to_setpoint(math.radians(10), math.radians(20),
                                 math.radians(30), 0.3)
    assert ok and not c
    r, p, y, u = sp
    assert abs(r - 10) < 1e-9          # roll: +omega_x
    assert abs(p - 20) < 1e-9          # pitch: +omega_y (cflib & fw negate twice)
    assert abs(y + 30) < 1e-9          # yaw: firmware inverts legacy yaw rate
    assert isinstance(u, int)


def test_rate_caps():
    ok, _, sp, c = R.to_setpoint(10.0, -10.0, 100.0, 0.3)
    assert ok and c
    assert sp[0] == 120.0 and sp[1] == -120.0 and sp[2] == -360.0
    ok, _, sp, _ = R.to_setpoint(10.0, 0, 0, 0.3, rp_max_dps=60.0)
    assert sp[0] == 60.0


def test_non_finite_rejected():
    for bad in (float("nan"), float("inf"), -float("inf")):
        for i in range(4):
            args = [0.0, 0.0, 0.0, 0.3]
            args[i] = bad
            ok, why, sp, _ = R.to_setpoint(*args)
            assert not ok and sp is None and why


def test_pwm_override():
    ok, _, sp, c = R.to_setpoint(0, 0, 0, 99.0, 40000)
    assert ok and sp[3] == 40000 and not c
    assert R.to_setpoint(0, 0, 0, 0, 65000)[2][3] == HI
    assert R.to_setpoint(0, 0, 0, 0, 5)[2][3] == LO


# --- gating -------------------------------------------------------------------
FM_RATE = {n: 0 for n in R.FLIGHTMODE_CHECK}


def test_confirmed_needs_all_three():
    assert R.rate_mode_confirmed(FM_RATE)
    for n in R.FLIGHTMODE_CHECK:
        assert not R.rate_mode_confirmed({**FM_RATE, n: 1})
        assert not R.rate_mode_confirmed({**FM_RATE, n: None})
    assert not R.rate_mode_confirmed({})


def test_yaw_is_never_written():
    assert "flightmode.stabModeYaw" not in R.FLIGHTMODE_WRITE


def test_gate():
    base = dict(enable=True, setup=True, confirmed=True, state="FLYING")
    assert R.rate_gate(**base) == (True, "")
    for k, v in (("enable", False), ("setup", False), ("confirmed", False),
                 ("state", "TAKEOFF"), ("state", "LANDING"), ("state", "IDLE"),
                 ("state", "STOPPED")):
        ok, why = R.rate_gate(**{**base, k: v})
        assert not ok and why


def test_tilt_guard():
    assert not R.tilt_exceeded(10.0)
    assert R.tilt_exceeded(61.0)
    assert R.tilt_exceeded(None, (0.0, -65.0))
    assert not R.tilt_exceeded(None, None)
    assert R.tilt_exceeded(30.0, (70.0, 0.0))
    assert not R.tilt_exceeded(50.0, limit=55.0)


# --- imu ----------------------------------------------------------------------
def test_log_period():
    assert R.log_period_ms(100) == 10
    assert R.log_period_ms(1000) == 10
    assert R.log_period_ms(50) == 20
    assert R.log_period_ms(33) == 30
    assert R.log_period_ms(0) == 2500       # floored at 0.4 Hz, never > 2550


def test_imu_si():
    w, a = R.imu_si(180.0, -90.0, 0.0, 0.0, 0.0, 1.0)
    assert abs(w[0] - math.pi) < 1e-12 and abs(w[1] + math.pi / 2) < 1e-12
    assert a == (0.0, 0.0, R.G)


# --- attitude freshness (finding 3) --------------------------------------------
def _chk(**kw):
    base = dict(mocap_stale=False, vicon_tilt=5.0, vicon_age=0.01,
                onboard_rp=None, onboard_age=None)
    base.update(kw)
    return R.attitude_check(**base)[0]


def test_attitude_check_ok_and_stale_and_tilt():
    assert _chk() == "ok"
    assert _chk(vicon_age=0.5) == "stale"
    assert _chk(vicon_tilt=None, vicon_age=1e9) == "stale"
    assert _chk(mocap_stale=True) == "stale"
    assert _chk(vicon_tilt=70.0) == "tilt"


def test_attitude_check_never_reuses_stale_or_nonfinite_samples():
    # stale vicon over the limit is ignored, but then nothing fresh remains
    assert _chk(vicon_tilt=80.0, vicon_age=0.5) == "stale"
    assert _chk(vicon_tilt=float("nan")) == "stale"
    assert _chk(vicon_age=0.5, onboard_rp=(float("nan"), 0.0),
                onboard_age=0.01) == "stale"
    assert _chk(vicon_age=0.5, onboard_rp=(1.0, 2.0), onboard_age=0.5) == "stale"
    assert _chk(vicon_age=0.5, onboard_rp=(1.0, 2.0), onboard_age=0.01) == "ok"
    assert _chk(onboard_rp=(70.0, 0.0), onboard_age=0.01) == "tilt"


def test_decoder_config_requires_all_params_read_as_zero():
    ok = {n: 0 for n in R.DECODER_PARAMS}
    assert R.decoder_config_ok(ok)
    for n in R.DECODER_PARAMS:
        assert not R.decoder_config_ok({**ok, n: 1})
        assert not R.decoder_config_ok({**ok, n: None})      # not in the TOC
    assert not R.decoder_config_ok({})


def test_flightmode_defaults_cover_every_checked_param():
    assert set(R.FLIGHTMODE_DEFAULT) == set(R.FLIGHTMODE_CHECK)
    assert R.FLIGHTMODE_DEFAULT["flightmode.stabModeYaw"] == R.RATE


if __name__ == "__main__":
    # Same calling convention as the other suites, for the Docker build loop.
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
