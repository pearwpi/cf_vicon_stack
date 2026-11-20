#!/usr/bin/env python3
"""Pure logic for the body-rate + collective-thrust (CTBR) command path.

Used by crazyflie_server.py (cmd_rate, IMU log) and scripts/calibrate_thrust.py.
Same design rule as cf_core.py: no ROS, no cflib, no clocks, no printing, so
every decision here is testable without hardware. See docs/RATE_MODE.md.

SIGN CONVENTIONS -- checked against source, 2026-11 master:
  cflib commander.send_setpoint(roll, pitch, yawrate, thrust) packs
      struct.pack('<fffH', roll, -pitch, yawrate, thrust)
  firmware crtp_commander_rpyt.c, with flightmode.stabModeRoll/Pitch = 0 (RATE):
      attitudeRate.roll  =  roll_pkt
      attitudeRate.pitch =  pitch_pkt          (= -cflib pitch)
      attitudeRate.yaw   = -yaw_pkt            ("legacy rate input is inverted")
  firmware controller_pid.c tracks those against
      gyro.x, -gyro.y, and (yaw integrated into an attitude target, then) gyro.z
  and the sensor frame is right-handed body FLU (x fwd, y left, z up). So:
      cflib roll    [deg/s] = +omega_x
      cflib pitch   [deg/s] = +omega_y      (positive = nose DOWN)
      cflib yawrate [deg/s] = -omega_z      (positive = CLOCKWISE seen from above)
  The yaw negation matches the sign flip the driver already applies to
  Hover.yaw_rate. ALL THREE MUST STILL BE CONFIRMED WITH THE PROPS OFF.

THRUST MAP
  The legacy RPYT thrust word u (0..65535) is not a newton value. What it means
  depends on the firmware build:

  * CONFIG_ENABLE_THRUST_BAT_COMPENSATED (default on current Crazyflie 2.x
    builds): motors.c rescales u LINEARLY to newtons per motor,
        f = u / 65535 * THRUST_MAX,   THRUST_MAX = 0.12 N   (platform_defaults_cf2.h)
    and inverts the motor-voltage cubic against the live battery voltage, so
    the onboard firmware already does battery compensation. That is the
    default polynomial below (FW_LINEAR_POLY). Sanity check: a 29 g drone
    hovers at u = 0.029*9.81/0.48*65535 ~ 38.8k, and cf_core.ATT_THRUST_HOVER
    is 38000 -- the number the standalone teleop was hand-tuned to.
  * Compensation disabled / old firmware: u is raw PWM duty and thrust depends
    on battery voltage. Then use Foerster's identification of the stock
    Crazyflie 2.0 motors+props (J. Foerster, "System Identification of the
    Crazyflie 2.0 Nano Quadrocopter", ETH Zurich BSc thesis, 2015):
        f [N per motor] = 2.130295e-11 u^2 + 1.032633e-6 u + 5.484560e-4
    (FOERSTER_POLY). It was measured at roughly full battery and on 2.0 props,
    not the 2.1+ props.
  Either way, thrust_scale (from scripts/calibrate_thrust.py) corrects the model
  so that the measured hover command maps exactly to m*g.
"""
from __future__ import annotations

import math

G = 9.80665

# per-motor force [N] = a0 + a1*u + a2*u^2, u = legacy thrust word
FW_LINEAR_POLY = (0.0, 0.12 / 65535.0, 0.0)
FOERSTER_POLY = (5.484560e-4, 1.032633e-6, 2.130295e-11)

# Training caps (isaaclab.bodyrate_rp_max_deg / bodyrate_yaw_max_deg).
RATE_RP_MAX_DPS = 120.0
RATE_YAW_MAX_DPS = 360.0
RATE_TILT_LIMIT_DEG = 60.0
# Rate output needs an attitude sample no older than this (2 control ticks).
RATE_ATT_MAX_AGE_S = 0.10

# Which flightmode params must read RATE (0) before cmd_rate is honoured.
# stabModeYaw is RATE by firmware default and is only VERIFIED, never written.
FLIGHTMODE_WRITE = ("flightmode.stabModeRoll", "flightmode.stabModePitch")
FLIGHTMODE_CHECK = FLIGHTMODE_WRITE + ("flightmode.stabModeYaw",)
RATE, ANGLE = 0, 1
# Firmware defaults. Anything else, found at connect, is stale RAM state from
# an earlier session and is restored (roll/pitch -> 1, yaw -> 0).
FLIGHTMODE_DEFAULT = {"flightmode.stabModeRoll": ANGLE,
                      "flightmode.stabModePitch": ANGLE,
                      "flightmode.stabModeYaw": RATE}
# Params that change how the legacy RPYT decoder interprets a packet:
# althold -> thrust is a climb rate, poshold/posSet -> roll/pitch are
# velocities/position offsets. All must read 0 before rates are handed off.
DECODER_PARAMS = ("flightmode.althold", "flightmode.poshold",
                  "flightmode.posSet")
# A param value only counts as verified if the firmware reported it this
# recently (ack of our write, or a readback), and while flying it is re-read
# every PARAM_REFRESH_S.
PARAM_MAX_AGE_S = 1.5
PARAM_REFRESH_S = 0.5
FM_RETRY_S = 0.5
# The thrust unlock (zero-thrust packet) is lost if the setpoint stream stops
# (the firmware commander times out and re-locks the decoder).
UNLOCK_TX_GAP_S = 1.0


# ==============================================================================
# THRUST
# ==============================================================================
def motor_force(u, poly=FW_LINEAR_POLY):
    """Per-motor force [N] at thrust word u."""
    a0, a1, a2 = poly
    return a0 + a1 * u + a2 * u * u


def check_poly(poly):
    """A usable map is increasing on [0, 65535]. Returns (ok, reason)."""
    if len(poly) != 3 or not all(math.isfinite(float(c)) for c in poly):
        return False, f"thrust_poly must be 3 finite numbers, got {poly}"
    a0, a1, a2 = (float(c) for c in poly)
    # derivative a1 + 2 a2 u must be > 0 at both ends of the range
    if a1 <= 0.0 or a1 + 2.0 * a2 * 65535.0 <= 0.0:
        return False, "thrust_poly is not increasing over 0..65535"
    return True, ""


def thrust_to_cmd(thrust_n, poly=FW_LINEAR_POLY, scale=1.0,
                  lo=10001, hi=55000):
    """Collective thrust [N] -> clamped legacy thrust word. Returns (u, clamped).

    The four motors are assumed to share the collective equally (the PID's
    roll/pitch/yaw corrections are added on top by the firmware mixer).
    `scale` multiplies the requested thrust before inversion; calibration sets
    it so that m*g lands on the measured hover command.
    """
    f = max(0.0, float(thrust_n)) * float(scale) / 4.0
    a0, a1, a2 = (float(c) for c in poly)
    if a2 == 0.0:
        u = (f - a0) / a1
    else:
        disc = a1 * a1 - 4.0 * a2 * (a0 - f)
        u = (-a1 + math.sqrt(max(disc, 0.0))) / (2.0 * a2)
    c = min(max(u, lo), hi)
    return int(round(c)), (c != u)


def cmd_to_thrust(u, poly=FW_LINEAR_POLY, scale=1.0):
    """Inverse of thrust_to_cmd (unclamped): thrust word -> collective [N]."""
    return 4.0 * motor_force(float(u), poly) / float(scale)


def fit_thrust_scale(hover_cmd, mass_kg, poly=FW_LINEAR_POLY):
    """scale such that thrust_to_cmd(mass*g, scale=...) == hover_cmd."""
    return 4.0 * motor_force(float(hover_cmd), poly) / (float(mass_kg) * G)


def robust_hover(samples, trim=0.1):
    """Trimmed mean of a hover thrust-word capture (drops spool-up spikes)."""
    xs = sorted(float(s) for s in samples)
    if not xs:
        return None
    k = int(len(xs) * trim)
    core = xs[k:len(xs) - k] or xs
    return sum(core) / len(core)


# ==============================================================================
# COMMAND VALIDATION
# ==============================================================================
def to_setpoint(roll_rate, pitch_rate, yaw_rate, thrust, thrust_pwm=0, *,
                rp_max_dps=RATE_RP_MAX_DPS, yaw_max_dps=RATE_YAW_MAX_DPS,
                poly=FW_LINEAR_POLY, scale=1.0, pwm_lo=10001, pwm_hi=55000):
    """RateThrust fields (rad/s FLU, N) -> cflib send_setpoint args.

    Returns (ok, reason, setpoint, clamped) with setpoint =
    (roll_dps, pitch_dps, yawrate_dps_cflib, thrust_word). Non-finite input is
    REJECTED (never clamped into something plausible); out-of-range finite
    input is saturated, like the training env's action clip.
    """
    vals = (roll_rate, pitch_rate, yaw_rate, thrust)
    if not all(math.isfinite(float(v)) for v in vals):
        return False, f"non-finite field in {vals}", None, False
    clamped = False

    def sat(v, m):
        nonlocal clamped
        c = min(max(v, -m), m)
        clamped |= (c != v)
        return c

    r = sat(math.degrees(roll_rate), rp_max_dps)
    p = sat(math.degrees(pitch_rate), rp_max_dps)
    y = sat(math.degrees(yaw_rate), yaw_max_dps)
    if thrust_pwm:
        u = int(min(max(int(thrust_pwm), pwm_lo), pwm_hi))
        clamped |= (u != int(thrust_pwm))
    else:
        u, c = thrust_to_cmd(thrust, poly, scale, pwm_lo, pwm_hi)
        clamped |= c
    # FLU -> cflib legacy: roll +, pitch + (cflib negates, firmware negates
    # back), yaw NEGATED (firmware inverts legacy yaw rate). See module doc.
    return True, "", (r, p, -y, u), clamped


# ==============================================================================
# MODE GATE
# ==============================================================================
def rate_mode_confirmed(flightmode):
    """True only if the firmware has REPORTED rate on roll, pitch AND yaw.

    `flightmode` maps full param name -> int or None (unknown). Unknown is
    not rate: sending a rate as an angle would command a 120 deg tilt.
    """
    return all(flightmode.get(n) == RATE for n in FLIGHTMODE_CHECK)


def decoder_config_ok(values):
    """True only if every decoder-affecting param was READ and is 0.
    Missing from the firmware TOC (None) is refusal, never assumed 0."""
    return all(values.get(n) == 0 for n in DECODER_PARAMS)


def rate_gate(*, enable, setup, confirmed, state):
    """Whether a cmd_rate message may be stored. Returns (ok, reason)."""
    if not enable:
        return False, "cmd_rate rejected: enable_rate_cmd is false"
    if not setup:
        return False, ("cmd_rate rejected: rate_mode_setup is false, so the "
                       "firmware flight mode was not switched to rate")
    if not confirmed:
        return False, ("cmd_rate rejected: firmware has not confirmed "
                       "flightmode.stabModeRoll/Pitch/Yaw = 0 (rate)")
    if state != "FLYING":
        return False, f"cmd_rate rejected: state is {state}, not FLYING"
    return True, ""


def tilt_exceeded(vicon_tilt_deg, onboard_rp_deg=None,
                  limit=RATE_TILT_LIMIT_DEG):
    """Rate-mode tilt guard. Vicon tilt is convention-free (cf_core.tilt_deg_of);
    onboard (roll, pitch) is a fallback/second opinion when it is logged."""
    if vicon_tilt_deg is not None and vicon_tilt_deg > limit:
        return True
    if onboard_rp_deg is not None:
        r, p = onboard_rp_deg
        if abs(r) > limit or abs(p) > limit:
            return True
    return False


def attitude_check(*, mocap_stale, vicon_tilt, vicon_age,
                   onboard_rp, onboard_age,
                   max_age=RATE_ATT_MAX_AGE_S, limit=RATE_TILT_LIMIT_DEG):
    """May rate setpoints be sent THIS tick? Returns (status, detail) with
    status "ok" | "tilt" | "stale".

    Under rates nothing onboard levels the drone, so a rate packet is only sent
    while at least one FRESH, FINITE attitude sample proves the tilt is inside
    the limit. vicon_tilt/vicon_age describe the last quaternion the flip
    filter ACCEPTED (None / huge age when there never was one); onboard_* the
    last stabilizer.roll/pitch sample. Every sample is age-gated, so a frozen
    or rejected stream can never be reused. A stale mocap POSITION also fails:
    the EKF is dead-reckoning and nothing may be commanded on it.
    """
    def good(v, age):
        return (v is not None and age is not None and math.isfinite(age)
                and age <= max_age)

    if mocap_stale:
        return "stale", "mocap pose stale"
    v_ok = good(vicon_tilt, vicon_age) and math.isfinite(vicon_tilt)
    o_ok = (good(onboard_rp, onboard_age)
            and all(math.isfinite(x) for x in onboard_rp))
    if not (v_ok or o_ok):
        return "stale", ("no fresh valid attitude (vicon age "
                         f"{vicon_age}, onboard age {onboard_age})")
    if tilt_exceeded(vicon_tilt if v_ok else None,
                     onboard_rp if o_ok else None, limit):
        return "tilt", (f"tilt over {limit:.0f} deg (vicon {vicon_tilt if v_ok else None}, "
                        f"onboard {onboard_rp if o_ok else None})")
    return "ok", ""


# ==============================================================================
# IMU
# ==============================================================================
def log_period_ms(hz):
    """cflib log periods are multiples of 10 ms, 10..2550. Nearest feasible."""
    hz = max(float(hz), 0.4)
    return int(min(max(round(100.0 / hz) * 10, 10), 2550))


def imu_si(gx, gy, gz, ax, ay, az):
    """gyro deg/s, acc g (body FLU) -> (rad/s, m/s^2), still body FLU.

    acc is specific force, so a drone at rest reads +9.81 on z, which is the
    sensor_msgs/Imu (REP-145) convention.
    """
    d = math.radians
    return (d(gx), d(gy), d(gz)), (ax * G, ay * G, az * G)
