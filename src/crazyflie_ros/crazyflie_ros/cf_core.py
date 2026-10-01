#!/usr/bin/env python3
"""The safety and control core, shared by both front ends.

Imported by crazyflie_vicon_teleop.py (standalone) AND by
src/crazyflie_ros/crazyflie_ros/crazyflie_server.py (the ROS 2 driver), which
imports an identical copy kept in its package; tests/test_duplicates.py fails
the moment the two differ.

Two programs can spin these motors. A guard that exists in one and not the
other is worse than no guard, because the two paths then behave differently
under exactly the conditions where you can least afford a surprise. So the
decision logic lives here once and both call it.

DESIGN RULE: everything here is PURE. No ROS at module scope, no cflib, no
printing, no clocks read implicitly -- time is always passed in, and watchdog
state is passed in and handed back rather than stored. That is what makes the
guards testable without hardware. vicon_capture() imports rclpy INSIDE the
function so this stays importable on a machine with no ROS.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

CORE_REVISION = "core-r1 2026-08-26  (extracted from teleop r12, no behaviour change)"

# ==============================================================================
# CONSTANTS
# ==============================================================================
M_PER_FT = 0.3048

CONTROL_HZ = 50.0          # setpoint transmit rate
EXTPOSE_HZ = 50.0          # mocap -> EKF injection rate (keep <= 100 on 2M radio)
MOCAP_EXPECTED_HZ = 100.0  # what Vicon should be publishing at

# --- Safety thresholds --------------------------------------------------------
# Every value below is one dense line of WHY. Do not change one without knowing
# which failure it was written for.
MIN_FLIP_DEG = 45.0        # smallest yaw step treated as a solver flip; jitter is 1-2 deg
FLIP_GAP_CAP_S = 0.05      # cap the rate divisor. Uncapped, a rejected frame stops the
                           # reference advancing, the gap grows, and ~130 ms later the
                           # same 90 deg flip implies <720 deg/s and is ACCEPTED -- the
                           # EKF then holds a permanently wrong yaw, which rotates the
                           # position loop. That IS the spiral this filter prevents.
PREARM_TILT_DEG = 30.0     # refuse to arm above this. A steady-but-INVERTED rigid body
                           # throws zero yaw rejects, so only an absolute check sees it.
QUAT_DEAD_S = 0.50         # orientation rejected this long while flying -> land
MOCAP_STALE_S = 0.15       # no fresh pose this long -> freeze setpoint (EKF dead-reckons)
MOCAP_DEAD_S = 0.40        # no fresh pose this long -> cut motors; IMU-only drift is now
                           # metres/s and a 1 m fall is cheaper than a wall
EKF_DIVERGE_M = 0.35       # onboard estimate vs Vicon truth mismatch -> land
EKF_DIVERGE_HOLD_S = 0.30  # ...sustained this long, so one bad frame does not trigger it
BATT_WARN_V = 3.40
BATT_LAND_V = 3.20
LEASH_M = 0.45             # setpoint may never sit further than this from the drone. If
                           # it is blocked (netting, bad tune) and you hold a key, the
                           # setpoint runs metres ahead and it LAUNCHES when it breaks
                           # free. Capping commanded error caps achievable acceleration.

# Two-stage geofence, same philosophy as the mocap watchdog: escalate, do not jump
# straight to cutting motors. A drone 30 cm out at 0.6 m is better landed than dropped.
GEOFENCE_MARGIN_M = 0.30   # soft bounds + this -> auto-LAND
GEOFENCE_KILL_M = 0.80     # soft bounds + this -> cut motors

# Auto-centred on the drone at startup: a fixed box around the Vicon origin is wrong
# whenever that origin sits in a corner of the capture space, which would put a
# grounded drone "out of bounds" before it ever moved.
DEFAULT_VOLUME = (2.0, 2.0, 1.20)   # width(x) m, depth(y) m, ceiling(z) m

# --- Motion rates -------------------------------------------------------------
POS_SPEED_MS = 0.45
POS_CLIMB_MS = 0.35
YAW_RATE_DPS = 70.0
TURBO = 2.0

DEFAULT_HOLD_FT = 2.0      # the "hold at 2 feet" button

ATT_MAX_ANGLE_DEG = 12.0
ATT_YAWRATE_DPS = 120.0
ATT_THRUST_HOVER = 38000
ATT_THRUST_MIN = 10001
ATT_THRUST_MAX = 55000
ATT_THRUST_STEP = 12000    # per second

TAKEOFF_CLIMB_MS = 0.35
LAND_DESCENT_MS = 0.30
LAND_CUTOFF_Z = 0.09

KALMAN_VAR_THRESHOLD = 0.001
KALMAN_CONVERGE_WINDOW = 10

# Firmware-side commander watchdog, from crazyflie-firmware supervisor.c:
#   COMMANDER_WDT_TIMEOUT_STABILIZE  M2T(500)   -> levels out and holds
#   COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)  -> cuts motors, drone falls
# Any command-source watchdog we impose must fire well inside the first of
# these, so that WE decide what happens rather than the firmware.
FW_WDT_STABILIZE_S = 0.50
FW_WDT_SHUTDOWN_S = 2.00
COMMAND_TIMEOUT_S = 0.30   # no cmd_* from any source for this long while flying
                           # -> hold position where we are. Deliberately below
                           # FW_WDT_STABILIZE_S.


# ==============================================================================
# QUATERNION / POSE
# ==============================================================================
def wrap180(d: float) -> float:
    """Wrap an angle in degrees to (-180, 180]."""
    return (d + 180.0) % 360.0 - 180.0


def quat_norm(qx, qy, qz, qw) -> float:
    return math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)


def yaw_deg_of(qx, qy, qz, qw) -> float:
    """Yaw about +Z from the quaternion (ENU, right-handed)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def tilt_deg_of(qx, qy, qz, qw) -> float:
    """Angle between the body's reported Z axis and world +Z.

    Convention-free: it reads only `R[2][2] = 1 - 2(qx^2 + qy^2)`, so no Euler
    order, no roll/pitch sign convention, and -- the point of it -- no +/-180
    wrap. A body created upside down reports Euler roll near +/-180, where a
    naive reading looks like plausible noise; this returns ~180 and cannot be
    mistaken for anything else.
    """
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.degrees(math.acos(max(-1.0, min(1.0, r22))))


# The Vicon SDK returns translation [0,0,0] and quaternion [1,0,0,0] for a
# segment it cannot see. vicon_receiver never reads the Occluded flag, so the
# topic keeps publishing those AT FULL RATE and perfectly FRESH -- meaning no
# staleness watchdog can see an occlusion, and the position controller flies
# toward the Vicon origin at full authority. The quaternion even has norm 1, so
# the zero-norm guard passes it.
#
# Exact equality is deliberate. These are the SDK's literal sentinel values,
# not a measurement, and a real pose is never bit-exactly zero in all three
# axes with a bit-exact identity-about-X quaternion. Both halves are required
# so a drone genuinely sitting near the origin is never rejected.
def occluded_sentinel(x, y, z, qx, qy, qz, qw):
    """True if this frame is the SDK's occluded-segment sentinel."""
    return (x == 0.0 and y == 0.0 and z == 0.0
            and qx == 1.0 and qy == 0.0 and qz == 0.0 and qw == 0.0)


@dataclass
class Pose:
    """One mocap sample, already in the Crazyflie's expected convention."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    stamp: float = 0.0        # header stamp, seconds
    recv_time: float = 0.0    # local wall clock at receive
    quat_ok: bool = True      # False if the orientation failed a sanity check

    @property
    def yaw_rad(self) -> float:
        siny_cosp = 2.0 * (self.qw * self.qz + self.qx * self.qy)
        cosy_cosp = 1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw_rad)

    @property
    def tilt_deg(self) -> float:
        return tilt_deg_of(self.qx, self.qy, self.qz, self.qw)


# ==============================================================================
# FLIP FILTER
# ==============================================================================
def flip_check(yaw, last_yaw, last_yaw_t, now, max_yaw_rate_dps):
    """Decide whether this yaw sample is a mocap solver flip.

    Returns (rejected, d_deg, rate_dps). State is the caller's; last_yaw /
    last_yaw_t are the last ACCEPTED sample and must NOT advance on rejection.

    BOTH conditions are required: an absolute jump > MIN_FLIP_DEG (a flip is
    ~90 or ~180 deg, no drone turns 45 deg in one frame) AND rate > threshold
    (so a genuine fast rotation seen across a long gap is not called a flip).
    Rate alone is far too twitchy at 240 Hz, where 4 deg of jitter over 4 ms
    already implies ~1000 deg/s. The divisor is capped -- see FLIP_GAP_CAP_S.
    """
    if last_yaw is None or last_yaw_t is None:
        return (False, 0.0, 0.0)
    gap = now - last_yaw_t
    if gap <= 1e-6:
        return (False, 0.0, 0.0)
    d = abs(wrap180(yaw - last_yaw))
    rate = d / min(gap, FLIP_GAP_CAP_S)
    return (d > MIN_FLIP_DEG and rate > max_yaw_rate_dps, d, rate)


# ==============================================================================
# GEOFENCE
# ==============================================================================
@dataclass
class Bounds:
    xmin: float = -1.2
    xmax: float = 1.2
    ymin: float = -1.2
    ymax: float = 1.2
    zmin: float = 0.05
    zmax: float = 1.50

    def clamp(self, x, y, z):
        return (min(max(x, self.xmin), self.xmax),
                min(max(y, self.ymin), self.ymax),
                min(max(z, self.zmin), self.zmax))

    def outside(self, x, y, z, margin=0.0):
        # NOTE: no lower margin on z. The floor is at z~0 and zmin is a setpoint
        # floor, not a violation -- a drone sitting on the ground is inside the
        # volume, it is just below the minimum commanded altitude.
        return not (self.xmin - margin <= x <= self.xmax + margin and
                    self.ymin - margin <= y <= self.ymax + margin and
                    z <= self.zmax + margin)

    def outside_hard(self, x, y, z):
        return self.outside(x, y, z, GEOFENCE_MARGIN_M)

    def outside_kill(self, x, y, z):
        return self.outside(x, y, z, GEOFENCE_KILL_M)

    @classmethod
    def centered_on(cls, x, y, w, d, h, zmin=0.05):
        return cls(x - w / 2.0, x + w / 2.0,
                   y - d / 2.0, y + d / 2.0, zmin, h)

    def describe(self) -> str:
        return (f"x[{self.xmin:+.2f},{self.xmax:+.2f}] "
                f"y[{self.ymin:+.2f},{self.ymax:+.2f}] "
                f"z[{self.zmin:.2f},{self.zmax:.2f}]")

    def as_cli(self) -> str:
        return (f"--bounds {self.xmin:.2f} {self.xmax:.2f} "
                f"{self.ymin:.2f} {self.ymax:.2f} "
                f"{self.zmin:.2f} {self.zmax:.2f}")


class State:
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    FLYING = "FLYING"
    LANDING = "LANDING"
    STOPPED = "STOPPED"

    FLYING_STATES = (TAKEOFF, FLYING, LANDING)


class Mode:
    POSITION = "POSITION"
    ATTITUDE = "ATTITUDE"


# ==============================================================================
# SETPOINT LIMITING
# ==============================================================================
def apply_leash(sp_x, sp_y, sp_z, px, py, pz, leash=LEASH_M):
    """Clamp the setpoint to a fixed radius around the MEASURED position. See LEASH_M."""
    dx, dy, dz = sp_x - px, sp_y - py, sp_z - pz
    d = math.sqrt(dx * dx + dy * dy + dz * dz)
    if d > leash:
        k = leash / d
        return (px + dx * k, py + dy * k, pz + dz * k)
    return (sp_x, sp_y, sp_z)


def clamp_alt(z, lo, hi):
    """Keep a requested altitude inside the flight volume. Returns (z, clamped)."""
    c = min(max(z, lo), hi)
    return (c, c != z)


# ==============================================================================
# PRE-ARM GATE
# ==============================================================================
def prearm_check(pose, bounds, pos_only=False, tilt_limit=PREARM_TILT_DEG):
    """Refuse takeoff BEFORE arming rather than after. Returns (ok, reasons).

    1. UPRIGHT, from `R[2][2]` so the +/-180 Euler wrap cannot disguise it. An
    inverted template reports a steady, well-behaved attitude with ZERO yaw
    rejects, so every flip-based test passes it; injecting it tells the EKF
    the drone is upside down and the controller drives into the floor at
    full thrust the moment motors spool.
    2. GEOFENCE. Checked here so a misconfigured volume costs a message, not a
    drop. Resting on the floor is below zmin and is NOT a violation: zmin
    floors the COMMANDED altitude, not the measured one.
    3. --pos-only withholds the quaternion, so EKF yaw is gyro-only and the
    drone must start near yaw 0 or the position loop begins rotated.
    """
    reasons = []

    tilt = pose.tilt_deg
    if tilt > tilt_limit:
        if tilt > 150.0:
            reasons.append(
                f"rigid body is UPSIDE DOWN: reported tilt {tilt:.1f} deg from "
                f"vertical ({180.0 - tilt:.1f} deg off an exact half-turn). "
                f"Re-create it in Tracker with the drone upright.")
        else:
            reasons.append(
                f"reported tilt {tilt:.1f} deg exceeds the {tilt_limit:.0f} deg "
                f"pre-arm limit. Cross-check against gravity with imu_tilt.py.")

    b = bounds
    inside_xy = (b.xmin <= pose.x <= b.xmax) and (b.ymin <= pose.y <= b.ymax)
    if not inside_xy:
        reasons.append(f"horizontal position ({pose.x:+.2f}, {pose.y:+.2f}) is "
                       f"outside {b.describe()}")
    if pose.z > b.zmax:
        reasons.append(f"already above the ceiling ({pose.z:.2f} > {b.zmax:.2f} m)")

    if pos_only and abs(wrap180(pose.yaw_deg)) > 25.0:
        reasons.append(f"--pos-only needs the drone started near yaw 0; "
                       f"reported {pose.yaw_deg:+.1f} deg")

    return (not reasons, reasons)


# ==============================================================================
# SUPERVISOR
# ==============================================================================
@dataclass
class SuperviseResult:
    action: str | None = None        # None | "land" | "kill"
    reason: str = ""
    mocap_stale: bool = False
    stale_since: float | None = None
    diverge_since: float | None = None
    became_stale: bool = False       # first tick of a stale episode, for logging


def supervise(*, now, flying, pose_age, quat_age, pos_only,
              pos, ekf, got_state, vbat, bounds,
              stale_since, diverge_since, yaw_rejects=0,
              cmd_age=None, command_timeout=None):
    """One tick of the safety supervisor. Pure: no clocks, no printing.

    Explicit severity ("kill" vs "land") rather than the caller substring-
    matching the reason, which is how a reworded log line silently downgrades a
    motor-cut to a landing. Watchdog state is passed in and handed back on the
    result. pos and ekf are (x, y, z) or None. Ordered by severity; each stage
    escalates rather than jumping straight to cutting motors.
    """
    r = SuperviseResult(stale_since=stale_since, diverge_since=diverge_since)

    # --- mocap staleness, two stages -----------------------------------------
    #   STALE (>150 ms): the EKF is dead-reckoning on the IMU. Survivable for a
    #     fraction of a second, but accepting new operator input on a stale
    #     estimate is not. Freeze the setpoint and coast.
    #   DEAD (>400 ms): IMU-only drift is now metres per second. There is no
    #     safe recovery. A 1 m fall is cheaper than a wall.
    r.mocap_stale = pose_age > MOCAP_STALE_S
    if r.mocap_stale:
        if stale_since is None:
            r.stale_since = now
            r.became_stale = True
        if pose_age > MOCAP_DEAD_S and flying:
            r.action, r.reason = "kill", f"MOCAP LOST ({pose_age * 1000:.0f} ms)"
            return r
    else:
        r.stale_since = None

    # --- orientation watchdog -------------------------------------------------
    # The position watchdog above cannot see this. During a latched solver flip
    # the position stream stays fresh, smooth and low-latency while every
    # quaternion is rejected, so extpose silently degrades to extpos and EKF yaw
    # runs on the gyro alone. Yaw error rotates the position loop, and past
    # ~90 deg the error grows instead of decaying: a spiral into the wall, not a
    # drift. Land rather than kill, because position is still trustworthy and a
    # controlled descent is strictly safer than dropping.
    # --pos-only withholds the quaternion deliberately and has its own pre-arm
    # gate, so a rejected orientation there is the stated configuration rather
    # than a silent degradation.
    if flying and not pos_only and quat_age > QUAT_DEAD_S:
        r.action = "land"
        r.reason = (f"MOCAP ORIENTATION UNUSABLE for {quat_age * 1000:.0f} ms "
                    f"({yaw_rejects} rejects) -- rigid body is rotationally "
                    f"ambiguous")
        return r

    # --- command-source watchdog ---------------------------------------------
    # New in the ROS split, and the reason the split needs it. When teleop and
    # the radio live in one process, a dead controller means a dead sender. Once
    # they are separate nodes, the teleop can die, the executor can stall, or
    # DDS can partition, and the driver would happily keep flying on the last
    # setpoint forever. Hold position instead: this is a "stop listening", not
    # an emergency, so it must not land or kill.
    if (flying and command_timeout is not None and cmd_age is not None
            and cmd_age > command_timeout):
        r.action = "hold"
        r.reason = f"no command for {cmd_age * 1000:.0f} ms -- holding position"
        return r

    # --- EKF vs Vicon divergence ---------------------------------------------
    if pos is not None and ekf is not None and got_state:
        err = math.dist(ekf, pos)
        if err > EKF_DIVERGE_M:
            if diverge_since is None:
                r.diverge_since = now
            elif now - diverge_since > EKF_DIVERGE_HOLD_S and flying:
                r.action = "land"
                r.reason = f"EKF DIVERGED from Vicon by {err * 100:.0f} cm"
                return r
        else:
            r.diverge_since = None

    # --- geofence, two stages -------------------------------------------------
    if pos is not None and flying:
        x, y, z = pos
        where = f"({x:+.2f},{y:+.2f},{z:+.2f})"
        if bounds.outside_kill(x, y, z):
            r.action = "kill"
            r.reason = (f"GEOFENCE: {where} is >{GEOFENCE_KILL_M:.2f} m outside "
                        f"{bounds.describe()}")
            return r
        if bounds.outside_hard(x, y, z):
            r.action = "land"
            r.reason = f"GEOFENCE: {where} outside {bounds.describe()}"
            return r

    # --- battery --------------------------------------------------------------
    if vbat and vbat < BATT_LAND_V and flying:
        r.action, r.reason = "land", f"BATTERY CRITICAL ({vbat:.2f} V)"
        return r

    return r


# ==============================================================================
# VICON CAPTURE
# ==============================================================================
# rclpy is imported inside the function, not at module scope, so cf_core stays
# importable (and testable) on a machine with no ROS distro sourced. Four
# passive tools used to carry their own copy of this.
def vicon_capture(topic, duration, live=None, live_period=1.0, quiet=False):
    """Collect PoseStamped samples from `topic` for `duration` seconds.

    Returns [(recv_t, stamp, x, y, z, qx, qy, qz, qw), ...].
    `live(rows, elapsed)` is called every live_period seconds if given.
    """
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

    rows = []

    class _Cap(Node):
        def __init__(self):
            super().__init__("vicon_capture")
            self.create_subscription(PoseStamped, topic, self._cb, 10)

        def _cb(self, msg):
            p, o = msg.pose.position, msg.pose.orientation
            rows.append((time.time(),
                         msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                         p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    owns_context = not rclpy.ok()
    if owns_context:
        rclpy.init()
    n = _Cap()
    if not quiet:
        print(f"[capture] {topic} for {duration:.0f}s ...")
    t0 = last = time.time()
    try:
        while time.time() - t0 < duration:
            rclpy.spin_once(n, timeout_sec=0.05)
            if live and time.time() - last >= live_period:
                last = time.time()
                live(rows, last - t0)
    finally:
        n.destroy_node()
        if owns_context:
            rclpy.shutdown()
    return rows


def capture_stats(rows, expected_hz=None):
    """Rate, stamp->receive latency and inter-arrival gaps for a capture.

    NOTE ON LATENCY: vicon_receiver stamps each message with its own clock when
    it pulls the frame, NOT with the Vicon capture time, so this measures ROS
    transport only (~0.2 ms) and says nothing about camera-to-bridge delay.
    The dropout threshold is derived from the MEASURED rate, because Vicon runs
    at ~237 Hz and a threshold built on an assumed 100 Hz only catches gaps
    about five frames long.
    """
    if len(rows) < 2:
        return None
    ts = [r[0] for r in rows]
    span = ts[-1] - ts[0]
    hz = (len(rows) - 1) / span if span > 0 else 0.0
    lat = sorted((r[0] - r[1]) * 1000.0 for r in rows)
    thresh = 2.0 / (expected_hz or hz or 1.0)
    gaps = [ts[i] - ts[i - 1] for i in range(1, len(ts))]
    drops = [g for g in gaps if g > thresh]
    return {
        "n": len(rows), "dur": span, "hz": hz,
        "lat_med": lat[len(lat) // 2], "lat_p95": lat[int(0.95 * len(lat))],
        "drop_thresh_ms": thresh * 1000.0, "drops": len(drops),
        "max_drop_ms": (max(drops) * 1000.0) if drops else 0.0,
    }
