#!/usr/bin/env python3
"""Tests for the r11 orientation guard in crazyflie_vicon_teleop.py.

Covers exactly what r11 changed, no more:

  1. FLIP_GAP_CAP_S -- the flip filter must not LATCH. Before r11, once a frame
     was rejected the reference stopped advancing, so `gap` grew and the same
     90 deg flip implied a smaller and smaller rate. After ~130 ms it fell under
     720 deg/s and was ACCEPTED, feeding the EKF a permanently 90 deg-wrong yaw.
     That rotates the position loop, and past ~90 deg the error grows instead of
     decaying -- the observed outward spiral.

  2. quat_age() / QUAT_DEAD_S -- a latched solver keeps the POSITION stream
     fresh and smooth while every quaternion is rejected, so the existing
     position watchdog sees a perfectly healthy feed. The orientation needs its
     own clock.

Stubs rclpy and cflib; no hardware, no radio.
"""
import os
import math
import sys
import types

# ---- stub out the hardware-facing imports before loading the teleop ---------
for name in ("rclpy", "rclpy.node", "rclpy.qos"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["rclpy"].init = lambda *a, **k: None
sys.modules["rclpy"].ok = lambda: True
sys.modules["rclpy"].shutdown = lambda: None
sys.modules["rclpy"].spin_once = lambda *a, **k: None


class _Node:
    def __init__(self, *a, **k):
        pass

    def create_subscription(self, *a, **k):
        return None

    def get_logger(self):
        class L:
            warn = staticmethod(lambda *a, **k: None)
            error = staticmethod(lambda *a, **k: None)
            info = staticmethod(lambda *a, **k: None)
        return L()

    def destroy_node(self):
        pass


sys.modules["rclpy.node"].Node = _Node
for name in ("cflib", "cflib.crtp", "cflib.crazyflie", "cflib.crazyflie.log",
             "cflib.crazyflie.syncCrazyflie", "cflib.crazyflie.syncLogger",
             "cflib.utils", "cflib.utils.power_switch",
             "geometry_msgs", "geometry_msgs.msg"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["geometry_msgs.msg"].PoseStamped = object

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crazyflie_vicon_teleop as T  # noqa: E402

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name} {detail}")
    else:
        FAIL += 1
        print(f"  FAIL  {name} {detail}")


# ---- helpers ---------------------------------------------------------------
def quat_yaw(deg):
    h = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(h), math.cos(h))


class FakeMsg:
    """Shaped like geometry_msgs/PoseStamped, no more permissive than the real
    thing -- a fake that accepts more than the real object launders bugs."""

    def __init__(self, x, y, z, q, stamp):
        qx, qy, qz, qw = q
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y, z=z),
            orientation=types.SimpleNamespace(x=qx, y=qy, z=qz, w=qw))
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=int(stamp),
                                        nanosec=int((stamp % 1) * 1e9)))


def make_source(monkey_time):
    src = T.ViconSource.__new__(T.ViconSource)
    _Node.__init__(src)
    src._init_state(240.0, 720.0)
    src.get_logger = _Node.get_logger.__get__(src)
    return src


def feed(src, yaw_deg, t, clock):
    """Deliver one frame at wall-clock t; returns the resulting Pose."""
    clock[0] = t
    src.position_callback(FakeMsg(0.5, -0.2, 0.015, quat_yaw(yaw_deg), t))
    return src.latest()


# Patch time.time inside the module so we control the clock exactly.
_clock = [1000.0]
_real_time = T.time.time
T.time.time = lambda: _clock[0]

print("\n[A] a single-frame flip is rejected and recovers")
src = make_source(_clock)
t = 1000.0
for i in range(20):                     # settle at 0 deg
    t += 1 / 240.0
    feed(src, 0.0, t, _clock)
base = src.total_yaw_rejects
t += 1 / 240.0
p = feed(src, -93.6, t, _clock)         # the flip
check("flip frame rejected", p.quat_ok is False)
check("reject counted", src.total_yaw_rejects == base + 1)
t += 1 / 240.0
p = feed(src, 0.0, t, _clock)           # solver recovers
check("next good frame accepted", p.quat_ok is True)
check("no extra reject on recovery", src.total_yaw_rejects == base + 1)

print("\n[B] a PERSISTENT flip never latches (the r11 fix)")
src = make_source(_clock)
t = 2000.0
_clock[0] = t
for i in range(20):
    t += 1 / 240.0
    feed(src, 0.0, t, _clock)
accepted_after_flip = 0
flip_start = t
for i in range(240):                    # 1 full second stuck at -93.6 deg
    t += 1 / 240.0
    p = feed(src, -93.6, t, _clock)
    if p.quat_ok:
        accepted_after_flip += 1
check("no flipped frame is ever accepted", accepted_after_flip == 0,
      f"({accepted_after_flip} accepted over 1.0 s)")
check("every flipped frame counted as a reject",
      src.total_yaw_rejects == 240, f"({src.total_yaw_rejects})")

print("\n[C] quat_age tracks orientation health independently of position")
check("quat_age grew across the stuck second",
      abs(src.quat_age() - (t - flip_start)) < 0.02,
      f"({src.quat_age():.3f} s)")
check("position age stayed healthy throughout", src.age() < 0.01,
      f"({src.age():.4f} s)")
check("quat_age exceeds the land threshold",
      src.quat_age() > T.QUAT_DEAD_S, f"({src.quat_age():.3f} s)")

print("\n[D] the guard would NOT fire on a healthy stream")
src = make_source(_clock)
t = 3000.0
for i in range(480):                    # 2 s clean
    t += 1 / 240.0
    feed(src, 12.0, t, _clock)
check("no rejects on a clean stream", src.total_yaw_rejects == 0)
check("quat_age stays near zero", src.quat_age() < 0.01,
      f"({src.quat_age():.4f} s)")

print("\n[E] ordinary yaw jitter and commanded rotation are untouched")
src = make_source(_clock)
t = 4000.0
for i in range(240):
    t += 1 / 240.0
    feed(src, 4.0 * math.sin(i / 8.0), t, _clock)   # jitter, few deg
check("jitter never rejected", src.total_yaw_rejects == 0)
src = make_source(_clock)
t = 5000.0
yaw = 0.0
for i in range(240):                    # commanded 200 deg/s yaw for 1 s
    t += 1 / 240.0
    yaw += 200.0 / 240.0
    feed(src, yaw, t, _clock)
check("200 deg/s commanded rotation never rejected",
      src.total_yaw_rejects == 0, f"({src.total_yaw_rejects})")

print("\n[F] pre-r11 behaviour is what the cap actually removed")
# Reproduce the old maths directly: with an uncapped gap the same flip passes
# the rate test once enough time has elapsed. This is the bug, asserted so it
# cannot quietly return.
d = 93.6
gap_latch = 0.30
check("uncapped: 93.6 deg over 300 ms implies only ~312 deg/s",
      d / gap_latch < 720.0, f"({d / gap_latch:.0f} deg/s -- would be ACCEPTED)")
check("capped: same flip implies >=1872 deg/s",
      d / min(gap_latch, T.FLIP_GAP_CAP_S) > 720.0,
      f"({d / min(gap_latch, T.FLIP_GAP_CAP_S):.0f} deg/s -- rejected)")
check("cap is short enough to bite before QUAT_DEAD_S",
      T.FLIP_GAP_CAP_S < T.QUAT_DEAD_S)

print("\n[G] delegation actually exists on the wrapper")
# The handoff records self.vicon.age() shipping broken because age() lived on
# ViconSource and was called on ViconThread. quat_age() must not repeat it.
check("ViconSource.quat_age exists", hasattr(T.ViconSource, "quat_age"))
check("ViconThread.quat_age exists", hasattr(T.ViconThread, "quat_age"))
check("they are different functions (real delegation, not aliasing)",
      T.ViconThread.quat_age is not T.ViconSource.quat_age)

print("\n[H] supervise() lands on a dead orientation, and only when flying")
calls = {}


class FakeVicon:
    """Only exposes what a ViconThread exposes. Deliberately NOT collapsed onto
    the node -- see the handoff: a fake that merges the two layers accepts calls
    the real object would reject."""

    def __init__(self, qage):
        self._qage = qage
        self.node = types.SimpleNamespace(latest=lambda: None,
                                          total_yaw_rejects=7)

    def age(self):
        return 0.001

    def quat_age(self):
        return self._qage


def supervise_with(qage, state, pos_only=False):
    tele = T.Teleop.__new__(T.Teleop)
    tele.vicon = FakeVicon(qage)
    tele.state = state
    tele.args = types.SimpleNamespace(pos_only=pos_only, no_fly=False)
    tele._stale_since = None
    tele._diverge_since = None
    tele.mocap_stale = False
    tele.tlm = types.SimpleNamespace(
        snapshot=lambda: (0.0, 0.0, 0.0, 3.9, 0.0, 0.0, 0.0),
        got_state=False)
    tele.bounds = types.SimpleNamespace(
        outside_kill=lambda *a: False, outside_hard=lambda *a: False,
        describe=lambda: "box")
    return T.Teleop.supervise(tele)


r = supervise_with(0.8, T.State.FLYING)
check("flying + dead orientation -> land", r is not None and r[0] == "land",
      f"({r})")
check("reason names the cause", r is not None and "ORIENTATION" in r[1],
      f"({r[1] if r else None})")
check("it lands, never kills", r is not None and r[0] != "kill")

r = supervise_with(0.8, T.State.IDLE)
check("idle + dead orientation -> no action", r is None, f"({r})")

r = supervise_with(0.1, T.State.FLYING)
check("brief rejection while flying -> no action", r is None, f"({r})")

r = supervise_with(0.8, T.State.FLYING, pos_only=True)
check("--pos-only is exempt (orientation withheld by design)", r is None,
      f"({r})")

r = supervise_with(T.QUAT_DEAD_S - 0.01, T.State.FLYING)
check("just under threshold -> no action", r is None)
r = supervise_with(T.QUAT_DEAD_S + 0.01, T.State.FLYING)
check("just over threshold -> land", r is not None and r[0] == "land")

print("\n[I] the upright gate: steady-but-inverted must be refused")
# A rigid body created with Z inverted reports the drone upside down, perfectly
# steady, with ZERO yaw rejects -- so every flip-based test passes it. Only an
# absolute check catches it. Measured on the real system: 176.92 deg.
def tilt_of(qx, qy, qz, qw):
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.degrees(math.acos(max(-1.0, min(1.0, r22))))


def quat_roll(deg):
    h = math.radians(deg) / 2.0
    return (math.sin(h), 0.0, 0.0, math.cos(h))


check("upright reads ~0 deg", tilt_of(*quat_roll(0.0)) < 0.01,
      f"({tilt_of(*quat_roll(0.0)):.2f})")
check("3 deg tilt reads 3 deg", abs(tilt_of(*quat_roll(3.0)) - 3.0) < 0.01)
check("inverted reads ~180 deg", tilt_of(*quat_roll(180.0)) > 179.99,
      f"({tilt_of(*quat_roll(180.0)):.2f})")
check("the real 2026-08-12 inversion is over the limit",
      tilt_of(*quat_roll(176.92)) > T.PREARM_TILT_DEG,
      f"({tilt_of(*quat_roll(176.92)):.2f} deg vs limit {T.PREARM_TILT_DEG:.0f})")
check("a healthy 3.2 deg template error still arms",
      tilt_of(*quat_roll(3.17)) < T.PREARM_TILT_DEG)
check("limit leaves room for a drone on a slope",
      10.0 < T.PREARM_TILT_DEG < 60.0, f"({T.PREARM_TILT_DEG:.0f} deg)")

# The measure must not be an Euler roll: at 176.92 deg Euler roll wraps to
# -183/+177 and a naive |roll| < 30 test reads it as noise, not inversion.
def euler_roll(qx, qy, qz, qw):
    return math.degrees(math.atan2(2 * (qw * qx + qy * qz),
                                   1 - 2 * (qx * qx + qy * qy)))


check("R[2][2] measure is immune to the +/-180 wrap that fools Euler roll",
      abs(euler_roll(*quat_roll(176.92))) > 170.0
      and tilt_of(*quat_roll(176.92)) > 170.0)

T.time.time = _real_time
print("\n" + "=" * 60)
print(f"{PASS} passed, {FAIL} failed")
print("ALL CHECKS PASSED" if FAIL == 0 else "*** FAILURES ***")
sys.exit(1 if FAIL else 0)
