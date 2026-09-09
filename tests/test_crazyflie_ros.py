#!/usr/bin/env python3
"""Tests for the ROS 2 driver and the shared core, rclpy and messages stubbed.

Same trick test_flip_guard.py uses for cflib: none of this needs ROS installed,
so it runs anywhere Python does, including a machine that has never had a
distro sourced. What it does NOT prove is that the package builds -- only
`colcon build` on a real Humble machine can say that.

Two things are being defended here.

  1. UNITS. The message definitions are Crazyswarm2's, and Crazyswarm2's unit
     conventions are mutually inconsistent (Hover.yaw_rate is rad/s and
     sign-flipped, VelocityWorld.yaw_rate is deg/s, FullState angular is deg/s,
     Position.yaw is degrees, GoTo.yaw is radians despite its own comment
     saying deg). They were reproduced exactly rather than tidied, so that
     dropping in the real crazyflie_server later does not silently change what
     a policy commands. A test is the only thing that keeps "exactly" true.

  2. NO DRIFT. The whole point of cf_core.py is that the standalone script and
     this node cannot disagree about a guard. That is asserted directly.
"""
import ast
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "src", "crazyflie_ros")
sys.path.insert(0, ROOT)
sys.path.insert(0, PKG)

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


# =============================================================================
# STUBS
# =============================================================================
class _Stamp:
    sec = 0
    nanosec = 0

    def to_msg(self):
        return self


class _Clock:
    def now(self):
        return _Stamp()


class _Logger:
    def __init__(self):
        self.msgs = []

    def _log(self, lvl, m):
        self.msgs.append((lvl, m))

    def info(self, m):  self._log("info", m)
    def warn(self, m):  self._log("warn", m)
    def error(self, m): self._log("error", m)
    def fatal(self, m): self._log("fatal", m)


class _Param:
    def __init__(self, v):
        self.value = v


class _FakeNode:
    """Enough of rclpy.node.Node that the real __init__ runs unmodified."""

    def __init__(self, name):
        self._name = name
        self._log = _Logger()
        self.subs, self.pubs, self.srvs, self.timers = [], [], [], []
        self._params = {}

    def declare_parameter(self, name, default):
        self._params[name] = default
        return _Param(default)

    def create_subscription(self, t, topic, cb, qos, callback_group=None):
        self.subs.append((t, topic, cb)); return types.SimpleNamespace()

    def create_publisher(self, t, topic, qos):
        p = types.SimpleNamespace(msgs=[])
        p.publish = p.msgs.append
        self.pubs.append((t, topic, p)); return p

    def create_service(self, t, name, cb, callback_group=None):
        self.srvs.append((t, name, cb)); return types.SimpleNamespace()

    def create_timer(self, period, cb):
        self.timers.append((period, cb)); return types.SimpleNamespace()

    def create_client(self, t, name):
        return types.SimpleNamespace(srv_name=name,
                                     wait_for_service=lambda timeout_sec=0: True,
                                     call_async=lambda r: None)

    def get_logger(self):  return self._log
    def get_clock(self):   return _Clock()
    def destroy_node(self): pass


def _msg_class(fields, consts=None):
    ns = dict(consts or {})

    def __init__(self):
        for k, v in fields.items():
            setattr(self, k, v() if callable(v) else v)
    ns["__init__"] = __init__
    return type("Msg", (), ns)


def _vec():   return types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
def _quat():  return types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
def _hdr():   return types.SimpleNamespace(stamp=_Stamp(), frame_id="")
def _pose():  return types.SimpleNamespace(position=_vec(), orientation=_quat())
def _twist(): return types.SimpleNamespace(linear=_vec(), angular=_vec())


rclpy = types.ModuleType("rclpy")
rclpy.init = lambda **k: None
rclpy.shutdown = lambda: None
rclpy.ok = lambda: True
rclpy.spin = lambda n: None
node_mod = types.ModuleType("rclpy.node"); node_mod.Node = _FakeNode
qos_mod = types.ModuleType("rclpy.qos")
qos_mod.QoSProfile = lambda **k: k
qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
cbg_mod = types.ModuleType("rclpy.callback_groups")
cbg_mod.MutuallyExclusiveCallbackGroup = lambda: None
rclpy.node, rclpy.qos, rclpy.callback_groups = node_mod, qos_mod, cbg_mod

gm = types.ModuleType("geometry_msgs"); gmm = types.ModuleType("geometry_msgs.msg")
gmm.PoseStamped = _msg_class({"header": _hdr, "pose": _pose})
ss = types.ModuleType("std_srvs"); ssv = types.ModuleType("std_srvs.srv")
ssv.Empty = type("Empty", (), {"Request": type("R", (), {})})

ci = types.ModuleType("crazyflie_interfaces")
cim = types.ModuleType("crazyflie_interfaces.msg")
cim.Position = _msg_class({"header": _hdr, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
cim.Hover = _msg_class({"header": _hdr, "vx": 0.0, "vy": 0.0,
                        "yaw_rate": 0.0, "z_distance": 0.0})
cim.VelocityWorld = _msg_class({"header": _hdr, "vel": _vec, "yaw_rate": 0.0})
cim.FullState = _msg_class({"header": _hdr, "pose": _pose,
                            "twist": _twist, "acc": _vec})
cim.LogDataGeneric = _msg_class({"header": _hdr, "timestamp": 0, "values": list})
cim.Status = _msg_class(
    {"header": _hdr, "supervisor_info": 0, "battery_voltage": 0.0,
     "pm_state": 0, "rssi": 0, "num_rx_broadcast": 0, "num_tx_broadcast": 0,
     "num_rx_unicast": 0, "num_tx_unicast": 0, "latency_unicast": 0},
    {"SUPERVISOR_INFO_CAN_BE_ARMED": 1, "SUPERVISOR_INFO_IS_ARMED": 2,
     "SUPERVISOR_INFO_AUTO_ARM": 4, "SUPERVISOR_INFO_CAN_FLY": 8,
     "SUPERVISOR_INFO_IS_FLYING": 16, "SUPERVISOR_INFO_IS_TUMBLED": 32,
     "SUPERVISOR_INFO_IS_LOCKED": 64})
cis = types.ModuleType("crazyflie_interfaces.srv")
for _n in ("Takeoff", "Land", "GoTo", "NotifySetpointsStop", "Arm", "Stop"):
    setattr(cis, _n, type(_n, (), {"Request": type("R", (), {})}))

for _k, _v in {"rclpy": rclpy, "rclpy.node": node_mod, "rclpy.qos": qos_mod,
               "rclpy.callback_groups": cbg_mod,
               "geometry_msgs": gm, "geometry_msgs.msg": gmm,
               "std_srvs": ss, "std_srvs.srv": ssv,
               "crazyflie_interfaces": ci, "crazyflie_interfaces.msg": cim,
               "crazyflie_interfaces.srv": cis}.items():
    sys.modules[_k] = _v

from crazyflie_ros import crazyflie_server as S   # noqa: E402
from crazyflie_ros import cf_core as C            # noqa: E402


class FakeCf:
    """Records what would have gone on the radio."""

    def __init__(self):
        self.sent = []
        me = self

        class _Cmd:
            def send_position_setpoint(s, x, y, z, yaw):
                me.sent.append(("position", x, y, z, yaw))

            def send_hover_setpoint(s, *a):     me.sent.append(("hover",) + a)
            def send_velocity_world_setpoint(s, *a): me.sent.append(("vel",) + a)
            def send_full_state_setpoint(s, *a): me.sent.append(("full",) + a)
            def send_stop_setpoint(s):          me.sent.append(("stop",))
            def send_notify_setpoint_stop(s, m): me.sent.append(("notify", m))
        self.commander = _Cmd()
        self.extpos = types.SimpleNamespace(
            send_extpos=lambda *a: me.sent.append(("extpos",) + a),
            send_extpose=lambda *a: me.sent.append(("extpose",) + a))


def make_server(**params):
    srv = S.CrazyflieServer()
    for k, v in params.items():
        setattr(srv, k, v)
    srv.cf = FakeCf()
    return srv


def pose_at(x=0.0, y=0.0, z=0.5, yaw=0.0, t=1000.0):
    h = math.radians(yaw) / 2.0
    return C.Pose(x=x, y=y, z=z, qx=0.0, qy=0.0, qz=math.sin(h), qw=math.cos(h),
                  stamp=t, recv_time=t)


print("=" * 62)
print("[A] units are reproduced EXACTLY, inconsistencies and all")
srv = make_server()

m = cim.Position(); m.x, m.y, m.z, m.yaw = 1.0, 2.0, 3.0, 90.0
srv._cmd_position(m)
check("Position.yaw passes through as DEGREES",
      srv._cmd[1] == (1.0, 2.0, 3.0, 90.0), f"({srv._cmd})")

m = cim.Hover(); m.vx, m.vy, m.yaw_rate, m.z_distance = 0.1, 0.2, 1.0, 0.5
srv._cmd_hover(m)
got = srv._cmd[1][2]
check("Hover.yaw_rate: rad/s -> deg/s AND sign-flipped",
      abs(got - (-math.degrees(1.0))) < 1e-9, f"got {got}, want {-math.degrees(1.0)}")
check("Hover other fields pass through",
      srv._cmd[1][0] == 0.1 and srv._cmd[1][1] == 0.2 and srv._cmd[1][3] == 0.5)

m = cim.VelocityWorld(); m.vel.x, m.vel.y, m.vel.z, m.yaw_rate = 1., 2., 3., 45.0
srv._cmd_velocity_world(m)
check("VelocityWorld.yaw_rate stays deg/s, NOT negated",
      srv._cmd[1] == (1., 2., 3., 45.0), f"({srv._cmd})")

m = cim.FullState()
m.pose.position.x, m.pose.position.y, m.pose.position.z = 1., 2., 3.
m.pose.orientation.x, m.pose.orientation.y = 0.1, 0.2
m.pose.orientation.z, m.pose.orientation.w = 0.3, 0.9
m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = 10., 20., 30.
srv._cmd_full_state(m)
q = srv._cmd[1][3]
check("FullState quaternion order is (x, y, z, w)",
      q == [0.1, 0.2, 0.3, 0.9], f"({q})")
check("FullState angular stays deg/s, NOT negated",
      srv._cmd[1][4:] == (10., 20., 30.), f"({srv._cmd[1][4:]})")

print("\n[B] the setpoint choke point clamps, leashes and applies yaw_sign")
srv = make_server(bounds=C.Bounds(-1, 1, -1, 1, 0.05, 1.5), yaw_sign=1)
p = pose_at(0.0, 0.0, 0.5)
srv._send_position(5.0, 0.0, 0.5, 0.0, p)
kind, x, y, z, yaw = srv.cf.sent[-1]
check("geofence clamps x before it ever reaches the radio", x <= 1.0 + 1e-9, f"x={x}")
check("leash caps the commanded error at LEASH_M",
      math.dist((x, y, z), (p.x, p.y, p.z)) <= C.LEASH_M + 1e-9,
      f"d={math.dist((x,y,z),(p.x,p.y,p.z)):.3f}")

srv = make_server(bounds=C.Bounds(-2, 2, -2, 2, 0.05, 1.5), yaw_sign=-1)
srv._send_position(0.0, 0.0, 0.5, 30.0, pose_at())
check("yaw_sign=-1 inverts the transmitted yaw",
      abs(srv.cf.sent[-1][4] - (-30.0)) < 1e-9, f"({srv.cf.sent[-1]})")

print("\n[C] the command watchdog HOLDS -- it must not land and must not kill")
b = C.Bounds(-2, 2, -2, 2, 0.05, 1.5)
base = dict(now=1000.0, pose_age=0.001, quat_age=0.001, pos_only=False,
            pos=(0., 0., 0.5), ekf=(0., 0., 0.5), got_state=True, vbat=3.9,
            bounds=b, stale_since=None, diverge_since=None)

r = C.supervise(flying=True, cmd_age=0.5, command_timeout=0.3, **base)
check("stale commands while flying -> hold", r.action == "hold", f"({r.action})")
check("hold is not a land", r.action != "land")
check("hold is not a kill", r.action != "kill")
r = C.supervise(flying=True, cmd_age=0.1, command_timeout=0.3, **base)
check("fresh commands -> no action", r.action is None, f"({r.action})")
r = C.supervise(flying=False, cmd_age=99.0, command_timeout=0.3, **base)
check("on the ground, silence is not a fault", r.action is None, f"({r.action})")

check("timeout default sits inside the firmware 500 ms stabilize window",
      C.COMMAND_TIMEOUT_S < C.FW_WDT_STABILIZE_S,
      f"{C.COMMAND_TIMEOUT_S} vs {C.FW_WDT_STABILIZE_S}")

print("\n[D] severity ordering: the worse fault wins")
r = C.supervise(flying=True, cmd_age=99.0, command_timeout=0.3,
                **{**base, "pose_age": 1.0})
check("mocap DEAD outranks a command timeout", r.action == "kill", f"({r.action})")
r = C.supervise(flying=True, cmd_age=99.0, command_timeout=0.3,
                **{**base, "quat_age": 1.0})
check("dead orientation outranks a command timeout", r.action == "land", f"({r.action})")
r = C.supervise(flying=True, cmd_age=0.01, command_timeout=0.3,
                **{**base, "pos": (9., 9., 0.5)})
check("far outside the fence -> kill", r.action == "kill", f"({r.action})")
r = C.supervise(flying=True, cmd_age=0.01, command_timeout=0.3,
                **{**base, "pos": (2.4, 0., 0.5)})
check("just outside the fence -> land, not kill", r.action == "land", f"({r.action})")
r = C.supervise(flying=True, cmd_age=0.01, command_timeout=0.3,
                **{**base, "vbat": 3.1})
check("critical battery -> land", r.action == "land", f"({r.action})")

print("\n[E] the pre-arm gate refuses the states that broke real hardware")
b2 = C.Bounds(-2, 2, -2, 2, 0.05, 1.5)
ok, why = C.prearm_check(pose_at(0, 0, 0.02), b2)
check("upright drone on the floor is allowed to arm", ok, f"({why})")
inv = C.Pose(x=0, y=0, z=0.02, qx=1.0, qy=0.0, qz=0.0, qw=0.0)
ok, why = C.prearm_check(inv, b2)
check("inverted rigid body is refused", not ok, f"({why})")
check("...and the message says UPSIDE DOWN",
      any("UPSIDE DOWN" in w for w in why), f"({why})")
check("an occluded frame reads as exactly 180 deg tilt",
      abs(C.tilt_deg_of(1, 0, 0, 0) - 180.0) < 1e-9)
ok, why = C.prearm_check(pose_at(9, 9, 0.02), b2)
check("drone outside the volume is refused", not ok, f"({why})")

print("\n[F] no drift: the node and the standalone share one implementation")
sys.modules.setdefault("cflib", types.ModuleType("cflib"))
for n in ("cflib.crtp", "cflib.crazyflie", "cflib.crazyflie.log",
          "cflib.crazyflie.syncCrazyflie", "cflib.crazyflie.syncLogger",
          "cflib.utils", "cflib.utils.power_switch"):
    sys.modules.setdefault(n, types.ModuleType(n))
import cf_core as TopCore              # noqa: E402  the standalone's import path
import crazyflie_vicon_teleop as T     # noqa: E402

# NOT an identity check. The standalone imports `cf_core`; the node imports
# `crazyflie_ros.cf_core`. Python builds a separate module object per import
# NAME even when both names resolve to one file, so `T.Pose is C.Pose` is
# false here -- and irrelevant, because the two programs never share an
# interpreter.
#
# These were once ONE file: the package copy was a symlink to the stack root,
# and the invariant asserted here was same-inode. fix_symlinks.py made them
# real files, because symlinks do not survive a transfer -- that is exactly how
# the copy to pear-2 lost them. The invariant is therefore BYTE IDENTITY now,
# held on the host by DUPLICATES.json + test_duplicates.py. What is being
# guarded has not changed: two diverging copies of the safety core, with the
# node and the standalone enforcing different limits and nothing complaining.
# Content equality also passes if they are ever symlinked again, so this check
# does not have to be revisited either way.
def _sha_of(path):
    import hashlib
    with open(os.path.realpath(path), "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()

check("both import paths load byte-identical files",
      _sha_of(TopCore.__file__) == _sha_of(C.__file__),
      f"{TopCore.__file__} vs {C.__file__}"
      "  -- fix with: python3 fix_symlinks.py --sync")
check("the standalone's names come from that same content",
      _sha_of(sys.modules[T.Pose.__module__].__file__) == _sha_of(C.__file__))

# Behavioural equivalence on the guard that matters most, across the exact
# boundary where the pre-r11 filter latched.
_cases = [(93.6, 0.0, 1000.0, 1000.004), (93.6, 0.0, 1000.0, 1000.300),
          (4.0, 0.0, 1000.0, 1000.004), (200.0, 0.0, 1000.0, 1000.500),
          (46.0, 0.0, 1000.0, 1000.0001)]
check("flip_check agrees on every boundary case",
      all(T.flip_check(*c, 720.0) == C.flip_check(*c, 720.0) for c in _cases))
check("Bounds agrees", T.Bounds(-1, 1, -1, 1, 0.05, 1.5).outside_kill(9, 0, 0.5)
      == C.Bounds(-1, 1, -1, 1, 0.05, 1.5).outside_kill(9, 0, 0.5))
check("prearm_check agrees on an inverted body",
      T.prearm_check(C.Pose(qx=1.0, qw=0.0), C.Bounds())[0]
      == C.prearm_check(C.Pose(qx=1.0, qw=0.0), C.Bounds())[0])
for k in ("QUAT_DEAD_S", "MOCAP_DEAD_S", "MOCAP_STALE_S", "FLIP_GAP_CAP_S",
          "PREARM_TILT_DEG", "LEASH_M", "GEOFENCE_KILL_M", "EKF_DIVERGE_M"):
    check(f"same {k}", getattr(T, k) == getattr(C, k))

print("\n[G] every message field the driver touches exists in the vendored .msg")
MSGDIR = os.path.join(ROOT, "src", "crazyflie_interfaces", "msg")


def msg_fields(name):
    out = set()
    for ln in open(os.path.join(MSGDIR, name)):
        ln = ln.split("#")[0].strip()
        if not ln or "=" in ln:
            continue
        parts = ln.split()
        if len(parts) >= 2:
            out.add(parts[1])
    return out


for msg, used in {
    "Position.msg": {"header", "x", "y", "z", "yaw"},
    "Hover.msg": {"header", "vx", "vy", "yaw_rate", "z_distance"},
    "VelocityWorld.msg": {"header", "vel", "yaw_rate"},
    "FullState.msg": {"header", "pose", "twist", "acc"},
    "Status.msg": {"header", "supervisor_info", "battery_voltage"},
}.items():
    have = msg_fields(msg)
    check(f"{msg} has {sorted(used)}", used <= have, f"missing {sorted(used - have)}")

src = open(os.path.join(PKG, "crazyflie_ros", "crazyflie_server.py")).read()
tree = ast.parse(src)
sends = {n.func.attr for n in ast.walk(tree)
         if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
         and n.func.attr.startswith("send_")}
check("driver only calls known cflib senders",
      sends <= {"send_position_setpoint", "send_hover_setpoint",
                "send_velocity_world_setpoint", "send_full_state_setpoint",
                "send_stop_setpoint", "send_notify_setpoint_stop",
                "send_extpos", "send_extpose", "send_arming_request"},
      f"unexpected: {sends}")

print("\n[H] the driver wires up the topics and services it advertises")
srv = make_server()
topics = {t for _, t, _ in srv.subs}
check("subscribes to all four cmd_* topics",
      {"cf1/cmd_position", "cf1/cmd_hover", "cf1/cmd_velocity_world",
       "cf1/cmd_full_state"} <= topics, f"({sorted(topics)})")
check("subscribes to the Vicon topic",
      "/vicon/crazyflie1/crazyflie1" in topics)
names = {n for _, n, _ in srv.srvs}
check("offers takeoff/land/arm/stop/emergency",
      {"cf1/takeoff", "cf1/land", "cf1/arm", "cf1/stop", "cf1/emergency"} <= names,
      f"({sorted(names)})")

print("\n[I] the merged vicon_probe keeps everything both old tools reported")
import csv as _csv, io as _io, contextlib as _ctx           # noqa: E402
import vicon_probe as V                                      # noqa: E402

_cap = os.path.join(HERE, "fixtures", "tilt_capture_1786563877.csv")
if os.path.exists(_cap):
    _rows = []
    for _r in _csv.DictReader(open(_cap)):
        _t = float(_r["t"])
        _rows.append((_t, _t - 0.00019, float(_r["x"]), float(_r["y"]),
                      float(_r["z"]), float(_r["qx"]), float(_r["qy"]),
                      float(_r["qz"]), float(_r["qw"])))
    _b = _io.StringIO()
    with _ctx.redirect_stdout(_b):
        V.analyse(_rows, _rows[-1][0] - _rows[0][0])
    _out = _b.getvalue()
    for _sec in ("--- LINK ---", "latency (stamp -> receive)", "dropouts >",
                 "--- POSITION", "--- TEMPLATE Z AXIS", "--- ORIENTATION",
                 "--- YAW STEP ANALYSIS", "--- VERDICT"):
        check(f"probe still reports {_sec.strip('- ')}", _sec in _out)
    _s = C.capture_stats(_rows)
    check("dropout threshold comes from the MEASURED rate, not an assumed 100 Hz",
          abs(_s["drop_thresh_ms"] - 2000.0 / _s["hz"]) < 1e-6,
          f"{_s['drop_thresh_ms']:.2f} ms at {_s['hz']:.1f} Hz")
    check("a 100 Hz assumption would have hidden most of these dropouts",
          C.capture_stats(_rows, expected_hz=100.0)["drops"] < _s["drops"],
          f"{C.capture_stats(_rows, expected_hz=100.0)['drops']} vs {_s['drops']}")
    _b2 = _io.StringIO()
    with _ctx.redirect_stdout(_b2):
        V.live_summary(_rows[:2000], 5.0)
    check("live mode prints a one-line summary", "Hz" in _b2.getvalue())
else:
    check("capture fixture present for the probe replay", False, "(missing)")

print("\n[J] the module must be internally consistent enough to START")
# 238 logic checks passed while crazyflie_vicon_teleop.py refused to run: every
# suite drives the classes through fakes, so nothing ever called selfcheck(),
# which is the one thing that verifies the control loop and the mocap wrapper
# agree about what attributes exist. That gap is what this closes.
try:
    T.selfcheck()
    check("selfcheck() accepts the current module", True)
except SystemExit as exc:
    check("selfcheck() accepts the current module", False, str(exc))
except Exception as exc:
    check("selfcheck() accepts the current module", False,
          f"{type(exc).__name__}: {exc}")

print("\n[K] occluded frames must not reach the control loop")
# The SDK's not-tracked sentinel arrives at full rate, fresh, with a unit-norm
# quaternion, so every existing guard passes it and the drone would be
# commanded toward the Vicon origin. These pin the fix.
check("sentinel is detected", C.occluded_sentinel(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
check("a normal pose is not", not C.occluded_sentinel(0.9, -0.1, 0.5, 0.0, 0.0, 0.0, 1.0))
check("a drone genuinely AT the origin is not rejected",
      not C.occluded_sentinel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      "position alone must never be enough")
check("position near but not at zero is not rejected",
      not C.occluded_sentinel(1e-9, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
check("the sentinel quaternion has unit norm, so the zero-norm guard misses it",
      abs(C.quat_norm(1.0, 0.0, 0.0, 0.0) - 1.0) < 1e-12)
check("...and reads as exactly 180 deg tilt",
      abs(C.tilt_deg_of(1.0, 0.0, 0.0, 0.0) - 180.0) < 1e-9)


class _OccMsg:
    def __init__(s, x, y, z, q, stamp=1000.0):
        s.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y, z=z),
            orientation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))
        s.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=int(stamp),
                                        nanosec=int((stamp % 1) * 1e9)))


_src = T.ViconSource.__new__(T.ViconSource)
_src._init_state(240.0)
_src.get_logger = lambda: types.SimpleNamespace(
    info=lambda *a: None, warn=lambda *a: None,
    error=lambda *a: None, fatal=lambda *a: None)

_src.position_callback(_OccMsg(0.9, -0.1, 0.5, (0.0, 0.0, 0.0, 1.0)))
_good = _src.latest()
check("a good frame is accepted", _good is not None and abs(_good.x - 0.9) < 1e-9)

_src.position_callback(_OccMsg(0.0, 0.0, 0.0, (1.0, 0.0, 0.0, 0.0)))
_after = _src.latest()
check("an occluded frame is DROPPED, not stored",
      _after is not None and abs(_after.x - 0.9) < 1e-9,
      f"latest became {(_after.x, _after.y, _after.z) if _after else None}")
check("...and is counted", getattr(_src, "total_occluded", 0) == 1,
      f"count {getattr(_src, 'total_occluded', None)}")

print("\n[L] --step moves the hold point, and refuses to leave the volume")
_te = T.Teleop.__new__(T.Teleop)
_te.args = types.SimpleNamespace(diagnose=30.0, step=0.30)
_te.bounds = C.Bounds(-1.0, 1.0, -1.0, 1.0, 0.05, 1.2)
_te.hold_x = 0.0
_te._diag_hold_x0 = 0.0
_te._diag_step_n = 0

_te._maybe_step(1.0)
check("no step before 1/3 of the capture", _te.hold_x == 0.0, f"{_te.hold_x}")
_te._maybe_step(10.1)
check("steps out at 1/3", abs(_te.hold_x - 0.30) < 1e-9, f"{_te.hold_x}")
_te._maybe_step(12.0)
check("does not step twice", abs(_te.hold_x - 0.30) < 1e-9, f"{_te.hold_x}")
_te._maybe_step(20.1)
check("steps back at 2/3", abs(_te.hold_x - 0.0) < 1e-9, f"{_te.hold_x}")

# A step that would leave the geofence must be refused, not silently clamped:
# a clipped step is a step of unknown size and useless for timing.
_te2 = T.Teleop.__new__(T.Teleop)
_te2.args = types.SimpleNamespace(diagnose=30.0, step=5.0)
_te2.bounds = C.Bounds(-1.0, 1.0, -1.0, 1.0, 0.05, 1.2)
_te2.hold_x = 0.0
_te2._diag_hold_x0 = 0.0
_te2._diag_step_n = 0
_te2._maybe_step(10.1)
check("an out-of-volume step is refused, not clamped", _te2.hold_x == 0.0,
      f"{_te2.hold_x}")
check("...and it does not retry later", (_te2._maybe_step(20.1) or True)
      and _te2.hold_x == 0.0)

_te3 = T.Teleop.__new__(T.Teleop)
_te3.args = types.SimpleNamespace(diagnose=30.0, step=0.0)
_te3.bounds = C.Bounds(-1.0, 1.0, -1.0, 1.0, 0.05, 1.2)
_te3.hold_x = 0.0
_te3._diag_hold_x0 = 0.0
_te3._diag_step_n = 0
_te3._maybe_step(25.0)
check("--step 0 is a plain hover capture, unchanged", _te3.hold_x == 0.0)

print("\n[M] --frame-test must refuse a run that crossed solver branches")
import frame_check as FC                                      # noqa: E402
_nose = {"distance_m": 0.28, "travel_heading_deg": 2.8, "mean_yaw_deg": -92.5,
         "yaw_wobble_deg": 3.0, "dz_m": 0.0, "yaw_error_deg": 95.3, "ok": False}
_left = {"distance_m": 0.32, "travel_heading_deg": 87.9, "mean_yaw_deg": 0.6,
         "yaw_wobble_deg": 3.0, "dz_m": 0.0, "yaw_error_deg": -2.8, "ok": True}
_out = FC.report(_nose, _left, 0.465, None, 15602, 0.6)
check("the real 2026-08-28 run is now flagged CONTAMINATED", "CONTAMINATED" in _out)
check("...and no longer invents a +46 deg offset",
      "CONSTANT YAW OFFSET" not in _out)
check("...and no longer claims a mirrored axis", "MIRRORED" not in _out)
check("...and still reports the ambiguity that caused it",
      "rotationally ambiguous" in _out)

_ok_n = dict(_nose); _ok_n["mean_yaw_deg"] = -1.0; _ok_n["yaw_error_deg"] = -2.0
_ok_l = dict(_left); _ok_l["mean_yaw_deg"] = -1.4; _ok_l["yaw_error_deg"] = -2.8
_out2 = FC.report(_ok_n, _ok_l, 0.465, None, 0, 0.4)
check("a clean single-branch run still passes", "PASS --" in _out2, _out2[-200:])

print("\n" + "=" * 62)
print(f"{PASS} passed, {FAIL} failed")
print("ALL CHECKS PASSED" if FAIL == 0 else "FAILURES ABOVE")
sys.exit(1 if FAIL else 0)
