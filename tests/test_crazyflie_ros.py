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


class _ParamType:
    """Stands in for a member of rclpy.Parameter.Type."""

    def __init__(self, name):
        self.name = name


class _FakeNode:
    """Enough of rclpy.node.Node that the real __init__ runs unmodified."""

    def __init__(self, name):
        self._name = name
        self._log = _Logger()
        self.subs, self.pubs, self.srvs, self.timers = [], [], [], []
        self._params = {}

    def declare_parameter(self, name, default):
        # Declared by type (rclpy.Parameter.Type), the value is None until set.
        value = None if isinstance(default, _ParamType) else default
        self._params[name] = value
        return _Param(value)

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
param_mod = types.ModuleType("rclpy.parameter")
param_mod.Parameter = type("Parameter", (), {
    "Type": types.SimpleNamespace(DOUBLE_ARRAY=_ParamType("DOUBLE_ARRAY"))})
rclpy.node, rclpy.qos, rclpy.callback_groups = node_mod, qos_mod, cbg_mod
rclpy.parameter = param_mod

gm = types.ModuleType("geometry_msgs"); gmm = types.ModuleType("geometry_msgs.msg")
gmm.PoseStamped = _msg_class({"header": _hdr, "pose": _pose})
sm = types.ModuleType("sensor_msgs"); smm = types.ModuleType("sensor_msgs.msg")
smm.Imu = _msg_class({"header": _hdr, "orientation": _quat,
                      "orientation_covariance": lambda: [0.0] * 9,
                      "angular_velocity": _vec, "linear_acceleration": _vec})
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
cim.RateThrust = _msg_class({"header": _hdr, "roll_rate": 0.0, "pitch_rate": 0.0,
                             "yaw_rate": 0.0, "thrust": 0.0, "thrust_pwm": 0})
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
               "rclpy.callback_groups": cbg_mod, "rclpy.parameter": param_mod,
               "geometry_msgs": gm, "geometry_msgs.msg": gmm,
               "sensor_msgs": sm, "sensor_msgs.msg": smm,
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

            def send_setpoint(s, *a):           me.sent.append(("rpyt",) + a)
            def send_hover_setpoint(s, *a):     me.sent.append(("hover",) + a)
            def send_velocity_world_setpoint(s, *a): me.sent.append(("vel",) + a)
            def send_full_state_setpoint(s, *a): me.sent.append(("full",) + a)
            def send_stop_setpoint(s):          me.sent.append(("stop",))
            def send_notify_setpoint_stop(s, m): me.sent.append(("notify", m))
        self.commander = _Cmd()
        # Firmware param store. set_value is ECHOED (applied) unless
        # echo_params is False, which models a write the firmware never acked.
        self.params = {"flightmode.stabModeRoll": "1",
                       "flightmode.stabModePitch": "1",
                       "flightmode.stabModeYaw": "0",
                       "flightmode.althold": "0",
                       "flightmode.poshold": "0",
                       "flightmode.posSet": "0"}
        self.echo_params = True
        self.param_writes = []
        self.throw_on_set = False
        self._cbs = []

        def _fire(n):
            for cb in list(me._cbs):
                cb(n, me.params[n])

        def _set(n, v):
            me.param_writes.append((n, v))
            if me.throw_on_set:
                raise IOError("radio write failed")
            if me.echo_params:
                me.params[n] = str(v)
                _fire(n)

        def _get(n):
            return me.params[n]           # KeyError == not in the TOC
        # A readback always reports the firmware's TRUE value, even when our
        # writes are not being applied.
        self.param = types.SimpleNamespace(
            set_value=_set, get_value=_get, request_param_update=_fire,
            add_update_callback=lambda group=None, name=None, cb=None:
                me._cbs.append(cb))
        self.extpos = types.SimpleNamespace(
            send_extpos=lambda *a: me.sent.append(("extpos",) + a),
            send_extpose=lambda *a: me.sent.append(("extpose",) + a))


def make_server(**params):
    srv = S.CrazyflieServer()
    for k, v in params.items():
        setattr(srv, k, v)
    srv.cf = FakeCf()
    srv._wire_cf_callbacks()
    srv.fm_shutdown_timeout = 0.05
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
    "RateThrust.msg": {"header", "roll_rate", "pitch_rate", "yaw_rate",
                       "thrust", "thrust_pwm"},
}.items():
    have = msg_fields(msg)
    check(f"{msg} has {sorted(used)}", used <= have, f"missing {sorted(used - have)}")

src = open(os.path.join(PKG, "crazyflie_ros", "crazyflie_server.py")).read()
tree = ast.parse(src)
sends = {n.func.attr for n in ast.walk(tree)
         if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
         and n.func.attr.startswith("send_")}
check("driver only calls known cflib senders",
      sends <= {"send_position_setpoint", "send_hover_setpoint", "send_setpoint",
                "send_velocity_world_setpoint", "send_full_state_setpoint",
                "send_stop_setpoint", "send_notify_setpoint_stop",
                "send_extpos", "send_extpose", "send_arming_request"},
      f"unexpected: {sends}")

print("\n[H] the driver wires up the topics and services it advertises")
srv = make_server()
topics = {t for _, t, _ in srv.subs}
check("subscribes to all five cmd_* topics",
      {"cf1/cmd_position", "cf1/cmd_hover", "cf1/cmd_velocity_world",
       "cf1/cmd_full_state", "cf1/cmd_rate"} <= topics, f"({sorted(topics)})")
check("advertises cf1/imu", "cf1/imu" in {t for _, t, _ in srv.pubs})
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
check("exact (0,0,0) with an identity quaternion IS the sentinel (seen 2026-11-19)",
      C.occluded_sentinel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))
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

print("\n[N] cmd_rate: opt-in, firmware-confirmed, falls back to HOLD")
from crazyflie_ros import rate_cmd as R                       # noqa: E402

B = C.Bounds(-2, 2, -2, 2, 0.05, 1.5)


def rate_server(enable=True, setup=True, echo=True):
    s = make_server(bounds=B, enable_rate_cmd=enable, rate_mode_setup=setup)
    s.cf.echo_params = echo
    s._est_ready = True
    p = pose_at(0, 0, 0.02, t=__import__("time").time())
    s._latest = p
    s._last_quat_ok_t = p.recv_time
    s._last_quat_tilt = p.tilt_deg
    return s


def fly(s):
    """Take off through the real path, then jump to FLYING (as _takeoff_step does)."""
    s.begin_takeoff(0.5)
    s.state = C.State.FLYING
    s._hold = (0.0, 0.0, 0.5, 0.0)
    s._latest = pose_at(0, 0, 0.5, t=__import__("time").time())
    s._last_quat_ok_t = s._latest.recv_time
    s._last_quat_tilt = s._latest.tilt_deg


def rmsg(r=0.0, p=0.0, y=0.0, t=0.3, pwm=0):
    m = cim.RateThrust()
    m.roll_rate, m.pitch_rate, m.yaw_rate, m.thrust, m.thrust_pwm = r, p, y, t, pwm
    return m


import time as _time                                           # noqa: E402

s = make_server(bounds=B)
check("defaults: enable_rate_cmd and rate_mode_setup are OFF",
      s.enable_rate_cmd is False and s.rate_mode_setup is False)
check("defaults: imu on at 100 Hz", s.enable_imu_log and s.imu_log_hz == 100.0)

s = rate_server(enable=False, setup=False)
fly(s)
check("disabled: takeoff writes NO flight-mode params", s.cf.param_writes == [])
check("disabled: no thrust-unlock RPYT packet",
      not any(e[0] == "rpyt" for e in s.cf.sent))
s._cmd_rate(rmsg(0.1))
check("disabled: cmd_rate is not stored", s._cmd is None)
check("disabled: refusal is logged", any("enable_rate_cmd" in m for _, m in s._log.msgs))
n = len(s._log.msgs)
for _ in range(20):
    s._cmd_rate(rmsg(0.1))
check("disabled: refusal warning is THROTTLED", len(s._log.msgs) == n)

s = rate_server(enable=True, setup=False)
fly(s)
s.tick()
s._cmd_rate(rmsg(0.1))
check("enable without setup: refused, params untouched",
      s._cmd is None and s.cf.param_writes == [])

s = rate_server(echo=False)
fly(s)
s.tick()
s._cmd_rate(rmsg(0.1))
check("setup but firmware never ACKed: refused (rates would be read as ANGLES)",
      s._cmd is None, f"{s._cmd}")

s = rate_server()
s.begin_takeoff(0.5)
check("takeoff with setup: thrust-unlock (0,0,0,0) sent on the ground",
      ("rpyt", 0, 0, 0, 0) in s.cf.sent, f"{s.cf.sent}")
check("takeoff with setup: writes stabModeRoll/Pitch=0, never stabModeYaw",
      sorted(s.cf.param_writes) == [("flightmode.stabModePitch", "0"),
                                    ("flightmode.stabModeRoll", "0")],
      f"{s.cf.param_writes}")
s.tick()
s._cmd_rate(rmsg(0.1))
check("TAKEOFF state: cmd_rate refused", s._cmd is None)

s = rate_server()
fly(s)
s.tick()
check("firmware echo -> rate mode confirmed", s._rate_active)
s._cmd_rate(rmsg(r=math.radians(30), p=math.radians(-20),
                 y=math.radians(90), t=0.30))
check("FLYING + confirmed: cmd_rate stored", s._cmd is not None and s._cmd[0] == "rate")
check("...and clears the post-takeoff hold", s._hold is None)
s.cf.sent.clear()
s.tick()
k = s.cf.sent[-1] if s.cf.sent else None
check("transmits send_setpoint", k is not None and k[0] == "rpyt", f"{k}")
check("roll deg/s +, pitch deg/s +, yaw deg/s NEGATED",
      k and abs(k[1] - 30) < 1e-6 and abs(k[2] + 20) < 1e-6 and abs(k[3] + 90) < 1e-6,
      f"{k}")
check("0.30 N -> thrust word on the linear firmware map",
      k and k[4] == round(0.30 / 4 / 0.12 * 65535), f"{k}")

s._cmd_rate(rmsg(r=math.radians(500), y=math.radians(-1000), t=99.0))
s.tick()
k = s.cf.sent[-1]
check("rates saturate at the training caps",
      abs(k[1] - 120) < 1e-6 and abs(k[3] - 360) < 1e-6, f"{k}")
check("thrust saturates at thrust_pwm_max", k[4] == C.ATT_THRUST_MAX, f"{k}")
before = s._cmd
s._cmd_rate(rmsg(r=float("nan")))
check("NaN is rejected, previous command kept", s._cmd == before)

s.rate_allow_raw_pwm = True
s._cmd_rate(rmsg(t=0.0, pwm=40000))
s.rate_allow_raw_pwm = False
s.tick()
check("thrust_pwm override sent verbatim", s.cf.sent[-1][4] == 40000, f"{s.cf.sent[-1]}")

# stale -> HOLD with position setpoints, then rate resumes cleanly
s._cmd_t = _time.time() - 1.0
s.cf.sent.clear()
s.tick()
check("stale cmd_rate -> HOLD position (not land, not kill)",
      s.state == C.State.FLYING and s._hold is not None
      and s.cf.sent and s.cf.sent[-1][0] == "position", f"{s.cf.sent[-1:]}")
s._cmd_rate(rmsg(0.1))
s.cf.sent.clear()
s.tick()
check("fresh cmd_rate after a hold -> rates again",
      s._hold is None and s.cf.sent[-1][0] == "rpyt", f"{s.cf.sent[-1:]}")

# switch to position and back
m = cim.Position(); m.x, m.y, m.z, m.yaw = 0.1, 0.0, 0.5, 0.0
s._cmd_position(m)
s.cf.sent.clear(); s.tick()
check("cmd_position after cmd_rate -> position setpoints",
      s.cf.sent[-1][0] == "position", f"{s.cf.sent[-1:]}")
s._cmd_rate(rmsg(0.1))
s.cf.sent.clear(); s.tick()
check("...and back to rates", s.cf.sent[-1][0] == "rpyt")

# firmware loses rate mode mid-flight -> never sends rates
s.cf.params["flightmode.stabModePitch"] = "1"
s.cf.sent.clear(); s.tick()
check("rate mode lost mid-flight -> HOLD, no RPYT packet",
      s._hold is not None and all(e[0] != "rpyt" for e in s.cf.sent), f"{s.cf.sent}")
s.cf.params["flightmode.stabModePitch"] = "0"

# tilt guard
s = rate_server(); fly(s); s.tick()
s._cmd_rate(rmsg(0.1))
h = math.radians(70) / 2
s._latest = C.Pose(x=0, y=0, z=0.5, qx=math.sin(h), qy=0, qz=0, qw=math.cos(h),
                   recv_time=_time.time(), stamp=_time.time())
s._last_quat_ok_t = _time.time()
s._last_quat_tilt = s._latest.tilt_deg
s.tick()
check("tilt > rate_tilt_limit_deg in rate mode -> LAND", s.state == C.State.LANDING)
_t0 = s._cmd_t
s._cmd_rate(rmsg(0.2))
check("LANDING: cmd_rate refused (watchdog not refreshed)",
      s._cmd_t == _t0 and s.state == C.State.LANDING)
s.cf.sent.clear(); s.tick()
check("landing sends position setpoints, not rates",
      s.cf.sent and all(e[0] == "position" for e in s.cf.sent), f"{s.cf.sent}")

# supervisor still governs rate flight
s = rate_server(); fly(s); s.tick(); s._cmd_rate(rmsg(0.1))
s._latest = pose_at(9, 9, 0.5, t=_time.time()); s._last_quat_ok_t = _time.time()
s.tick()
check("geofence kill still fires in rate mode", s.state == C.State.STOPPED)
check("...and flight mode is reverted to angle",
      s.cf.params["flightmode.stabModeRoll"] == "1"
      and s.cf.params["flightmode.stabModePitch"] == "1")

s = rate_server(); fly(s); s.tick(); s._cmd_rate(rmsg(0.1))
s._latest = pose_at(2.4, 0, 0.5, t=_time.time()); s._last_quat_ok_t = _time.time()
s.tick()
check("geofence land still fires in rate mode", s.state == C.State.LANDING)
s._hold = (0, 0, 0.05, 0)
s.tick()
check("landing complete -> IDLE and flight mode reverted",
      s.state == C.State.IDLE and s.cf.params["flightmode.stabModeRoll"] == "1")

s = rate_server(); fly(s); s.tick()
s._srv_emergency(None, None)
check("emergency reverts flight mode", s.cf.params["flightmode.stabModePitch"] == "1")
s = rate_server(); fly(s); s.tick()
s._srv_stop(None, None)
check("stop reverts flight mode", s.cf.params["flightmode.stabModeRoll"] == "1")
s = rate_server(); fly(s); s.tick()
s.shutdown()
check("shutdown reverts flight mode", s.cf.params["flightmode.stabModeRoll"] == "1")

s = rate_server(enable=False, setup=False)
s.cf.params["flightmode.stabModeRoll"] = "0"
s.cf.params["flightmode.stabModePitch"] = "0"
s._check_flightmode_at_connect()
check("connect finds a leftover rate mode without setup -> restores angle",
      s.cf.params["flightmode.stabModeRoll"] == "1")
s = rate_server(enable=False, setup=False)
s._check_flightmode_at_connect()
check("connect with default params writes nothing", s.cf.param_writes == [])
s = rate_server(enable=False, setup=False)
s._srv_stop(None, None)
check("stop without rate setup writes nothing", s.cf.param_writes == [])

print("\n[O] imu: cflib log units -> sensor_msgs/Imu SI, body FLU")
s = make_server()
s._on_imu(0, {"gyro.x": 90.0, "gyro.y": -45.0, "gyro.z": 180.0,
              "acc.x": 0.0, "acc.y": 0.0, "acc.z": 1.0}, None)
imu = [pp for _, t, pp in s.pubs if t == "cf1/imu"][0].msgs[-1]
check("gyro deg/s -> rad/s", abs(imu.angular_velocity.x - math.pi / 2) < 1e-9
      and abs(imu.angular_velocity.y + math.pi / 4) < 1e-9
      and abs(imu.angular_velocity.z - math.pi) < 1e-9)
check("acc g -> m/s^2, +z at rest", abs(imu.linear_acceleration.z - 9.80665) < 1e-9)
check("orientation flagged as not provided", imu.orientation_covariance[0] == -1.0)
check("frame_id is the body frame", imu.header.frame_id == "cf1/body")

print("\n[P2] command loss must NOT mask the other guards (finding 2)")
_b = C.Bounds(-2, 2, -2, 2, 0.05, 1.5)
_base = dict(now=1000.0, flying=True, pose_age=0.001, quat_age=0.001,
             pos_only=False, pos=(0., 0., 0.5), ekf=(0., 0., 0.5),
             got_state=True, vbat=3.9, bounds=_b, stale_since=None,
             diverge_since=None, cmd_age=5.0, command_timeout=0.3)
r = C.supervise(**{**_base, "pos": (2.4, 0., 0.5)})
check("P2 cmd loss + fence violation -> land (not hold)", r.action == "land", r.action)
r = C.supervise(**{**_base, "pos": (9., 0., 0.5)})
check("P2 cmd loss + far outside fence -> kill", r.action == "kill", r.action)
r = C.supervise(**{**_base, "vbat": 3.0})
check("P2 cmd loss + 3.0 V battery -> land", r.action == "land", r.action)
r = C.supervise(**{**_base, "ekf": (1.0, 0., 0.5), "diverge_since": 999.0})
check("P2 cmd loss + sustained EKF mismatch -> land", r.action == "land", r.action)
r = C.supervise(**{**_base, "ekf": (1.0, 0., 0.5)})
check("P2 cmd loss + fresh EKF mismatch still tracks onset while holding",
      r.action == "hold" and r.diverge_since == 1000.0, f"{r.action} {r.diverge_since}")
r = C.supervise(**_base)
check("P2 cmd loss alone still holds", r.action == "hold", r.action)

print("\n[P1] takeoff only from an eligible ground state; emergency latches (finding 1)")
for _st in (C.State.TAKEOFF, C.State.FLYING, C.State.LANDING, C.State.STOPPED):
    s = rate_server()
    s.state = _st
    s._latest = pose_at(0, 0, 0.02, t=_time.time())
    s._last_quat_ok_t = s._latest.recv_time
    s._last_quat_tilt = s._latest.tilt_deg
    ok = s.begin_takeoff(0.5)
    check(f"P1 takeoff refused in {_st}: no arm/unlock/param write, state kept",
          (not ok) and s.cf.sent == [] and s.cf.param_writes == []
          and not s._armed and s.state == _st,
          f"ok={ok} sent={s.cf.sent} w={s.cf.param_writes} armed={s._armed}")
s = rate_server(); fly(s); s.tick()
s.cf.sent.clear(); s.cf.param_writes.clear()
check("P1 second /takeoff while airborne sends zero-thrust packet? no",
      s.begin_takeoff(0.5) is False and not any(e[0] == "rpyt" for e in s.cf.sent)
      and s.cf.param_writes == [] and s.state == C.State.FLYING)
s = rate_server()
s._latest = pose_at(0, 0, 0.8, t=_time.time()); s._last_quat_ok_t = s._latest.recv_time
check("P1 takeoff refused when the drone is not on the ground",
      s.begin_takeoff(0.5) is False and s.cf.sent == [] and not s._armed)
for _h in (float("nan"), float("inf"), -1.0, 0.0, 9.0, 0.01):
    s = rate_server()
    check(f"P1 takeoff target {_h} rejected before arm/unlock",
          s.begin_takeoff(_h) is False and s.cf.sent == [] and not s._armed
          and s.cf.param_writes == [] and s.state == C.State.IDLE)
s = rate_server()
check("P1 sane takeoff from IDLE still accepted", s.begin_takeoff(0.5) is True)
s = rate_server(); fly(s); s.tick()
s.emergency("test")
s._srv_stop(None, None)
check("P1 /stop does not clear an emergency latch", s.state == C.State.STOPPED)
s._latest = pose_at(0, 0, 0.02, t=_time.time()); s._last_quat_ok_t = s._latest.recv_time
check("P1 takeoff refused after emergency", s.begin_takeoff(0.5) is False)
s._srv_arm(types.SimpleNamespace(arm=True), None)
check("P1 /arm true refused while latched", not s._armed)
s._srv_reset_emergency(None, None)
check("P1 explicit reset clears the latch", s.state == C.State.IDLE)
check("P1 takeoff accepted after explicit reset", s.begin_takeoff(0.5) is True)
s = rate_server(); fly(s); s._armed = False
s._srv_arm(types.SimpleNamespace(arm=True), None)
check("P1 /arm true refused while airborne", not s._armed)
check("P1 reset service advertised",
      "cf1/reset_emergency" in {n for _, n, _ in make_server().srvs})

print("\n[P3] rate output needs a FRESH VALID attitude; otherwise land (finding 3)")


def _rate_flying():
    s = rate_server(); fly(s); s.tick()
    s._cmd_rate(rmsg(0.1))
    s.cf.sent.clear(); s.tick()
    assert s.cf.sent and s.cf.sent[-1][0] == "rpyt"
    s.cf.sent.clear()
    return s


def _no_rpyt(s):
    return not any(e[0] == "rpyt" for e in s.cf.sent)


s = _rate_flying()
s._latest = pose_at(0, 0, 0.5, t=_time.time() - 0.2)      # stale mocap, > 150 ms
s._last_quat_ok_t = _time.time() - 0.2
s.tick()
check("P3 stale mocap (>150 ms, <400 ms): NO rate packet, lands instead",
      _no_rpyt(s) and s.state == C.State.LANDING, f"{s.state} {s.cf.sent}")

s = _rate_flying()
s._last_quat_ok_t = _time.time() - 0.3                    # quat rejected 300 ms
s.tick()
check("P3 quaternion rejected, no onboard attitude: NO rate packet, lands",
      _no_rpyt(s) and s.state == C.State.LANDING, f"{s.state} {s.cf.sent}")

s = _rate_flying()
s._last_quat_ok_t = _time.time() - 0.3
s._att, s._att_t = (2.0, -3.0), _time.time()
s.cf.sent.clear(); s.tick()
check("P3 quaternion rejected but FRESH onboard attitude: rates continue",
      s.state == C.State.FLYING and s.cf.sent and s.cf.sent[-1][0] == "rpyt",
      f"{s.state} {s.cf.sent}")

s = _rate_flying()
s._last_quat_ok_t = _time.time() - 0.3
s._att, s._att_t = (2.0, -3.0), _time.time() - 0.5        # onboard sample is old
s.tick()
check("P3 stale onboard attitude is not reused: lands",
      _no_rpyt(s) and s.state == C.State.LANDING)

s = _rate_flying()
s._last_quat_ok_t = _time.time() - 0.3
s._last_quat_tilt = 80.0                                  # stale AND over limit
s._att, s._att_t = (float("nan"), 0.0), _time.time()
s.tick()
check("P3 non-finite onboard attitude does not count as valid: lands",
      _no_rpyt(s) and s.state == C.State.LANDING)

s = _rate_flying()
s._att, s._att_t = (75.0, 0.0), _time.time()
s.tick()
check("P3 fresh onboard tilt over limit lands even if Vicon says upright",
      _no_rpyt(s) and s.state == C.State.LANDING)

s = _rate_flying()
check("P3 transmit itself refuses rates when the per-tick attitude gate is closed",
      (setattr(s, "_rate_att_ok", False), s._transmit(_time.time(), s.latest()),
       _no_rpyt(s))[-1])

print("\n[P4] flight-mode restore is tracked per param, acked, and gates arming (finding 4)")
RF = ("flightmode.stabModeRoll", "flightmode.stabModePitch")
s = rate_server(); s.cf.throw_on_set = True
s.begin_takeoff(0.5)
check("P4 write that throws still leaves a restore debt (not silently cleared)",
      set(s._fm_target) == set(RF), f"{s._fm_target}")
s.cf.throw_on_set = False
s.state = C.State.IDLE; s._armed = False; s.cf.sent.clear()
check("P4 takeoff inhibited while mode state is uncertain (debt unacked)",
      s.begin_takeoff(0.5) is False and s.cf.sent == [] and not s._armed)
s = rate_server(); fly(s); s.tick()
s.cf.echo_params = False
s._srv_stop(None, None)
check("P4 unacknowledged revert: params still 0 and debt kept for BOTH axes",
      s.cf.params[RF[0]] == "0" and s.cf.params[RF[1]] == "0"
      and set(s._fm_target) == set(RF), f"{s._fm_target}")
s._latest = pose_at(0, 0, 0.02, t=_time.time()); s._last_quat_ok_t = s._latest.recv_time
check("P4 takeoff refused while a restore is unacknowledged",
      s.begin_takeoff(0.5) is False and s.state == C.State.IDLE and not s._armed)
s._srv_arm(types.SimpleNamespace(arm=True), None)
check("P4 /arm true refused while a restore is unacknowledged", not s._armed)
s.cf.echo_params = True                     # the radio recovers; service retries
for n in RF:
    s._fm_wt[n] -= 5.0
s._service_flightmode(_time.time()); s._service_flightmode(_time.time())
check("P4 retry restores both axes and clears the debt",
      s.cf.params[RF[0]] == "1" and s.cf.params[RF[1]] == "1"
      and s._fm_target == {}, f"{s.cf.params} {s._fm_target}")
s._latest = pose_at(0, 0, 0.02, t=_time.time()); s._last_quat_ok_t = s._latest.recv_time
check("P4 takeoff allowed again once acknowledged", s.begin_takeoff(0.5) is True)

s = rate_server(); fly(s); s.tick()
s.cf.echo_params = False
s.cf.params[RF[0]] = "1"                    # roll happened to be restored already
s._fm_wt[RF[0]] = 0.0
s._fm_seen[RF[0]] = _time.time()
s._revert_flightmode("test")
check("P4 per-axis tracking: only the unacknowledged axis stays owed",
      (s._service_flightmode(_time.time()) or True)
      and RF[1] in s._fm_target, f"{s._fm_target}")

s = rate_server(); fly(s); s.tick(); s.cf.echo_params = False
s.cf.param_writes.clear()
s.shutdown()
check("P4 shutdown does not claim success without acks (loud error)",
      any("NOT confirmed" in m for _, m in s._log.msgs))
s = rate_server(); fly(s); s.tick()
s.shutdown()
check("P4 shutdown with acks: restored, no error",
      s.cf.params[RF[0]] == "1" and not any("NOT confirmed" in m for _, m in s._log.msgs))

for roll, pitch, yaw in ((0, 1, 0), (1, 0, 0), (0, 0, 1), (1, 1, 1), (0, 0, 0), (2, 1, 0)):
    s = rate_server(enable=False, setup=False)
    s.cf.params.update({RF[0]: str(roll), RF[1]: str(pitch),
                        "flightmode.stabModeYaw": str(yaw)})
    s._check_flightmode_at_connect()
    s._service_flightmode(_time.time())
    check(f"P4 connect recovery from roll={roll} pitch={pitch} yaw={yaw}",
          s.cf.params[RF[0]] == "1" and s.cf.params[RF[1]] == "1"
          and s.cf.params["flightmode.stabModeYaw"] == "0",
          f"{s.cf.params}")
s = rate_server(enable=True, setup=True)
del s.cf.params["flightmode.stabModeYaw"]
check("P4 unreadable flight mode with rate enabled: takeoff refused",
      s.begin_takeoff(0.5) is False and s.cf.sent == [])

s = rate_server(); fly(s); s.tick(); s._cmd_rate(rmsg(0.1))
s._on_link_lost("uri", "lost")
check("P4 link lost invalidates the rate command buffer and latches STOPPED",
      s._cmd is None and s.state == C.State.STOPPED and not s._rate_active)
s._srv_reset_emergency(None, None)
s._latest = pose_at(0, 0, 0.02, t=_time.time()); s._last_quat_ok_t = s._latest.recv_time
check("P4 takeoff still refused after link loss until reconnect",
      s.begin_takeoff(0.5) is False)

s = rate_server(); fly(s)
_real_sleep = _time.sleep
_time.sleep = lambda *_: (_ for _ in ()).throw(AssertionError("tick slept"))
try:
    s.tick(); s._srv_stop(None, None); s.tick()
    check("P4 no parameter waits inside the control tick or stop path", True)
except AssertionError as exc:
    check("P4 no parameter waits inside the control tick or stop path", False, str(exc))
finally:
    _time.sleep = _real_sleep

print("\n[P5] rate handoff needs verified decoder params and a ground unlock (finding 5)")
def _ready():
    s = rate_server(); fly(s); s.tick()
    return s
s = _ready()
check("P5 baseline: handoff allowed when everything is verified", s._rate_active, s._rate_why)
for n in ("flightmode.althold", "flightmode.poshold", "flightmode.posSet"):
    s = rate_server(); s.cf.params[n] = "1"; fly(s); s.tick()
    s._cmd_rate(rmsg(0.1))
    check(f"P5 {n}=1 -> handoff refused, cmd_rate not stored",
          not s._rate_active and s._cmd is None, s._rate_why)
    s = rate_server(); del s.cf.params[n]; fly(s); s.tick()
    check(f"P5 {n} missing from the TOC -> refused", not s._rate_active, s._rate_why)
s = _ready(); s._cmd_rate(rmsg(0.1))
s.cf.params["flightmode.althold"] = "1"           # another client flips it
s._fm_refresh_t = 0.0
s.cf.sent.clear(); s.tick()
check("P5 althold changed mid-flight: readback catches it, rates stop, hold",
      not s._rate_active and all(e[0] != "rpyt" for e in s.cf.sent), f"{s.cf.sent}")
s = rate_server(); fly(s); s.cf._cbs.clear(); s._fm_seen.clear(); s.tick()
check("P5 cached values with no firmware report are NOT fresh -> refused",
      not s._rate_active, s._rate_why)
s = _ready(); s._unlock_t = None; s.tick()
check("P5 no unlock since the ground state -> refused", not s._rate_active, s._rate_why)
s = _ready(); s._last_tx_t = _time.time() - 5.0
s._fm_refresh_t = _time.time()
s._rate_active, s._rate_why = s._rate_ready(_time.time())
check("P5 setpoint stream gap re-locks thrust -> unlock no longer valid",
      not s._rate_active, s._rate_why)
s = rate_server()
s.begin_takeoff(0.5)
check("P5 unlock recorded at takeoff from the ground", s._unlock_t is not None)
s.cf.sent.clear(); s._srv_stop(None, None)
check("P5 stop clears the unlock", s._unlock_t is None)
s = _ready(); s.emergency("x")
check("P5 emergency clears the unlock", s._unlock_t is None)
s = rate_server()
def _boom(*a): raise IOError("radio")
s.cf.commander.send_setpoint = _boom
s.begin_takeoff(0.5); s.state = C.State.FLYING; s.tick()
check("P5 unlock send failure -> handoff refused", not s._rate_active, s._rate_why)

print("\n[P6] thrust <= 0 rejected, raw pwm gated, config validated at startup (finding 6)")
s = rate_server(); fly(s); s.tick()
for _t in (0.0, -1.0):
    s._cmd = None
    s._cmd_rate(rmsg(0.1, t=_t))
    check(f"P6 thrust {_t} N rejected, not stored", s._cmd is None)
s._cmd_rate(rmsg(t=0.3, pwm=40000))
check("P6 raw thrust_pwm refused by default (same topic cannot bypass the map)",
      s._cmd is None and s.rate_allow_raw_pwm is False)
for _k, _v in (("thrust_scale", 0.0), ("thrust_scale", float("nan")),
               ("rate_max_rp_dps", -1.0), ("rate_tilt_limit_deg", 0.0),
               ("thrust_pwm_min", 60000), ("thrust_pwm_max", 70000)):
    class _N(S.CrazyflieServer):
        pass
    _orig = _FakeNode.declare_parameter
    def _decl(self, name, default, _k=_k, _v=_v):
        if name == "enable_rate_cmd":
            return _Param(True)
        if name == _k:
            return _Param(_v)
        return _orig(self, name, default)
    _FakeNode.declare_parameter = _decl
    try:
        _s = S.CrazyflieServer()
    finally:
        _FakeNode.declare_parameter = _orig
    check(f"P6 startup: {_k}={_v} disables cmd_rate", _s.enable_rate_cmd is False)

print("\n[G] geofence: soft-box land, wall zone cut, floor cut")
_gb = C.Bounds(0.0, 5.0, 0.0, 4.0, 0.0, 1.8)
_gw = (-1.0, 6.0, -1.0, 5.0)            # walls, outside the soft box
_gbase = dict(now=1000.0, flying=True, state=C.State.FLYING, pose_age=0.001,
              quat_age=0.001, pos_only=False, pos=(2., 2., 1.0), ekf=(2., 2., 1.0),
              got_state=True, vbat=3.9, bounds=_gb, walls=_gw, stale_since=None,
              diverge_since=None, cmd_age=0.01, command_timeout=0.3)
def _g(**kw):
    return C.supervise(**{**_gbase, **kw})
r = _g()
check("G inside everything -> no action", r.action is None, r.action)
r = _g(pos=(5.01, 2., 1.0))
check("G 1 cm outside box -> land", r.action == "land", r.action)
r = _g(pos=(2., 2., 1.81))
check("G above box top -> land (no ceiling cut)", r.action == "land", r.action)
r = _g(pos=(2., 2., 1.9))
check("G just above box top, away from walls -> land", r.action == "land", r.action)
r = _g(pos=(5.0, 2., 1.0))
check("G exactly on box edge -> no action", r.action is None, r.action)
r = _g(pos=(5.5, 2., 1.0))
check("G 0.5 m outside box, 0.5 m from wall -> land, not cut", r.action == "land", r.action)
r = _g(pos=(5.95, 2., 1.0))
check("G 5 cm from wall at z 1.0 -> kill", r.action == "kill", r.action)
r = _g(pos=(-0.95, 2., 1.0))
check("G 5 cm from xmin wall -> kill", r.action == "kill", r.action)
r = _g(pos=(2., 4.95, 1.0))
check("G 5 cm from ymin/ymax wall -> kill", r.action == "kill", r.action)
r = _g(pos=(6.5, 2., 1.0))
check("G beyond a wall -> kill", r.action == "kill", r.action)
r = _g(pos=(5.85, 2., 1.0))
check("G 15 cm from wall (outside zone) -> land", r.action == "land", r.action)
r = _g(pos=(5.95, 2., 2.0))
check("G wall zone at z exactly 2.0 -> kill", r.action == "kill", r.action)
r = _g(pos=(5.95, 2., 2.4))
check("G 5 cm from wall at z 2.4 -> land + timer started",
      r.action == "land" and r.wall_since == 1000.0, f"{r.action} {r.wall_since}")
r2 = _g(pos=(5.95, 2., 2.4), now=1000.5, wall_since=r.wall_since)
check("G ...still land at 0.5 s", r2.action == "land" and r2.wall_since == 1000.0, r2.action)
r2 = _g(pos=(5.95, 2., 2.4), now=1001.0, wall_since=r.wall_since)
check("G ...kill once the timeout expires", r2.action == "kill", r2.action)
r2 = _g(pos=(5.95, 2., 1.9), now=1000.2, wall_since=r.wall_since)
check("G ...kill immediately once z <= 2.0 in the zone", r2.action == "kill", r2.action)
r2 = _g(pos=(5.5, 2., 2.4), now=1000.6, wall_since=r.wall_since)
check("G leaving the zone resets the timer", r2.wall_since is None, r2.wall_since)
r3 = _g(pos=(5.95, 2., 2.4), now=1001.5, wall_since=r2.wall_since)
check("G ...and re-entering starts a fresh one (land, not kill)",
      r3.action == "land" and r3.wall_since == 1001.5, f"{r3.action} {r3.wall_since}")
r = _g(pos=(2., 2., 0.05))
check("G z 0.05 while FLYING -> kill", r.action == "kill", r.action)
r = _g(pos=(2., 2., 0.10))
check("G z 0.10 while FLYING -> no floor cut", r.action is None, r.action)
for _st in (C.State.TAKEOFF, C.State.LANDING):
    r = _g(pos=(2., 2., 0.05), state=_st)
    check(f"G z 0.05 during {_st} -> no action", r.action is None, r.action)
r = _g(pos=(2., 2., 0.05), state=None)
check("G state unknown -> no floor cut", r.action is None, r.action)
r = _g(pos=(2., 2., 0.05), flying=False, state=C.State.IDLE)
check("G on the ground (IDLE) -> no action", r.action is None, r.action)
r = _g(pos=(5.95, 2., 1.0), state=C.State.LANDING)
check("G wall zone still cuts while LANDING", r.action == "kill", r.action)
r = _g(pos=(2., 2., 1.0), walls=None)
check("G walls unset: inside -> no action", r.action is None, r.action)
r = _g(pos=(5.5, 2., 1.0), walls=None)
check("G walls unset: 0.5 m outside box -> land", r.action == "land", r.action)
r = _g(pos=(5.9, 2., 1.0), walls=None)
check("G walls unset: 0.9 m outside box -> old 0.80 m kill", r.action == "kill", r.action)
r = _g(pos=(5.85, 2., 1.0))
check("G walls set: 0.9 m outside box is NOT a kill (old margin removed)",
      r.action == "land", r.action)
r = _g(pos=(5.95, 2., 1.0), cmd_age=5.0)
check("G command timeout + wall zone -> kill beats hold", r.action == "kill", r.action)
r = _g(pos=(5.95, 2., 2.4), cmd_age=5.0)
check("G command timeout + high wall zone -> land beats hold", r.action == "land", r.action)
r = _g(cmd_age=5.0)
check("G command timeout alone still holds", r.action == "hold", r.action)
r = _g(pos=(5.95, 2., 1.0), pose_age=0.5)
check("G mocap dead still first", r.action == "kill" and "MOCAP" in r.reason, r.reason)
r = _g(pos=(5.95, 2., 1.0), quat_age=1.0)
check("G orientation dead (land) precedes wall cut", r.action == "land", r.action)
r = _g(pos=(2., 2., 0.05), ekf=(3., 2., 0.05), diverge_since=900.0)
check("G floor cut precedes EKF land", r.action == "kill", r.action)

print("\n" + "=" * 62)
print(f"{PASS} passed, {FAIL} failed")
print("ALL CHECKS PASSED" if FAIL == 0 else "FAILURES ABOVE")
sys.exit(1 if FAIL else 0)
