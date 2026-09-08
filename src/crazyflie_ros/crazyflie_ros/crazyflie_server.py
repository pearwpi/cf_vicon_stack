#!/usr/bin/env python3
"""
crazyflie_server.py -- ROS 2 driver for a Vicon-fed Crazyflie.

This is the "commands out" half of the stack, as a ROS 2 node. It owns the
radio and it owns every safety decision. Nothing else in the system is allowed
to talk to the Crazyflie.

    /vicon/<body>/<body>   (PoseStamped, from vicon_receiver)
             |
             v
    +--------------------------------+
    |  crazyflie_server              |
    |    flip filter  -> extpose --> | ==(Crazyradio)==> Crazyflie EKF
    |    supervisor                  |
    |    setpoint choke point -----> | ==(Crazyradio)==> Crazyflie commander
    +--------------------------------+
             ^                    |
    cmd_position / cmd_hover      +--> status, pose
    cmd_velocity_world            
    cmd_full_state                
    services: takeoff land arm stop emergency notify_setpoints_stop

WHY COMMANDS ARE BUFFERED, NOT RELAYED
--------------------------------------
Upstream Crazyswarm2 calls cflib directly from each cmd_* subscription
callback. This node does not. Every command is stored and then transmitted by
one 50 Hz timer, because:

  1. The supervisor gets a veto. A setpoint sent straight from a callback is
     already on the radio before any watchdog has looked at it.
  2. The radio is rate-limited by construction. A publisher looping at 500 Hz
     cannot flood a 2M link that is also carrying extpose.
  3. There is exactly one line in this file that transmits a setpoint, so
     "what did we actually send" has one answer.

The cost is that commands are downsampled to control_hz. For a 50 Hz policy or
a human on a keyboard that is not a cost at all.

THE COMMAND WATCHDOG, AND WHY THE SPLIT CREATES THE NEED FOR IT
---------------------------------------------------------------
In the standalone script the operator and the radio live in one process: if the
controller dies, the sender dies with it. Split into two nodes, teleop can
crash, the executor can stall, or DDS can partition, and this node would
cheerfully keep re-transmitting the last setpoint forever.

So: no new command for command_timeout seconds while flying and the node stops
relaying and holds position at the drone's measured location. That is a "stop
listening", not an emergency, so it must not land and must not kill.

command_timeout defaults to 0.30 s, deliberately inside the firmware's own
COMMANDER_WDT_TIMEOUT_STABILIZE of 500 ms. We decide what happens, not the
firmware. (Firmware behaviour, for reference: at 500 ms it levels the attitude
and holds; at 2000 ms it cuts the motors and the drone falls.)

UNITS -- READ THIS BEFORE PUBLISHING ANYTHING
---------------------------------------------
The message definitions are Crazyswarm2's, verbatim, so that this node stays
swap-compatible with upstream. Upstream's unit conventions are internally
inconsistent, and they have been reproduced here EXACTLY rather than tidied.
Fixing them locally would mean that swapping in the real crazyflie_server later
silently changes what your policy commands, which is far worse than an ugly
table:

    Position.yaw            DEGREES        passed straight to the firmware
    Hover.yaw_rate          RADIANS/s      AND SIGN-FLIPPED  (-degrees(v))
    VelocityWorld.yaw_rate  DEGREES/s      passed straight through
    FullState.twist.angular DEGREES/s      passed straight through
    GoTo.yaw                RADIANS        despite the "# deg" comment in the .srv

If you want a sane interface, put a converter node in front of this one. Do not
"fix" it here.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

from crazyflie_interfaces.msg import (FullState, Hover, Position, Status,
                                      VelocityWorld)
from crazyflie_interfaces.srv import Arm, GoTo, Land, NotifySetpointsStop, Stop, Takeoff

from crazyflie_ros import cf_core as C


# ==============================================================================
# cflib is imported lazily so that --help, tests and a dry `ros2 pkg` sweep do
# not require it. A missing cflib must fail loudly at connect time, not silently
# degrade into a node that publishes nothing.
# ==============================================================================
def _import_cflib():
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    return cflib.crtp, Crazyflie, LogConfig, SyncCrazyflie


def _arm(cf, do_arm: bool) -> bool:
    """Send an arming request across cflib/firmware generations.

    Newer cflib exposes cf.supervisor.send_arming_request and deprecates
    cf.platform.send_arming_request. Firmware below CRTP v12 has no supervisor
    subsystem and needs no arming step at all. Returns True if a request was
    actually delivered. Identical to the standalone script's helper.
    """
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        for holder in ("supervisor", "platform"):
            svc = getattr(cf, holder, None)
            fn = getattr(svc, "send_arming_request", None) if svc else None
            if fn is None:
                continue
            try:
                fn(do_arm)
                return True
            except Exception:
                continue
    return False


class CrazyflieServer(Node):

    def __init__(self):
        super().__init__("crazyflie_server")

        # ---- parameters ------------------------------------------------------
        d = self.declare_parameter
        self.uri = d("uri", "radio://0/80/2M/E7E7E7E701").value
        self.vicon_topic = d("vicon_topic", "/vicon/crazyflie1/crazyflie1").value
        self.prefix = d("prefix", "cf1").value
        self.control_hz = float(d("control_hz", C.CONTROL_HZ).value)
        self.extpose_hz = float(d("extpose_hz", C.EXTPOSE_HZ).value)
        self.expected_hz = float(d("mocap_expected_hz", C.MOCAP_EXPECTED_HZ).value)
        self.command_timeout = float(d("command_timeout", C.COMMAND_TIMEOUT_S).value)
        self.max_yaw_rate_dps = float(d("max_yaw_rate_dps", 720.0).value)
        self.yaw_sign = int(d("yaw_sign", 1).value)
        self.pos_only = bool(d("pos_only", False).value)
        self.no_fly = bool(d("no_fly", False).value)
        self.hover_z = float(d("hover_z", 0.60).value)
        self.yaw_offset = float(d("yaw_offset", 0.0).value)
        vol = list(d("volume", list(C.DEFAULT_VOLUME)).value)
        self.volume = (float(vol[0]), float(vol[1]), float(vol[2]))
        bounds_p = list(d("bounds", []).value or [])

        # ---- mocap state (written by the executor thread, read by others) ----
        self._pose_lock = threading.Lock()
        self._latest = None            # cf_core.Pose
        self._last_yaw = None
        self._last_yaw_t = None
        self._last_quat_ok_t = None
        self._yaw_rejects = 0
        self._occluded = 0
        self._occl_warned = False
        self._yaw_warned = False
        self._unit_warned = False
        self._recv_times = []
        self._max_recv_times = max(int(self.expected_hz), 50)

        # ---- telemetry (written by cflib's RX thread) ------------------------
        self._tlm_lock = threading.Lock()
        self._est = None               # (x, y, z) onboard estimate
        self._vbat = 0.0
        self._got_state = False

        # ---- command state (written by executor, read by executor) -----------
        self._cmd = None               # ("position"|"hover"|"velocity_world"|"full_state", payload)
        self._cmd_t = 0.0
        self._cmd_timed_out = False

        # ---- flight state ----------------------------------------------------
        self.state = C.State.IDLE
        self.bounds = (C.Bounds(*bounds_p) if len(bounds_p) == 6 else None)
        self._stale_since = None
        self._diverge_since = None
        self._mocap_stale = False
        self._hold = None              # (x, y, z, yaw_deg) frozen setpoint
        self._takeoff_target = None
        self._armed = False

        self.cf = None
        self.scf = None
        self._extpose_stop = threading.Event()
        self._extpose_thread = None
        self._sent_extpose = 0

        # ---- ROS interfaces --------------------------------------------------
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        cb = MutuallyExclusiveCallbackGroup()

        self.create_subscription(PoseStamped, self.vicon_topic,
                                 self._vicon_cb, sensor_qos, callback_group=cb)
        p = self.prefix
        self.create_subscription(Position, f"{p}/cmd_position",
                                 self._cmd_position, 10, callback_group=cb)
        self.create_subscription(Hover, f"{p}/cmd_hover",
                                 self._cmd_hover, 10, callback_group=cb)
        self.create_subscription(VelocityWorld, f"{p}/cmd_velocity_world",
                                 self._cmd_velocity_world, 10, callback_group=cb)
        self.create_subscription(FullState, f"{p}/cmd_full_state",
                                 self._cmd_full_state, 10, callback_group=cb)

        self.pub_status = self.create_publisher(Status, f"{p}/status", 10)
        self.pub_pose = self.create_publisher(PoseStamped, f"{p}/pose", 10)

        self.create_service(Takeoff, f"{p}/takeoff", self._srv_takeoff, callback_group=cb)
        self.create_service(Land, f"{p}/land", self._srv_land, callback_group=cb)
        self.create_service(Arm, f"{p}/arm", self._srv_arm, callback_group=cb)
        self.create_service(Stop, f"{p}/stop", self._srv_stop, callback_group=cb)
        self.create_service(Empty, f"{p}/emergency", self._srv_emergency, callback_group=cb)
        self.create_service(NotifySetpointsStop, f"{p}/notify_setpoints_stop",
                            self._srv_notify_stop, callback_group=cb)
        self.create_service(GoTo, f"{p}/go_to", self._srv_go_to, callback_group=cb)

        self.get_logger().info(f"core: {C.CORE_REVISION}")
        self.get_logger().info(
            f"uri={self.uri} mocap={self.vicon_topic} prefix={p} "
            f"control={self.control_hz:.0f}Hz extpose={self.extpose_hz:.0f}Hz "
            f"cmd_timeout={self.command_timeout*1000:.0f}ms"
            + ("  [NO-FLY]" if self.no_fly else ""))

    # =========================================================================
    # MOCAP
    # =========================================================================
    def _vicon_cb(self, msg):
        now = time.time()
        p = C.Pose(
            x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z,
            qx=msg.pose.orientation.x, qy=msg.pose.orientation.y,
            qz=msg.pose.orientation.z, qw=msg.pose.orientation.w,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            recv_time=now)

        # Same occluded-segment guard as the standalone script; see
        # cf_core.occluded_sentinel for why exact equality is correct here.
        if C.occluded_sentinel(p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw):
            self._occluded += 1
            if not self._occl_warned:
                self._occl_warned = True
                self.get_logger().error(
                    "OCCLUDED: the Vicon SDK's not-tracked sentinel. Frames "
                    "dropped so the mocap watchdog can act.")
            return

        # Unit sanity: a Crazyflie flying in a room is never 100 m from origin.
        # If it is, someone is publishing millimetres and the drone will be
        # commanded to fly through a wall at full tilt.
        if not self._unit_warned and max(abs(p.x), abs(p.y), abs(p.z)) > 100.0:
            self._unit_warned = True
            self.get_logger().error(
                "Pose magnitude > 100 -- this stream looks like MILLIMETRES, "
                "not metres. DO NOT FLY. Fix the publisher or add a /1000 scale.")

        # Quaternion sanity: an unnormalised or zero quat silently corrupts the
        # EKF's attitude update.
        n = C.quat_norm(p.qx, p.qy, p.qz, p.qw)
        if n < 1e-6:
            self.get_logger().warn("Zero-norm quaternion received; frame dropped.")
            return
        if abs(n - 1.0) > 1e-3:
            p.qx, p.qy, p.qz, p.qw = p.qx / n, p.qy / n, p.qz / n, p.qw / n

        # Same flip decision as the standalone script -- literally the same
        # function, so the two front ends cannot disagree about what a solver
        # flip is.
        rejected, d, rate = C.flip_check(p.yaw_deg, self._last_yaw,
                                         self._last_yaw_t, now,
                                         self.max_yaw_rate_dps)
        if rejected:
            p.quat_ok = False
            self._yaw_rejects += 1
            if not self._yaw_warned:
                self._yaw_warned = True
                self.get_logger().error(
                    f"Yaw jumped {d:.0f} deg ({rate:.0f} deg/s). The mocap "
                    f"rigid body is rotationally AMBIGUOUS -- Tracker is "
                    f"switching between two solutions. Quaternion rejected; "
                    f"position still used. Fix the marker layout.")
        else:
            self._last_yaw = p.yaw_deg
            self._last_yaw_t = now
            self._last_quat_ok_t = now

        self._recv_times.append(now)
        if len(self._recv_times) > self._max_recv_times:
            del self._recv_times[0]

        with self._pose_lock:
            self._latest = p

    def latest(self):
        with self._pose_lock:
            return self._latest

    def pose_age(self) -> float:
        p = self.latest()
        return 1e9 if p is None else (time.time() - p.recv_time)

    def quat_age(self) -> float:
        """Seconds since the orientation was last usable.

        Deliberately NOT pose_age(): a solver latched onto a mirrored solution
        keeps delivering fresh, smooth, low-latency POSITION while every
        quaternion fails. To a position watchdog that looks perfectly healthy.
        """
        t = self._last_quat_ok_t
        return 1e9 if t is None else (time.time() - t)

    def rate_hz(self) -> float:
        if len(self._recv_times) < 2:
            return 0.0
        span = self._recv_times[-1] - self._recv_times[0]
        return (len(self._recv_times) - 1) / span if span > 0 else 0.0

    # =========================================================================
    # RADIO
    # =========================================================================
    def connect(self) -> bool:
        try:
            crtp, Crazyflie, LogConfig, SyncCrazyflie = _import_cflib()
        except Exception as exc:
            self.get_logger().fatal(f"cflib is not importable: {exc}")
            return False

        crtp.init_drivers(enable_debug_driver=False)
        self.get_logger().info(f"connecting to {self.uri} ...")
        try:
            self.scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
            self.scf.open_link()
        except Exception as exc:
            self.get_logger().fatal(f"could not open {self.uri}: {exc}")
            return False
        self.cf = self.scf.cf
        self.get_logger().info("connected")

        # Onboard estimate + battery. These land on cflib's RX thread, NOT on
        # the ROS executor, hence _tlm_lock.
        lg = LogConfig(name="state", period_in_ms=50)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
            lg.add_variable(v, "float")
        lg.add_variable("pm.vbat", "float")
        lg.data_received_cb.add_callback(self._on_state)
        try:
            self.cf.log.add_config(lg)
            lg.start()
        except Exception as exc:
            self.get_logger().warn(f"telemetry log failed to start: {exc}")

        self._start_extpose()
        return True

    def _on_state(self, ts, data, cfg):
        with self._tlm_lock:
            self._est = (data.get("stateEstimate.x", 0.0),
                         data.get("stateEstimate.y", 0.0),
                         data.get("stateEstimate.z", 0.0))
            self._vbat = data.get("pm.vbat", self._vbat)
            self._got_state = True

    def _start_extpose(self):
        """Push mocap into the EKF on its own thread, independent of control.

        Separate from the control timer on purpose: the EKF wants a steady pose
        stream even during the ticks where the supervisor has decided not to
        send a setpoint. Tying the two together means a control-side stall also
        starves the estimator, which is the opposite of what you want.
        """
        hz = self.extpose_hz
        if self.yaw_offset:
            self.get_logger().info(
                f"rotating injected quaternion by {self.yaw_offset:+.1f} deg about Z")
        self.get_logger().info(
            f"injecting {'extpos (position only)' if self.pos_only else 'extpose'} "
            f"at {hz:.0f} Hz")

        def loop():
            period = 1.0 / hz
            nxt = time.time()
            while not self._extpose_stop.is_set():
                p = self.latest()
                if p is not None and (time.time() - p.recv_time) < 0.25:
                    try:
                        if self.pos_only or not p.quat_ok:
                            self.cf.extpos.send_extpos(p.x, p.y, p.z)
                        else:
                            qx, qy, qz, qw = p.qx, p.qy, p.qz, p.qw
                            if self.yaw_offset:
                                h = math.radians(self.yaw_offset) / 2.0
                                cz, sz = math.cos(h), math.sin(h)
                                qx, qy, qz, qw = (cz * qx - sz * qy,
                                                  cz * qy + sz * qx,
                                                  cz * qz + sz * qw,
                                                  cz * qw - sz * qz)
                            self.cf.extpos.send_extpose(p.x, p.y, p.z, qx, qy, qz, qw)
                        self._sent_extpose += 1
                    except Exception as exc:
                        self.get_logger().warn(f"extpose send failed: {exc}")
                nxt += period
                sleep = nxt - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    nxt = time.time()   # fell behind; resync rather than spin

        self._extpose_thread = threading.Thread(target=loop, daemon=True,
                                                name="extpose")
        self._extpose_thread.start()

    # =========================================================================
    # COMMAND INTAKE -- record only. Transmission happens in _tick().
    # =========================================================================
    def _accept(self, kind, payload):
        self._cmd = (kind, payload)
        self._cmd_t = time.time()
        if self._cmd_timed_out:
            self._cmd_timed_out = False
            self._hold = None
            self.get_logger().info("commands resumed")

    def _cmd_position(self, msg):
        # Position.yaw is DEGREES upstream. Do not convert.
        self._accept("position", (msg.x, msg.y, msg.z, msg.yaw))

    def _cmd_hover(self, msg):
        # Hover.yaw_rate is RADIANS/s upstream AND sign-flipped on the way out.
        self._accept("hover", (msg.vx, msg.vy,
                               -1.0 * math.degrees(msg.yaw_rate),
                               msg.z_distance))

    def _cmd_velocity_world(self, msg):
        # VelocityWorld.yaw_rate is DEGREES/s upstream. Straight through.
        self._accept("velocity_world", (msg.vel.x, msg.vel.y, msg.vel.z,
                                        msg.yaw_rate))

    def _cmd_full_state(self, msg):
        # FullState.twist.angular is DEGREES/s upstream. Straight through.
        q = msg.pose.orientation
        self._accept("full_state", (
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            [msg.acc.x, msg.acc.y, msg.acc.z],
            [q.x, q.y, q.z, q.w],
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z))

    # =========================================================================
    # CONTROL TICK
    # =========================================================================
    def tick(self):
        now = time.time()
        p = self.latest()

        self._publish_status(now, p)

        if self.cf is None:
            return

        flying = self.state in C.State.FLYING_STATES

        with self._tlm_lock:
            est, vbat, got_state = self._est, self._vbat, self._got_state

        # Bounds default to a box centred on wherever the drone actually is, so
        # a Vicon origin parked in the corner of the capture space does not put
        # a grounded drone "out of bounds".
        if self.bounds is None and p is not None:
            self.bounds = C.Bounds.centered_on(p.x, p.y, *self.volume)
            self.get_logger().info(f"flight volume {self.bounds.describe()}")
        if self.bounds is None:
            return

        cmd_age = (now - self._cmd_t) if self._cmd_t else 1e9

        r = C.supervise(
            now=now, flying=flying,
            pose_age=self.pose_age(), quat_age=self.quat_age(),
            pos_only=self.pos_only,
            pos=(p.x, p.y, p.z) if p is not None else None,
            ekf=est, got_state=got_state, vbat=vbat,
            bounds=self.bounds,
            stale_since=self._stale_since, diverge_since=self._diverge_since,
            yaw_rejects=self._yaw_rejects,
            cmd_age=cmd_age, command_timeout=self.command_timeout)

        self._stale_since = r.stale_since
        self._diverge_since = r.diverge_since
        self._mocap_stale = r.mocap_stale
        if r.became_stale and flying:
            self.get_logger().warn(
                f"mocap stale ({self.pose_age()*1000:.0f} ms) -- setpoint frozen")

        if r.action == "kill":
            self.emergency(r.reason)
            return
        if r.action == "land":
            self.get_logger().error(f"AUTO-LAND: {r.reason}")
            self.begin_land()
            # fall through: a landing still needs setpoints sent this tick
        elif r.action == "hold":
            if not self._cmd_timed_out:
                self._cmd_timed_out = True
                self.get_logger().warn(r.reason)
                if p is not None:
                    self._hold = (p.x, p.y, p.z, p.yaw_deg)

        if not flying or self.no_fly:
            return
        self._transmit(now, p)

    def _transmit(self, now, p):
        """THE ONLY PLACE A SETPOINT LEAVES THIS NODE."""
        try:
            if self.state == C.State.LANDING:
                self._land_step(now, p)
                return
            if self.state == C.State.TAKEOFF:
                self._takeoff_step(now, p)
                return
            if self._hold is not None:
                x, y, z, yaw = self._hold
                self._send_position(x, y, z, yaw, p)
                return
            if self._cmd is None:
                return
            kind, a = self._cmd
            if kind == "position":
                self._send_position(a[0], a[1], a[2], a[3], p)
            elif kind == "hover":
                self.cf.commander.send_hover_setpoint(*a)
            elif kind == "velocity_world":
                self.cf.commander.send_velocity_world_setpoint(*a)
            elif kind == "full_state":
                self.cf.commander.send_full_state_setpoint(*a)
        except Exception as exc:
            self.get_logger().error(f"setpoint send failed: {exc}")

    def _send_position(self, x, y, z, yaw_deg, p):
        """Geofence clamp, then leash, then transmit.

        yaw_sign exists because the sign convention on the CRTP position
        setpoint has differed across firmware generations. Verify it WITH THE
        PROPELLERS OFF. Do not compensate by swapping the operator's keys --
        that leaves the sign error in place for every other consumer.
        """
        x, y, z = self.bounds.clamp(x, y, z)
        if p is not None:
            x, y, z = C.apply_leash(x, y, z, p.x, p.y, p.z)
        self.cf.commander.send_position_setpoint(x, y, z,
                                                 self.yaw_sign * yaw_deg)

    # =========================================================================
    # FLIGHT STATE
    # =========================================================================
    def _takeoff_step(self, now, p):
        x, y, z0, yaw = self._takeoff_target
        z = min(z0, self._climb_z)
        self._climb_z += C.TAKEOFF_CLIMB_MS / self.control_hz
        self._send_position(x, y, z, yaw, p)
        if self._climb_z >= z0:
            self.state = C.State.FLYING
            self._hold = (x, y, z0, yaw)
            self.get_logger().info(f"hovering at {z0:.2f} m")

    def _land_step(self, now, p):
        x, y, z, yaw = self._hold if self._hold else (0.0, 0.0, 0.0, 0.0)
        z -= C.LAND_DESCENT_MS / self.control_hz
        self._hold = (x, y, z, yaw)
        if z <= C.LAND_CUTOFF_Z:
            self.get_logger().info("landed; motors off")
            self.stop_motors()
            self.state = C.State.IDLE
            self._hold = None
            return
        self._send_position(x, y, z, yaw, p)

    def begin_takeoff(self, height) -> bool:
        if self.no_fly:
            self.get_logger().error("takeoff refused: no_fly is set")
            return False
        p = self.latest()
        if p is None:
            self.get_logger().error("takeoff refused: no mocap pose yet")
            return False
        if self.pose_age() > C.MOCAP_STALE_S:
            self.get_logger().error("takeoff refused: mocap pose is stale")
            return False
        if self.bounds is None:
            self.bounds = C.Bounds.centered_on(p.x, p.y, *self.volume)

        ok, reasons = C.prearm_check(p, self.bounds, pos_only=self.pos_only)
        if not ok:
            self.get_logger().error("TAKEOFF REFUSED:")
            for why in reasons:
                self.get_logger().error(f"  - {why}")
            return False

        if not self._armed:
            _arm(self.cf, True)
            self._armed = True
        self._takeoff_target = (p.x, p.y, float(height), p.yaw_deg)
        self._climb_z = p.z
        self._hold = None
        self.state = C.State.TAKEOFF
        self.get_logger().info(f"taking off to {height:.2f} m")
        return True

    def begin_land(self):
        if self.state in (C.State.IDLE, C.State.STOPPED, C.State.LANDING):
            return
        p = self.latest()
        if self._hold is None and p is not None:
            self._hold = (p.x, p.y, p.z, p.yaw_deg)
        self.state = C.State.LANDING
        self.get_logger().info("landing")

    def stop_motors(self):
        try:
            self.cf.commander.send_stop_setpoint()
            try:
                self.cf.commander.send_notify_setpoint_stop(0)
            except Exception:
                pass          # older cflib; the stop setpoint alone suffices
            _arm(self.cf, False)
            self._armed = False
        except Exception as exc:
            self.get_logger().error(f"stop failed: {exc}")

    def emergency(self, reason):
        self.get_logger().fatal(f"EMERGENCY STOP: {reason}")
        self.state = C.State.STOPPED
        self._hold = None
        try:
            self.cf.commander.send_stop_setpoint()
            _arm(self.cf, False)
            self._armed = False
        except Exception:
            pass

    # =========================================================================
    # SERVICES
    # =========================================================================
    def _srv_takeoff(self, req, resp):
        self.begin_takeoff(req.height if req.height > 0.0 else self.hover_z)
        return resp

    def _srv_land(self, req, resp):
        self.begin_land()
        return resp

    def _srv_arm(self, req, resp):
        if self.cf is not None:
            _arm(self.cf, bool(req.arm))
            self._armed = bool(req.arm)
            self.get_logger().info(f"{'armed' if req.arm else 'disarmed'}")
        return resp

    def _srv_stop(self, req, resp):
        self.stop_motors()
        self.state = C.State.IDLE
        self._hold = None
        return resp

    def _srv_emergency(self, req, resp):
        self.emergency("commanded via /emergency")
        return resp

    def _srv_notify_stop(self, req, resp):
        try:
            self.cf.commander.send_notify_setpoint_stop(
                int(req.remain_valid_millisecs))
        except Exception as exc:
            self.get_logger().warn(f"notify_setpoints_stop failed: {exc}")
        return resp

    def _srv_go_to(self, req, resp):
        """Move the hold point. GoTo.yaw is RADIANS upstream despite the
        '# deg' comment in the .srv file; matched here deliberately.

        This is NOT the firmware high-level commander. Streaming CRTP setpoints
        sit at priority 2 and the high-level commander at priority 1, so a
        stream of setpoints permanently masks it. Rather than interleave the
        two and depend on notify_setpoints_stop landing at the right moment,
        go_to just retargets the position setpoint this node is already
        sending.
        """
        p = self.latest()
        if p is None:
            return resp
        if req.relative:
            base = self._hold if self._hold else (p.x, p.y, p.z, p.yaw_deg)
            self._hold = (base[0] + req.goal.x, base[1] + req.goal.y,
                          base[2] + req.goal.z,
                          base[3] + math.degrees(req.yaw))
        else:
            self._hold = (req.goal.x, req.goal.y, req.goal.z,
                          math.degrees(req.yaw))
        self._accept("position", self._hold)
        return resp

    # =========================================================================
    # STATUS
    # =========================================================================
    def _publish_status(self, now, p):
        with self._tlm_lock:
            est, vbat = self._est, self._vbat
        s = Status()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = self.prefix
        s.battery_voltage = float(vbat)
        bits = 0
        if self.state in C.State.FLYING_STATES:
            bits |= Status.SUPERVISOR_INFO_IS_FLYING
        if self._armed:
            bits |= Status.SUPERVISOR_INFO_IS_ARMED
        if p is not None and self.bounds is not None:
            ok, _ = C.prearm_check(p, self.bounds, pos_only=self.pos_only)
            if ok:
                bits |= Status.SUPERVISOR_INFO_CAN_BE_ARMED
                bits |= Status.SUPERVISOR_INFO_CAN_FLY
        if p is not None and p.tilt_deg > 120.0:
            bits |= Status.SUPERVISOR_INFO_IS_TUMBLED
        s.supervisor_info = bits
        self.pub_status.publish(s)

        if est is not None:
            m = PoseStamped()
            m.header.stamp = s.header.stamp
            m.header.frame_id = "map"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = est
            m.pose.orientation.w = 1.0
            self.pub_pose.publish(m)

    def shutdown(self):
        self._extpose_stop.set()
        if self._extpose_thread:
            self._extpose_thread.join(timeout=1.0)
        if self.cf is not None:
            try:
                self.stop_motors()
            except Exception:
                pass
        if self.scf is not None:
            try:
                self.scf.close_link()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieServer()
    if not node.connect():
        node.get_logger().fatal("no radio link; shutting down")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.create_timer(1.0 / node.control_hz, node.tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
