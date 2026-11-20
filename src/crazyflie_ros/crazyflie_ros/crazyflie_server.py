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
    cmd_position / cmd_hover      +--> status, pose, imu
    cmd_velocity_world            
    cmd_full_state                
    cmd_rate  (opt-in; docs/RATE_MODE.md)
    services: takeoff land arm stop emergency reset_emergency notify_setpoints_stop

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

The one exception is cmd_rate (crazyflie_interfaces/RateThrust), which is local
to this stack, not Crazyswarm2's, and is therefore plain SI:

    RateThrust.*_rate       RADIANS/s      body FLU, converted here (yaw negated)
    RateThrust.thrust       NEWTONS        collective, mapped to the thrust word
    imu (sensor_msgs/Imu)   rad/s, m/s^2   body FLU, gravity included

cmd_rate is OFF unless enable_rate_cmd AND rate_mode_setup are both set and the
firmware has confirmed flightmode.stabModeRoll/Pitch/Yaw = 0. See
docs/RATE_MODE.md before enabling either.
"""
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty

from crazyflie_interfaces.msg import (FullState, Hover, LogDataGeneric,
                                      Position, RateThrust, Status,
                                      VelocityWorld)
from crazyflie_interfaces.srv import Arm, GoTo, Land, NotifySetpointsStop, Stop, Takeoff

from crazyflie_ros import cf_core as C
from crazyflie_ros import rate_cmd as R


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
        # Takeoff is only eligible from the ground: state IDLE AND measured z
        # at or below this (a /stop mid-air leaves state IDLE with the drone
        # still up there).
        self.takeoff_ground_z_max = float(d("takeoff_ground_z_max", 0.25).value)
        self.yaw_offset = float(d("yaw_offset", 0.0).value)
        vol = list(d("volume", list(C.DEFAULT_VOLUME)).value)
        self.volume = (float(vol[0]), float(vol[1]), float(vol[2]))
        # Declared by TYPE, not by an empty-list default. rclpy infers the
        # type of [] as BYTE_ARRAY, so a YAML file supplying six floats is
        # rejected at startup with "expecting type BYTE_ARRAY" -- the node
        # dies before it ever reads the geofence it was handed.
        bounds_p = list(d("bounds", Parameter.Type.DOUBLE_ARRAY).value or [])

        # ---- body-rate path (docs/RATE_MODE.md). Both switches default OFF:
        # enable_rate_cmd lets cmd_rate be accepted at all; rate_mode_setup
        # lets this node WRITE flightmode.stabModeRoll/Pitch = 0 at takeoff
        # (and restore 1 on land/stop/emergency/shutdown).
        self.enable_rate_cmd = bool(d("enable_rate_cmd", False).value)
        self.rate_mode_setup = bool(d("rate_mode_setup", False).value)
        self.rate_rp_max_dps = float(d("rate_max_rp_dps", R.RATE_RP_MAX_DPS).value)
        self.rate_yaw_max_dps = float(d("rate_max_yaw_dps", R.RATE_YAW_MAX_DPS).value)
        self.rate_tilt_limit = float(d("rate_tilt_limit_deg", R.RATE_TILT_LIMIT_DEG).value)
        self.thrust_poly = tuple(float(c) for c in
                                 d("thrust_poly", list(R.FW_LINEAR_POLY)).value)
        self.thrust_scale = float(d("thrust_scale", 1.0).value)
        self.thrust_pwm_min = int(d("thrust_pwm_min", C.ATT_THRUST_MIN).value)
        self.thrust_pwm_max = int(d("thrust_pwm_max", C.ATT_THRUST_MAX).value)
        ok, why = R.check_poly(self.thrust_poly)
        if not ok:
            self.get_logger().error(f"{why} -- cmd_rate disabled")
            self.enable_rate_cmd = False
        # ---- IMU stream
        self.enable_imu_log = bool(d("enable_imu_log", True).value)
        self.imu_log_hz = float(d("imu_log_hz", 100.0).value)
        self.imu_log_attitude = bool(d("imu_log_attitude", False).value)

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
        self._kvar = None              # (varPX, varPY, varPZ) from the EKF
        self._est_ready = False        # reset done AND settled onto Vicon
        self._est_thread = None
        self._logs = []                # LogConfigs to stop on the way out
        self._att = None               # (roll, pitch) deg, stabilizer.*, if logged

        # ---- command state (written by executor, read by executor) -----------
        self._cmd = None               # ("position"|"hover"|"velocity_world"|"full_state"|"rate", payload)
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
        self._fm_written = False       # WE set stabModeRoll/Pitch=0 and owe a revert
        self._rate_active = False      # firmware-confirmed rate mode, refreshed per tick
        self._warn_t = {}              # throttled-warning timestamps

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
        # Always subscribed, so a policy publishing with the path disabled gets
        # a loud refusal instead of silence.
        self.create_subscription(RateThrust, f"{p}/cmd_rate",
                                 self._cmd_rate, 10, callback_group=cb)

        self.pub_status = self.create_publisher(Status, f"{p}/status", 10)
        self.pub_pose = self.create_publisher(PoseStamped, f"{p}/pose", 10)
        # Reliable, depth 10: matches both reliable and best-effort subscribers.
        self.pub_imu = self.create_publisher(Imu, f"{p}/imu", 10)
        # [stabilizer.thrust (thrust word), pm.vbat (V)], for calibrate_thrust.py
        self.pub_thrust = self.create_publisher(LogDataGeneric,
                                                f"{p}/log/thrust", 10)

        self.create_service(Takeoff, f"{p}/takeoff", self._srv_takeoff, callback_group=cb)
        self.create_service(Land, f"{p}/land", self._srv_land, callback_group=cb)
        self.create_service(Arm, f"{p}/arm", self._srv_arm, callback_group=cb)
        self.create_service(Stop, f"{p}/stop", self._srv_stop, callback_group=cb)
        self.create_service(Empty, f"{p}/emergency", self._srv_emergency, callback_group=cb)
        self.create_service(Empty, f"{p}/reset_emergency",
                            self._srv_reset_emergency, callback_group=cb)
        self.create_service(NotifySetpointsStop, f"{p}/notify_setpoints_stop",
                            self._srv_notify_stop, callback_group=cb)
        self.create_service(GoTo, f"{p}/go_to", self._srv_go_to, callback_group=cb)

        self.get_logger().info(f"core: {C.CORE_REVISION}")
        self.get_logger().info(
            f"uri={self.uri} mocap={self.vicon_topic} prefix={p} "
            f"control={self.control_hz:.0f}Hz extpose={self.extpose_hz:.0f}Hz "
            f"cmd_timeout={self.command_timeout*1000:.0f}ms"
            + ("  [NO-FLY]" if self.no_fly else ""))
        if self.enable_rate_cmd:
            self.get_logger().warn(
                f"cmd_rate ENABLED (rate_mode_setup={self.rate_mode_setup}) "
                f"caps rp={self.rate_rp_max_dps:.0f} yaw={self.rate_yaw_max_dps:.0f} "
                f"deg/s tilt={self.rate_tilt_limit:.0f} deg "
                f"thrust_poly={self.thrust_poly} scale={self.thrust_scale:.4f} "
                f"pwm=[{self.thrust_pwm_min},{self.thrust_pwm_max}]")

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
        self._configure_estimator()

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
            self._logs.append(lg)
        except Exception as exc:
            self.get_logger().warn(f"telemetry log failed to start: {exc}")

        # Kalman position variance. This is how we learn the estimator has
        # actually settled, instead of assuming it because time passed.
        lgv = LogConfig(name="kvar", period_in_ms=100)
        for v in ("kalman.varPX", "kalman.varPY", "kalman.varPZ"):
            lgv.add_variable(v, "float")
        lgv.data_received_cb.add_callback(self._on_kvar)
        try:
            self.cf.log.add_config(lgv)
            lgv.start()
            self._logs.append(lgv)
        except Exception as exc:
            self.get_logger().warn(f"kalman variance log failed to start: {exc}")

        if self.enable_imu_log:
            self._start_imu_log(LogConfig)

        # Controller thrust output, in the SAME units as the RPYT thrust word.
        # Its own block so an old firmware without the variable cannot take
        # the state block down with it. 8 bytes at 10 Hz.
        lgt = LogConfig(name="thrust", period_in_ms=100)
        lgt.add_variable("stabilizer.thrust", "float")
        lgt.add_variable("pm.vbat", "float")
        lgt.data_received_cb.add_callback(self._on_thrust)
        try:
            self.cf.log.add_config(lgt)
            lgt.start()
            self._logs.append(lgt)
        except Exception as exc:
            self.get_logger().warn(f"thrust log failed to start: {exc}")
        self._check_flightmode_at_connect()
        self._start_extpose()
        return True

    # -------------------------------------------------------------------------
    # IMU stream. Radio budget on 2M: extpose 50 Hz + setpoints 50 Hz uplink;
    # downlink rides the ACKs of those packets plus cflib's idle polling, and
    # is state 20 Hz + kvar 10 Hz + this block. One log packet carries at most
    # 26 data bytes, so gyro+acc (6 floats, 24 B) is ONE packet per sample;
    # attitude goes in its own block, off by default. Watch for the log
    # rate on the imu topic falling short of imu_log_hz: that is congestion.
    # -------------------------------------------------------------------------
    def _start_imu_log(self, LogConfig):
        period = R.log_period_ms(self.imu_log_hz)
        lg = LogConfig(name="imu", period_in_ms=period)
        for v in ("gyro.x", "gyro.y", "gyro.z", "acc.x", "acc.y", "acc.z"):
            lg.add_variable(v, "float")
        lg.data_received_cb.add_callback(self._on_imu)
        try:
            self.cf.log.add_config(lg)
            lg.start()
            self._logs.append(lg)
            self.get_logger().info(f"imu log at {1000.0 / period:.0f} Hz "
                                   f"-> {self.prefix}/imu")
        except Exception as exc:
            self.get_logger().warn(f"imu log failed to start: {exc}")
        if not self.imu_log_attitude:
            return
        lga = LogConfig(name="att", period_in_ms=max(period, 20))
        for v in ("stabilizer.roll", "stabilizer.pitch"):
            lga.add_variable(v, "float")
        lga.data_received_cb.add_callback(self._on_att)
        try:
            self.cf.log.add_config(lga)
            lga.start()
            self._logs.append(lga)
        except Exception as exc:
            self.get_logger().warn(f"attitude log failed to start: {exc}")

    def _on_imu(self, ts, data, cfg):
        # cflib RX thread. rclpy publish is thread-safe; publishing here
        # rather than from tick() keeps the full log rate (tick is 50 Hz).
        (gx, gy, gz), (ax, ay, az) = R.imu_si(
            data.get("gyro.x", 0.0), data.get("gyro.y", 0.0),
            data.get("gyro.z", 0.0), data.get("acc.x", 0.0),
            data.get("acc.y", 0.0), data.get("acc.z", 0.0))
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = f"{self.prefix}/body"      # FLU, see RATE_MODE.md
        m.orientation_covariance[0] = -1.0              # REP-145: not provided
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gx, gy, gz
        m.linear_acceleration.x = ax
        m.linear_acceleration.y = ay
        m.linear_acceleration.z = az
        self.pub_imu.publish(m)

    def _on_thrust(self, ts, data, cfg):
        m = LogDataGeneric()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.prefix
        m.timestamp = int(ts)
        m.values = [float(data.get("stabilizer.thrust", 0.0)),
                    float(data.get("pm.vbat", 0.0))]
        self.pub_thrust.publish(m)

    def _on_att(self, ts, data, cfg):
        # stabilizer.pitch is the firmware's LEGACY sign (inverted); only the
        # magnitude is used, by the tilt guard.
        with self._tlm_lock:
            self._att = (data.get("stabilizer.roll", 0.0),
                         data.get("stabilizer.pitch", 0.0))

    # -------------------------------------------------------------------------
    # Firmware flight mode (RPYT commander). See docs/RATE_MODE.md.
    # -------------------------------------------------------------------------
    def _read_flightmode(self):
        """Firmware-ACKNOWLEDGED values, from cflib's param cache. A write is
        only reflected here once the Crazyflie echoes it, so this is the
        firmware's word, not our intent. Unreadable -> None (= not rate)."""
        out = {}
        for n in R.FLIGHTMODE_CHECK:
            try:
                out[n] = int(float(self.cf.param.get_value(n)))
            except Exception:
                out[n] = None
        return out

    def _set_flightmode(self, mode):
        """Write stabModeRoll/Pitch. mode is R.RATE or R.ANGLE. Non-blocking:
        confirmation arrives asynchronously and is read by _read_flightmode."""
        for n in R.FLIGHTMODE_WRITE:
            try:
                self.cf.param.set_value(n, str(mode))
            except Exception as exc:
                self.get_logger().error(f"could not set {n}={mode}: {exc}")
        self._fm_written = (mode == R.RATE)

    def _revert_flightmode(self, why):
        """Put roll/pitch back to ANGLE if, and only if, this node changed them."""
        if not self._fm_written or self.cf is None:
            return
        self._set_flightmode(R.ANGLE)
        self._rate_active = False
        self.get_logger().info(f"flightmode.stabModeRoll/Pitch restored to 1 "
                               f"(angle) on {why}")

    def _check_flightmode_at_connect(self):
        fm = self._read_flightmode()
        if R.rate_mode_confirmed(fm) and not self.rate_mode_setup:
            # Firmware params live in RAM: an earlier session that lost its link
            # mid-rate-flight leaves them at 0 until a reboot. The standalone
            # teleop's ATTITUDE mode would then read its angles as RATES.
            self.get_logger().error(
                "firmware is in RATE mode on roll/pitch but rate_mode_setup is "
                "false -- restoring the default (1 = angle).")
            self._fm_written = True
            self._revert_flightmode("connect")
        else:
            self.get_logger().info(f"firmware flight mode: {fm}")

    def _on_state(self, ts, data, cfg):
        with self._tlm_lock:
            self._est = (data.get("stateEstimate.x", 0.0),
                         data.get("stateEstimate.y", 0.0),
                         data.get("stateEstimate.z", 0.0))
            self._vbat = data.get("pm.vbat", self._vbat)
            self._got_state = True

    def _on_kvar(self, ts, data, cfg):
        with self._tlm_lock:
            self._kvar = (data.get("kalman.varPX", 0.0),
                          data.get("kalman.varPY", 0.0),
                          data.get("kalman.varPZ", 0.0))

    def _configure_estimator(self):
        """Put the firmware into the only state where injected pose means anything.

        The complementary estimator cannot consume an absolute position. With it
        selected, stateEstimate.x/y sit at exactly 0 and z reports barometric
        altitude ABOVE SEA LEVEL -- on 2026-09-17 this node published
        (0, 0, 139.5) for a drone on the floor and every extpose packet we had
        sent was discarded. The position controller would then have flown on a
        state with no relationship to the room.

        The standalone teleop has always set these. The ROS split did not.
        """
        p = self.cf.param
        p.set_value("stabilizer.estimator", "2")    # Kalman. Mocap requires it.
        p.set_value("stabilizer.controller", "1")   # PID, not Mellinger: far
                                                    # more forgiving of a pose
                                                    # stream that drops frames.
        p.set_value("commander.enHighLevel", "0")
        # What the EKF assumes about our pose noise. Too small and one bad frame
        # yanks the estimate; too large and it ignores mocap and drifts on IMU.
        for name, val in (("locSrv.extPosStdDev", "0.01"),
                          ("locSrv.extQuatStdDev", "0.06")):
            try:
                p.set_value(name, val)
            except Exception as exc:
                self.get_logger().warn(f"could not set {name}: {exc}")
        time.sleep(0.3)
        self.get_logger().info("estimator=kalman controller=pid "
                               "extPosStdDev=0.01 extQuatStdDev=0.06")

    def _start_estimator_reset(self):
        """Reset the EKF once pose is really flowing, then wait for it to settle.

        Order matters. resetEstimation initialises the filter at the ORIGIN, so
        it has to happen while extpose is being injected -- reset it on a silent
        link and the filter converges confidently onto (0, 0, 0), and the first
        setpoint flies the drone there.

        Settled is not the same as correct: a filter can be beautifully
        converged onto the wrong place. So this checks the variance has stopped
        moving AND that the result agrees with Vicon before saying ready.
        """
        def run():
            self.get_logger().info("resetting Kalman estimator ...")
            try:
                self.cf.param.set_value("kalman.resetEstimation", "1")
                time.sleep(0.15)
                self.cf.param.set_value("kalman.resetEstimation", "0")
            except Exception as exc:
                self.get_logger().error(f"estimator reset failed: {exc}")
                return
            hist = deque(maxlen=C.KALMAN_CONVERGE_WINDOW)
            deadline = time.time() + 12.0
            while time.time() < deadline:
                time.sleep(0.1)
                with self._tlm_lock:
                    kv, est = self._kvar, self._est
                if kv is None:
                    continue
                hist.append(kv)
                if len(hist) < hist.maxlen:
                    continue
                spread = max(max(h[i] for h in hist) - min(h[i] for h in hist)
                             for i in range(3))
                if spread >= C.KALMAN_VAR_THRESHOLD:
                    continue
                pose = self.latest()
                if pose is None or est is None:
                    continue
                err = math.dist(est, (pose.x, pose.y, pose.z))
                if err > C.EKF_DIVERGE_M:
                    self.get_logger().error(
                        f"estimator settled {err * 100:.0f} cm from Vicon -- it "
                        "converged onto the wrong place. NOT ready to fly.")
                    return
                self._est_ready = True
                self.get_logger().info(
                    f"estimator converged (EKF-Vicon error {err * 100:.1f} cm)")
                return
            self.get_logger().error(
                "estimator did not converge in 12 s -- takeoff will be refused")

        self._est_thread = threading.Thread(target=run, daemon=True,
                                            name="estreset")
        self._est_thread.start()

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

    def _warn_throttled(self, key, msg, period=2.0):
        now = time.time()
        if now - self._warn_t.get(key, 0.0) >= period:
            self._warn_t[key] = now
            self.get_logger().warn(msg)

    def _cmd_rate(self, msg):
        """RateThrust: SI, body FLU. Refused unless every gate is open.

        A refused message is NOT stored and does NOT refresh the command
        watchdog, so a policy publishing into a closed gate ends up holding
        position, never flying on a stale setpoint.
        """
        ok, why = R.rate_gate(enable=self.enable_rate_cmd,
                              setup=self.rate_mode_setup,
                              confirmed=self._rate_active, state=self.state)
        if not ok:
            self._warn_throttled("rate_gate", why)
            return
        ok, why, sp, clamped = R.to_setpoint(
            msg.roll_rate, msg.pitch_rate, msg.yaw_rate, msg.thrust,
            msg.thrust_pwm, rp_max_dps=self.rate_rp_max_dps,
            yaw_max_dps=self.rate_yaw_max_dps, poly=self.thrust_poly,
            scale=self.thrust_scale, pwm_lo=self.thrust_pwm_min,
            pwm_hi=self.thrust_pwm_max)
        if not ok:
            self._warn_throttled("rate_bad", f"cmd_rate rejected: {why}")
            return
        if clamped:
            self._warn_throttled("rate_clamp", f"cmd_rate saturated -> {sp}", 5.0)
        self._accept("rate", sp)
        # A fresh rate command supersedes the post-takeoff / go_to hold point:
        # the policy, not the hold, is now flying. (LANDING and TAKEOFF are
        # refused by the gate above, so this cannot cancel a landing.)
        self._hold = None

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

        # Reset the EKF once, after extpose has actually been going out for a
        # second. Earlier than that and it resets onto an empty link.
        if (not self._est_ready and self._est_thread is None
                and self._sent_extpose > int(self.extpose_hz)):
            self._start_estimator_reset()

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

        if self.enable_rate_cmd and self.rate_mode_setup:
            self._rate_active = R.rate_mode_confirmed(self._read_flightmode())

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
        elif self._flying_rate() and p is not None:
            # Rate-only tilt guard: under rates nothing onboard levels the drone.
            with self._tlm_lock:
                att = self._att
            tilt = p.tilt_deg if p.quat_ok else None
            if R.tilt_exceeded(tilt, att, self.rate_tilt_limit):
                self.get_logger().error(
                    f"AUTO-LAND: tilt over {self.rate_tilt_limit:.0f} deg in "
                    f"rate mode (vicon {tilt}, onboard {att})")
                self.begin_land()

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
            if kind == "rate":
                if not self._rate_active:
                    # Firmware no longer confirms rate mode: NEVER send these
                    # numbers, which it would read as ANGLES. Hold instead.
                    self.get_logger().error(
                        "rate mode lost while flying rates -- holding position")
                    self._cmd = None
                    if p is not None:
                        self._hold = (p.x, p.y, p.z, p.yaw_deg)
                        self._send_position(*self._hold, p)
                    return
                self.cf.commander.send_setpoint(*a)
            elif kind == "position":
                self._send_position(a[0], a[1], a[2], a[3], p)
            elif kind == "hover":
                self.cf.commander.send_hover_setpoint(*a)
            elif kind == "velocity_world":
                self.cf.commander.send_velocity_world_setpoint(*a)
            elif kind == "full_state":
                self.cf.commander.send_full_state_setpoint(*a)
        except Exception as exc:
            self.get_logger().error(f"setpoint send failed: {exc}")

    def _flying_rate(self):
        """True while rate setpoints are what _transmit will actually send."""
        return (self.state == C.State.FLYING and self._hold is None
                and self._cmd is not None and self._cmd[0] == "rate")

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
            self._revert_flightmode("landing")
            return
        self._send_position(x, y, z, yaw, p)

    def begin_takeoff(self, height) -> bool:
        # ELIGIBILITY FIRST, before anything is armed, unlocked or written. A
        # second /takeoff while airborne used to reach the rate-setup branch
        # and put a zero-thrust RPYT packet on the radio mid-air. Only IDLE is
        # eligible; STOPPED (emergency) latches until /reset_emergency.
        if self.state != C.State.IDLE:
            why = ("emergency latched; call reset_emergency first"
                   if self.state == C.State.STOPPED else
                   f"state is {self.state}, not IDLE")
            self.get_logger().error(f"takeoff refused: {why}")
            return False
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
        if not (math.isfinite(p.z) and p.z <= self.takeoff_ground_z_max):
            self.get_logger().error(
                f"takeoff refused: measured z {p.z:.2f} m is not on the ground "
                f"(limit {self.takeoff_ground_z_max:.2f} m)")
            return False
        try:
            height = float(height)
        except (TypeError, ValueError):
            height = float("nan")
        if not (math.isfinite(height)
                and self.bounds.zmin <= height <= self.bounds.zmax):
            self.get_logger().error(
                f"takeoff refused: target height {height} m is not a finite "
                f"value inside [{self.bounds.zmin:.2f}, {self.bounds.zmax:.2f}] m")
            return False

        if not self._est_ready:
            self.get_logger().error(
                "takeoff refused: the Kalman estimator has not converged onto "
                "Vicon. Wait for the 'estimator converged' line.")
            return False

        # An ambiguous rigid body can be mid-flip on exactly the frame we latch
        # the climb's yaw target from. supervise() would catch a SUSTAINED
        # rejection later, but by then we would already be rotating toward a
        # heading 90 deg from the one the drone is actually holding.
        if not self.pos_only and self.quat_age() > C.QUAT_DEAD_S:
            self.get_logger().error(
                f"takeoff refused: no usable mocap orientation for "
                f"{self.quat_age() * 1000:.0f} ms ({self._yaw_rejects} yaw "
                f"rejects) -- the rigid body is ambiguous right now")
            return False

        ok, reasons = C.prearm_check(p, self.bounds, pos_only=self.pos_only)
        if not ok:
            self.get_logger().error("TAKEOFF REFUSED:")
            for why in reasons:
                self.get_logger().error(f"  - {why}")
            return False

        if not self._armed:
            _arm(self.cf, True)
            self._armed = True
        if self.enable_rate_cmd and self.rate_mode_setup:
            # On the ground, before any position setpoint. (1) The RPYT
            # decoder zeroes thrust until it has seen one zero-thrust packet
            # (thrustLocked, crtp_commander_rpyt.c); without this the first
            # rate setpoint mid-air would cut the motors. (2) Roll/pitch to
            # RATE. Only the RPYT path reads these, so the position-controlled
            # takeoff is unaffected; cmd_rate opens once the firmware echoes.
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            self._set_flightmode(R.RATE)
            self.get_logger().warn("flightmode.stabModeRoll/Pitch -> 0 (rate)")
        # Last ACCEPTED yaw, not this frame's. p.yaw_deg carries whatever
        # Tracker last said, including a solution we rejected.
        yaw0 = self._last_yaw if self._last_yaw is not None else p.yaw_deg
        self._takeoff_target = (p.x, p.y, float(height), yaw0)
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
        try:
            self._revert_flightmode("emergency")   # after the motors are cut
        except Exception:
            pass

    # =========================================================================
    # SERVICES
    # =========================================================================
    def _srv_takeoff(self, req, resp):
        h = float(req.height)
        # height == 0 is the "use the default" convention; NaN is NOT (NaN > 0
        # is False, so it used to silently become hover_z). Negative is a bug
        # in the caller: refuse rather than guess.
        if h == 0.0:
            h = self.hover_z
        self.begin_takeoff(h)
        return resp

    def _srv_land(self, req, resp):
        self.begin_land()
        return resp

    def _srv_arm(self, req, resp):
        if self.cf is not None:
            if req.arm and self.state != C.State.IDLE:
                self.get_logger().error(
                    f"arm refused: state is {self.state}, not IDLE")
                return resp
            _arm(self.cf, bool(req.arm))
            self._armed = bool(req.arm)
            self.get_logger().info(f"{'armed' if req.arm else 'disarmed'}")
        return resp

    def _srv_stop(self, req, resp):
        self.stop_motors()
        if self.state != C.State.STOPPED:     # an emergency latch survives /stop
            self.state = C.State.IDLE
        self._hold = None
        self._revert_flightmode("stop")
        return resp

    def _srv_emergency(self, req, resp):
        self.emergency("commanded via /emergency")
        return resp

    def _srv_reset_emergency(self, req, resp):
        """The ONLY way out of STOPPED. Motors are already off."""
        if self.state == C.State.STOPPED:
            self.state = C.State.IDLE
            self.get_logger().warn("emergency latch cleared by reset_emergency")
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
        for lg in self._logs:
            # An unstopped block keeps streaming from the firmware after we are
            # gone. The next connection then sees "no LogEntry to handle id=N"
            # and can lose the link outright, which cost us 20 minutes on
            # 2026-09-17.
            try:
                lg.stop()
            except Exception:
                pass
        if self.cf is not None:
            try:
                self.stop_motors()
            except Exception:
                pass
            try:
                self._revert_flightmode("shutdown")
                time.sleep(0.2)      # let the param write leave before close
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
