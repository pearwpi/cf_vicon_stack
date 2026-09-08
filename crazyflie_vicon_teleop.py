#!/usr/bin/env python3
"""Keyboard teleop for a Crazyflie 2.x whose state estimate comes from Vicon.

    radio://0/80/2M/E7E7E7E701   <-  /vicon/crazyflie1/crazyflie1 (PoseStamped)

BEFORE YOU ARM MOTORS
  1. A quadrotor has 4 controllable DOF, not 6: roll/pitch are slaved to
     horizontal acceleration. POSITION mode gives x/y/z/yaw; ATTITUDE mode is
     manual roll/pitch/yawrate/thrust with no position hold.
  2. Mocap fails HARD where a Flow Deck fails soft: one occluded marker for
     300 ms and the EKF position innovation explodes. Every guard in cf_core
     exists because of that asymmetry. Do not delete them.
  3. Frame contract (verify with --check): metres, ENU Z-up right-handed,
     rigid-body X out the FRONT of the drone, quaternion (x,y,z,w) same frame.
     A rigid-body X that is not the nose is the #1 cause of mocap CF crashes.
  4. First-flight order: --check (no radio) -> --no-fly (radio, no motors) ->
     props OFF for real -> props on, netted, --volume 1.5 1.5 0.8.

CONTROLS
  T takeoff   L land   SPACE E-STOP (motors off, it falls)   ESC quit
  H  hold at --hold-ft (default 2 ft), latching panic button; freezes x/y/yaw
     and drives altitude. Ignores movement keys until pressed again. L and
     SPACE always override it.
  M  toggle POSITION <-> ATTITUDE (only while landed)
  POSITION (setpoint moves in the drone's BODY frame):
     W/S fwd/back   A/D left/right   R/F up/down   Q/E yaw CCW/CW
     LSHIFT 2x      Z re-centre setpoint on the drone's current position
  ATTITUDE (raw, you are the only controller):
     W/S pitch   A/D roll   Q/E yaw rate   R/F thrust (decays toward hover)

WHAT THIS DOES NOT DO
  * No trajectory generation: setpoints step at 50 Hz with a rate limit. Fine
    for slow teleop, wrong for aggressive flight.
  * No multi-drone: one URI, one rigid body.
  * Yaw sign on the CRTP position setpoint has changed across firmware. If yaw
    goes the wrong way use --yaw-sign -1, verified with props OFF.
  * Latency stats need the Vicon PC and this machine clock-synced; a constant
    never-varying "latency" is clock skew, not delay.
  * The ~237 Hz Vicon stream is downsampled to 50 Hz for injection. Raise
    EXTPOSE_HZ only after measuring the radio's packet budget.

DEPENDENCIES: pip install cflib; pygame optional; ROS 2 rclpy+geometry_msgs sourced.

USAGE
  python3 crazyflie_vicon_teleop.py --check | --no-fly | --frame-test |
      --motor-test | --diagnose 30
  python3 crazyflie_vicon_teleop.py --uri radio://0/80/2M/E7E7E7E701 \
      --topic /vicon/crazyflie1/crazyflie1 --hover-z 0.6 \
      --bounds -1.2 1.2 -1.2 1.2 0.05 1.5
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

# ==============================================================================
# CONFIGURATION DEFAULTS
# ==============================================================================

# Bump this on every edit. Printed at startup so "which version am I actually
# running?" is never a guess -- a question that has already cost us one debug
# cycle chasing a bug that was fixed in a file sitting on another machine.
REVISION = "r12 2026-08-12  (flip-latch fix, orientation watchdog, upright gate)"

DEFAULT_URI = "radio://0/80/2M/E7E7E7E701"
DEFAULT_TOPIC = "/vicon/crazyflie1/crazyflie1"

# The control path is metric end to end, because the firmware, the EKF, and
# Vicon all are. Feet appear only at the CLI and in the status readout, converted
# exactly once. Mixing units inside a control loop is how you lose vehicles.
# --- the shared core ----------------------------------------------------------
# Every constant, guard and geofence below used to be defined here. They now
# live in cf_core.py, which is imported unchanged by the ROS 2 driver node as
# well. Two programs can spin these motors; a guard that exists in one and not
# the other is worse than no guard, because the two paths then behave
# differently under exactly the conditions where you can least afford surprise.
# cf_core.py is pure Python: no ROS, no cflib, no I/O.
import cf_core
from cf_core import (  # noqa: F401  (re-exported for tests and for --version)
    M_PER_FT, DEFAULT_HOLD_FT,
    CONTROL_HZ, EXTPOSE_HZ, MOCAP_EXPECTED_HZ,
    FLIP_GAP_CAP_S, PREARM_TILT_DEG, QUAT_DEAD_S,
    MOCAP_STALE_S, MOCAP_DEAD_S, EKF_DIVERGE_M, EKF_DIVERGE_HOLD_S,
    BATT_WARN_V, LEASH_M,
    GEOFENCE_MARGIN_M, GEOFENCE_KILL_M, DEFAULT_VOLUME,
    POS_SPEED_MS, POS_CLIMB_MS, YAW_RATE_DPS, TURBO,
    ATT_MAX_ANGLE_DEG, ATT_YAWRATE_DPS, ATT_THRUST_HOVER,
    ATT_THRUST_MIN, ATT_THRUST_MAX, ATT_THRUST_STEP,
    TAKEOFF_CLIMB_MS, LAND_DESCENT_MS, LAND_CUTOFF_Z,
    KALMAN_VAR_THRESHOLD, KALMAN_CONVERGE_WINDOW,
    Pose, Bounds, State, Mode,
    flip_check, prearm_check, apply_leash, clamp_alt,
    occluded_sentinel,
    supervise as core_supervise,
    wrap180,
)

# ==============================================================================
# ROS 2 / VICON SOURCE
# ==============================================================================
# The monitoring logic below (rate, latency, dropout accounting) is carried over
# from vicon_pose_subscriber.py. It is not decoration: `healthy()` is what the
# control loop uses to decide whether it is safe to keep flying.

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped
    ROS_AVAILABLE = True
    _ROS_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - environment dependent
    ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = exc
    Node = object  # type: ignore


def diagnose_ros_import_failure() -> str:
    """
    Turn an rclpy ImportError into the actual root cause.

    "Source your setup.bash" is the wrong advice most of the time. The common
    failures are distinguishable, and telling someone to re-source when their
    interpreter is the problem sends them down a dead end.
    """
    import glob as _glob
    err = str(_ROS_IMPORT_ERROR)
    mine = f"{sys.version_info.major}.{sys.version_info.minor}"
    distro = os.environ.get("ROS_DISTRO")
    lines = [f"rclpy/geometry_msgs unavailable: {err}", ""]

    if "_rclpy_pybind11" in err or "C extension" in err:
        required = None
        if distro:
            for c in sorted(_glob.glob(f"/opt/ros/{distro}/lib/python3.*")):
                tag = os.path.basename(c)
                if tag.startswith("python3."):
                    required = tag[len("python"):]
                    break
        lines += [
            "ROOT CAUSE: Python version mismatch, NOT a missing source.",
            f"  ROS 2 {distro or '?'} compiled its C extensions for Python "
            f"{required or '?'}.",
            f"  You are running Python {mine} ({sys.executable}).",
            "",
        ]
        if os.environ.get("CONDA_PREFIX") or "conda" in sys.executable:
            lines += [
                "Anaconda is active and is shadowing the system Python. Fix:",
                "  conda deactivate",
                "  conda config --set auto_activate_base false",
                "  exec bash -l",
                "  source /opt/ros/%s/setup.bash" % (distro or "humble"),
            ]
        else:
            lines += [
                "Use the system interpreter explicitly:",
                "  /usr/bin/python3 crazyflie_vicon_teleop.py",
            ]
    elif not distro:
        lines += [
            "ROOT CAUSE: ROS 2 is not sourced in this shell.",
            "  source /opt/ros/humble/setup.bash",
            "  source ~/your_ws/install/setup.bash",
        ]
    elif sys.prefix != sys.base_prefix:
        lines += [
            "ROOT CAUSE: you are in a virtualenv that cannot see system packages.",
            "  /usr/bin/python3 -m venv --system-site-packages ~/cfenv",
            "  source ~/cfenv/bin/activate && pip install cflib pygame",
        ]
    else:
        lines += ["Run 'python3 check_setup.py' for a full diagnosis."]

    lines += ["", "Full diagnosis: python3 check_setup.py"]
    return "\n".join(lines)




class ViconSource(Node):
    """
    Thread-safe latest-pose holder + link health monitor.

    QoS note: a BEST_EFFORT subscription matches both BEST_EFFORT and RELIABLE
    publishers, so this is the permissive choice. If you see zero messages, the
    problem is the topic name or the ROS_DOMAIN_ID, not the QoS.
    """

    def __init__(self, topic: str, expected_hz: float,
                 max_yaw_rate_dps: float = 720.0):
        super().__init__("crazyflie_vicon_teleop")

        self.topic = topic
        self._init_state(expected_hz, max_yaw_rate_dps)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # depth 1: we only ever want the newest pose
        )
        self.sub = self.create_subscription(
            PoseStamped, topic, self.position_callback, qos
        )
        self.start_wall = time.time()
        self.get_logger().info(
            f"Subscribed to {topic} (expecting ~{expected_hz:.0f} Hz)"
        )

    def _init_state(self, expected_hz: float, max_yaw_rate_dps: float = 720.0):
        """
        All non-ROS state, in one place.

        Separated from __init__ so tests can build a ViconSource without a ROS
        graph and still get EXACTLY the fields the real object has. The tests
        used to initialise these by hand, which meant every new field silently
        desynchronised the test double from the real class -- the same trap that
        hid the self.vicon.age() bug.
        """
        self.expected_period = 1.0 / expected_hz
        self.dropout_threshold = 2.0 * self.expected_period
        self._rate_calibrated = False

        window = max(int(expected_hz), 50)
        self.recv_times = deque(maxlen=window)
        self.latencies = deque(maxlen=window)

        self.total_messages = 0
        self.total_dropouts = 0
        self.max_dropout_s = 0.0
        self.last_recv_time = None

        self._lock = threading.Lock()
        self._latest: Pose | None = None
        self._unit_warned = False

        # Orientation sanity. A rotationally symmetric marker layout gives the
        # mocap solver two valid answers for the rigid body, and it will swap
        # between them -- a 180 deg yaw step in a single 4 ms frame, i.e.
        # ~43,000 deg/s, which no Crazyflie does. Feeding such a quaternion to
        # the EKF inverts the frame the position controller corrects in, so the
        # loop pushes the wrong way and the drone spirals outward.
        self.max_yaw_rate_dps = max_yaw_rate_dps
        self._last_yaw = None
        self._last_yaw_t = None
        # When the orientation was last USABLE. Distinct from pose age: the
        # position can be perfectly healthy while the quaternion is garbage,
        # and that combination injects position-only, leaving EKF yaw on the
        # gyro alone. supervise() lands on this.
        self._last_quat_ok_t = None
        self.total_yaw_rejects = 0
        self.total_occluded = 0
        self._occl_warned = False
        self.max_yaw_rate_seen = 0.0
        self._yaw_warned = False

    # -- callback -------------------------------------------------------------
    def position_callback(self, msg):
        now = time.time()

        p = Pose(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
            qx=msg.pose.orientation.x,
            qy=msg.pose.orientation.y,
            qz=msg.pose.orientation.z,
            qw=msg.pose.orientation.w,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            recv_time=now,
        )

        # An occluded segment arrives looking perfectly healthy: full rate,
        # fresh timestamp, unit-norm quaternion. Dropping it here is what lets
        # MOCAP_STALE_S and MOCAP_DEAD_S notice that the body is not tracked.
        if occluded_sentinel(p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw):
            self.total_occluded += 1
            if not self._occl_warned:
                self._occl_warned = True
                self.get_logger().error(
                    "OCCLUDED: position exactly (0,0,0) with quaternion "
                    "exactly (1,0,0,0) is the Vicon SDK's not-tracked "
                    "sentinel, not a measurement. Dropping these frames so "
                    "the mocap watchdog can act. Fix marker visibility.")
            return

        # Unit sanity: a Crazyflie flying in a room is never 100 m from origin.
        # If it is, someone is publishing millimetres and the drone will be
        # commanded to fly through a wall at full tilt.
        if not self._unit_warned and max(abs(p.x), abs(p.y), abs(p.z)) > 100.0:
            self._unit_warned = True
            self.get_logger().error(
                "Pose magnitude > 100 -- this stream looks like MILLIMETRES, "
                "not metres. DO NOT FLY. Fix the publisher or add a /1000 scale."
            )

        # Quaternion sanity: an unnormalised or zero quat silently corrupts the
        # EKF's attitude update.
        n = math.sqrt(p.qx**2 + p.qy**2 + p.qz**2 + p.qw**2)
        if n < 1e-6:
            self.get_logger().warn("Zero-norm quaternion received; frame dropped.")
            return
        if abs(n - 1.0) > 1e-3:
            p.qx, p.qy, p.qz, p.qw = p.qx / n, p.qy / n, p.qz / n, p.qw / n

        # ---- orientation sanity check ---------------------------------------
        yaw = p.yaw_deg
        # The decision itself lives in cf_core.flip_check so that this node and
        # the ROS 2 driver cannot disagree about what a solver flip is.
        rejected, d, rate = flip_check(yaw, self._last_yaw, self._last_yaw_t,
                                       now, self.max_yaw_rate_dps)
        if rate:
            self.max_yaw_rate_seen = max(self.max_yaw_rate_seen, rate)
        if rejected:
            gap = now - self._last_yaw_t
            # Physically impossible -- treat the orientation as invalid but
            # KEEP the position, which stays continuous through these flips.
            p.quat_ok = False
            self.total_yaw_rejects += 1
            if not self._yaw_warned:
                self._yaw_warned = True
                self.get_logger().error(
                    f"Yaw jumped {d:.0f} deg in {gap*1000:.1f} ms "
                    f"({rate:.0f} deg/s). The mocap rigid body is "
                    f"rotationally AMBIGUOUS -- Tracker is switching "
                    f"between two solutions. Quaternion rejected; "
                    f"position still used. Fix the marker layout.")
        if p.quat_ok:
            self._last_yaw = yaw
            self._last_yaw_t = now
            self._last_quat_ok_t = now

        # Latency. Accept a small negative value: when the publisher runs on
        # this same machine the stamp can land a hair ahead of our read, and
        # discarding those made the latency readout show a flat 0 ms.
        latency = now - p.stamp
        if -0.05 < latency < 1.0:
            self.latencies.append(max(0.0, latency))

        if self.last_recv_time is not None:
            gap = now - self.last_recv_time
            if gap > self.dropout_threshold:
                self.total_dropouts += 1
                self.max_dropout_s = max(self.max_dropout_s, gap)

        self.recv_times.append(now)
        self.last_recv_time = now
        self.total_messages += 1

        with self._lock:
            self._latest = p

    # -- accessors ------------------------------------------------------------
    def latest(self) -> Pose | None:
        with self._lock:
            return self._latest

    def age(self) -> float:
        """Seconds since the last pose. Large number if we never got one."""
        with self._lock:
            p = self._latest
        return 1e9 if p is None else (time.time() - p.recv_time)

    def quat_age(self) -> float:
        """Seconds since the orientation was last usable.

        Deliberately NOT the same as age(): a solver that has latched onto a
        mirrored solution keeps delivering fresh, low-latency, perfectly
        continuous POSITION while every quaternion fails the flip test. To the
        position watchdog that looks completely healthy.
        """
        # Read without the lock, matching how the callback writes it: a single
        # float rebind is atomic under the GIL, and being one frame stale is
        # meaningless against a 500 ms threshold. Taking the lock here would
        # imply a synchronisation that the write side does not honour.
        t = self._last_quat_ok_t
        return 1e9 if t is None else (time.time() - t)

    def rate_hz(self) -> float:
        if len(self.recv_times) < 2:
            return 0.0
        span = self.recv_times[-1] - self.recv_times[0]
        return (len(self.recv_times) - 1) / span if span > 0 else 0.0

    def calibrate_rate(self, measured_hz: float):
        """
        Set the dropout threshold from the rate we ACTUALLY observe.

        Otherwise --expected-hz is a footgun: leave it at 100 while Vicon runs
        at 240 and the threshold sits at 20 ms against a real 4.2 ms period, so
        a gap has to be nearly 5 frames long before it registers. Occlusions
        show up first as short gaps, which is exactly what you'd stop seeing.
        """
        if measured_hz < 1.0:
            return
        self.expected_period = 1.0 / measured_hz
        self.dropout_threshold = 2.0 * self.expected_period
        self._rate_calibrated = True
        self.get_logger().info(
            f"Dropout threshold auto-calibrated to "
            f"{self.dropout_threshold*1000:.1f} ms (measured {measured_hz:.1f} Hz)")

    def latency_ms(self) -> float | None:
        if not self.latencies:
            return None
        return statistics.median(self.latencies) * 1000.0

    def health_line(self) -> str:
        lat = self.latency_ms()
        lat_s = f"{lat:.0f}ms" if lat is not None else "n/a"
        yj = (f" YAWFLIP={self.total_yaw_rejects}"
              if self.total_yaw_rejects else "")
        return (
            f"mocap {self.rate_hz():5.1f}Hz age={self.age()*1000:5.1f}ms "
            f"lat={lat_s} drops={self.total_dropouts}{yj}"
        )


class ViconThread:
    """Runs rclpy's executor on a background thread so the control loop owns main."""

    def __init__(self, topic: str, expected_hz: float,
                 max_yaw_rate_dps: float = 720.0):
        if not ROS_AVAILABLE:
            raise RuntimeError(diagnose_ros_import_failure())
        rclpy.init(args=None)
        self.node = ViconSource(topic, expected_hz, max_yaw_rate_dps)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True, name="vicon")
        self._thread.start()

    def _spin(self):
        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # Delegation to the node. Without these, every `self.vicon.age()` in the
    # control loop is an AttributeError that only fires once you are already
    # connected and armed -- the worst possible time to discover it.
    def latest(self) -> "Pose | None":
        return self.node.latest()

    def age(self) -> float:
        return self.node.age()

    def quat_age(self) -> float:
        return self.node.quat_age()

    def rate_hz(self) -> float:
        return self.node.rate_hz()

    def latency_ms(self):
        return self.node.latency_ms()

    def health_line(self) -> str:
        return self.node.health_line()

    def wait_for_first_pose(self, timeout=10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.node.latest() is not None:
                return True
            time.sleep(0.05)
        return False

    def shutdown(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


# ==============================================================================
# KEYBOARD BACKENDS
# ==============================================================================

KEYS = ["w", "s", "a", "d", "q", "e", "r", "f",
        "t", "l", "m", "z", "h", "space", "shift", "esc"]


# Keyboard backends live in cf_keyboard.py, shared with the ROS 2 teleop node.
from cf_keyboard import (  # noqa: F401
    KeyboardBase, make_keyboard,
)
# ==============================================================================
# CRAZYFLIE TELEOP
# ==============================================================================

# Imported lazily so that --check (frame verification) works on a machine with
# ROS but no cflib, and so an import error surfaces as a clear message rather
# than a traceback before argparse has even run.
try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    CFLIB_AVAILABLE = True
    _CFLIB_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover
    CFLIB_AVAILABLE = False
    _CFLIB_IMPORT_ERROR = exc
    SyncCrazyflie = object  # type: ignore


def _arm(cf, do_arm: bool) -> bool:
    """
    Send an arming request across cflib/firmware generations.

    Newer cflib exposes cf.supervisor.send_arming_request and deprecates
    cf.platform.send_arming_request. Firmware below CRTP v12 has no supervisor
    subsystem and requires no arming step at all. Returns True if a request was
    actually delivered.
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


def prepare_radio() -> None:
    """
    Best-effort: release any kernel driver holding the Crazyradio.

    cflib calls set_configuration(1), which libusb rejects with EBUSY if any
    interface of the device is claimed. On Ubuntu the claimant is normally
    ModemManager, which probes every new USB device. Detaching first turns a
    hard failure into a successful connect.

    Deliberately scoped to Bitcraze vendor IDs only. Detaching kernel drivers
    from arbitrary USB hardware is not something a flight script should be
    doing, and a bug here would be worse than the problem it solves.
    """
    try:
        import usb.core
    except Exception:
        return
    for vid, pid in ((0x1915, 0x7777), (0x1915, 0x7778), (0x1915, 0x0101)):
        try:
            for dev in usb.core.find(find_all=True, idVendor=vid, idProduct=pid):
                try:
                    cfg = dev.get_active_configuration()
                except Exception:
                    continue
                for intf in cfg:
                    i = intf.bInterfaceNumber
                    try:
                        if dev.is_kernel_driver_active(i):
                            dev.detach_kernel_driver(i)
                            print(f"[cf] detached kernel driver from "
                                  f"{vid:04x}:{pid:04x} interface {i}")
                    except Exception:
                        pass  # no permission or not supported; cflib will report
        except Exception:
            continue


def diagnose_radio_failure(exc: Exception, uri: str) -> str:
    """Turn a cflib link error into the specific remedy, by errno."""
    msg = str(exc)
    L = ["", "=" * 70, f"RADIO CONNECT FAILED: {msg.splitlines()[0]}", "=" * 70]

    if "Errno 16" in msg or "Resource busy" in msg:
        L += [
            "errno 16 = EBUSY. libusb OPENED the device, then found its",
            "interfaces already claimed. This is NOT a permissions problem --",
            "permissions would be errno 13. Something else holds the radio.",
            "",
            "Remedies, in order:",
            "  1. Close cfclient and any other terminal running this script.",
            "     ps aux | grep -Ei 'cfclient|teleop' | grep -v grep",
            "  2. sudo systemctl stop ModemManager      # then retry",
            "  3. Physically unplug and replug the dongle.",
            "  4. python3 radio_doctor.py --fix",
        ]
    elif "Errno 13" in msg or "denied" in msg.lower():
        L += [
            "errno 13 = EACCES. This IS a udev/permissions problem.",
            "  sudo bash setup_crazyradio_ubuntu.sh",
            "  # then UNPLUG/REPLUG the dongle and start a NEW login session",
        ]
    elif "Errno 19" in msg or "No such device" in msg:
        L += ["errno 19 = ENODEV. The dongle vanished. Reseat it."]
    elif "Cannot find" in msg or "no driver" in msg.lower():
        L += [
            "No radio interface matched the URI.",
            f"  Is the dongle plugged into THIS machine?  lsusb | grep 1915",
            f"  URI in use: {uri}",
        ]
    else:
        L += ["Run: python3 radio_doctor.py"]

    L += ["=" * 70]
    return "\n".join(L)




class Telemetry:
    """Onboard log data. Everything here is a cross-check on the mocap loop."""

    def __init__(self):
        self.lock = threading.Lock()
        self.est_x = self.est_y = self.est_z = 0.0
        self.vbat = 0.0
        self.var_x = self.var_y = self.var_z = 1.0
        self.got_state = False
        self.got_var = False

    def on_state(self, ts, data, cfg):
        with self.lock:
            self.est_x = data.get("stateEstimate.x", self.est_x)
            self.est_y = data.get("stateEstimate.y", self.est_y)
            self.est_z = data.get("stateEstimate.z", self.est_z)
            self.vbat = data.get("pm.vbat", self.vbat)
            self.got_state = True

    def on_var(self, ts, data, cfg):
        with self.lock:
            self.var_x = data.get("kalman.varPX", self.var_x)
            self.var_y = data.get("kalman.varPY", self.var_y)
            self.var_z = data.get("kalman.varPZ", self.var_z)
            self.got_var = True

    def snapshot(self):
        with self.lock:
            return (self.est_x, self.est_y, self.est_z, self.vbat,
                    self.var_x, self.var_y, self.var_z)


class Teleop:
    def __init__(self, scf: SyncCrazyflie, vicon: ViconThread,
                 kb: KeyboardBase, args):
        self.scf = scf
        self.cf = scf.cf
        self.vicon = vicon
        self.kb = kb
        self.args = args
        self.bounds = Bounds(*args.bounds) if args.bounds else Bounds()
        self.yaw_sign = float(args.yaw_sign)
        self.tlm = Telemetry()

        self.state = State.IDLE
        self.mode = Mode.POSITION
        self.running = True
        self.abort_reason: str | None = None

        # Virtual setpoint
        self.sp_x = self.sp_y = 0.0
        self.sp_z = self.bounds.zmin
        self.sp_yaw_deg = 0.0

        # Attitude-mode state
        self.att_thrust = float(ATT_THRUST_HOVER)

        # HOLD state -- latching freeze at a fixed altitude
        self.hold_active = False
        self.hold_x = self.hold_y = 0.0
        self.hold_yaw_deg = 0.0
        self.hold_z = args.hold_z

        # Altitude that TAKEOFF climbs to. Normally --hover-z, but H from the
        # ground takes off to the hold altitude instead.
        self.target_z = args.hover_z

        self._diverge_since: float | None = None
        self._stale_since: float | None = None
        self.mocap_stale = False
        self._extpose_stop = threading.Event()
        self._extpose_thread: threading.Thread | None = None
        self._last_print = 0.0
        self._sent_extpose = 0

        # --diagnose recording
        self.samples: list[dict] = []
        self._diag_started = False
        self._diag_t0: float | None = None
        self._diag_step_n = 0
        self._diag_hold_x0: float | None = None

    # -- setup ----------------------------------------------------------------
    def configure(self):
        cf = self.cf
        print("[cf] configuring estimator for external pose...")

        # Kalman estimator. On mocap the complementary estimator is not an
        # option -- it has no way to consume an absolute position.
        cf.param.set_value("stabilizer.estimator", "2")
        # PID controller. Mellinger (2) tracks aggressive trajectories better
        # but is far less forgiving of a noisy/dropping pose stream.
        cf.param.set_value("stabilizer.controller", "1")

        # Measurement noise the EKF assumes for our injected pose. Too small and
        # a single bad frame yanks the estimate; too large and the EKF ignores
        # mocap and drifts on IMU alone.
        for name, val in (("locSrv.extPosStdDev", "0.01"),
                          ("locSrv.extQuatStdDev", "0.06")):
            try:
                cf.param.set_value(name, val)
            except Exception as exc:
                print(f"[cf] warn: could not set {name}: {exc}")

        cf.param.set_value("commander.enHighLevel", "0")
        time.sleep(0.3)

        # Telemetry blocks. Each block is capped at 26 bytes of payload.
        lg_state = LogConfig(name="state", period_in_ms=50)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
            lg_state.add_variable(v, "float")
        lg_state.add_variable("pm.vbat", "float")

        lg_var = LogConfig(name="kvar", period_in_ms=100)
        for v in ("kalman.varPX", "kalman.varPY", "kalman.varPZ"):
            lg_var.add_variable(v, "float")

        cf.log.add_config(lg_state)
        cf.log.add_config(lg_var)
        lg_state.data_received_cb.add_callback(self.tlm.on_state)
        lg_var.data_received_cb.add_callback(self.tlm.on_var)
        lg_state.start()
        lg_var.start()
        self._logs = (lg_state, lg_var)

    def start_extpose_feed(self):
        """Push mocap into the EKF on its own thread, independent of control."""
        hz = float(getattr(self.args, "extpose_hz", EXTPOSE_HZ))
        pos_only = bool(getattr(self.args, "pos_only", False))
        yaw_off = float(getattr(self.args, "yaw_offset", 0.0))
        if yaw_off:
            print(f"[cf] rotating injected quaternion by {yaw_off:+.1f} deg "
                  f"about Z (empirical yaw correction)")
        print(f"[cf] injecting {'extpos (position only)' if pos_only else 'extpose'}"
              f" at {hz:.0f} Hz")

        def loop():
            period = 1.0 / hz
            nxt = time.time()
            while not self._extpose_stop.is_set():
                p = self.vicon.node.latest()
                if p is not None and (time.time() - p.recv_time) < 0.25:
                    try:
                        if pos_only or not p.quat_ok:
                            # Diagnostic path: withhold the quaternion so the
                            # EKF uses gyro-integrated attitude instead. If the
                            # drone suddenly holds position well, the injected
                            # quaternion (i.e. the rigid-body axes) is the bug.
                            self.cf.extpos.send_extpos(p.x, p.y, p.z)
                        else:
                            qx, qy, qz, qw = p.qx, p.qy, p.qz, p.qw
                            if yaw_off:
                                # Compose a rotation about world Z:
                                # q_out = q_z(offset) * q_in
                                h = math.radians(yaw_off) / 2.0
                                cz, sz = math.cos(h), math.sin(h)
                                qx, qy, qz, qw = (cz * qx - sz * qy,
                                                  cz * qy + sz * qx,
                                                  cz * qz + sz * qw,
                                                  cz * qw - sz * qz)
                            self.cf.extpos.send_extpose(p.x, p.y, p.z,
                                                        qx, qy, qz, qw)
                        self._sent_extpose += 1
                    except Exception as exc:
                        print(f"[cf] extpose send failed: {exc}")
                nxt += period
                sleep = nxt - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    nxt = time.time()  # we fell behind; resync rather than spin
        self._extpose_thread = threading.Thread(target=loop, daemon=True,
                                                name="extpose")
        self._extpose_thread.start()

    def reset_estimator_and_wait(self, timeout=12.0) -> bool:
        print("[cf] resetting Kalman estimator...")
        self.cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.15)
        self.cf.param.set_value("kalman.resetEstimation", "0")

        hist_x, hist_y, hist_z = (deque(maxlen=KALMAN_CONVERGE_WINDOW),
                                  deque(maxlen=KALMAN_CONVERGE_WINDOW),
                                  deque(maxlen=KALMAN_CONVERGE_WINDOW))
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.1)
            ex, ey, ez, _, vx, vy, vz = self.tlm.snapshot()
            hist_x.append(vx); hist_y.append(vy); hist_z.append(vz)
            if len(hist_x) < KALMAN_CONVERGE_WINDOW:
                continue
            spread = max(max(hist_x) - min(hist_x),
                         max(hist_y) - min(hist_y),
                         max(hist_z) - min(hist_z))
            if spread >= KALMAN_VAR_THRESHOLD:
                continue

            # Variance settling is necessary but not sufficient: the EKF can
            # converge confidently onto the WRONG position if extpose is not
            # actually arriving. Cross-check against Vicon truth.
            p = self.vicon.node.latest()
            if p is None:
                continue
            err = math.dist((ex, ey, ez), (p.x, p.y, p.z))
            if err < 0.10:
                print(f"[cf] estimator converged (EKF-Vicon error {err*100:.1f} cm)")
                return True
            print(f"[cf] variance settled but EKF is {err*100:.0f} cm from Vicon "
                  f"-- still waiting (check extpose is arriving)")
        print("[cf] ESTIMATOR DID NOT CONVERGE. Not arming.")
        return False

    def arm(self):
        """CF firmware >= 2023.11 requires an explicit arming request."""
        if self.args.no_fly:
            print("[cf] --no-fly: NOT arming. Motors stay disabled.")
            return
        # cflib moved arming from platform -> supervisor. Firmware older than
        # CRTP v12 has no supervisor at all and needs no arming step, so a
        # failure here is expected and harmless on those builds.
        if _arm(self.cf, True):
            print("[cf] arming request sent")
        else:
            print("[cf] no arming stage on this firmware (pre-CRTP-12) -- ok")
        # Unlock the thrust safety latch before any position setpoint.
        self.cf.commander.send_setpoint(0, 0, 0, 0)

    # -- safety ---------------------------------------------------------------
    def supervise(self):
        """One tick of the safety supervisor -> None | ("kill"|"land", reason).

        Delegates to cf_core.supervise, which the ROS driver also calls, so the
        two front ends cannot disagree about when to land or cut motors.
        """
        p = self.vicon.node.latest()
        ex, ey, ez, vbat, _, _, _ = self.tlm.snapshot()
        r = core_supervise(
            now=time.time(),
            flying=self.state in State.FLYING_STATES,
            pose_age=self.vicon.age(), quat_age=self.vicon.quat_age(),
            pos_only=self.args.pos_only,
            pos=(p.x, p.y, p.z) if p is not None else None,
            ekf=(ex, ey, ez) if self.tlm.got_state else None,
            got_state=self.tlm.got_state, vbat=vbat, bounds=self.bounds,
            stale_since=self._stale_since, diverge_since=self._diverge_since,
            yaw_rejects=self.vicon.node.total_yaw_rejects)
        self._stale_since = r.stale_since
        self._diverge_since = r.diverge_since
        self.mocap_stale = r.mocap_stale
        if r.became_stale and self.state in State.FLYING_STATES:
            print(f"\n[safety] mocap stale "
                  f"({self.vicon.age()*1000:.0f} ms) -- setpoint frozen")
        return (r.action, r.reason) if r.action else None

    # -- setpoint integration -------------------------------------------------
    def update_position_setpoint(self, dt: float):
        # Refuse operator input while the pose is stale -- see supervise().
        if self.mocap_stale:
            return
        if self.hold_active:
            self._drive_to_hold(dt)
            return
        kb = self.kb
        scale = TURBO if kb.held("shift") else 1.0
        v = POS_SPEED_MS * scale * dt
        vz = POS_CLIMB_MS * scale * dt
        vyaw = YAW_RATE_DPS * scale * dt

        fwd = (1.0 if kb.held("w") else 0.0) - (1.0 if kb.held("s") else 0.0)
        left = (1.0 if kb.held("a") else 0.0) - (1.0 if kb.held("d") else 0.0)
        up = (1.0 if kb.held("r") else 0.0) - (1.0 if kb.held("f") else 0.0)
        yaw = (1.0 if kb.held("q") else 0.0) - (1.0 if kb.held("e") else 0.0)

        # Body-frame -> world. ENU: yaw measured from +X toward +Y.
        c, s = math.cos(math.radians(self.sp_yaw_deg)), math.sin(math.radians(self.sp_yaw_deg))
        self.sp_x += v * (fwd * c - left * s)
        self.sp_y += v * (fwd * s + left * c)
        self.sp_z += vz * up
        self.sp_yaw_deg = (self.sp_yaw_deg + vyaw * yaw + 180.0) % 360.0 - 180.0

        self._apply_limits()

    def _drive_to_hold(self, dt: float):
        """
        HOLD mode: converge on the frozen pose captured when H was pressed.

        Rate-limited rather than a step change, so engaging hold from a moving
        drone does not command an instantaneous position jump the controller
        would answer with a hard pitch. Operator translation and yaw input is
        ignored entirely while held -- that is the point of the button.
        """
        step = POS_SPEED_MS * dt
        stepz = POS_CLIMB_MS * dt
        self.sp_x += max(-step, min(step, self.hold_x - self.sp_x))
        self.sp_y += max(-step, min(step, self.hold_y - self.sp_y))
        self.sp_z += max(-stepz, min(stepz, self.hold_z - self.sp_z))
        self.sp_yaw_deg = self.hold_yaw_deg
        self._apply_limits()

    def _apply_limits(self):
        """Geofence clamp, then leash. Applied to every position setpoint."""
        self.sp_x, self.sp_y, self.sp_z = self.bounds.clamp(
            self.sp_x, self.sp_y, self.sp_z)
        p = self.vicon.node.latest()
        if p is not None:
            self.sp_x, self.sp_y, self.sp_z = apply_leash(
                self.sp_x, self.sp_y, self.sp_z, p.x, p.y, p.z)

    def engage_hold(self, alt_m: float | None = None):
        """Freeze horizontal position and yaw; command a fixed altitude."""
        p = self.vicon.node.latest()
        if p is None:
            print("\n[hold] refused: no mocap pose")
            return False
        self.hold_x, self.hold_y = p.x, p.y
        self.hold_yaw_deg = p.yaw_deg
        self.hold_z = self.clamp_alt(alt_m if alt_m is not None
                                     else self.args.hold_z)
        self.hold_active = True
        print(f"\n[hold] ENGAGED at ({self.hold_x:+.2f}, {self.hold_y:+.2f}) "
              f"alt {self.hold_z:.2f} m / {self.hold_z / M_PER_FT:.1f} ft "
              f"-- movement keys ignored, press H to release")
        return True

    def release_hold(self):
        if not self.hold_active:
            return
        self.hold_active = False
        # Hand control back from where the drone actually is, not from a stale
        # pre-hold setpoint, so the first keypress does not jump it.
        self.recenter_setpoint()
        self.sp_z = self.hold_z
        print("\n[hold] released -- manual control restored")

    def clamp_alt(self, z: float) -> float:
        """Keep a requested altitude inside the flight volume."""
        c, was_clamped = clamp_alt(z, self.bounds.zmin, self.bounds.zmax)
        if was_clamped:
            print(f"\n[hold] {z:.2f} m is outside the flight volume "
                  f"[{self.bounds.zmin:.2f}, {self.bounds.zmax:.2f}] -- "
                  f"clamped to {c:.2f} m ({c / M_PER_FT:.1f} ft)")
        return c

    def _send_pos(self):
        """Single choke point for position setpoints.

        yaw_sign exists because the CRTP position-setpoint sign convention has
        differed across firmware generations. Verify WITH PROPS OFF: press Q
        and confirm the motor mix implies CCW yaw viewed from above. If it is
        wrong in flight use --yaw-sign -1; do NOT compensate by swapping your
        Q/E keys, which leaves the sign error in place for everything else.
        """
        self.cf.commander.send_position_setpoint(
            self.sp_x, self.sp_y, self.sp_z, self.yaw_sign * self.sp_yaw_deg)

    def recenter_setpoint(self):
        p = self.vicon.node.latest()
        if p is None:
            return
        self.sp_x, self.sp_y = p.x, p.y
        self.sp_z = max(p.z, self.bounds.zmin)
        self.sp_yaw_deg = p.yaw_deg

    def send_attitude(self, dt: float):
        kb = self.kb
        pitch = ATT_MAX_ANGLE_DEG * ((1.0 if kb.held("w") else 0.0) -
                                     (1.0 if kb.held("s") else 0.0))
        roll = ATT_MAX_ANGLE_DEG * ((1.0 if kb.held("d") else 0.0) -
                                    (1.0 if kb.held("a") else 0.0))
        yawrate = ATT_YAWRATE_DPS * ((1.0 if kb.held("e") else 0.0) -
                                     (1.0 if kb.held("q") else 0.0))
        up = (1.0 if kb.held("r") else 0.0) - (1.0 if kb.held("f") else 0.0)
        if up != 0.0:
            self.att_thrust += ATT_THRUST_STEP * up * dt
        else:
            # Bleed back toward hover so a missed keypress does not leave the
            # drone climbing at full thrust.
            self.att_thrust += (ATT_THRUST_HOVER - self.att_thrust) * min(1.0, dt * 1.5)
        self.att_thrust = min(max(self.att_thrust, ATT_THRUST_MIN), ATT_THRUST_MAX)
        self.cf.commander.send_setpoint(roll, pitch, yawrate, int(self.att_thrust))

    # -- state machine --------------------------------------------------------
    def handle_keys(self):
        kb = self.kb
        if kb.pressed("space"):
            self.emergency_stop("operator E-STOP")
            return
        if kb.pressed("esc"):
            self.running = False
            if self.state in (State.TAKEOFF, State.FLYING):
                self.state = State.LANDING
            return
        if kb.pressed("m") and self.state == State.IDLE:
            self.mode = Mode.ATTITUDE if self.mode == Mode.POSITION else Mode.POSITION
            print(f"\n[mode] {self.mode}")
        if kb.pressed("z") and self.mode == Mode.POSITION and not self.hold_active:
            self.recenter_setpoint()

        # H -- panic hold. Latching: freeze in place at the hold altitude.
        if kb.pressed("h") and self.mode == Mode.POSITION:
            if self.hold_active:
                self.release_hold()
            elif self.state in (State.TAKEOFF, State.FLYING):
                self.engage_hold()
            elif self.state == State.IDLE:
                # From the ground, H means "take off to the hold altitude and
                # stay there" -- the single-button hover.
                if self.begin_takeoff(target_z=self.args.hold_z):
                    self.engage_hold()

        if kb.pressed("t") and self.state == State.IDLE:
            self.begin_takeoff()
        if kb.pressed("l") and self.state in (State.TAKEOFF, State.FLYING):
            self.hold_active = False   # landing overrides hold
            print("\n[state] LANDING")
            self.state = State.LANDING

    def pre_arm_ok(self, p) -> bool:
        """Refuse takeoff if the drone is outside the geofence or not upright.

        Delegates to cf_core.prearm_check, then prints a suggested volume.
        """
        b = self.bounds
        inside_xy = (b.xmin <= p.x <= b.xmax) and (b.ymin <= p.y <= b.ymax)
        if inside_xy and p.z <= b.zmax:
            return True

        sug = Bounds.centered_on(p.x, p.y, *DEFAULT_VOLUME)
        print("\n" + "=" * 70)
        print("TAKEOFF REFUSED -- the drone is outside the flight volume.")
        print("=" * 70)
        print(f"  drone is at   ({p.x:+.2f}, {p.y:+.2f}, {p.z:+.2f}) m")
        print(f"  volume is     {b.describe()}")
        if not inside_xy:
            print("  -> horizontal position is out of bounds")
        if p.z > b.zmax:
            print(f"  -> already above the ceiling ({b.zmax:.2f} m)")
        print("")
        print("  Your Vicon origin is probably not centred on the flight area.")
        print("  Either re-run with a volume centred where the drone actually is:")
        print(f"      {sug.as_cli()}")
        print("  or size one automatically around it:")
        print("      --volume 2.0 2.0 1.2")
        print("=" * 70)
        return False

    def begin_takeoff(self, target_z: float | None = None) -> bool:
        if self.args.no_fly:
            print("\n[state] --no-fly is set; takeoff refused.")
            return False
        p = self.vicon.node.latest()
        if p is None or self.vicon.age() > MOCAP_STALE_S:
            print("\n[state] takeoff refused: mocap not fresh.")
            return False
        if not self.pre_arm_ok(p):
            return False
        if self.mode == Mode.ATTITUDE:
            self.state = State.FLYING
            self.att_thrust = float(ATT_THRUST_HOVER)
            print("\n[state] ATTITUDE mode live -- you are the controller.")
            return True
        self.target_z = self.clamp_alt(
            self.args.hover_z if target_z is None else target_z)
        self.recenter_setpoint()
        self.sp_z = max(p.z, self.bounds.zmin)
        self.state = State.TAKEOFF
        print(f"\n[state] TAKEOFF to {self.target_z:.2f} m "
              f"/ {self.target_z / M_PER_FT:.1f} ft")
        return True

    def emergency_stop(self, reason: str):
        self.abort_reason = reason
        self.hold_active = False
        self.state = State.STOPPED
        try:
            self.cf.commander.send_stop_setpoint()
            self.cf.commander.send_notify_setpoint_stop()
        except Exception:
            pass
        try:
            _arm(self.cf, False)
        except Exception:
            pass
        print(f"\n\n*** EMERGENCY STOP: {reason} ***")
        # The motor cut above is the only thing that matters here. Everything
        # below is diagnostics, so it is wrapped: an AttributeError while
        # formatting a log line must never propagate out of the one function
        # whose job is to make the vehicle safe.
        try:
            self._print_post_mortem()
        except Exception as exc:
            print(f"(post-mortem unavailable: {exc})\n")

    def _print_post_mortem(self):
        """Evidence for the next run, so diagnosis doesn't start from a guess."""
        p = self.vicon.node.latest()
        print("--- post-mortem ---")
        if p is not None:
            print(f"  position at abort : ({p.x:+.2f},{p.y:+.2f},{p.z:+.2f}) m")
        print(f"  setpoint at abort : ({self.sp_x:+.2f},{self.sp_y:+.2f},"
              f"{self.sp_z:+.2f}) m")
        print(f"  flight volume     : {self.bounds.describe()}")
        if p is not None:
            # z deliberately has NO lower bound here, matching
            # Bounds.outside(): zmin floors the COMMANDED altitude, not the
            # measured one, so a drone resting on the floor is inside the
            # volume. Checking z against zmin made every landed post-mortem
            # report a violation that the supervisor does not agree exists.
            for name, lo, hi, val in (("x", self.bounds.xmin, self.bounds.xmax, p.x),
                                      ("y", self.bounds.ymin, self.bounds.ymax, p.y),
                                      ("z", None, self.bounds.zmax, p.z)):
                below = lo is not None and val < lo
                if below or val > hi:
                    over = (lo - val) if below else (val - hi)
                    print(f"    -> {name} out of bounds by {over:+.2f} m")
        print(f"  mocap             : {self.vicon.node.rate_hz():.1f} Hz, "
              f"age {self.vicon.age()*1000:.0f} ms, "
              f"{getattr(self.vicon.node, 'total_dropouts', -1)} dropouts, "
              f"max gap "
              f"{getattr(self.vicon.node, 'max_dropout_s', 0.0)*1000:.0f} ms")
        ex, ey, ez, vbat, _, _, _ = self.tlm.snapshot()
        print(f"  onboard estimate  : ({ex:+.2f},{ey:+.2f},{ez:+.2f}) m, "
              f"battery {vbat:.2f} V")
        print("-------------------\n")

    # -- main loop ------------------------------------------------------------
    def run(self):
        period = 1.0 / CONTROL_HZ
        last = time.time()
        print(f"\nReady. T=takeoff  H=hold@{self.args.hold_z / M_PER_FT:.1f}ft  "
              f"L=land  SPACE=E-STOP  ESC=quit\n")

        while self.running or self.state == State.LANDING:
            now = time.time()
            dt = min(now - last, 0.1)
            last = now

            self.kb.poll()
            if getattr(self.kb, "window_closed", False):
                self.running = False
                if self.state in (State.TAKEOFF, State.FLYING):
                    self.state = State.LANDING

            self.handle_keys()
            if self.state == State.STOPPED:
                break
            self.run_diagnose_sequence(now)

            verdict = self.supervise()
            if verdict:
                action, reason = verdict
                if action == "kill":
                    self.emergency_stop(reason)
                    break
                if self.state != State.LANDING:
                    print(f"\n[safety] {reason} -> auto-land")
                    self.hold_active = False
                    self.state = State.LANDING

            p = self.vicon.node.latest()

            if self.state == State.IDLE:
                if self.mode == Mode.POSITION:
                    self.recenter_setpoint()
                # Keep the link warm without arming: zero thrust setpoint.
                self.cf.commander.send_setpoint(0, 0, 0, 0)

            elif self.state == State.TAKEOFF:
                self.sp_z = min(self.sp_z + TAKEOFF_CLIMB_MS * dt, self.target_z)
                self._send_pos()
                if p is not None and p.z > self.target_z - 0.05:
                    self.state = State.FLYING
                    print("\n[state] FLYING")

            elif self.state == State.FLYING:
                if self.mode == Mode.POSITION:
                    self.update_position_setpoint(dt)
                    self._send_pos()
                else:
                    self.send_attitude(dt)

            elif self.state == State.LANDING:
                if self.mode == Mode.ATTITUDE:
                    self.att_thrust = max(ATT_THRUST_MIN,
                                          self.att_thrust - 9000 * dt)
                    self.cf.commander.send_setpoint(0, 0, 0, int(self.att_thrust))
                    if self.att_thrust <= ATT_THRUST_MIN + 1:
                        self.finish_landing()
                else:
                    self.sp_z = max(self.sp_z - LAND_DESCENT_MS * dt, 0.0)
                    self._send_pos()
                    landed = (p is not None and p.z < LAND_CUTOFF_Z)
                    if landed or self.sp_z <= 0.01:
                        self.finish_landing()

            self.print_status()

            sleep = period - (time.time() - now)
            if sleep > 0:
                time.sleep(sleep)

    def run_diagnose_sequence(self, now: float):
        """
        Automated hover-quality capture: take off, hold, record, land.

        Deliberately uses the ordinary hold path and the ordinary safety
        supervisor -- a diagnostic that bypasses the safety layer would measure
        a configuration you never actually fly.
        """
        secs = getattr(self.args, "diagnose", 0.0)
        if not secs:
            return
        if self.state == State.IDLE and not self._diag_started:
            self._diag_started = True
            print(f"\n[diagnose] taking off to hold and recording {secs:.0f}s "
                  f"of hover...")
            if self.begin_takeoff(target_z=self.args.hold_z):
                self.engage_hold()
            else:
                self.running = False
            return
        if self.state == State.FLYING and self.hold_active:
            if self._diag_t0 is None:
                self._diag_t0 = now
                self._diag_step_n = 0
                self._diag_hold_x0 = self.hold_x
                print("[diagnose] settled -- recording")
            self._maybe_step(now - self._diag_t0)
            self.record_sample(now)
            if now - self._diag_t0 >= secs:
                print(f"\n[diagnose] captured {len(self.samples)} samples; landing")
                self.hold_active = False
                self.state = State.LANDING
                self.running = False

    def _maybe_step(self, elapsed: float):
        """Move the hold point out at 1/3 of the capture and back at 2/3.

        WHY THE HOLD POINT AND NOT sp_x. Moving hold_x keeps the setpoint on
        the ordinary _drive_to_hold path, so the geofence clamp, the leash and
        the supervisor all still apply. A diagnostic that bypasses the safety
        layer measures a configuration you never actually fly.

        The move is therefore a rate-limited ramp at POS_SPEED_MS, not an
        instantaneous jump. That is correct for real hardware and it does not
        harm the measurement: dead time is read from the ONSET, the delay
        between the setpoint starting to move and the position starting to
        move. Time-to-settle would mix in controller dynamics; onset does not.
        """
        d = float(getattr(self.args, "step", 0.0) or 0.0)
        if d == 0.0 or self._diag_hold_x0 is None:
            return
        secs = float(getattr(self.args, "diagnose", 0.0) or 0.0)
        if secs <= 0.0:
            return

        if self._diag_step_n == 0 and elapsed >= secs / 3.0:
            target = self._diag_hold_x0 + d
            # Refuse rather than let the geofence silently clamp it: a clipped
            # step is a step of unknown size, which is worthless for timing.
            if not (self.bounds.xmin <= target <= self.bounds.xmax):
                print(f"\n[diagnose] STEP SKIPPED -- target x={target:+.2f} m is "
                      f"outside {self.bounds.describe()}. Use a smaller --step "
                      f"or a wider --volume.")
                self._diag_step_n = 2
                return
            self._diag_step_n = 1
            self.hold_x = target
            print(f"\n[diagnose] STEP +{d:.2f} m along world +X  "
                  f"(hold_x {self._diag_hold_x0:+.2f} -> {self.hold_x:+.2f})")
        elif self._diag_step_n == 1 and elapsed >= 2.0 * secs / 3.0:
            self._diag_step_n = 2
            self.hold_x = self._diag_hold_x0
            print(f"\n[diagnose] STEP back to {self.hold_x:+.2f} m")

    def record_sample(self, now: float):
        p = self.vicon.node.latest()
        if p is None:
            return
        ex, ey, ez, _, _, _, _ = self.tlm.snapshot()
        lat = self.vicon.node.latency_ms()
        self.samples.append({
            "t": now, "sp_x": self.sp_x, "sp_y": self.sp_y, "sp_z": self.sp_z,
            "x": p.x, "y": p.y, "z": p.z,
            "ex": ex, "ey": ey, "ez": ez,
            "yaw_deg": p.yaw_deg, "latency_ms": lat if lat else 0.0,
        })

    def write_samples(self, path="hover_log.csv") -> bool:
        if len(self.samples) < 20:
            return False
        import csv
        cols = ["t", "sp_x", "sp_y", "sp_z", "x", "y", "z",
                "ex", "ey", "ez", "yaw_deg", "latency_ms"]
        t0 = self.samples[0]["t"]
        with open(path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            for s in self.samples:
                row = dict(s)
                row["t"] = s["t"] - t0     # relative time; absolute is useless here
                w.writerow({k: row[k] for k in cols})
        print(f"[diagnose] wrote {len(self.samples)} samples to {path}")
        return True

    def finish_landing(self):
        try:
            self.cf.commander.send_stop_setpoint()
            self.cf.commander.send_notify_setpoint_stop()
        except Exception:
            pass
        self.state = State.IDLE if self.running else State.STOPPED
        print("\n[state] landed")

    def print_status(self):
        now = time.time()
        if now - self._last_print < 0.2:
            return
        self._last_print = now
        p = self.vicon.node.latest()
        ex, ey, ez, vbat, _, _, _ = self.tlm.snapshot()
        if p is None:
            pos = "vicon --.-- --.-- --.--"
            err = float("nan")
        else:
            pos = f"vicon {p.x:+.2f} {p.y:+.2f} {p.z:+.2f} yaw {p.yaw_deg:+6.1f}"
            err = math.dist((ex, ey, ez), (p.x, p.y, p.z))
        batt = f"{vbat:.2f}V" + ("!" if vbat and vbat < BATT_WARN_V else " ")
        sp = (f"sp {self.sp_x:+.2f} {self.sp_y:+.2f} "
              f"{self.sp_z:.2f}m/{self.sp_z / M_PER_FT:.1f}ft")
        tag = "HOLD" if self.hold_active else self.mode[:4]
        line = (f"\r[{self.state:8s}|{tag:4s}] {pos} | {sp} | "
                f"ekf_err {err*100:4.1f}cm | {batt} | {self.vicon.node.health_line()}   ")
        sys.stdout.write(line)
        sys.stdout.flush()

    def cleanup(self):
        self._extpose_stop.set()
        if self._extpose_thread:
            self._extpose_thread.join(timeout=1.0)
        try:
            self.cf.commander.send_stop_setpoint()
            self.cf.commander.send_notify_setpoint_stop()
        except Exception:
            pass
        for lg in getattr(self, "_logs", ()):
            try:
                lg.stop()
            except Exception:
                pass


# ==============================================================================
# FRAME VERIFICATION MODE (no radio, no motors)
# ==============================================================================

def run_check(args):
    print(__doc__.split("CONTROLS")[0])
    print("=" * 78)
    print("FRAME CHECK -- no radio connection, no motors. Ctrl-C to exit.")
    print("Pick the drone up and move it. Confirm ALL of the following:")
    print("  * Move it AWAY from you along its own nose  -> X increases")
    print("  * Move it to ITS left                       -> Y increases")
    print("  * Lift it                                   -> Z increases")
    print("  * Rotate CCW seen from above                -> yaw increases")
    print("  * On the floor at your chosen origin        -> ~0, 0, ~0")
    print("If any of these is wrong, fix the Vicon rigid-body definition or add")
    print("a transform. DO NOT compensate for it in the flight code.")
    print("=" * 78)
    vic = ViconThread(args.topic, args.expected_hz,
                      getattr(args, "max_yaw_rate", 720.0))
    if not vic.wait_for_first_pose(10.0):
        print("\nNO POSE RECEIVED in 10 s. Check: topic name, ROS_DOMAIN_ID, "
              "and whether the Vicon object is labelled/tracked.")
        vic.shutdown()
        return 1
    try:
        while True:
            p = vic.node.latest()
            sys.stdout.write(
                f"\rX {p.x:+8.3f}  Y {p.y:+8.3f}  Z {p.z:+8.3f} m   "
                f"yaw {p.yaw_deg:+7.2f} deg   |  {vic.node.health_line()}   ")
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n")
    finally:
        vic.shutdown()
    return 0



# ==============================================================================
# BENCH FRAME TEST (no radio, no motors, props can stay off)
# ==============================================================================

def run_frame_test(args):
    """
    Guided rigid-body validation. Measures, without flying:
      * yaw solver flips and stationary yaw jitter
      * the CONSTANT yaw offset between the object's X axis and the drone's nose
      * frame handedness (mirrored / swapped axes)
      * Z sign
      * whether the object's origin sits at the drone's centre of rotation

    The yaw-offset measurement is convention-free: it compares the direction the
    drone ACTUALLY travelled against the yaw it reported. No assumption about
    quaternion order, handedness or firmware sign is involved -- which matters,
    because those assumptions are exactly what you are trying to test.
    """
    try:
        import frame_check as FC
    except Exception as exc:
        print(f"frame_check.py not importable ({exc}). "
              f"Put it next to this script.")
        return 1

    vic = ViconThread(args.topic, args.expected_hz,
                      getattr(args, "max_yaw_rate", 720.0))
    if not vic.wait_for_first_pose(10.0):
        print("NO POSE RECEIVED in 10 s. Check the topic and that the rigid "
              "body is labelled and tracked in Tracker.")
        vic.shutdown()
        return 1

    def avg_pose(secs=1.0):
        """Mean position and yaw over a window, for a drone held still."""
        end = time.time() + secs
        xs, ys, zs, yaws = [], [], [], []
        while time.time() < end:
            p = vic.latest()
            if p is not None:
                xs.append(p.x); ys.append(p.y); zs.append(p.z)
                yaws.append(p.yaw_deg)
            time.sleep(0.02)
        if not xs:
            return None, []
        return (sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)), yaws

    def collect(secs):
        end = time.time() + secs
        out = []
        while time.time() < end:
            p = vic.latest()
            if p is not None:
                out.append((p.x, p.y, p.yaw_deg))
            time.sleep(0.01)
        return out

    print("\n" + "=" * 72)
    print("BENCH FRAME TEST -- no radio, no motors. Take the props OFF anyway.")
    print("=" * 72)
    print("Five steps, all by hand. Follow the prompts; ENTER advances.\n")

    try:
        # -- 1. stationary --------------------------------------------------
        input("STEP 1/5  Put the drone on the floor, hands off. ENTER when still: ")
        base_flips = vic.node.total_yaw_rejects
        _, yaws0 = avg_pose(3.0)
        jitter = FC.circular_span_deg(yaws0) if yaws0 else 0.0
        flips = vic.node.total_yaw_rejects - base_flips
        print(f"          yaw moved {jitter:.1f} deg while stationary, "
              f"{flips} flips\n")

        # -- 2. slide along the nose ----------------------------------------
        print("STEP 2/5  Slide the drone at least 40 cm in the direction its")
        print("          NOSE points. Keep it level and DO NOT rotate it.")
        input("          ENTER when it is sitting at the start point: ")
        start, _ = avg_pose(1.0)
        input("          Now slide it forward, then ENTER: ")
        end_p, yaws_n = avg_pose(1.0)
        nose = FC.analyze_slide(start, end_p, yaws_n) if start and end_p else None
        if nose and "error" in nose:
            print(f"          UNUSABLE: {nose['error']}")
        elif nose:
            print(f"          travelled {nose['travel_heading_deg']:+.1f} deg, "
                  f"reported yaw {nose['mean_yaw_deg']:+.1f} deg  ->  "
                  f"offset {nose['yaw_error_deg']:+.1f} deg\n")

        # -- 3. slide to its left -------------------------------------------
        print("STEP 3/5  Back to the start, then slide it 40 cm to ITS OWN LEFT")
        print("          (the drone's left, not yours). Again, no rotating.")
        input("          ENTER when at the start point: ")
        start2, _ = avg_pose(1.0)
        input("          Now slide it left, then ENTER: ")
        end2, yaws_l = avg_pose(1.0)
        left = (FC.analyze_slide(start2, end2, yaws_l, expected_offset_deg=90.0)
                if start2 and end2 else None)
        if left and "error" in left:
            print(f"          UNUSABLE: {left['error']}")
        elif left:
            print(f"          offset {left['yaw_error_deg']:+.1f} deg\n")

        # -- 4. lift --------------------------------------------------------
        input("STEP 4/5  Put it down, ENTER, then lift it ~40 cm and hold: ")
        low, _ = avg_pose(1.0)
        input("          Lift it now, hold steady, then ENTER: ")
        high, _ = avg_pose(1.0)
        lift_dz = (high[2] - low[2]) if (low and high) else None
        print(f"          dz = {lift_dz:+.3f} m\n" if lift_dz is not None else "")

        # -- 5. rotate in place ---------------------------------------------
        print("STEP 5/5  Hold the drone at one spot and rotate it SLOWLY through")
        print("          a full turn, keeping its centre as fixed as you can.")
        input("          ENTER, then start turning (10 s): ")
        pts = collect(10.0)
        origin = FC.fit_origin_offset(pts)
        if "error" in origin:
            print(f"          UNUSABLE: {origin['error']}\n")
        else:
            print(f"          origin offset {origin['offset_mag']*100:.1f} cm\n")

        print(FC.report(nose, left, lift_dz, origin,
                        vic.node.total_yaw_rejects, jitter))
    except (KeyboardInterrupt, EOFError):
        print("\naborted")
    finally:
        vic.shutdown()
    return 0



def run_motor_test(args):
    """
    Closed-loop sign test with the PROPS OFF and the drone in your hand.

    Measures the whole chain -- Vicon -> EKF -> position -> velocity -> attitude
    -> motor mix -- and answers one question: when displaced from its setpoint,
    does the drone try to tilt TOWARD the setpoint?

    Made convention-free by one piece of setup: the nose is held along world +X
    throughout, so physical yaw is 0 and the body frame equals the world frame.
    No quaternion order, handedness or firmware sign assumption is used.
    """
    try:
        import frame_check as FC
    except Exception as exc:
        print(f"frame_check.py not importable ({exc}).")
        return 1
    if not CFLIB_AVAILABLE:
        print(f"cflib not importable ({_CFLIB_IMPORT_ERROR}).")
        return 3

    print("\n" + "!" * 72)
    print("MOTOR TEST -- THE MOTORS WILL SPIN.")
    print("!" * 72)
    print("  * REMOVE ALL FOUR PROPELLERS. Check each one is off, by hand.")
    print("  * Hold the drone firmly, NOSE POINTING ALONG VICON +X, and keep")
    print("    that heading for the whole test. Do not rotate it.")
    print("  * You will move it around by hand while the motors run.")
    print("!" * 72)
    if input('Type exactly "PROPS OFF" to continue: ').strip() != "PROPS OFF":
        print("aborted")
        return 1

    vic = ViconThread(args.topic, args.expected_hz,
                      getattr(args, "max_yaw_rate", 720.0))
    if not vic.wait_for_first_pose(10.0):
        print("NO POSE RECEIVED.")
        vic.shutdown()
        return 1

    prepare_radio()
    cflib.crtp.init_drivers(enable_debug_driver=False)
    samples = []
    motors = {"m1": 0.0, "m2": 0.0, "m3": 0.0, "m4": 0.0}
    lock = threading.Lock()

    def on_motor(ts, data, cfg):
        with lock:
            for k in motors:
                motors[k] = data.get(f"motor.{k}", motors[k])

    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            cf = scf.cf
            cf.param.set_value("stabilizer.estimator", "2")
            cf.param.set_value("stabilizer.controller", "1")
            cf.param.set_value("commander.enHighLevel", "0")
            time.sleep(0.3)

            lg = LogConfig(name="motors", period_in_ms=20)
            for v in ("motor.m1", "motor.m2", "motor.m3", "motor.m4"):
                lg.add_variable(v, "uint16_t")
            cf.log.add_config(lg)
            lg.data_received_cb.add_callback(on_motor)
            lg.start()

            stop = threading.Event()

            def feed():
                period = 1.0 / 100.0
                while not stop.is_set():
                    p = vic.latest()
                    if p is not None and (time.time() - p.recv_time) < 0.25:
                        try:
                            if args.pos_only or not p.quat_ok:
                                cf.extpos.send_extpos(p.x, p.y, p.z)
                            else:
                                cf.extpos.send_extpose(p.x, p.y, p.z,
                                                       p.qx, p.qy, p.qz, p.qw)
                        except Exception:
                            pass
                    time.sleep(period)
            t = threading.Thread(target=feed, daemon=True)
            t.start()

            print("[cf] resetting estimator...")
            cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(0.15)
            cf.param.set_value("kalman.resetEstimation", "0")
            time.sleep(2.5)

            p0 = vic.latest()
            yaw0 = p0.yaw_deg
            print(f"[cf] reported yaw with the nose on +X: {yaw0:+.1f} deg")
            if abs(FC.wrap180(yaw0)) > 25.0:
                print(f"      ^^ THIS IS ALREADY THE ANSWER: with the nose")
                print(f"         physically along +X, a correct rigid body")
                print(f"         reports ~0 deg, not {yaw0:+.1f}. The object's")
                print(f"         X axis is {FC.wrap180(yaw0):+.0f} deg off the nose.")
            _arm(cf, True)
            cf.commander.send_setpoint(0, 0, 0, 0)

            sp = (p0.x, p0.y, p0.z + 0.02)
            print("\nMove the drone around by hand, 15-40 cm from where it")
            print("started, in several directions. Keep the nose on +X.")
            print("Recording for 25 s...\n")
            end = time.time() + 25.0
            while time.time() < end:
                p = vic.latest()
                if p is None:
                    continue
                cf.commander.send_position_setpoint(sp[0], sp[1], sp[2], 0.0)
                ex, ey = sp[0] - p.x, sp[1] - p.y
                with lock:
                    m = (motors["m1"], motors["m2"], motors["m3"], motors["m4"])
                if sum(m) > 100:
                    samples.append((ex, ey) + m)
                sys.stdout.write(f"\r  err {math.hypot(ex, ey)*100:5.1f} cm  "
                                 f"motors {int(m[0]):5d} {int(m[1]):5d} "
                                 f"{int(m[2]):5d} {int(m[3]):5d}  "
                                 f"samples {len(samples):4d}   ")
                sys.stdout.flush()
                time.sleep(0.02)

            stop.set()
            cf.commander.send_stop_setpoint()
            try:
                cf.commander.send_notify_setpoint_stop()
            except Exception:
                pass
            _arm(cf, False)
            lg.stop()
    except Exception as exc:
        print(diagnose_radio_failure(exc, args.uri))
        vic.shutdown()
        return 4

    print("\n")
    print(FC.report_motor_test(FC.analyze_motor_response(samples)))
    vic.shutdown()
    return 0


# ==============================================================================
# MAIN
# ==============================================================================

def selfcheck() -> None:
    """
    Fail before the radio opens if this file is internally inconsistent.

    Catches a stale or half-applied copy: every attribute the control loop
    reads off the mocap wrapper must actually exist on it. Cheap, static, and
    it fires at startup instead of at the first supervise() tick -- which is
    after the drone is armed.

    Uses AST rather than a regex: a regex over the source also matches example
    code inside docstrings, which produced phantom failures the first time this
    was written.
    """
    import ast
    import inspect
    try:
        src = inspect.getsource(sys.modules[__name__])
        tree = ast.parse(src)
    except Exception:
        return  # can't read or parse our own source; not worth failing over

    want_thread, want_node = set(), set()
    for n in ast.walk(tree):
        if not isinstance(n, ast.Attribute):
            continue
        v = n.value
        if not isinstance(v, ast.Attribute):
            continue
        # self.vicon.node.<attr>
        if (v.attr == "node" and isinstance(v.value, ast.Attribute)
                and v.value.attr == "vicon"
                and isinstance(v.value.value, ast.Name)
                and v.value.value.id == "self"):
            want_node.add(n.attr)
        # self.vicon.<attr>
        elif (v.attr == "vicon" and isinstance(v.value, ast.Name)
                and v.value.id == "self" and n.attr != "node"):
            want_thread.add(n.attr)

    # Fields assigned in _init_state are INSTANCE attributes, so
    # hasattr(ViconSource, ...) cannot see them and every data field the
    # control loop reads gets reported missing. Only methods would pass.
    # Probe a real instance the way the tests do: __new__ plus _init_state,
    # which needs no ROS graph.
    probe = None
    try:
        probe = ViconSource.__new__(ViconSource)
        probe._init_state(MOCAP_EXPECTED_HZ)
    except Exception:
        probe = None

    def _has(cls, obj, a):
        return hasattr(cls, a) or (obj is not None and hasattr(obj, a))

    missing = [f"ViconThread.{a}" for a in want_thread
               if not hasattr(ViconThread, a)]
    missing += [f"ViconSource.{a}" for a in want_node
                if not _has(ViconSource, probe, a)]

    if missing:
        raise SystemExit(
            "\n".join([
                "",
                "=" * 70,
                "SELF-CHECK FAILED -- refusing to connect to the drone.",
                "=" * 70,
                "The control loop references attributes that do not exist:",
                *(f"    {m}" for m in sorted(missing)),
                "",
                "This file is stale or was only partially updated. Replace it",
                "with the current version and verify:",
                "    python3 crazyflie_vicon_teleop.py --version",
                "=" * 70,
            ]))


def main():
    ap = argparse.ArgumentParser(
        description="Vicon-guided keyboard teleop for Crazyflie 2.x",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uri", default=DEFAULT_URI)
    ap.add_argument("--topic", default=DEFAULT_TOPIC)
    ap.add_argument("--expected-hz", type=float, default=MOCAP_EXPECTED_HZ)
    ap.add_argument("--hover-z", type=float, default=0.60,
                    help="takeoff altitude for T, metres")
    ap.add_argument("--hold-ft", type=float, default=DEFAULT_HOLD_FT,
                    help="altitude for the H (hold) button, in FEET "
                         f"(default {DEFAULT_HOLD_FT})")
    ap.add_argument("--hold-z", type=float, default=None,
                    help="hold altitude in METRES; overrides --hold-ft")
    ap.add_argument("--bounds", type=float, nargs=6, metavar=(
        "XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        help="absolute soft flight volume in Vicon world coordinates")
    ap.add_argument("--volume", type=float, nargs=3, metavar=("W", "D", "H"),
                    default=list(DEFAULT_VOLUME),
                    help="flight volume size, AUTO-CENTRED on the drone's "
                         "position at startup (default %(default)s). Ignored "
                         "if --bounds is given.")
    ap.add_argument("--keyboard", choices=["auto", "pygame", "termios"],
                    default="auto")
    ap.add_argument("--yaw-sign", type=int, choices=[1, -1], default=1,
                    help="flip if the drone yaws the wrong way (firmware "
                         "convention differs across versions)")
    ap.add_argument("--motor-test", action="store_true",
                    help="PROPS OFF bench test: measures whether the closed "
                         "loop actually pushes the drone toward its setpoint. "
                         "The definitive control-frame check.")
    ap.add_argument("--yaw-offset", type=float, default=0.0,
                    help="rotate the INJECTED quaternion by this many degrees "
                         "about Z. Empirical correction for a rigid body whose "
                         "X axis is not the nose. Prefer fixing Tracker.")
    ap.add_argument("--frame-test", action="store_true",
                    help="GUIDED BENCH TEST, no radio and no motors: measures "
                         "the constant yaw offset, frame handedness, Z sign and "
                         "rigid-body origin offset. Run this before blaming the "
                         "controller.")
    ap.add_argument("--check", action="store_true",
                    help="frame/units verification only -- no radio, no motors")
    ap.add_argument("--no-fly", action="store_true",
                    help="connect and feed the EKF but refuse to take off")
    ap.add_argument("--version", action="store_true",
                    help="print the file revision and exit")
    ap.add_argument("--extpose-hz", type=float, default=EXTPOSE_HZ,
                    help="rate at which mocap is injected into the EKF "
                         "(default %(default)s). The EKF DERIVES velocity from "
                         "these updates, so this rate limits velocity-estimate "
                         "quality -- and velocity is the 2nd cascade level.")
    ap.add_argument("--max-yaw-rate", type=float, default=720.0,
                    help="reject the quaternion when reported yaw changes "
                         "faster than this (deg/s). Catches mocap solver flips "
                         "from an ambiguous marker layout. Default %(default)s.")
    ap.add_argument("--pos-only", action="store_true",
                    help="inject position only, no quaternion. Diagnostic: if "
                         "hover improves, the rigid-body axes are misaligned.")
    ap.add_argument("--diagnose", type=float, metavar="SECONDS", default=0.0,
                    help="auto: take off, hold, record SECONDS of hover to "
                         "hover_log.csv, land, then analyse and report why it "
                         "drifts")
    ap.add_argument("--step", type=float, metavar="METRES", default=0.0,
                    help="with --diagnose: move the hold point this far along "
                         "world +X at 1/3 of the capture and back at 2/3. The "
                         "delay between the setpoint moving and the drone "
                         "moving is the loop dead time -- the only way to "
                         "measure real end-to-end latency. 0.30 is a good "
                         "value; keep --volume wide enough to contain it")
    args, ros_args = ap.parse_known_args()

    print(f"[teleop] revision {REVISION}")
    print(f"[teleop] core     {cf_core.CORE_REVISION}")
    if args.version:
        return 0
    selfcheck()

    # Feet -> metres exactly once, here at the boundary. Everything downstream
    # is metres.
    if args.hold_z is None:
        args.hold_z = args.hold_ft * M_PER_FT
    print(f"[teleop] hold altitude (H key): {args.hold_z:.3f} m "
          f"/ {args.hold_z / M_PER_FT:.2f} ft")

    if args.frame_test:
        return run_frame_test(args)

    if args.motor_test:
        return run_motor_test(args)

    if args.check:
        return run_check(args)

    if not CFLIB_AVAILABLE:
        print(f"cflib is not importable ({_CFLIB_IMPORT_ERROR}).\n"
              f"  pip install cflib\n"
              f"(--check still works without it.)")
        return 3

    # --- Vicon first. No radio link until we know the pose stream is alive. ---
    vic = ViconThread(args.topic, args.expected_hz, args.max_yaw_rate)
    if not vic.wait_for_first_pose(10.0):
        print("NO POSE RECEIVED in 10 s. Refusing to connect to the drone.")
        vic.shutdown()
        return 1
    time.sleep(1.0)
    rate = vic.rate_hz()
    print(f"[vicon] streaming at {rate:.1f} Hz")
    if rate < 50.0:
        print(f"[vicon] WARNING: {rate:.1f} Hz is too low for position control. "
              f"You want >=50 Hz, ideally 100+.")
    # Trust the measurement over the CLI flag.
    vic.node.calibrate_rate(rate)

    # ---- Flight volume -------------------------------------------------------
    # Auto-centre on where the drone actually is unless told otherwise. This is
    # the fix for a Vicon origin that sits at a corner of the capture space.
    p0 = vic.latest()
    if args.bounds:
        box = Bounds(*args.bounds)
        origin = "explicit --bounds"
    else:
        box = Bounds.centered_on(p0.x, p0.y, *args.volume)
        args.bounds = [box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax]
        origin = (f"auto-centred on drone at ({p0.x:+.2f},{p0.y:+.2f}), "
                  f"size {args.volume[0]}x{args.volume[1]}x{args.volume[2]} m")
    print(f"[volume] {box.describe()}   ({origin})")
    print(f"[volume] drone at ({p0.x:+.2f},{p0.y:+.2f},{p0.z:+.2f}) m  "
          f"-> {'INSIDE' if not box.outside(p0.x, p0.y, p0.z) else 'OUTSIDE'}")
    print(f"[volume] escalation: >{GEOFENCE_MARGIN_M:.2f} m outside = auto-land, "
          f">{GEOFENCE_KILL_M:.2f} m outside = motors off")
    if box.outside(p0.x, p0.y, p0.z):
        print(f"[volume] WARNING: takeoff will be refused. Suggested:\n"
              f"         {Bounds.centered_on(p0.x, p0.y, *args.volume).as_cli()}")

    # ---- orientation integrity gate -----------------------------------------
    # Checked BEFORE the radio opens. A flipping quaternion inverts the position
    # loop, so arming into it is how you get an expanding spiral.
    if vic.node.total_yaw_rejects and not args.pos_only:
        print("\n" + "=" * 70)
        print("MOCAP ORIENTATION IS UNRELIABLE -- refusing to arm.")
        print("=" * 70)
        print(f"  {vic.node.total_yaw_rejects} impossible yaw jumps already, "
              f"peak {vic.node.max_yaw_rate_seen:.0f} deg/s, "
              f"while the drone has not even taken off.")
        print("  Your Vicon rigid body is rotationally AMBIGUOUS: the marker")
        print("  layout maps onto itself under rotation, so Tracker has two")
        print("  valid solutions and keeps swapping. Every swap inverts the")
        print("  frame the position controller works in.")
        print("")
        print("  FIX (do this): re-place markers so no rotation maps the set")
        print("  onto itself -- 4-5 markers at DIFFERENT HEIGHTS, irregular")
        print("  spacing. Then delete and re-create the rigid body in Tracker.")
        print("")
        print("  FLY NOW anyway, ignoring mocap orientation:")
        print("      python3 crazyflie_vicon_teleop.py --pos-only")
        print("=" * 70)
        vic.shutdown()
        return 5

    # ---- upright gate --------------------------------------------------------
    # The flip gate above catches an orientation that CHANGES impossibly. It
    # says nothing about one that is impossibly WRONG but perfectly steady --
    # a rigid body created with its Z axis inverted reports the drone upside
    # down, rock solid, with zero yaw rejects. Injecting that quaternion tells
    # the EKF the drone is inverted, and the attitude controller then drives
    # toward that "correction" the instant motors spool: a flip into the floor
    # at full thrust, not a drift.
    #
    # A drone that is about to take off is sitting on the ground, so its
    # reported tilt from vertical must be small. Uses only R[2][2], so no
    # roll/pitch sign or Euler-order convention is involved -- and near 180 deg
    # Euler roll wraps at +/-180, which is exactly where a naive check would
    # read plausible noise instead of an inversion.
    if p0 is not None and not args.pos_only:
        r22 = 1.0 - 2.0 * (p0.qx * p0.qx + p0.qy * p0.qy)
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, r22))))
        if tilt_deg > PREARM_TILT_DEG:
            print("\n" + "=" * 70)
            print("MOCAP SAYS THE DRONE IS NOT UPRIGHT -- refusing to arm.")
            print("=" * 70)
            print(f"  Reported tilt from vertical: {tilt_deg:.1f} deg "
                  f"(limit {PREARM_TILT_DEG:.0f} deg).")
            if tilt_deg > 120.0:
                print("  That is upside down. The rigid body's Z axis points DOWN.")
            print("")
            print("  The drone is sitting on the ground, so this is the RIGID")
            print("  BODY, not the drone. It was created from an inverted or")
            print("  badly tilted pose, and the axes are baked in that way.")
            print("")
            print("  Confirm with the drone's own accelerometer (no motors):")
            print("      python3 imu_tilt.py")
            print("  If the IMU says level and Vicon does not, it is the template.")
            print("")
            print("  FIX: stand the drone upright, flat and level, nose along")
            print("  Vicon +X, then DELETE and RE-CREATE the rigid body in")
            print("  Tracker. Editing it keeps the old template.")
            print("=" * 70)
            vic.shutdown()
            return 6

    kb = None
    teleop = None
    prepare_radio()
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print(f"[cf] connecting to {args.uri} ...")
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            kb = make_keyboard(None if args.keyboard == "auto" else args.keyboard)
            teleop = Teleop(scf, vic, kb, args)
            teleop.configure()
            teleop.start_extpose_feed()
            time.sleep(1.0)

            if not teleop.reset_estimator_and_wait():
                print("Aborting: estimator never converged onto the Vicon pose.")
                return 2

            if args.pos_only:
                p0 = vic.latest()
                if p0 is not None and abs((p0.yaw_deg + 180) % 360 - 180) > 25.0:
                    print("\n" + "=" * 70)
                    print("--pos-only REQUIRES the drone to start facing world +X.")
                    print("=" * 70)
                    print(f"  Reported yaw is {p0.yaw_deg:+.1f} deg, not ~0.")
                    print("  Without the quaternion, the EKF's yaw is gyro-only and")
                    print("  kalman.resetEstimation initialises it to ZERO. If the")
                    print("  drone is not physically pointing along +X, the position")
                    print("  loop is rotated by that angle and WILL diverge -- which")
                    print("  is a fault introduced by this flag, not the one you are")
                    print("  trying to isolate.")
                    print("")
                    print("  Physically point the nose along Vicon +X and retry.")
                    print("  If the nose IS on +X and yaw still reads far from 0,")
                    print("  the rigid body itself is rotated by that amount --")
                    print("  that is your bug. Confirm with --frame-test.")
                    print("=" * 70)
                    return 6
            teleop.arm()
            teleop.recenter_setpoint()
            if args.no_fly:
                print("\n[--no-fly] Monitoring only. Watch ekf_err; it should "
                      "settle under 5 cm. Ctrl-C to exit.\n")
            teleop.run()
    except KeyboardInterrupt:
        print("\n[main] interrupted")
        if teleop:
            teleop.emergency_stop("KeyboardInterrupt")
    except Exception as exc:
        if teleop is None:
            # Failed before the link was up: no motors involved, so print the
            # targeted remedy instead of three nested libusb tracebacks.
            print(diagnose_radio_failure(exc, args.uri))
            return 4
        print(f"\n[main] FATAL: {exc}")
        teleop.emergency_stop(f"exception: {exc}")
        raise
    finally:
        if teleop:
            teleop.cleanup()
            if teleop.abort_reason:
                print(f"[main] aborted: {teleop.abort_reason}")
            if teleop.samples:
                if teleop.write_samples("hover_log.csv"):
                    try:
                        import hover_diagnostics
                        print(hover_diagnostics.report(
                            hover_diagnostics.analyze(teleop.samples)))
                    except Exception as exc:
                        print(f"[diagnose] analysis skipped ({exc}). Run:\n"
                              f"    python3 hover_diagnostics.py hover_log.csv")
                else:
                    print("[diagnose] too few samples to analyse "
                          "-- did it abort early?")
        if kb:
            kb.close()
        vic.shutdown()
        print("[main] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
