#!/usr/bin/env python3
"""
circle_node.py -- take off, fly a horizontal circle, return, land.

A scripted stand-in for teleop / a policy: it publishes {prefix}/cmd_position
and calls the takeoff / land services. It holds NO safety logic and never
touches the radio; crazyflie_server's supervisor, geofence and leash apply to
every setpoint it sends. Run it with the keyboard teleop disabled:

    ros2 launch crazyflie_ros crazyflie.launch.py teleop:=false config:=<cfg>
    ros2 run crazyflie_ros circle --ros-args -p vicon_topic:=/vicon/<b>/<b>

SEQUENCE
  1. wait for a Vicon pose; latch the start point and yaw
  2. call takeoff(height) and stay SILENT on cmd_position during the climb
     (the server owns the climb, and it only lets go of its takeoff hold
     after a command timeout -- publishing during the climb would leave the
     drone parked on the takeoff point and ignore every command)
  3. wait until Vicon says the drone has held `height` for settle_s
  4. glide out to the circle, fly `laps` laps (ramped angular speed in and
     out), glide back to the centre, hover, land

Yaw is held at the start yaw throughout. Position.yaw is DEGREES.

ABORTS: Ctrl-C -> land. The server dropping IS_FLYING mid-flight (auto-land,
geofence, e-stop) -> this node stops publishing and exits. If this node dies
outright, the server's command watchdog holds position; call {prefix}/land.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Position, Status
from crazyflie_interfaces.srv import Land, Takeoff

from crazyflie_ros import cf_core as C


# ==============================================================================
# TRAJECTORY -- pure, so it is testable without ROS
# ==============================================================================
def _smooth(u):
    """0..1 -> 0..1 with zero velocity at both ends."""
    u = min(max(u, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * u)


class CirclePlan:
    """Setpoint (x, y) as a function of time since the plan started.

    Phases: 'out' (start -> circle at angle 0), 'circle', 'back' (-> centre),
    'settle' (hold centre), then done.
    """

    def __init__(self, start_xy, center_xy, radius, period_s, laps,
                 transit_ms=0.25, ramp_s=2.0, settle_s=2.0):
        self.sx, self.sy = start_xy
        self.cx, self.cy = center_xy
        self.r = radius
        self.omega = 2.0 * math.pi / period_s
        self.total_angle = 2.0 * math.pi * laps
        self.ramp = min(ramp_s, self.total_angle / self.omega / 2.0)

        ax, ay = self.cx + self.r, self.cy
        self.t_out = max(math.hypot(ax - self.sx, ay - self.sy) / transit_ms, 1.0)
        cruise_angle = self.total_angle - self.omega * self.ramp
        self.t_circle = 2.0 * self.ramp + cruise_angle / self.omega
        self.t_back = max(self.r / transit_ms, 1.0)
        self.t_settle = settle_s
        self.duration = self.t_out + self.t_circle + self.t_back + self.t_settle

    def _angle(self, t):
        """Angle swept after t seconds of the circle phase, ramped in and out."""
        w, T, R = self.omega, self.t_circle, self.ramp
        if t <= R:
            return 0.5 * w * t * t / R
        if t >= T - R:
            tr = T - t
            return self.total_angle - 0.5 * w * tr * tr / R
        return 0.5 * w * R + w * (t - R)

    def at(self, t):
        """Returns (x, y, phase). phase is None once the plan is complete."""
        if t < self.t_out:
            s = _smooth(t / self.t_out)
            return (self.sx + s * (self.cx + self.r - self.sx),
                    self.sy + s * (self.cy - self.sy), "out")
        t -= self.t_out
        if t < self.t_circle:
            a = self._angle(t)
            return (self.cx + self.r * math.cos(a),
                    self.cy + self.r * math.sin(a), "circle")
        t -= self.t_circle
        if t < self.t_back:
            s = _smooth(t / self.t_back)
            return (self.cx + (1.0 - s) * self.r, self.cy, "back")
        t -= self.t_back
        if t < self.t_settle:
            return (self.cx, self.cy, "settle")
        return (self.cx, self.cy, None)


# ==============================================================================
# NODE
# ==============================================================================
class CircleNode(Node):

    def __init__(self):
        super().__init__("crazyflie_circle")
        d = self.declare_parameter
        self.prefix = d("prefix", "cf1").value
        self.vicon_topic = d("vicon_topic", "/vicon/crazyflie_a2/crazyflie_a2").value
        self.height = float(d("height", 1.0).value)
        self.diameter = float(d("diameter", 1.0).value)
        self.period_s = float(d("period_s", 10.0).value)     # seconds per lap
        self.laps = float(d("laps", 2.0).value)
        self.center_mode = d("center", "start").value        # "start" | "origin"
        self.rate_hz = float(d("rate_hz", C.CONTROL_HZ).value)
        self.settle_s = float(d("settle_s", 1.5).value)

        if self.center_mode not in ("start", "origin"):
            raise ValueError("center must be 'start' or 'origin'")
        v = math.pi * self.diameter / self.period_s
        if v > C.POS_SPEED_MS * C.TURBO:
            raise ValueError(f"{v:.2f} m/s around the circle is faster than this "
                             f"stack is tuned for; raise period_s")

        p = self.prefix
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PoseStamped, self.vicon_topic, self._pose_cb, sensor_qos)
        self.create_subscription(Status, f"{p}/status", self._status_cb, 10)
        self.pub = self.create_publisher(Position, f"{p}/cmd_position", 10)
        self.cli_takeoff = self.create_client(Takeoff, f"{p}/takeoff")
        self.cli_land = self.create_client(Land, f"{p}/land")

        self.pose = None              # (x, y, z, yaw_deg, recv_time)
        self.server_flying = False
        self.phase = "wait_pose"
        self.t_phase = time.time()
        self.at_height_since = None
        self.plan = None
        self.yaw0 = 0.0
        self._yaw_hist = []
        self.done = False
        self._last_log = None

        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f"circle: height {self.height:.2f} m, diameter {self.diameter:.2f} m, "
            f"{self.laps:g} laps at {self.period_s:.1f} s/lap ({v:.2f} m/s), "
            f"centred on {self.center_mode}; pose from {self.vicon_topic}")

    # ---- inputs --------------------------------------------------------------
    def _pose_cb(self, msg):
        q = msg.pose.orientation
        # A lost object arrives as an exact (0, 0, 0) position. cf_core's sentinel
        # also needs a specific quaternion and missed a (0,0,0)+identity frame
        # seen on 2026-11-19, which would latch the START POINT at the origin.
        if (C.occluded_sentinel(msg.pose.position.x, msg.pose.position.y,
                                msg.pose.position.z, q.x, q.y, q.z, q.w)
                or (msg.pose.position.x == 0.0 and msg.pose.position.y == 0.0
                    and msg.pose.position.z == 0.0)):
            return
        self.pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                     C.yaw_deg_of(q.x, q.y, q.z, q.w), time.time())

    def _status_cb(self, msg):
        self.server_flying = bool(msg.supervisor_info & Status.SUPERVISOR_INFO_IS_FLYING)

    # ---- helpers -------------------------------------------------------------
    def _goto_phase(self, name):
        self.phase = name
        self.t_phase = time.time()
        self.get_logger().info(f"-> {name}")

    def _send(self, x, y, z):
        m = Position()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.x, m.y, m.z, m.yaw = float(x), float(y), float(z), float(self.yaw0)
        self.pub.publish(m)

    def _abort(self, why):
        self.get_logger().error(f"ABORT: {why}")
        self.land()

    def land(self):
        if self.phase in ("landing", "finished"):
            return
        if self.cli_land.wait_for_service(timeout_sec=0.5):
            self.cli_land.call_async(Land.Request())
            self.get_logger().info("land requested")
        else:
            self.get_logger().error(f"{self.cli_land.srv_name} unavailable -- "
                                    "the server will hold position; land it by hand")
        self._goto_phase("landing")

    # ---- state machine -------------------------------------------------------
    def tick(self):
        now = time.time()
        elapsed = now - self.t_phase
        pose_fresh = self.pose is not None and now - self.pose[4] < 0.25

        if self.phase == "wait_pose":
            if not pose_fresh:
                if elapsed > 10.0:
                    self.get_logger().error(f"no pose on {self.vicon_topic} after 10 s")
                    self._finish()
                return
            if not self.cli_takeoff.wait_for_service(timeout_sec=0.0):
                if elapsed > 10.0:
                    self.get_logger().error(f"{self.cli_takeoff.srv_name} unavailable")
                    self._finish()
                return
            # Latch yaw from 1 s of AGREEING samples, never from one frame: an
            # ambiguous rigid body can be mid-flip on exactly the frame we read,
            # and a yaw target 167 deg off makes the drone spin on the circle.
            self._yaw_hist.append(self.pose[3])
            if len(self._yaw_hist) < int(self.rate_hz):
                return
            ref = self._yaw_hist[-1]
            spread = max(abs(C.wrap180(v - ref)) for v in self._yaw_hist)
            if spread > 5.0:
                self._yaw_hist.clear()
                self.get_logger().warn(
                    f"Vicon yaw unstable ({spread:.0f} deg spread in 1 s) -- "
                    "rigid body is flipping; waiting for it to settle")
                return
            x, y, z, yaw, _ = self.pose
            self.start_xy = (x, y)
            self.center_xy = (x, y) if self.center_mode == "start" else (0.0, 0.0)
            self.yaw0 = yaw
            self.get_logger().info(
                f"start ({x:+.2f}, {y:+.2f}, {z:.2f}) yaw {yaw:+.1f} deg; circle "
                f"centre ({self.center_xy[0]:+.2f}, {self.center_xy[1]:+.2f})")
            req = Takeoff.Request()
            req.height = self.height
            self.cli_takeoff.call_async(req)
            self._goto_phase("climb")
            return

        if self.phase == "climb":
            # SILENT here on purpose -- see the module docstring, step 2.
            if elapsed > 2.0 and not self.server_flying:
                self.get_logger().error("server did not start flying -- takeoff was "
                                        "refused; its log says why")
                self._finish()
                return
            if elapsed > 15.0:
                self._abort("did not reach height within 15 s")
                return
            if pose_fresh and abs(self.pose[2] - self.height) < 0.08:
                if self.at_height_since is None:
                    self.at_height_since = now
                elif now - self.at_height_since >= self.settle_s:
                    x, y = self.pose[0], self.pose[1]
                    self.plan = CirclePlan((x, y), self.center_xy,
                                           self.diameter / 2.0, self.period_s,
                                           self.laps)
                    self.get_logger().info(f"at height; plan {self.plan.duration:.1f} s")
                    self._goto_phase("fly")
            else:
                self.at_height_since = None
            return

        if self.phase == "fly":
            if not self.server_flying:
                self.get_logger().error("server stopped flying (auto-land, geofence "
                                        "or e-stop) -- no longer commanding")
                self._finish()
                return
            x, y, step = self.plan.at(elapsed)
            if step is None:
                self.land()
                return
            if step != self._last_log:
                self._last_log = step
                self.get_logger().info(f"   {step}")
            self._send(x, y, self.height)
            return

        if self.phase == "landing":
            if not self.server_flying and elapsed > 1.0:
                self.get_logger().info("landed")
                self._finish()
            elif elapsed > 15.0:
                self.get_logger().error("still flying 15 s after land request")
                self._finish()

    def _finish(self):
        self.phase = "finished"
        self.done = True


def main(args=None):
    # No ROS signal handler: on Ctrl-C we still need a live context to call land.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = CircleNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C")
        if node.phase in ("climb", "fly"):
            node.land()
            t0 = time.time()
            while time.time() - t0 < 1.0:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
