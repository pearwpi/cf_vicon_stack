#!/usr/bin/env python3
"""
teleop_node.py -- keyboard front end for crazyflie_server.

Publishes crazyflie_interfaces/Position on {prefix}/cmd_position and calls the
takeoff / land / emergency services. It holds NO safety logic and never touches
the radio: if this node dies, crazyflie_server notices the command stream stop
and holds position by itself. That separation is the whole point of the split.

Because this node only integrates a setpoint, an RL policy can replace it by
publishing to the same topic. Nothing downstream changes.

CONTROLS
  T           take off to hover_z
  L           land
  SPACE       emergency stop (cuts motors; the drone WILL fall)
  W/S A/D     forward/back, left/right   (setpoint moves in the WORLD frame)
  R/F         up / down
  Q/E         yaw left (CCW) / right (CW)
  Z           re-centre the setpoint on the drone's current position
  LSHIFT      hold for 2x speed
  ESC         quit (lands first if flying)

NOTE ON THE FRAME: the standalone script moves the setpoint in the drone's BODY
frame. This node moves it in the WORLD frame, because a ROS command topic with
a body-frame meaning is a trap for anything that is not a human holding a key.
If you want body-frame teleop, use the standalone script.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

from crazyflie_interfaces.msg import Position
from crazyflie_interfaces.srv import Land, Takeoff

from crazyflie_ros import cf_core as C
from crazyflie_ros import cf_keyboard


class TeleopNode(Node):

    def __init__(self):
        super().__init__("crazyflie_teleop")
        d = self.declare_parameter
        self.prefix = d("prefix", "cf1").value
        self.rate_hz = float(d("rate_hz", C.CONTROL_HZ).value)
        self.hover_z = float(d("hover_z", 0.60).value)
        self.backend = d("keyboard", "termios").value

        p = self.prefix
        self.pub = self.create_publisher(Position, f"{p}/cmd_position", 10)
        self.cli_takeoff = self.create_client(Takeoff, f"{p}/takeoff")
        self.cli_land = self.create_client(Land, f"{p}/land")
        self.cli_emergency = self.create_client(Empty, f"{p}/emergency")

        self.kb = cf_keyboard.make_keyboard(self.backend)
        self.sp = None                 # (x, y, z, yaw_deg); None until takeoff
        self.flying = False

        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f"teleop ready on {p}/cmd_position at {self.rate_hz:.0f} Hz "
            f"({type(self.kb).__name__}). T=takeoff L=land SPACE=e-stop ESC=quit")

    def tick(self):
        self.kb.poll()
        dt = 1.0 / self.rate_hz

        if self.kb.pressed("space"):
            self.get_logger().error("EMERGENCY STOP")
            self._call(self.cli_emergency, Empty.Request())
            self.flying = False
            self.sp = None
            return

        if self.kb.pressed("escape"):
            if self.flying:
                self._call(self.cli_land, Land.Request())
            raise KeyboardInterrupt

        if self.kb.pressed("t") and not self.flying:
            req = Takeoff.Request()
            req.height = self.hover_z
            self._call(self.cli_takeoff, req)
            self.flying = True
            self.sp = None            # server owns the setpoint until it hovers
            self.get_logger().info(f"takeoff requested ({self.hover_z:.2f} m)")
            return

        if self.kb.pressed("l") and self.flying:
            self._call(self.cli_land, Land.Request())
            self.flying = False
            self.sp = None
            self.get_logger().info("land requested")
            return

        if not self.flying:
            return

        # The server drives the climb, so we do not publish until the operator
        # actually asks for a move. The first movement key seeds the setpoint
        # from the server's hover point via a Z (recentre) style zero command.
        speed = C.POS_SPEED_MS * (C.TURBO if self.kb.held("shift") else 1.0)
        climb = C.POS_CLIMB_MS * (C.TURBO if self.kb.held("shift") else 1.0)
        dx = (self.kb.held("w") - self.kb.held("s")) * speed * dt
        dy = (self.kb.held("a") - self.kb.held("d")) * speed * dt
        dz = (self.kb.held("r") - self.kb.held("f")) * climb * dt
        dyaw = (self.kb.held("q") - self.kb.held("e")) * C.YAW_RATE_DPS * dt

        if self.sp is None:
            if not (dx or dy or dz or dyaw):
                return                # nothing asked for yet; stay quiet
            self.sp = (0.0, 0.0, self.hover_z, 0.0)
            self.get_logger().warn(
                "seeding the setpoint at (0, 0, hover_z). Press Z to recentre "
                "on the drone if that is not where it is.")

        x, y, z, yaw = self.sp
        self.sp = (x + dx, y + dy, z + dz, C.wrap180(yaw + dyaw))

        m = Position()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.x, m.y, m.z, m.yaw = (float(self.sp[0]), float(self.sp[1]),
                                float(self.sp[2]), float(self.sp[3]))
        self.pub.publish(m)           # Position.yaw is DEGREES; see the server

    def _call(self, client, req):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(f"{client.srv_name} is not available")
            return None
        return client.call_async(req)

    def destroy_node(self):
        try:
            self.kb.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
