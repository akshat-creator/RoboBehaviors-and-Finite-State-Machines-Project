"""
Draw Pentagon
--------------------------
This node uses a simple time-based approach to drive a robot in a continuous
regular pentagon loop: move straight for a fixed distance, then turn left 72°,
repeat forever.
"""
import math
from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawPentagon(Node):
    """Continuously pilots a robot along a regular pentagon."""

    def __init__(self,
                 edge_length_m: float = 0.5,
                 forward_vel_mps: float = 0.1,
                 angular_vel_rps: float = 0.3):
        super().__init__('draw_pentagon')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Pentagon geometry and speed parameters
        self.edge_length = edge_length_m
        self.forward_vel = forward_vel_mps
        self.angular_vel = angular_vel_rps
        self.external_turn_rad = 2 * math.pi / 5.0  # 72° in radians

        # Kick off control loop in a background thread
        self.run_loop_thread = Thread(target=self.run_loop, daemon=True)
        self.run_loop_thread.start()

    def run_loop(self):
        """Continuously runs pentagon edges and turns while the node is active."""
        # Send a zero velocity first to ensure robot is stopped
        self.drive(0.0, 0.0)
        sleep(1.0)

        while rclpy.ok():
            for side in range(5):
                self.get_logger().info(f"Pentagon: edge {side+1}/5 → driving forward")
                self.drive_forward(self.edge_length)
                self.get_logger().info("Pentagon: turning left 72°")
                self.turn_left(self.external_turn_rad)

            self.get_logger().info("Completed a pentagon; looping again.")

        # On shutdown, ensure we stop the robot
        self.drive(0.0, 0.0)

    # -- Low-level helpers -------------------------------------------------

    def drive(self, linear: float, angular: float):
        """Publish a Twist with specified linear/ang velocities."""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def drive_forward(self, distance_m: float):
        """Drive straight for a specified distance (time-based)."""
        t = abs(distance_m) / self.forward_vel if self.forward_vel > 0 else 0.0
        self.drive(linear=self.forward_vel, angular=0.0)
        sleep(t)
        self.drive(0.0, 0.0)

    def turn_left(self, angle_rad: float):
        """Execute a left turn by angle_rad (time-based)."""
        t = abs(angle_rad) / self.angular_vel if self.angular_vel > 0 else 0.0
        self.drive(linear=0.0, angular=self.angular_vel)
        sleep(t)
        self.drive(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DrawPentagon(
        edge_length_m=0.5,       # change to make the pentagon larger/smaller
        forward_vel_mps=0.1,     # linear speed
        angular_vel_rps=0.3      # turn speed
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
