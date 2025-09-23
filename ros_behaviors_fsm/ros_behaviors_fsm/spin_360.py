"""
Spin Once (360°)
----------------
This node rotates the robot in place exactly once (time-based) and exits.
It publishes to 'cmd_vel' and uses a constant angular velocity for a
duration equal to 2π / |angular_vel_rps|.
"""
import math
from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SpinOnce(Node):
    """Rotates the robot in place by 360 degrees and stops."""

    def __init__(self,
                 angular_vel_rps: float = 0.5,
                 clockwise: bool = False):
        """
        Args:
            angular_vel_rps: Magnitude of angular speed.
            clockwise: If True, spins clockwise (negative z); else CCW.
        """
        super().__init__('spin_360')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.angular_vel = float(angular_vel_rps) * (-1.0 if clockwise else 1.0)

        # Kick off control in a background thread
        self.run_loop_thread = Thread(target=self.run_loop, daemon=True)
        self.run_loop_thread.start()

    def run_loop(self):
        """Publish a constant angular velocity for the required duration, once."""
        # Some setups drop the first message; send a zero first.
        self.drive(0.0, 0.0)
        sleep(1.0)

        total_angle_rad = 2.0 * math.pi
        duration_s = total_angle_rad / abs(self.angular_vel)

        self.get_logger().info(
            f"Spinning {'clockwise' if self.angular_vel < 0 else 'counter-clockwise'} "
            f"for {duration_s:.2f}s at |ω|={abs(self.angular_vel):.2f} rad/s."
        )

        # Start spinning
        self.drive(0.0, self.angular_vel)
        sleep(duration_s)

        # Stop and finish
        self.drive(0.0, 0.0)
        self.get_logger().info("Done spinning 360°. Stopping node.")

    def drive(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpinOnce(
        angular_vel_rps=0.3,  # adjust for faster/slower spin
        clockwise=False       # set True to spin CW instead of CCW
    )
    try:
        # No subscriptions/timers; just wait for the worker to finish.
        node.run_loop_thread.join()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
