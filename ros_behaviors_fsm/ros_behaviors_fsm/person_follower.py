import rclpy
from rclpy.node import Node
from threading import Thread, Event
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math, time

class ObjectFollower(Node):
    """Threaded node that follows the closest object in view."""

    def __init__(self):
        super().__init__('object_follower')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

        # control parameters
        self.target_distance = 0.5   # desired distance to object [m]
        self.Kp_lin = 0.5            # proportional gain (linear)
        self.Kp_ang = 0.01           # proportional gain (angular)

        # state variables
        self.distance = None         # distance to closest object
        self.angle_index = None      # index of closest object
        self.scan_len = None         # number of rays in the scan

        self.stop_event = Event()
        self.thread = Thread(target=self.run_loop)
        self.thread.start()

    def process_scan(self, msg: LaserScan):
        """Find the closest object in the scan."""
        self.scan_len = len(msg.ranges)
        valid_ranges = [(i, d) for i, d in enumerate(msg.ranges) if d > 0.0 and not math.isinf(d)]
        if not valid_ranges:
            self.distance = None
            self.angle_index = None
            return

        # pick the closest object
        i, d = min(valid_ranges, key=lambda x: x[1])
        self.distance = d
        self.angle_index = i
        print(f"Closest object: dist={d:.2f} m at index={i}")

    def run_loop(self):
        """Continuously compute and publish velocity commands."""
        while not self.stop_event.is_set():
            msg = Twist()
            if self.distance is None or self.angle_index is None or self.scan_len is None:
                # nothing detected → stop
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                print("No object detected → stopping")
            else:
                # distance control (linear velocity)
                error_dist = self.distance - self.target_distance
                msg.linear.x = self.Kp_lin * error_dist

                # angle control (angular velocity)
                center_index = self.scan_len // 2
                error_ang = self.angle_index - center_index
                msg.angular.z = -self.Kp_ang * error_ang

                # clamp speeds
                msg.linear.x = max(min(msg.linear.x, 0.3), -0.3)
                msg.angular.z = max(min(msg.angular.z, 0.3), -0.3)

                print(f"cmd_vel → linear: {msg.linear.x:.2f}, angular: {msg.angular.z:.2f}")

            self.vel_pub.publish(msg)
            time.sleep(0.1)

    def destroy_node(self):
        """Stop thread cleanly on shutdown."""
        self.stop_event.set()
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
