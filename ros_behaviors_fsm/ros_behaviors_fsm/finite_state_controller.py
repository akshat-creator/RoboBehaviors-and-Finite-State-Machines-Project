"""
Finite-State Supervisor for Behavior Switching
---------------------------------------------
Runs ONE of two existing behaviors as a subprocess:
  - draw_pentagon (time-based driving)
  - person_follower (LaserScan-based following)

Transitions:
  - /bumper contact (ContactsState.non-empty):  PENTAGON → FOLLOW
  - lost target N sec in FOLLOW (from /scan):    FOLLOW → PENTAGON
  - estop=True:                                  any → IDLE (kill behavior, stop)
  - estop=False:                                 IDLE → PENTAGON

Topics:
  - 'bumper' : gazebo_msgs/ContactsState   (from Gazebo)
  - 'scan'   : sensor_msgs/LaserScan
  - 'estop'  : std_msgs/Bool
  - 'cmd_vel': geometry_msgs/Twist (only to force a stop on estop)

Assumes console entry points exist:
  'person_follower = ros_behaviors_fsm.person_follower:main'
  'draw_pentagon   = ros_behaviors_fsm.draw_pentagon:main'
"""

import math
import os
import signal
import subprocess
import sys
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from gazebo_msgs.msg import ContactsState  # <-- bumper messages


class Mode(Enum):
    PENTAGON = 0
    FOLLOW = 1
    IDLE = 2


class BehaviorFSM(Node):
    def __init__(self):
        super().__init__('behavior_fsm_supervisor')

        # ---- Parameters (override via ROS params if desired)
        self.declare_parameter('object_present_threshold_m', 1.5)  # FOLLOW sees target within this
        self.declare_parameter('lost_object_timeout_s', 2.0)       # FOLLOW → PENTAGON if no target for this long
        self.declare_parameter('poll_rate_hz', 10.0)               # timer rate for FOLLOW fallback checks

        # Bumper robustness
        self.declare_parameter('bumper_cooldown_s', 0.5)           # min seconds between switches from bumper
        self.declare_parameter('bumper_min_msgs', 1)               # consecutive contact msgs required

        self.object_present_threshold_m = float(
            self.get_parameter('object_present_threshold_m').value
        )
        self.lost_object_timeout_s = float(
            self.get_parameter('lost_object_timeout_s').value
        )
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.poll_dt = 1.0 / max(self.poll_rate_hz, 1.0)

        self._bumper_cooldown_s = float(self.get_parameter('bumper_cooldown_s').value)
        self._bumper_min_msgs = int(self.get_parameter('bumper_min_msgs').value)

        # ---- Pub/Sub
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(ContactsState, 'bumper', self._on_bumper_contacts, 10)

        # ---- Internal state
        self.mode = Mode.PENTAGON
        self.child_proc = None
        self.estopped = False

        # LaserScan bookkeeping (to decide when FOLLOW lost the target)
        self._last_seen_ts = None
        self._has_target_now = False

        # Bumper debounce/cooldown
        self._bumper_contact_streak = 0
        self._last_bumper_switch_ts = 0.0

        # Start default behavior
        self._start_pentagon()

        # Timer to evaluate FOLLOW → PENTAGON fallback
        self.create_timer(self.poll_dt, self._tick)

        self.get_logger().info(
            f"BehaviorFSM ready. scan_threshold={self.object_present_threshold_m:.2f} m, "
            f"lost_timeout={self.lost_object_timeout_s:.1f} s, "
            f"bumper_min_msgs={self._bumper_min_msgs}, bumper_cooldown={self._bumper_cooldown_s:.2f}s"
        )

    # -------------------- Subscribers --------------------

    def _on_bumper_contacts(self, msg: ContactsState):
        """Trigger FOLLOW when Gazebo reports any contact."""
        if self.estopped:
            return

        has_contact = len(msg.states) > 0
        now = time.monotonic()

        if has_contact:
            self._bumper_contact_streak += 1
        else:
            self._bumper_contact_streak = 0

        should_switch = (
            has_contact
            and self._bumper_contact_streak >= self._bumper_min_msgs
            and (now - self._last_bumper_switch_ts) >= self._bumper_cooldown_s
            and self.mode != Mode.FOLLOW
        )

        if should_switch:
            self.get_logger().info("Bumper contact (confirmed) → switching to FOLLOW.")
            self._last_bumper_switch_ts = now
            self._switch_to_follow()

    def _on_estop(self, msg: Bool):
        if msg.data and not self.estopped:
            self.estopped = True
            self.get_logger().warn("ESTOP asserted → entering IDLE and stopping.")
            self._kill_child()
            self._publish_stop()
            self.mode = Mode.IDLE
        elif not msg.data and self.estopped:
            self.estopped = False
            self.get_logger().warn("ESTOP cleared → resuming PENTAGON.")
            self._start_pentagon()

    def _on_scan(self, msg: LaserScan):
        # Consider there is a "target" if ANY finite reading is within threshold
        has_target = False
        thr = self.object_present_threshold_m
        for d in msg.ranges:
            if d is None or math.isinf(d) or math.isnan(d) or d <= 0.0:
                continue
            if d <= thr:
                has_target = True
                break

        self._has_target_now = has_target
        if has_target:
            self._last_seen_ts = time.monotonic()

    # -------------------- Timer: FOLLOW fallback --------------------

    def _tick(self):
        if self.estopped:
            return
        if self.mode == Mode.FOLLOW:
            now = time.monotonic()
            if self._last_seen_ts is None:
                no_view_duration = float('inf')
            else:
                no_view_duration = now - self._last_seen_ts

            if no_view_duration > self.lost_object_timeout_s:
                self.get_logger().info(
                    f"FOLLOW: lost target for {no_view_duration:.1f}s → switching back to PENTAGON."
                )
                self._start_pentagon()

    # -------------------- Process Management --------------------

    def _start_pentagon(self):
        self._spawn_behavior(['ros2', 'run', 'ros_behaviors_fsm', 'draw_pentagon'])
        self.mode = Mode.PENTAGON
        # reset detection history
        self._last_seen_ts = None
        self._has_target_now = False

    def _switch_to_follow(self):
        self._spawn_behavior(['ros2', 'run', 'ros_behaviors_fsm', 'person_follower'])
        self.mode = Mode.FOLLOW
        # Reset detection timer so FOLLOW gets a grace period to see the target
        self._last_seen_ts = time.monotonic() if self._has_target_now else None

    def _spawn_behavior(self, cmd):
        # Kill any existing child, then start new one
        self._kill_child()
        self.get_logger().info(f"Launching behavior: {' '.join(cmd)}")
        try:
            # Use a new process group so we can signal the whole tree on shutdown
            self.child_proc = subprocess.Popen(
                cmd,
                stdout=sys.stdout,
                stderr=sys.stderr,
                preexec_fn=os.setsid  # so we can SIGINT the group
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start behavior '{cmd}': {e}")
            self.child_proc = None

    def _kill_child(self):
        if self.child_proc is None:
            return
        if self.child_proc.poll() is not None:
            self.child_proc = None
            return

        try:
            self.get_logger().info("Stopping current behavior...")
            # Graceful stop: SIGINT to the whole process group (like Ctrl-C)
            os.killpg(os.getpgid(self.child_proc.pid), signal.SIGINT)
            # Wait a moment for clean shutdown
            waited = 0.0
            while self.child_proc.poll() is None and waited < 2.0:
                time.sleep(0.1)
                waited += 0.1

            # If still alive, escalate to SIGTERM
            if self.child_proc.poll() is None:
                self.get_logger().warn("Behavior not exiting; sending SIGTERM.")
                os.killpg(os.getpgid(self.child_proc.pid), signal.SIGTERM)

            # Final guard: if STILL alive after another short wait, SIGKILL
            waited2 = 0.0
            while self.child_proc.poll() is None and waited2 < 1.0:
                time.sleep(0.1)
                waited2 += 0.1
            if self.child_proc.poll() is None:
                self.get_logger().error("Force-killing behavior (SIGKILL).")
                os.killpg(os.getpgid(self.child_proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        finally:
            self.child_proc = None

    # -------------------- Utilities --------------------

    def _publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

    # Ensure child is stopped on node shutdown
    def destroy_node(self):
        try:
            self._kill_child()
            self._publish_stop()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
