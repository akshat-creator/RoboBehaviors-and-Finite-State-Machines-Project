"""
Finite-State Supervisor for Behavior Switching (+ periodic 360° spin during FOLLOW)
===================================================================================

This module implements a ROS 2 node (`BehaviorFSM`) that supervises and switches between two
pre-existing behaviors that are launched as **subprocesses**:

- `draw_pentagon`  – time-based open-loop driving that traces a pentagon.
- `person_follower` – closed-loop following using `sensor_msgs/LaserScan`.

While the FOLLOW behavior is active, the supervisor periodically pauses FOLLOW, runs a
separate `spin_360` node to perform a full rotation, and then resumes FOLLOW. This spin
is scheduled on a timer-like background thread and obeys a configurable interval.

The supervisor also supports **real robot** and **Gazebo** bumper inputs:

- Gazebo: subscribes to `/bumper` (`gazebo_msgs/ContactsState`).
- Real hardware: dynamically discovers and subscribes to `/bump` (arbitrary message type),
  treating any non-zero/true field (e.g., `left_front`, `left_side`, `right_front`, `right_side`)
  as a bump. Dynamic message import avoids compile-time dependencies on a specific HW message.

Console entry points assumed to exist (provided by your package setup):
  'person_follower = ros_behaviors_fsm.person_follower:main'
  'draw_pentagon   = ros_behaviors_fsm.draw_pentagon:main'
  'spin_360        = ros_behaviors_fsm.spin_360:main'
"""

import math
import os
import signal
import subprocess
import sys
import time
from enum import Enum
from threading import Thread, Event, Lock

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ContactsState

# For dynamic message loading & safe dict conversion (real robot /bump)
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict


class Mode(Enum):
    """Behavior modes managed by the supervisor."""
    PENTAGON = 0   # Actively running the draw_pentagon behavior
    FOLLOW = 1     # Actively running the person_follower behavior


class BehaviorFSM(Node):
    """Finite-state supervisor for behavior switching and scheduled spins."""

    def __init__(self):
        super().__init__('finite_state_controller')

        # ---- Parameters (declarations + reads)
        self.declare_parameter('object_present_threshold_m', 1.5)  # LaserScan threshold for target presence (meters)
        self.declare_parameter('lost_object_timeout_s', 8.0)       # FOLLOW → PENTAGON fallback timeout (seconds)
        self.declare_parameter('poll_rate_hz', 10.0)               # _tick rate (Hz)

        # Bumper debounce/cooldown parameters
        self.declare_parameter('bumper_cooldown_s', 0.5)
        self.declare_parameter('bumper_min_msgs', 1)

        # Real robot bump topic and Gazebo topic
        self.declare_parameter('bump_topic', '/bump')             # neato
        self.declare_parameter('gazebo_bumper_topic', 'bumper')   # gazebo

        # Spin controls (FOLLOW mode)
        self.declare_parameter('spin_interval_s', 15.0)
        self.declare_parameter('spin_pkg', 'ros_behaviors_fsm')
        self.declare_parameter('spin_exec', 'spin_360')

        # Resolve parameter values
        self.object_present_threshold_m = float(self.get_parameter('object_present_threshold_m').value)
        self.lost_object_timeout_s = float(self.get_parameter('lost_object_timeout_s').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.poll_dt = 1.0 / max(self.poll_rate_hz, 1.0)

        self._bumper_cooldown_s = float(self.get_parameter('bumper_cooldown_s').value)
        self._bumper_min_msgs = int(self.get_parameter('bumper_min_msgs').value)

        self._spin_interval_s = float(self.get_parameter('spin_interval_s').value)
        self._spin_pkg = str(self.get_parameter('spin_pkg').value)
        self._spin_exec = str(self.get_parameter('spin_exec').value)

        self._bump_topic = str(self.get_parameter('bump_topic').value)
        self._gazebo_bumper_topic = str(self.get_parameter('gazebo_bumper_topic').value)

        # ---- Publishers / Subscribers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)

        # Gazebo bumper (ContactsState): any contact ⇒ bump
        self.create_subscription(ContactsState, self._gazebo_bumper_topic, self._on_bumper_contacts, 10)

        # Real robot bump subscriber: resolve & subscribe dynamically (retries)
        self._bump_sub_dyn = None
        self.create_timer(1.0, self._ensure_bump_subscription)  # retry every second until success

        # ---- Internal state
        self.mode = Mode.PENTAGON          # Start in PENTAGON behavior
        self.child_proc = None             # Handle to the current behavior subprocess

        # LaserScan bookkeeping
        self._last_seen_ts = None          # monotonic timestamp when target last seen
        self._has_target_now = False       # last "instantaneous" detection result

        # Bumper debounce/cooldown tracking
        self._bumper_contact_streak = 0
        self._last_bumper_switch_ts = 0.0

        # Spin scheduler state
        self._last_spin_ts = 0.0
        self._spin_thread = Thread(target=self._spin_scheduler_loop, daemon=True)
        self._spin_in_progress = Event()
        self._proc_lock = Lock()           # guard child_proc mutation between threads

        # Start default behavior
        self._start_pentagon()

        # Timer to evaluate FOLLOW → PENTAGON fallback
        self.create_timer(self.poll_dt, self._tick)

        # Kick off spin scheduler
        self._spin_thread.start()

        self.get_logger().info(
            f"BehaviorFSM ready. scan_threshold={self.object_present_threshold_m:.2f} m, "
            f"lost_timeout={self.lost_object_timeout_s:.1f} s, "
            f"bumper_min_msgs={self._bumper_min_msgs}, bumper_cooldown={self._bumper_cooldown_s:.2f}s, "
            f"spin_interval={self._spin_interval_s:.1f}s, "
            f"gazebo_bumper_topic='{self._gazebo_bumper_topic}', bump_topic='{self._bump_topic}'"
        )

    # -------------------- Dynamic /bump subscription --------------------

    def _ensure_bump_subscription(self):
        if self._bump_sub_dyn is not None or not rclpy.ok():
            return
        try:
            msg_type_str = subprocess.check_output(
                ['ros2', 'topic', 'type', self._bump_topic],
                text=True, stderr=subprocess.STDOUT
            ).strip()
            if not msg_type_str:
                return
            MsgT = get_message(msg_type_str)
            self._bump_sub_dyn = self.create_subscription(MsgT, self._bump_topic, self._on_bump_generic, 10)
            self.get_logger().info(f"Subscribed to '{self._bump_topic}' (type: {msg_type_str}).")
        except subprocess.CalledProcessError:
            pass
        except Exception as e:
            self.get_logger().warn(f"Waiting for bump topic '{self._bump_topic}': {e}")

    def _on_bump_generic(self, msg):
        try:
            d = message_to_ordereddict(msg)
        except Exception:
            d = None

        pressed = False
        if isinstance(d, dict):
            preferred = ('left_front', 'left_side', 'right_front', 'right_side')
            any_preferred = any(k in d for k in preferred)
            keys = preferred if any_preferred else tuple(d.keys())

            for k in keys:
                v = d.get(k, 0)
                if isinstance(v, bool):
                    pressed = pressed or v
                else:
                    try:
                        pressed = pressed or (int(v) != 0)
                    except Exception:
                        pass
        else:
            for k in ('left_front', 'left_side', 'right_front', 'right_side', 'left', 'right', 'front'):
                if hasattr(msg, k):
                    v = getattr(msg, k)
                    if isinstance(v, bool) and v:
                        pressed = True
                        break
                    try:
                        if int(v) != 0:
                            pressed = True
                            break
                    except Exception:
                        pass

        self._register_bump_event(pressed)

    # -------------------- Gazebo ContactsState path --------------------

    def _on_bumper_contacts(self, msg: ContactsState):
        has_contact = len(msg.states) > 0
        self._register_bump_event(has_contact)

    # -------------------- Unified bump handling --------------------

    def _register_bump_event(self, pressed: bool):
        now = time.monotonic()

        if pressed:
            self._bumper_contact_streak += 1
        else:
            self._bumper_contact_streak = 0

        should_switch = (
            pressed
            and self._bumper_contact_streak >= self._bumper_min_msgs
            and (now - self._last_bumper_switch_ts) >= self._bumper_cooldown_s
            and self.mode != Mode.FOLLOW
        )

        if should_switch:
            self.get_logger().info("Bump detected → switching to FOLLOW.")
            self._last_bumper_switch_ts = now
            self._switch_to_follow()

    def _on_scan(self, msg: LaserScan):
        thr = self.object_present_threshold_m
        has_target = any(
            (d is not None) and (not math.isinf(d)) and (not math.isnan(d)) and (d > 0.0) and (d <= thr)
            for d in msg.ranges
        )
        self._has_target_now = has_target
        if has_target:
            self._last_seen_ts = time.monotonic()

    # -------------------- Timer: FOLLOW fallback --------------------

    def _tick(self):
        if self.mode == Mode.FOLLOW:
            now = time.monotonic()
            no_view_duration = float('inf') if self._last_seen_ts is None else (now - self._last_seen_ts)
            if no_view_duration > self.lost_object_timeout_s and not self._spin_in_progress.is_set():
                self.get_logger().info(
                    f"FOLLOW: lost target for {no_view_duration:.1f}s → switching back to PENTAGON."
                )
                self._start_pentagon()

    # -------------------- Spin scheduler --------------------

    def _spin_scheduler_loop(self):
        while rclpy.ok():
            time.sleep(0.1)
            if self.mode != Mode.FOLLOW:
                continue
            now = time.monotonic()
            if (now - self._last_spin_ts) >= self._spin_interval_s and not self._spin_in_progress.is_set():
                self._perform_spin_once()

    def _perform_spin_once(self):
        if self.mode != Mode.FOLLOW:
            return

        self._spin_in_progress.set()
        self.get_logger().info("FOLLOW active → scheduling 360° spin.")

        with self._proc_lock:
            self._kill_child()

        spin_cmd = ['ros2', 'run', self._spin_pkg, self._spin_exec]
        self.get_logger().info(f"Running spin: {' '.join(spin_cmd)}")
        try:
            spin_proc = subprocess.Popen(
                spin_cmd, stdout=sys.stdout, stderr=sys.stderr, preexec_fn=os.setsid
            )

            while spin_proc.poll() is None:
                # Wait indefinitely until the spin process exits on its own.
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"Failed to launch spin: {e}")
        # Always resume FOLLOW after completing the spin
        self.get_logger().info("Resuming FOLLOW after spin.")
        self._spawn_behavior(['ros2', 'run', 'ros_behaviors_fsm', 'person_follower'])
        self.mode = Mode.FOLLOW
        self._last_seen_ts = time.monotonic() if self._has_target_now else None

        self._last_spin_ts = time.monotonic()
        self._spin_in_progress.clear()

    # -------------------- Process Management --------------------

    def _start_pentagon(self):
        with self._proc_lock:
            self._spawn_behavior(['ros2', 'run', 'ros_behaviors_fsm', 'draw_pentagon'])
            self.mode = Mode.PENTAGON
            self._last_seen_ts = None
            self._has_target_now = False

    def _switch_to_follow(self):
        with self._proc_lock:
            self._spawn_behavior(['ros2', 'run', 'ros_behaviors_fsm', 'person_follower'])
            self.mode = Mode.FOLLOW
            self._last_seen_ts = time.monotonic() if self._has_target_now else None
            self._last_spin_ts = time.monotonic()

    def _spawn_behavior(self, cmd):
        self._kill_child()
        self.get_logger().info(f"Launching behavior: {' '.join(cmd)}")
        try:
            self.child_proc = subprocess.Popen(
                cmd, stdout=sys.stdout, stderr=sys.stderr, preexec_fn=os.setsid
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
            os.killpg(os.getpgid(self.child_proc.pid), signal.SIGINT)
            waited = 0.0
            while self.child_proc.poll() is None and waited < 2.0:
                time.sleep(0.1)
                waited += 0.1
            if self.child_proc.poll() is None:
                self.get_logger().warn("Behavior not exiting; sending SIGTERM.")
                os.killpg(os.getpgid(self.child_proc.pid), signal.SIGTERM)
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
