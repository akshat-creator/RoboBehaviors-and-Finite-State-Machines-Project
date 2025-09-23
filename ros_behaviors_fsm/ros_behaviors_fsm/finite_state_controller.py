"""
Finite-State Supervisor for Behavior Switching and 360 Degree Spin
===================================================================================

This FSM implements a ROS 2 node (`BehaviorFSM`) that supervises and switches between two
pre-existing behaviors that are launched as **subprocesses**:

- `draw_pentagon`  – time-based open-loop driving that traces a pentagon.
- `person_follower` – closed-loop following using `sensor_msgs/LaserScan`.

While the FOLLOW behavior is active, the supervisor periodically pauses FOLLOW, runs a
separate `spin_360` node to perform a full rotation, and then resumes FOLLOW. This spin
is scheduled on a timer-like background thread and obeys a configurable interval.

Console entry points assumed to exist:
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

# For dynamic message loading & safe dict conversion (/bump)
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict


class Mode(Enum):
    """Behavior modes"""
    PENTAGON = 0   # Actively running the draw_pentagon behavior
    FOLLOW = 1     # Actively running the person_follower behavior


class BehaviorFSM(Node):
    """Finite-state for behavior switching and scheduled spins."""

    def __init__(self):
        super().__init__('finite_state_controller')

        # Declare parameters
        self.declare_parameter('object_present_threshold_m', 1.5)  # LaserScan threshold for target presence (meters)
        self.declare_parameter('lost_object_timeout_s', 8.0)       # FOLLOW → PENTAGON fallback timeout (seconds)
        self.declare_parameter('poll_rate_hz', 10.0)               # tick rate (Hz): how often the FSM checks for mode switches
        self.declare_parameter('bumper_cooldown_s', 0.5)           # min time between bump events (seconds)
        self.declare_parameter('bump_topic', '/bump')             # neato
        self.declare_parameter('gazebo_bumper_topic', 'bumper')   # gazebo
        self.declare_parameter('spin_interval_s', 15.0)           # min time between 360 spins (seconds)
        self.declare_parameter('spin_pkg', 'ros_behaviors_fsm')
        self.declare_parameter('spin_exec', 'spin_360')

        # Resolve parameter values
        self.object_present_threshold_m = float(self.get_parameter('object_present_threshold_m').value) 
        self.lost_object_timeout_s = float(self.get_parameter('lost_object_timeout_s').value)
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.poll_dt = 1.0 / max(self.poll_rate_hz, 1.0)  # polling interval (seconds) ticks
        self._bumper_cooldown_s = float(self.get_parameter('bumper_cooldown_s').value)
        self._bump_topic = str(self.get_parameter('bump_topic').value)
        self._gazebo_bumper_topic = str(self.get_parameter('gazebo_bumper_topic').value)
        self._spin_interval_s = float(self.get_parameter('spin_interval_s').value)
        self._spin_pkg = str(self.get_parameter('spin_pkg').value)
        self._spin_exec = str(self.get_parameter('spin_exec').value)

        # Publishers / Subscribers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_subscription(ContactsState, self._gazebo_bumper_topic, self._on_bumper_contacts, 10)

        # Real robot bump subscriber: resolve & subscribe dynamically (retries)
        self._bump_sub = None
        self.create_timer(1.0, self._ensure_bump_subscription)  # subscribe to the bump topic, retry until it succeeds
        self.mode = Mode.PENTAGON         
        self.child_proc = None             # Stores the running process for the current behavior

        self._last_seen_ts = None          # how long the target has been out of view.
        self._has_target_now = False       # whether the target is currently detected

        self._bumper_contact_streak = 0   # counts consecutive bump detections
        self._last_bumper_switch_ts = 0.0 # time when last bump-triggered switch happened

        # Spin scheduler state
        self._last_spin_ts = 0.0
        self._spin_thread = Thread(target=self._spin_scheduler_loop, daemon=True)
        self._spin_in_progress = Event()
        self._proc_lock = Lock()  # ensures only one thread can start/stop the behavior process at a time

        self._start_pentagon()

        # Timer to evaluate FOLLOW → PENTAGON fallback
        self.create_timer(self.poll_dt, self._tick)

        self._spin_thread.start()

        self.get_logger().info(
            f"BehaviorFSM ready. scan_threshold={self.object_present_threshold_m:.2f} m, "
            f"lost_timeout={self.lost_object_timeout_s:.1f} s, "
            f"bumper_cooldown={self._bumper_cooldown_s:.2f}s, "
            f"spin_interval={self._spin_interval_s:.1f}s, "
            f"gazebo_bumper_topic='{self._gazebo_bumper_topic}', bump_topic='{self._bump_topic}'"
        )


    def _ensure_bump_subscription(self):
        """ (Co-Pilot was used to write this docstring)
        Try to subscribe to the bump sensor topic for the neato.
        This function is called periodically until the subscription succeeds.
        It dynamically determines the message type of the bump topic and creates the subscriber.
        If the topic is not available yet, it will keep retrying every second.
        """
        try:
            msg_type_str = subprocess.check_output(
                ['ros2', 'topic', 'type', self._bump_topic],
                text=True, stderr=subprocess.STDOUT
            ).strip()
            if not msg_type_str:
                return
            MsgT = get_message(msg_type_str)
            self._bump_sub = self.create_subscription(MsgT, self._bump_topic, self._on_bump_generic, 10)
            self.get_logger().info(f"Subscribed to '{self._bump_topic}' (type: {msg_type_str}).")
        except subprocess.CalledProcessError:
            pass
        except Exception as e:
            self.get_logger().warn(f"Waiting for bump topic '{self._bump_topic}': {e}")

    def _on_bump_generic(self, msg):
        """
        Converts the message to a dictionary and checks for any pressed bumpers.
        Handles both boolean and integer fields (was trying to code this without access to the neato).
        If any bumper is pressed, triggers the bump event logic.
        """
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

        self._register_bump_event(pressed)

    def _on_bumper_contacts(self, msg: ContactsState):
        """
        Checks if there is any contact in the message and triggers the bump event logic if so.
        Used to detect bumps in Gazebo.
        """
        has_contact = len(msg.states) > 0
        self._register_bump_event(has_contact)


    def _register_bump_event(self, pressed: bool):
        """ (Co-Pilot was used to write this docstring)
        Handle bump (collision) events from neato or simulation.

        This function tracks consecutive bump detections and enforces a cooldown period between mode switches.
        When a valid bump is detected and the cooldown has elapsed, the FSM switches the robot to FOLLOW mode.

        The switch to FOLLOW mode occurs only if all of the following are true:
            - A bump is currently detected (pressed is True)
            - There is at least one consecutive bump event (contact streak >= 1)
            - Enough time has passed since the last bump-triggered switch (cooldown period)
            - The robot is not already in FOLLOW mode

        Args:
            pressed (bool): True if a bump is currently detected, False otherwise.
        """
        now = time.monotonic()

        if pressed:
            self._bumper_contact_streak += 1
        else:
            self._bumper_contact_streak = 0

        should_switch = (
            pressed
            and self._bumper_contact_streak >= 1
            and (now - self._last_bumper_switch_ts) >= self._bumper_cooldown_s
            and self.mode != Mode.FOLLOW
        )

        if should_switch:
            self.get_logger().info("Bump detected → switching to FOLLOW.")
            self._last_bumper_switch_ts = now
            self._switch_to_follow()

    def _on_scan(self, msg: LaserScan):
        """ (Co-Pilot was used to write this docstring)
        Function for LaserScan messages to detect if a target object is present within a threshold distance.

        Scans all valid range readings in the incoming LaserScan message. If any reading is within the
        configured threshold (object_present_threshold_m), sets the internal flag to indicate a target is present
        and updates the timestamp for when the target was last seen. This is used by the FSM to decide when to
        switch between FOLLOW and PENTAGON modes.

        Args:
            msg (LaserScan): The incoming LaserScan message containing range data.
        """
        thr = self.object_present_threshold_m
        has_target = any(
            (d is not None) and (not math.isinf(d)) and (not math.isnan(d)) and (d > 0.0) and (d <= thr)
            for d in msg.ranges
        )
        self._has_target_now = has_target
        if has_target:
            self._last_seen_ts = time.monotonic()

    def _tick(self):
        """
        Periodic timer callback to check if the FOLLOW mode should be exited due to target loss.

        If the FSM is in FOLLOW mode, this function checks how long it has been since the target was last seen.
        If the target has been lost for longer than the configured timeout (lost_object_timeout_s), and a spin
        is not currently in progress, the FSM switches back to PENTAGON mode. This ensures the robot does not
        continue following when the target is no longer detected.
        """
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
        """ (Co-Pilot was used to write this docstring)
        Background thread loop that schedules periodic 360° spins while in FOLLOW mode.

        This loop runs continuously as long as the ROS node is active. Every 0.1 seconds, it checks if the FSM
        is in FOLLOW mode. If so, and if enough time has passed since the last spin (spin_interval_s), and no
        spin is currently in progress, it triggers a new spin by calling _perform_spin_once().

        This ensures the robot periodically pauses following, performs a 360° spin, and then resumes following.
        """
        while rclpy.ok():
            time.sleep(0.1)
            if self.mode != Mode.FOLLOW:
                continue
            now = time.monotonic()
            if (now - self._last_spin_ts) >= self._spin_interval_s and not self._spin_in_progress.is_set():
                self._perform_spin_once()

    def _perform_spin_once(self):
        """ (Co-Pilot was used to write this docstring)
        Perform a single 360° spin while in FOLLOW mode, then resume following.

        This method is called by the spin scheduler when it is time to perform a spin. It:
            - Ensures the FSM is in FOLLOW mode.
            - Sets a flag to indicate a spin is in progress.
            - Stops the current behavior process (if any) to avoid conflicts.
            - Launches the configured spin node as a subprocess and waits for it to finish.
            - After the spin completes, resumes the person_follower behavior and updates relevant state.
            - Clears the spin-in-progress flag so future spins can be scheduled.

        If launching the spin process fails, an error is logged and the FSM attempts to recover by resuming FOLLOW.
        """
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
                # Wait until the spin process exits
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"Failed to launch spin: {e}")
        # Resume FOLLOW after completing the spin
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
        """ (Co-Pilot was used to write this docstring)
        Start a new subprocess to run the specified robot behavior.

        This method first stops any currently running behavior subprocess to ensure only one behavior runs at a time.
        It then launches the new behavior as a subprocess using the provided command list. 
        If launching the subprocess fails, it logs an error and clears the process handle.

        Args:
            cmd (list of str): The command and arguments to launch the new behavior as a subprocess.
        """
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
            time.sleep(0.2)
        finally:
            self.child_proc = None

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
