# Launch file for FSM node

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_ns = LaunchConfiguration("robot_ns")
    params = PathJoinSubstitution(
        [FindPackageShare("ros_behaviors_fsm"), "config", "params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("robot_ns", default_value="neato"),
            Node(
                package="ros_behaviors_fsm",
                executable="finite_state_controller",
                name="finite_state_controller",
                output="screen",
                parameters=[params, {"use_sim_time": use_sim_time}],
                remappings=[
                    # Remap scan and cmd_vel topics to robot namespace
                    ("scan", ["/", robot_ns, "/scan"]),
                    ("cmd_vel", ["/", robot_ns, "/cmd_vel"]),
                ],
                arguments=["--ros-args", "--log-level", "ros_behaviors_fsm:=debug"],
            ),
        ]
    )
