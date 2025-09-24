from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="ros_behaviors_fsm",
                executable="finite_state_controller",
                name="finite_state_controller",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"fsm.start_state": "IDLE"},
                ],
            ),
            # Optional: bring up individual behaviors for testing
            # Node(package='ros_behaviors_fsm', executable='draw_pentagon', name='draw_pentagon', output='screen'),
            # Node(package='ros_behaviors_fsm', executable='spin_360', name='spin_360', output='screen'),
            # Node(package='ros_behaviors_fsm', executable='person_follower', name='person_follower', output='screen'),
        ]
    )
