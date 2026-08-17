"""Launch frontier exploration against a running Linorobot2 SLAM/Nav2 stack."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share = get_package_share_directory("pudu_coverage_planner")
    default_params = os.path.join(
        package_share, "config", "linorobot_frontier_exploration.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Frontier exploration parameter file",
            ),
            Node(
                package="explore_lite",
                executable="explore",
                name="explore_node",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
        ]
    )
