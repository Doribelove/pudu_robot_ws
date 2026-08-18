"""Attach known-map Boustrophedon coverage to an already running Nav2 stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("pudu_coverage_planner")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    goal_checker_id = LaunchConfiguration("goal_checker_id")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                package_share, "rviz", "area_coverage.rviz"),
        ),
        DeclareLaunchArgument(
            "goal_checker_id", default_value="general_goal_checker"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                package_share, "config", "area_coverage.yaml"),
        ),
        Node(
            package="pudu_coverage_planner",
            executable="area_coverage.py",
            name="area_coverage",
            output="screen",
            parameters=[params_file, {
                "use_sim_time": use_sim_time,
                "goal_checker_id": goal_checker_id,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
