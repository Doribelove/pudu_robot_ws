"""Minimal static-map Nav2 stack used by planner_benchmark."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml", description="Hospital map YAML"),
        DeclareLaunchArgument("params_file", description="Generated Nav2 benchmark params"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
                "topic_name": "/map",
                "frame_id": "map",
            }],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="benchmark_map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="benchmark_odom_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planner",
            output="screen",
            parameters=[params_file, {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server", "planner_server"],
                "bond_timeout": 0.0,
            }],
        ),
    ])
