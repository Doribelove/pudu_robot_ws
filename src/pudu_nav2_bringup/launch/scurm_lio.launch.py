"""Run the SCURM FAST-LIO pipeline as an isolated, optional hardware feature."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sentry_share = get_package_share_directory("sentry_bringup")
    livox_share = get_package_share_directory("livox_ros_driver2")

    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_livox = LaunchConfiguration("start_livox")
    params_file = LaunchConfiguration("params_file")
    map_path = LaunchConfiguration("map_path")

    localization = PythonExpression(["'", mode, "' == 'localization'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="mapping",
            choices=["mapping", "localization"],
            description="FAST-LIO operating mode"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "start_livox", default_value="true",
            description="Start the bundled Livox MID360 ROS driver"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                sentry_share, "params", "fast_lio_mapping_param.yaml"),
            description=(
                "FAST-LIO YAML. For localization, pass the upstream "
                "fast_lio_relocalization_param.yaml or a robot-specific copy.")),
        DeclareLaunchArgument(
            "map_path",
            default_value=os.path.join(sentry_share, "maps", "GlobalMap.pcd"),
            description="Prior PCD map used in localization mode"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                livox_share, "launch_ROS2", "msg_MID360_launch.py")),
            condition=IfCondition(start_livox),
        ),
        Node(
            package="icp_relocalization",
            executable="transform_publisher",
            name="scurm_transform_publisher",
            output="screen",
            condition=IfCondition(localization),
        ),
        Node(
            package="icp_relocalization",
            executable="icp_node",
            name="scurm_icp_relocalization",
            output="screen",
            parameters=[{
                "initial_x": 0.0,
                "initial_y": 0.0,
                "initial_z": 0.0,
                "initial_a": 0.0,
                "map_voxel_leaf_size": 0.5,
                "cloud_voxel_leaf_size": 0.3,
                "map_frame_id": "map",
                "solver_max_iter": 100,
                "max_correspondence_distance": 0.1,
                "RANSAC_outlier_rejection_threshold": 0.5,
                "map_path": map_path,
                "fitness_score_thre": 0.9,
                "converged_count_thre": 40,
                "pcl_type": "livox",
                "use_sim_time": use_sim_time,
            }],
            condition=IfCondition(localization),
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="scurm_fast_lio",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": use_sim_time,
                    "locate_in_prior_map": ParameterValue(
                        localization, value_type=bool),
                    "prior_map_path": map_path,
                },
            ],
            remappings=[("/Odometry", "/state_estimation")],
        ),
    ])
