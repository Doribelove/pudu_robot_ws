"""Run FAST-LIO mapping or ICP localization on the Gazebo PointCloud2 sensor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share = get_package_share_directory("pudu_nav2_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    linorobot_navigation_share = get_package_share_directory("linorobot2_navigation")

    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_path = LaunchConfiguration("map_path")
    map_output_path = LaunchConfiguration("map_output_path")
    navigation = LaunchConfiguration("navigation")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    localization = PythonExpression(["'", mode, "' == 'localization'"])
    navigation_localized = PythonExpression([
        "'", mode, "' == 'localization' and '", navigation, "' == 'true'",
    ])

    bt_xml = os.path.join(
        bringup_share, "behavior_trees", "scurm_navigation.xml")
    configured_nav2_params = RewrittenYaml(
        source_file=os.path.join(bringup_share, "config", "scurm_nav2.yaml"),
        root_key="",
        param_rewrites={"default_nav_to_pose_bt_xml": bt_xml},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="localization",
            choices=["mapping", "localization"]),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_share, "config", "scurm_fast_lio_sim_localization.yaml")),
        DeclareLaunchArgument(
            "map_path",
            default_value=os.path.join(
                bringup_share, "maps", "playground_3d.pcd")),
        DeclareLaunchArgument(
            "map_output_path",
            default_value="/tmp/scurm_playground_map.pcd"),
        DeclareLaunchArgument(
            "map_2d",
            default_value=os.path.join(
                linorobot_navigation_share, "maps", "playground.yaml")),
        DeclareLaunchArgument("navigation", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                linorobot_navigation_share,
                "rviz", "linorobot2_navigation.rviz")),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_z", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),

        Node(
            package="icp_relocalization",
            executable="transform_publisher",
            name="scurm_transform_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(localization),
        ),
        Node(
            package="icp_relocalization",
            executable="icp_node",
            name="scurm_icp_relocalization",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_z": LaunchConfiguration("initial_z"),
                "initial_a": LaunchConfiguration("initial_yaw"),
                "map_voxel_leaf_size": 0.15,
                "cloud_voxel_leaf_size": 0.12,
                "map_frame_id": "map",
                "solver_max_iter": 80,
                "max_correspondence_distance": 0.80,
                "RANSAC_outlier_rejection_threshold": 0.40,
                "map_path": map_path,
                "fitness_score_thre": 0.15,
                "converged_count_thre": 3,
                "pcl_type": "pointcloud2",
                # Transform scurm_lidar points to base_link before ICP.
                "source_to_base_x": 0.12,
                "source_to_base_y": 0.0,
                "source_to_base_z": 0.33,
                "source_to_base_roll": 0.0,
                "source_to_base_pitch": 0.0,
                "source_to_base_yaw": 0.0,
            }],
            remappings=[("/pointcloud2", "/scurm/lidar_points")],
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
                    "map_file_path": map_output_path,
                },
            ],
            remappings=[("/Odometry", "/odom")],
        ),
        Node(
            package="pudu_scurm_sim",
            executable="localization_error.py",
            name="scurm_localization_error",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(localization),
        ),

        # FAST-LIO needs the first IMU window and point cloud before it can
        # publish odom -> base_footprint. Starting Nav2/RViz immediately makes
        # their TF consumers report a disconnected robot model during this
        # normal initialization window. Keep the localization nodes immediate,
        # then bring up all consumers after the transform tree is available.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    parameters=[
                        configured_nav2_params,
                        {"yaml_filename": LaunchConfiguration("map_2d")},
                    ],
                    condition=IfCondition(navigation_localized),
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_scurm_map",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server"],
                    }],
                    condition=IfCondition(navigation_localized),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        nav2_share, "launch", "navigation_launch.py")),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "params_file": configured_nav2_params,
                        "autostart": "true",
                        "use_composition": "False",
                    }.items(),
                    condition=IfCondition(navigation_localized),
                ),
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                    condition=IfCondition(PythonExpression([
                        "'", navigation, "' == 'true' and '", rviz,
                        "' == 'true'",
                    ])),
                ),
            ],
        ),
    ])
