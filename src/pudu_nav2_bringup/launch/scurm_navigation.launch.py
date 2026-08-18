"""Launch the optional SCURM-inspired Nav2 profile on the PUDU 2WD model."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_share = get_package_share_directory("pudu_nav2_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    linorobot_share = get_package_share_directory("linorobot2_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    bt_xml = os.path.join(
        bringup_share, "behavior_trees", "scurm_navigation.xml")
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={"default_nav_to_pose_bt_xml": bt_xml},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use the Gazebo clock"),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                linorobot_share, "maps", "playground.yaml"),
            description="Map YAML passed to the Nav2 map server"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_share, "config", "scurm_nav2.yaml"),
            description="SCURM-inspired, differential-drive Nav2 profile"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start the Linorobot navigation RViz view"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                linorobot_share, "rviz", "linorobot2_navigation.rviz"),
            description="RViz configuration file"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "bringup_launch.py")),
            launch_arguments={
                "map": map_file,
                "use_sim_time": use_sim_time,
                "params_file": configured_params,
                "autostart": "true",
                "use_composition": "False",
                # Humble's bringup launch evaluates this value as Python.
                "slam": "False",
            }.items(),
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
