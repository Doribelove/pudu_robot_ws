"""Start Gazebo with the simulation-only 3D lidar/IMU SCURM robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("pudu_robot_description")
    gazebo_share = get_package_share_directory("linorobot2_gazebo")

    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf = LaunchConfiguration("urdf")
    world = LaunchConfiguration("world")
    lidar_horizontal_samples = LaunchConfiguration("lidar_horizontal_samples")
    robot_description = ParameterValue(Command([
        "xacro ", urdf,
        " lidar_horizontal_samples:=", lidar_horizontal_samples,
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("lidar_horizontal_samples", default_value="360"),
        DeclareLaunchArgument(
            "urdf",
            default_value=os.path.join(
                description_share, "urdf", "scurm_sim_2wd.urdf.xacro")),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(gazebo_share, "worlds", "playground.world")),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),
        ExecuteProcess(
            cmd=[
                "gazebo", "--verbose",
                "-s", "libgazebo_ros_factory.so",
                "-s", "libgazebo_ros_init.so",
                world,
            ],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="scurm_robot_spawner",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-entity", "linorobot2",
                "-x", LaunchConfiguration("spawn_x"),
                "-y", LaunchConfiguration("spawn_y"),
                "-z", LaunchConfiguration("spawn_z"),
                "-Y", LaunchConfiguration("spawn_yaw"),
            ],
        ),
        Node(
            package="linorobot2_gazebo",
            executable="command_timeout.py",
            name="command_timeout",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
