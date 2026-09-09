"""Isolated Jackal + full Nav2 launch for frozen 2A-V1-r2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from pathlib import Path

from two_a_v1_nav2.contracts import verify_2a_run_inputs


def validate_inputs(context):
    verify_2a_run_inputs(Path(LaunchConfiguration("run_dir").perform(context)))
    output = Path(LaunchConfiguration("result_dir").perform(context))
    if output.exists() and any(p.name != "launch.log" for p in output.iterdir()):
        raise RuntimeError(f"refusing nonempty result directory: {output}")
    return []


def generate_launch_description():
    package = FindPackageShare("two_a_v1_nav2")
    harness_package = FindPackageShare("three_d_v1_nav2")
    nav2_bringup = FindPackageShare("nav2_bringup")
    arena_bringup = FindPackageShare("arena_bringup")
    arena_simulation = FindPackageShare("arena_simulation_setup")
    run_dir = LaunchConfiguration("run_dir")
    result_dir = LaunchConfiguration("result_dir")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    start_mission = LaunchConfiguration("start_mission")
    record_bag = LaunchConfiguration("record_bag")
    rviz = LaunchConfiguration("rviz")
    query_id = LaunchConfiguration("query_id")

    index = PathJoinSubstitution([run_dir, "path_bank", "index.csv"])
    map_yaml = PathJoinSubstitution([run_dir, "derived_map", "extracted", "optemap.yaml"])
    bt_xml = PathJoinSubstitution([package, "behavior_trees", "navigate_online_r2.xml"])
    rewritten_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites={
            "path_bank_index": index,
            "default_nav_to_pose_bt_xml": bt_xml,
            "default_nav_through_poses_bt_xml": bt_xml,
            "planner_server.ros__parameters.TwoAV1R2.plugin": "two_a_v1_nav2/TwoAV1R2Planner",
        },
        convert_types=True,
    )
    params = ParameterFile(rewritten_params, allow_substs=True)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            arena_bringup, "launch", "simulator", "sim", "gazebo", "gazebo.launch.py",
        ])),
        launch_arguments={
            "use_sim_time": use_sim_time, "headless": headless, "world": "",
        }.items(),
    )

    xacro = PathJoinSubstitution([
        arena_simulation, "entities", "robots", "jackal", "urdf", "jackal.urdf.xacro",
    ])
    description = Command(["xacro ", xacro, " name:=jackal is_sim:=true"])
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="jackal_robot_state_publisher", output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": ParameterValue(description, value_type=str),
            "frame_prefix": "jackal/",
        }],
    )
    spawn = Node(
        package="ros_gz_sim", executable="create", name="spawn_frozen_query_jackal",
        output="screen",
        arguments=[
            "-world", "default", "-topic", "robot_description", "-name", "jackal",
            "-allow_renaming", "false", "-x", "-25.800998999999997",
            "-y", "-33.633104", "-z", "0.20", "-Y", "1.6539375586833447",
        ],
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="jackal_gz_bridge",
        output="screen",
        arguments=[
            "/model/jackal/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/jackal/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/jackal/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/world/default/model/jackal/link/base_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/model/jackal/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        remappings=[
            ("/model/jackal/cmd_vel", "/cmd_vel_guarded"),
            ("/model/jackal/tf", "/tf"),
            ("/model/jackal/joint_states", "/joint_states"),
            ("/world/default/model/jackal/link/base_link/sensor/gpu_lidar/scan", "/scan"),
        ],
    )
    map_to_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="map_to_jackal_odom_truth", output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "jackal/odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen", parameters=[params, {"yaml_filename": map_yaml}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen", parameters=[params],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            nav2_bringup, "launch", "navigation_launch.py",
        ])),
        launch_arguments={
            "namespace": "", "use_sim_time": use_sim_time, "autostart": "true",
            "params_file": rewritten_params, "use_composition": "False", "use_respawn": "False",
            "log_level": "info",
        }.items(),
    )
    localization = Node(
        package="three_d_v1_nav2", executable="truth_localization",
        name="two_a_v1_truth_localization", output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    guard = Node(
        package="three_d_v1_nav2", executable="cmd_vel_guard",
        name="three_d_v1_cmd_vel_guard", output="screen",
        parameters=[{
            "use_sim_time": use_sim_time, "input_topic": "/cmd_vel",
            "output_topic": "/cmd_vel_guarded",
            "dynamic_stop_hold_speed": 0.001,
            "audit_csv": PathJoinSubstitution([result_dir, "cmd_vel_audit.csv"]),
        }],
    )
    online_bridge = Node(
        package="two_a_v1_nav2", executable="online_r2_bridge", output="screen",
        parameters=[{
            "use_sim_time": True, "run_dir": run_dir, "output_dir": result_dir,
            "l3_domain_id": ParameterValue(LaunchConfiguration("l3_domain_id"), value_type=int),
        }],
    )
    mission = Node(
        package="three_d_v1_nav2", executable="seq8_mission",
        name="two_a_v1_seq8_mission", output="screen",
        condition=IfCondition(start_mission),
        parameters=[{
            "use_sim_time": use_sim_time, "output_dir": result_dir,
            "path_bank_index": index, "query_id": query_id,
            "behavior_tree": bt_xml,
            "action_timeout_s": ParameterValue(LaunchConfiguration("action_timeout_s"), value_type=float),
            "online": True,
        }],
    )
    bag = ExecuteProcess(
        condition=IfCondition(record_bag), output="screen",
        cmd=[
            "ros2", "bag", "record", "-o", PathJoinSubstitution([result_dir, "rosbags", "nav2_seq8"]),
            "/tf", "/tf_static", "/map", "/scan", "/model/jackal/odometry",
            "/cmd_vel_nav", "/cmd_vel", "/cmd_vel_guarded", "/plan",
            "/local_plan", "/local_costmap/costmap", "/global_costmap/costmap",
            "/three_d_v1/global_layer_trace", "/three_d_v1/cmd_guard_trace",
            "/three_d_v1/localization_trace", "/three_d_v1/teb_trace",
            "/three_d_v1/mission_state",
            "/three_d_v1/online_audit", "/three_d_v1/dynamic_snapshot", "/three_d_v1/dynamic_stop",
            "/two_a_v1/global_layer_trace",
        ],
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        additional_env={"QT_QPA_PLATFORM_PLUGIN_PATH": "/usr/lib/x86_64-linux-gnu/qt5/plugins",
                        "QT_QPA_FONTDIR": "/usr/share/fonts"},
        condition=IfCondition(rviz),
        arguments=["-d", PathJoinSubstitution([harness_package, "rviz", "seq8.rviz"])],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    after_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[
            map_server, map_lifecycle, navigation, localization, guard, mission, bag, rviz_node,
        ],
    ))
    return LaunchDescription([
        DeclareLaunchArgument("run_dir", description="Fresh experiment output directory"),
        DeclareLaunchArgument(
            "result_dir", default_value=run_dir,
            description="Stage-specific result directory (path-bank input remains under run_dir)",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "params_file", default_value=PathJoinSubstitution([package, "config", "nav2_seq8.yaml"]),
            description="Nav2 configuration recorded for this experiment",
        ),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("start_mission", default_value="true"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("query_id", default_value=""),
        DeclareLaunchArgument("action_timeout_s", default_value="900.0"),
        DeclareLaunchArgument("l3_domain_id", default_value="221"),
        OpaqueFunction(function=validate_inputs),
        online_bridge, gazebo, state_publisher, bridge, map_to_odom, spawn, after_spawn,
    ])
