"""Launch the static Stage 5 planner baseline with a configured RViz view."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("arena_evaluation")
    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    queries_file = LaunchConfiguration("queries_file")
    query_id = LaunchConfiguration("query_id")
    cycle_interval = LaunchConfiguration("cycle_interval")
    planner_label = LaunchConfiguration("planner_label")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    visualization_mode = LaunchConfiguration("visualization_mode")
    stage8_directory = LaunchConfiguration("stage8_directory")
    stage6_directory = LaunchConfiguration("stage6_directory")
    topology_directory = LaunchConfiguration("topology_directory")
    stage_label = LaunchConfiguration("stage_label")

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{share}/launch/planner_benchmark_stack.launch.py"
        ),
        launch_arguments={
            "map_yaml": map_yaml,
            "params_file": params_file,
            "use_sim_time": "false",
        }.items(),
    )
    visualizer = Node(
        package="arena_evaluation",
        executable="baseline_path_visualizer",
        name="baseline_path_visualizer",
        output="screen",
        parameters=[{
            "queries_file": queries_file,
            "query_id": query_id,
            "cycle_interval_seconds": cycle_interval,
            "planner_label": planner_label,
            "action_timeout_seconds": 30.0,
        }],
        condition=UnlessCondition(
            PythonExpression(["'", visualization_mode, "' == 'layered'"])
        ),
    )
    layered_visualizer = Node(
        package="arena_evaluation",
        executable="layered_path_visualizer",
        name="layered_path_visualizer",
        output="screen",
        parameters=[{
            "queries_file": queries_file,
            "query_id": query_id,
            "cycle_interval_seconds": cycle_interval,
            "stage8_directory": stage8_directory,
            "stage6_directory": stage6_directory,
            "topology_directory": topology_directory,
            "stage_label": stage_label,
        }],
        condition=IfCondition(
            PythonExpression(["'", visualization_mode, "' == 'layered'"])
        ),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="baseline_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
        on_exit=Shutdown(reason="RViz closed"),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("queries_file"),
        DeclareLaunchArgument("query_id", default_value="all"),
        DeclareLaunchArgument("cycle_interval", default_value="1.0"),
        DeclareLaunchArgument("planner_label", default_value="baseline"),
        DeclareLaunchArgument("visualization_mode", default_value="baseline"),
        DeclareLaunchArgument("stage8_directory", default_value=""),
        DeclareLaunchArgument("stage6_directory", default_value=""),
        DeclareLaunchArgument("topology_directory", default_value=""),
        DeclareLaunchArgument("stage_label", default_value="Stage 8A layered replay"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config", default_value=f"{share}/rviz/planner_baseline.rviz"
        ),
        stack,
        visualizer,
        layered_visualizer,
        rviz,
    ])
