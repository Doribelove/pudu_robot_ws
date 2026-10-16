"""Original Nav2 nodes in one process, with their original parameters/topics."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    m=LaunchConfiguration('map_yaml');p=LaunchConfiguration('params_file')
    sim=LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'),DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        ComposableNodeContainer(name='curve_nav2_container',namespace='',
            package='rclcpp_components',executable='component_container_isolated',output='screen',
            parameters=[p],
            composable_node_descriptions=[
                ComposableNode(package='nav2_map_server',plugin='nav2_map_server::MapServer',
                    name='map_server',parameters=[p,{'yaml_filename':m,'use_sim_time':sim,
                    'topic_name':'/map','frame_id':'map'}]),
                ComposableNode(package='nav2_planner',plugin='nav2_planner::PlannerServer',
                    name='planner_server',parameters=[p,{'use_sim_time':sim}],
                    remappings=[('/tf','tf'),('/tf_static','tf_static')]),
                ComposableNode(package='nav2_lifecycle_manager',plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_planner',parameters=[p,{'use_sim_time':sim,
                    'autostart':True,'node_names':['map_server','planner_server'],'bond_timeout':0.}]),
            ])])
