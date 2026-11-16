"""Original Nav2 map/planner/lifecycle nodes; static TF lives in the worker."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
 m=LaunchConfiguration('map_yaml');p=LaunchConfiguration('params_file');sim=LaunchConfiguration('use_sim_time')
 return LaunchDescription([
  DeclareLaunchArgument('map_yaml'),DeclareLaunchArgument('params_file'),DeclareLaunchArgument('use_sim_time',default_value='false'),
  Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',parameters=[p,{'yaml_filename':m,'use_sim_time':sim,'topic_name':'/map','frame_id':'map'}]),
  Node(package='nav2_planner',executable='planner_server',name='planner_server',output='screen',parameters=[p,{'use_sim_time':sim}],remappings=[('/tf','tf'),('/tf_static','tf_static')]),
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_planner',output='screen',parameters=[p,{'use_sim_time':sim,'autostart':True,'node_names':['map_server','planner_server'],'bond_timeout':0.}])])
