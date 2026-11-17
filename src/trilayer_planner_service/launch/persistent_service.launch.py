# SPDX-License-Identifier: Apache-2.0
"""User-started long-running broker; no automatic restart or robot controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output',description='New short private output directory; must not exist'),
        DeclareLaunchArgument('domain_id',description='Reserved child DDS domain, distinct from host Nav2'),
        DeclareLaunchArgument('solver_root',description='Verified installed OSQP/QDLDL/Clarabel root'),
        DeclareLaunchArgument('request_timeout',default_value='55'),
        DeclareLaunchArgument('min_free_mib',default_value='512',description='Output filesystem free-space floor; minimum 512 MiB'),
        DeclareLaunchArgument('lifetime',default_value='0',description='Explicit persistent mode at 0; otherwise bounded 5..3600 seconds'),
        ExecuteProcess(cmd=[PathJoinSubstitution([FindPackagePrefix('trilayer_planner_service'),'lib','trilayer_planner_service','serve']),
            '--output',LaunchConfiguration('output'),'--domain-id',LaunchConfiguration('domain_id'),
            '--solver-root',LaunchConfiguration('solver_root'),'--request-timeout',LaunchConfiguration('request_timeout'),
            '--min-free-mib',LaunchConfiguration('min_free_mib'),'--lifetime',LaunchConfiguration('lifetime')],output='screen',respawn=False,sigterm_timeout='25',sigkill_timeout='10'),
    ])
