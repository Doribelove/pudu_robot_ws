from __future__ import annotations
from pathlib import Path
from typing import Any, Dict
import yaml

def load_yaml(path: str | Path) -> Dict[str, Any]:
    return yaml.safe_load(Path(path).read_text()) or {}

def stack_parameters(*, protocol: Dict[str, Any], planner_config: Dict[str, Any]) -> Dict[str, Any]:
    footprint = protocol['footprint']
    footprint_string = '[' + ', '.join(('[%.6f, %.6f]' % (float(x), float(y)) for (x, y) in footprint)) + ']'
    variant = protocol.get('variants', {}).get(planner_config.get('config_variant', 'product'), {})
    planner_ros = dict(planner_config)
    planner_ros.pop('planner_id', None)
    planner_ros.pop('config_variant', None)
    planner_plugins = planner_ros.pop('planner_plugins', ['GridBased'])
    resolution = float(protocol['resolution'])
    width_cells = int(protocol.get('width_cells', protocol.get('map_width_cells', round(float(protocol.get('width_m', 80.0)) / resolution))))
    height_cells = int(protocol.get('height_cells', protocol.get('map_height_cells', round(float(protocol.get('height_m', 80.0)) / resolution))))
    if width_cells <= 0 or height_cells <= 0:
        raise ValueError('protocol must provide positive width_cells and height_cells')
    origin = protocol.get('origin', [-40.0, -40.0, 0.0])
    return {'map_server': {'ros__parameters': {'use_sim_time': False, 'yaml_filename': '', 'topic_name': '/map', 'frame_id': 'map'}}, 'planner_server': {'ros__parameters': {'use_sim_time': False, 'expected_planner_frequency': 20.0, 'planner_plugins': planner_plugins, **planner_ros}}, 'planner_server_rclcpp_node': {'ros__parameters': {'use_sim_time': False}}, 'global_costmap': {'global_costmap': {'ros__parameters': {'use_sim_time': False, 'global_frame': 'map', 'robot_base_frame': 'base_link', 'update_frequency': float(protocol.get('costmap_update_frequency', 1.0)), 'publish_frequency': float(protocol.get('costmap_publish_frequency', 1.0)), 'resolution': resolution, 'width': int(round(width_cells * resolution)), 'height': int(round(height_cells * resolution)), 'origin_x': float(origin[0]), 'origin_y': float(origin[1]), 'rolling_window': False, 'track_unknown_space': True, 'footprint': footprint_string, 'plugins': ['static_layer', 'inflation_layer'], 'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_topic': '/map', 'map_subscribe_transient_local': True, 'subscribe_to_updates': bool(protocol.get('static_layer_subscribe_to_updates', False))}, 'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer', 'inflation_radius': float(variant.get('inflation_radius', 0.25)), 'cost_scaling_factor': float(variant.get('cost_scaling_factor', 3.0))}, 'always_send_full_costmap': True}}}, 'global_costmap_client': {'ros__parameters': {'use_sim_time': False}}, 'global_costmap_rclcpp_node': {'ros__parameters': {'use_sim_time': False}}}
