// Host the existing Nav2 node libraries with one executor per node. The full
// original parameter file is a process argument so the nested costmap receives
// its own parameters too. No planner/costmap implementation is replaced.
#include <csignal>
#include <chrono>
#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav2_map_server/map_server.hpp"
#include "nav2_planner/planner_server.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"

static volatile std::sig_atomic_t stopping = 0;
static void request_stop(int) { stopping = 1; }

int main(int argc, char **argv) {
  if (argc < 2) { std::cerr << "map YAML argument required\n"; return 2; }
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop); std::signal(SIGTERM, request_stop);
  rclcpp::NodeOptions map_options;
  map_options.append_parameter_override("yaml_filename", std::string(argv[1]));
  map_options.append_parameter_override("use_sim_time", false);
  map_options.append_parameter_override("topic_name", std::string("/map"));
  map_options.append_parameter_override("frame_id", std::string("map"));
  auto map = std::make_shared<nav2_map_server::MapServer>(map_options);
  auto planner = std::make_shared<nav2_planner::PlannerServer>(rclcpp::NodeOptions());
  rclcpp::NodeOptions manager_options;
  manager_options.arguments({"--ros-args", "-r", "__node:=lifecycle_manager_planner"});
  manager_options.append_parameter_override("use_sim_time", false);
  manager_options.append_parameter_override("autostart", true);
  manager_options.append_parameter_override("node_names", std::vector<std::string>{"map_server", "planner_server"});
  manager_options.append_parameter_override("bond_timeout", 0.0);
  auto manager = std::make_shared<nav2_lifecycle_manager::LifecycleManager>(manager_options);
  rclcpp::executors::SingleThreadedExecutor map_executor, planner_executor, manager_executor;
  map_executor.add_node(map->get_node_base_interface());
  planner_executor.add_node(planner->get_node_base_interface());
  manager_executor.add_node(manager->get_node_base_interface());
  std::thread map_thread([&]{ map_executor.spin(); });
  std::thread planner_thread([&]{ planner_executor.spin(); });
  std::thread manager_thread([&]{ manager_executor.spin(); });
  std::cout << "CURVE_NAV2_NODES_CREATED map_server planner_server lifecycle_manager_planner\n" << std::flush;
  while (!stopping && rclcpp::ok()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // Keep executors alive for the original lifecycle manager's pre-shutdown.
  rclcpp::shutdown();
  map_executor.cancel(); planner_executor.cancel(); manager_executor.cancel();
  map_thread.join(); planner_thread.join(); manager_thread.join();
  return 0;
}
