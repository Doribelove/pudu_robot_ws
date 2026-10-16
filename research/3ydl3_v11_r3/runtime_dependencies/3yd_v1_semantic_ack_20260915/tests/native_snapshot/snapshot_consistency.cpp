#include <future>
#include <thread>
#include <iostream>
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
using namespace std::chrono_literals;
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto server = std::make_shared<nav2_util::LifecycleNode>("snapshot_probe", "", rclcpp::NodeOptions());
  auto client_node = std::make_shared<rclcpp::Node>("snapshot_client");
  nav2_costmap_2d::Costmap2D map(20, 20, .05, 0., 0., 253);
  nav2_costmap_2d::Costmap2DPublisher pub(server, &map, "map", "probe");
  auto client = client_node->create_client<nav2_msgs::srv::GetCostmap>("get_costmap");
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(server->get_node_base_interface()); exec.add_node(client_node);
  std::thread spinner([&]{exec.spin();});
  if (!client->wait_for_service(5s)) {exec.cancel();spinner.join();rclcpp::shutdown();return 2;}
  int early_reads = 0, wrong_final = 0;
  for (int trial=0; trial<10; ++trial) {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
    // Simulate StaticLayer intermediate content inside updateMap's mutex.
    std::fill(map.getCharMap(), map.getCharMap()+400, 76);
    auto future = client->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>());
    const bool early = future.wait_for(100ms)==std::future_status::ready;
    early_reads += early;
    std::fill(map.getCharMap(), map.getCharMap()+400, 253);
    lock.unlock();
    if (future.wait_for(2s)!=std::future_status::ready) {wrong_final++;continue;}
    const auto result = future.get();
    wrong_final += !std::all_of(result->map.data.begin(), result->map.data.end(), [](auto c){return c==253;});
  }
  exec.cancel(); spinner.join(); rclcpp::shutdown();
  std::cout << "{\"trials\":10,\"read_during_update\":" << early_reads << ",\"nonfinal_content\":" << wrong_final << "}" << std::endl;
  return (early_reads || wrong_final) ? 1 : 0;
}
