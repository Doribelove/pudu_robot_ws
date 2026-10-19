#ifndef THREE_D_V1_NAV2__FROZEN_R1_PLANNER_HPP_
#define THREE_D_V1_NAV2__FROZEN_R1_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace three_d_v1_nav2
{

struct FrozenPathRecord
{
  std::string query_id;
  double start_x{0.0};
  double start_y{0.0};
  double start_yaw{0.0};
  double goal_x{0.0};
  double goal_y{0.0};
  double goal_yaw{0.0};
  std::string path_file;
  std::string path_sha256;
  std::string canonical_path_hash;
  std::string route_edge_ids;
  std::string l2_binding_hash;
  std::string l2_backend;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
};

class FrozenR1PathPlanner : public nav2_core::GlobalPlanner
{
public:
  FrozenR1PathPlanner() = default;
  ~FrozenR1PathPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  void load_index(const std::string & path);
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("FrozenR1PathPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr trace_pub_;
  std::string name_;
  std::string global_frame_;
  double goal_position_tolerance_{0.02};
  double goal_yaw_tolerance_{0.02};
  double start_position_tolerance_{0.50};
  std::vector<FrozenPathRecord> records_;
  bool active_{false};
};

}  // namespace three_d_v1_nav2

#endif  // THREE_D_V1_NAV2__FROZEN_R1_PLANNER_HPP_
