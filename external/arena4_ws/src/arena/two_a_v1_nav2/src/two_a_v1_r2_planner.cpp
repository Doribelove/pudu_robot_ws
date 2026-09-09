#include "two_a_v1_nav2/two_a_v1_r2_planner.hpp"

#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace two_a_v1_nav2
{
void TwoAV1R2Planner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string,
  std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  auto node = parent.lock();
  if (!node) {throw nav2_core::PlannerException("2A-V1-r2 parent expired");}
  client_node_ = std::make_shared<rclcpp::Node>(
    "two_a_v1_r2_planner_client",
    rclcpp::NodeOptions().context(node->get_node_base_interface()->get_context()));
  client_ = client_node_->create_client<nav_msgs::srv::GetPlan>("/two_a_v1/compute_plan");
}

void TwoAV1R2Planner::cleanup()
{
  active_ = false;
  client_.reset();
  client_node_.reset();
}

nav_msgs::msg::Path TwoAV1R2Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!active_ || !client_->wait_for_service(std::chrono::seconds(1))) {
    throw nav2_core::PlannerException("TWO_A_V1_R2_UNAVAILABLE");
  }
  auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
  request->start = start;
  request->goal = goal;
  auto future = client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(client_node_, future, std::chrono::seconds(15)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    client_->remove_pending_request(future);
    throw nav2_core::PlannerException("TWO_A_V1_R2_DEADLINE");
  }
  auto path = future.get()->plan;
  if (path.header.frame_id != "map" || path.poses.size() < 2) {
    throw nav2_core::PlannerException("TWO_A_V1_R2_REJECTED_SEE_LAYER_TRACE");
  }
  return path;
}
}  // namespace two_a_v1_nav2

PLUGINLIB_EXPORT_CLASS(two_a_v1_nav2::TwoAV1R2Planner, nav2_core::GlobalPlanner)
