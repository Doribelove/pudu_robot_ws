#pragma once

#include <memory>
#include <string>
#include "nav2_core/global_planner.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "rclcpp/rclcpp.hpp"

namespace two_a_v1_nav2
{
class TwoAV1R2Planner : public nav2_core::GlobalPlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void cleanup() override;
  void activate() override {active_ = true;}
  void deactivate() override {active_ = false;}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
private:
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr client_;
  bool active_{false};
};
}  // namespace two_a_v1_nav2
