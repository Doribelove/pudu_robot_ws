#include "pudu_nav2_plugins/back_up_to_free_space.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_value.hpp"
#include "tf2/utils.h"

namespace pudu_nav2_plugins
{

void BackUpToFreeSpace::onConfigure()
{
  nav2_behaviors::DriveOnHeading<Action>::onConfigure();

  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock behavior server node");
  }

  const auto prefix = this->behavior_name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "allow_direction_change", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "angular_samples", rclcpp::ParameterValue(7));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "max_angular_velocity", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "search_distance", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "simulation_step", rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "minimum_free_distance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "direction_change_penalty", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "turning_penalty", rclcpp::ParameterValue(0.05));

  node->get_parameter(prefix + "allow_direction_change", allow_direction_change_);
  node->get_parameter(prefix + "angular_samples", angular_samples_);
  node->get_parameter(prefix + "max_angular_velocity", max_angular_velocity_);
  node->get_parameter(prefix + "search_distance", search_distance_);
  node->get_parameter(prefix + "simulation_step", simulation_step_);
  node->get_parameter(prefix + "minimum_free_distance", minimum_free_distance_);
  node->get_parameter(prefix + "direction_change_penalty", direction_change_penalty_);
  node->get_parameter(prefix + "turning_penalty", turning_penalty_);

  angular_samples_ = std::max(1, angular_samples_);
  if (angular_samples_ > 1 && angular_samples_ % 2 == 0) {
    ++angular_samples_;
  }
  max_angular_velocity_ = std::max(0.0, max_angular_velocity_);
  search_distance_ = std::max(0.05, search_distance_);
  simulation_step_ = std::clamp(simulation_step_, 0.01, 0.1);
  minimum_free_distance_ = std::max(simulation_step_, minimum_free_distance_);
  direction_change_penalty_ = std::max(0.0, direction_change_penalty_);
  turning_penalty_ = std::max(0.0, turning_penalty_);
}

BackUpToFreeSpace::Status BackUpToFreeSpace::onRun(
  const std::shared_ptr<const Action::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_ERROR(this->logger_, "BackUpToFreeSpace only supports planar X motion");
    return Status::FAILED;
  }

  target_distance_ = std::abs(command->target.x);
  const double speed = std::abs(command->speed);
  if (target_distance_ < simulation_step_ || speed < 1e-3) {
    RCLCPP_ERROR(this->logger_, "Recovery distance and speed must be non-zero");
    return Status::FAILED;
  }

  if (!nav2_util::getCurrentPose(
      this->initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    RCLCPP_ERROR(this->logger_, "Initial robot pose is not available");
    return Status::FAILED;
  }

  geometry_msgs::msg::Pose2D pose;
  pose.x = this->initial_pose_.pose.position.x;
  pose.y = this->initial_pose_.pose.position.y;
  pose.theta = tf2::getYaw(this->initial_pose_.pose.orientation);

  const double requested_direction = command->target.x < 0.0 ? -1.0 : 1.0;
  const auto candidate = selectCandidate(
    pose, requested_direction, speed, std::max(target_distance_, search_distance_));

  const double required_free_distance = std::min(target_distance_, minimum_free_distance_);
  if (candidate.free_distance + 1e-6 < required_free_distance) {
    RCLCPP_WARN(
      this->logger_,
      "No recovery arc has %.2f m of clearance (best %.2f m)",
      required_free_distance, candidate.free_distance);
    return Status::FAILED;
  }

  selected_linear_velocity_ = candidate.direction * speed;
  selected_angular_velocity_ = candidate.angular_velocity;
  traveled_distance_ = 0.0;
  last_pose_ = this->initial_pose_;
  this->command_time_allowance_ = command->time_allowance;
  this->end_time_ = this->clock_->now() + this->command_time_allowance_;

  RCLCPP_INFO(
    this->logger_,
    "Selected recovery arc: v=%.3f m/s, w=%.3f rad/s, free=%.2f m",
    selected_linear_velocity_, selected_angular_velocity_, candidate.free_distance);
  return Status::SUCCEEDED;
}

BackUpToFreeSpace::Status BackUpToFreeSpace::onCycleUpdate()
{
  if (
    this->command_time_allowance_.seconds() > 0.0 &&
    this->clock_->now() > this->end_time_)
  {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Recovery exceeded its time allowance");
    return Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
      this->transform_tolerance_))
  {
    this->stopRobot();
    RCLCPP_ERROR(this->logger_, "Current robot pose is not available");
    return Status::FAILED;
  }

  traveled_distance_ += std::hypot(
    current_pose.pose.position.x - last_pose_.pose.position.x,
    current_pose.pose.position.y - last_pose_.pose.position.y);
  last_pose_ = current_pose;
  this->feedback_->distance_traveled = traveled_distance_;
  this->action_server_->publish_feedback(this->feedback_);

  if (traveled_distance_ >= target_distance_) {
    this->stopRobot();
    return Status::SUCCEEDED;
  }

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  const double remaining = target_distance_ - traveled_distance_;
  const double lookahead = std::min(
    remaining,
    std::max(minimum_free_distance_, std::abs(selected_linear_velocity_) * this->simulate_ahead_time_));
  const double free_distance = collisionFreeDistance(
    pose, selected_linear_velocity_, selected_angular_velocity_, lookahead, true);
  if (free_distance + simulation_step_ < lookahead) {
    this->stopRobot();
    RCLCPP_WARN(this->logger_, "Selected recovery arc became obstructed");
    return Status::FAILED;
  }

  auto command = std::make_unique<geometry_msgs::msg::Twist>();
  command->linear.x = selected_linear_velocity_;
  command->angular.z = selected_angular_velocity_;
  this->vel_pub_->publish(std::move(command));
  return Status::RUNNING;
}

void BackUpToFreeSpace::onCleanup()
{
  nav2_behaviors::DriveOnHeading<Action>::onCleanup();
  traveled_distance_ = 0.0;
}

void BackUpToFreeSpace::onActionCompletion()
{
  nav2_behaviors::DriveOnHeading<Action>::onActionCompletion();
  traveled_distance_ = 0.0;
}

BackUpToFreeSpace::Candidate BackUpToFreeSpace::selectCandidate(
  const geometry_msgs::msg::Pose2D & pose,
  double requested_direction,
  double speed,
  double distance)
{
  Candidate best;
  std::vector<double> directions{requested_direction};
  if (allow_direction_change_) {
    directions.push_back(-requested_direction);
  }

  for (const double direction : directions) {
    for (int index = 0; index < angular_samples_; ++index) {
      const double ratio = angular_samples_ == 1 ? 0.0 :
        -1.0 + 2.0 * static_cast<double>(index) /
        static_cast<double>(angular_samples_ - 1);
      const double angular_velocity = ratio * max_angular_velocity_;
      const double free_distance = collisionFreeDistance(
        pose, direction * speed, angular_velocity, distance, true);
      const double direction_penalty =
        direction == requested_direction ? 0.0 : direction_change_penalty_;
      const double angular_penalty = max_angular_velocity_ > 0.0 ?
        turning_penalty_ * std::abs(angular_velocity) / max_angular_velocity_ : 0.0;
      const double score = free_distance - direction_penalty - angular_penalty;

      if (score > best.score) {
        best.direction = direction;
        best.angular_velocity = angular_velocity;
        best.free_distance = free_distance;
        best.score = score;
      }
    }
  }
  return best;
}

double BackUpToFreeSpace::collisionFreeDistance(
  const geometry_msgs::msg::Pose2D & start,
  double linear_velocity,
  double angular_velocity,
  double distance,
  bool fetch_data)
{
  if (distance <= 0.0 || std::abs(linear_velocity) < 1e-6) {
    return 0.0;
  }

  auto pose = start;
  double traveled = 0.0;
  const double speed = std::abs(linear_velocity);
  while (traveled + 1e-9 < distance) {
    const double step = std::min(simulation_step_, distance - traveled);
    pose = integrateArc(pose, linear_velocity, angular_velocity, step / speed);
    traveled += step;
    if (!this->collision_checker_->isCollisionFree(pose, fetch_data)) {
      return std::max(0.0, traveled - step);
    }
    fetch_data = false;
  }
  return traveled;
}

geometry_msgs::msg::Pose2D BackUpToFreeSpace::integrateArc(
  const geometry_msgs::msg::Pose2D & pose,
  double linear_velocity,
  double angular_velocity,
  double dt)
{
  auto result = pose;
  if (std::abs(angular_velocity) < 1e-6) {
    result.x += linear_velocity * dt * std::cos(pose.theta);
    result.y += linear_velocity * dt * std::sin(pose.theta);
    return result;
  }

  const double next_theta = pose.theta + angular_velocity * dt;
  const double radius = linear_velocity / angular_velocity;
  result.x += radius * (std::sin(next_theta) - std::sin(pose.theta));
  result.y -= radius * (std::cos(next_theta) - std::cos(pose.theta));
  result.theta = next_theta;
  return result;
}

}  // namespace pudu_nav2_plugins

PLUGINLIB_EXPORT_CLASS(pudu_nav2_plugins::BackUpToFreeSpace, nav2_core::Behavior)
