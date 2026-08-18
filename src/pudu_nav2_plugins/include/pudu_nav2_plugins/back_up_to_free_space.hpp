#ifndef PUDU_NAV2_PLUGINS__BACK_UP_TO_FREE_SPACE_HPP_
#define PUDU_NAV2_PLUGINS__BACK_UP_TO_FREE_SPACE_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"

namespace pudu_nav2_plugins
{

/**
 * @brief Differential-drive recovery that selects the safest collision-free arc.
 *
 * The behavior keeps the standard nav2_msgs/BackUp action interface. Before
 * moving, it samples forward/reverse arcs against Nav2's local costmap and
 * chooses the candidate with the longest free distance. It is inspired by
 * SCURM's free-space recovery, but does not command lateral velocity.
 */
class BackUpToFreeSpace
  : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  using Action = nav2_msgs::action::BackUp;
  using Status = nav2_behaviors::Status;

  Status onRun(const std::shared_ptr<const Action::Goal> command) override;
  Status onCycleUpdate() override;
  void onCleanup() override;
  void onActionCompletion() override;

protected:
  void onConfigure() override;

private:
  struct Candidate
  {
    double direction{0.0};
    double angular_velocity{0.0};
    double free_distance{0.0};
    double score{-1.0};
  };

  Candidate selectCandidate(
    const geometry_msgs::msg::Pose2D & pose,
    double requested_direction,
    double speed,
    double distance);

  double collisionFreeDistance(
    const geometry_msgs::msg::Pose2D & start,
    double linear_velocity,
    double angular_velocity,
    double distance,
    bool fetch_data);

  static geometry_msgs::msg::Pose2D integrateArc(
    const geometry_msgs::msg::Pose2D & pose,
    double linear_velocity,
    double angular_velocity,
    double dt);

  bool allow_direction_change_{true};
  int angular_samples_{7};
  double max_angular_velocity_{0.6};
  double search_distance_{0.75};
  double simulation_step_{0.025};
  double minimum_free_distance_{0.2};
  double direction_change_penalty_{0.15};
  double turning_penalty_{0.05};

  double target_distance_{0.0};
  double selected_linear_velocity_{0.0};
  double selected_angular_velocity_{0.0};
  double traveled_distance_{0.0};
  geometry_msgs::msg::PoseStamped last_pose_;
};

}  // namespace pudu_nav2_plugins

#endif  // PUDU_NAV2_PLUGINS__BACK_UP_TO_FREE_SPACE_HPP_
