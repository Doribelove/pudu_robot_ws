#include "three_d_v1_nav2/frozen_r1_planner.hpp"

#include <openssl/sha.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace three_d_v1_nav2
{
namespace
{

std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t index = 0; index < line.size(); ++index) {
    const char value = line[index];
    if (value == '"') {
      if (quoted && index + 1 < line.size() && line[index + 1] == '"') {
        field.push_back('"');
        ++index;
      } else {
        quoted = !quoted;
      }
    } else if (value == ',' && !quoted) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(value);
    }
  }
  if (quoted) {
    throw std::runtime_error("unterminated quoted CSV field");
  }
  fields.push_back(field);
  // Python's csv module intentionally emits CRLF by default. std::getline()
  // removes only LF on Linux, so normalize the final CSV field explicitly.
  if (!fields.empty() && !fields.back().empty() && fields.back().back() == '\r') {
    fields.back().pop_back();
  }
  return fields;
}

std::string sha256_file(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("unable to open file for SHA-256: " + path.string());
  }
  SHA256_CTX context;
  SHA256_Init(&context);
  std::array<unsigned char, 64 * 1024> buffer{};
  while (stream.good()) {
    stream.read(reinterpret_cast<char *>(buffer.data()), buffer.size());
    const auto size = stream.gcount();
    if (size > 0) {
      SHA256_Update(&context, buffer.data(), static_cast<std::size_t>(size));
    }
  }
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  SHA256_Final(digest.data(), &context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const unsigned char value : digest) {
    output << std::setw(2) << static_cast<unsigned int>(value);
  }
  return output.str();
}

double normalized_angle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

bool close_to(double first, double second, double tolerance)
{
  return std::abs(normalized_angle(first - second)) <= tolerance;
}

bool exact_text_bool(const std::string & value, bool expected)
{
  return value == (expected ? "true" : "false");
}

}  // namespace

void FrozenR1PathPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("planner lifecycle parent expired during configure");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = std::move(name);
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_bank_index", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_position_tolerance", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_yaw_tolerance", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".start_position_tolerance", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".expected_query_count", rclcpp::ParameterValue(8));

  const auto index = node->get_parameter(name_ + ".path_bank_index").as_string();
  goal_position_tolerance_ = node->get_parameter(name_ + ".goal_position_tolerance").as_double();
  goal_yaw_tolerance_ = node->get_parameter(name_ + ".goal_yaw_tolerance").as_double();
  start_position_tolerance_ = node->get_parameter(name_ + ".start_position_tolerance").as_double();
  const auto expected_count = node->get_parameter(name_ + ".expected_query_count").as_int();
  if (index.empty()) {
    throw std::runtime_error(name_ + ".path_bank_index must be set");
  }
  load_index(index);
  if (static_cast<int64_t>(records_.size()) != expected_count) {
    throw std::runtime_error(
            "path bank query count mismatch: expected " + std::to_string(expected_count) +
            ", loaded " + std::to_string(records_.size()));
  }
  trace_pub_ = node->create_publisher<std_msgs::msg::String>(
    "/three_d_v1/global_layer_trace", rclcpp::QoS(10).reliable());
  RCLCPP_INFO(
    logger_, "Configured frozen 3D-V1-r1 path-bank planner with %zu audited paths from %s",
    records_.size(), index.c_str());
}

void FrozenR1PathPlanner::cleanup()
{
  active_ = false;
  records_.clear();
  trace_pub_.reset();
}

void FrozenR1PathPlanner::activate()
{
  active_ = true;
  if (trace_pub_) {
    trace_pub_->on_activate();
  }
}

void FrozenR1PathPlanner::deactivate()
{
  active_ = false;
  if (trace_pub_) {
    trace_pub_->on_deactivate();
  }
}

void FrozenR1PathPlanner::load_index(const std::string & raw_path)
{
  const std::filesystem::path index_path = std::filesystem::absolute(raw_path);
  std::ifstream stream(index_path);
  if (!stream) {
    throw std::runtime_error("unable to open path-bank index: " + index_path.string());
  }
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("path-bank index is empty");
  }
  const auto header = split_csv(line);
  std::map<std::string, std::size_t> columns;
  for (std::size_t index = 0; index < header.size(); ++index) {
    columns[header[index]] = index;
  }
  const std::vector<std::string> required = {
    "query_id", "start_x", "start_y", "start_yaw", "goal_x", "goal_y", "goal_yaw",
    "path_file", "path_sha256", "canonical_path_hash", "route_edge_ids", "l2_binding_hash",
    "l2_backend", "final_audit_passed", "l1_algorithm", "l3_algorithm", "angle_bins",
    "motion_model", "reverse_allowed", "rotate_in_place_allowed", "min_turning_radius_m",
    "max_curvature_1pm", "roi_ack_mismatches"};
  for (const auto & key : required) {
    if (columns.count(key) == 0) {
      throw std::runtime_error("path-bank index is missing required column: " + key);
    }
  }
  const auto get = [&columns](const std::vector<std::string> & row, const std::string & key) {
      const auto index = columns.at(key);
      if (index >= row.size()) {
        throw std::runtime_error("short path-bank CSV row at column: " + key);
      }
      return row[index];
    };

  records_.clear();
  std::map<std::string, bool> seen_ids;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    const auto row = split_csv(line);
    FrozenPathRecord record;
    record.query_id = get(row, "query_id");
    if (record.query_id.empty() || seen_ids[record.query_id]) {
      throw std::runtime_error("empty or duplicate query ID in path bank: " + record.query_id);
    }
    seen_ids[record.query_id] = true;
    record.start_x = std::stod(get(row, "start_x"));
    record.start_y = std::stod(get(row, "start_y"));
    record.start_yaw = std::stod(get(row, "start_yaw"));
    record.goal_x = std::stod(get(row, "goal_x"));
    record.goal_y = std::stod(get(row, "goal_y"));
    record.goal_yaw = std::stod(get(row, "goal_yaw"));
    record.path_file = get(row, "path_file");
    record.path_sha256 = get(row, "path_sha256");
    record.canonical_path_hash = get(row, "canonical_path_hash");
    record.route_edge_ids = get(row, "route_edge_ids");
    record.l2_binding_hash = get(row, "l2_binding_hash");
    record.l2_backend = get(row, "l2_backend");

    if (!exact_text_bool(get(row, "final_audit_passed"), true) ||
      get(row, "l1_algorithm") != "deterministic_graph_astar" ||
      get(row, "l3_algorithm") != "smac_hybrid_astar" ||
      get(row, "angle_bins") != "48" || get(row, "motion_model") != "DUBIN" ||
      !exact_text_bool(get(row, "reverse_allowed"), false) ||
      !exact_text_bool(get(row, "rotate_in_place_allowed"), false) ||
      std::abs(std::stod(get(row, "min_turning_radius_m")) - 0.40) > 1.0e-9 ||
      std::abs(std::stod(get(row, "max_curvature_1pm")) - 2.50) > 1.0e-9 ||
      std::stoi(get(row, "roi_ack_mismatches")) != 0 ||
      (record.l2_backend != "compact_persistent_dstar" &&
      record.l2_backend != "compact_dstar_cache_restore" &&
      record.l2_backend != "deterministic_grid_astar"))
    {
      throw std::runtime_error("frozen 3D-V1-r1 contract rejected path row: " + record.query_id);
    }

    std::filesystem::path path_file = record.path_file;
    if (path_file.is_relative()) {
      path_file = index_path.parent_path() / path_file;
    }
    path_file = std::filesystem::weakly_canonical(path_file);
    if (sha256_file(path_file) != record.path_sha256) {
      throw std::runtime_error("path file SHA-256 mismatch: " + record.query_id);
    }
    std::ifstream path_stream(path_file);
    std::string path_line;
    if (!std::getline(path_stream, path_line) || split_csv(path_line) != std::vector<std::string>{"x", "y", "yaw"}) {
      throw std::runtime_error("invalid path CSV header: " + record.query_id);
    }
    while (std::getline(path_stream, path_line)) {
      if (path_line.empty()) {
        continue;
      }
      const auto point = split_csv(path_line);
      if (point.size() != 3) {
        throw std::runtime_error("invalid path point: " + record.query_id);
      }
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = std::stod(point[0]);
      pose.pose.position.y = std::stod(point[1]);
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, std::stod(point[2]));
      pose.pose.orientation.x = orientation.x();
      pose.pose.orientation.y = orientation.y();
      pose.pose.orientation.z = orientation.z();
      pose.pose.orientation.w = orientation.w();
      record.poses.push_back(pose);
    }
    if (record.poses.size() < 2) {
      throw std::runtime_error("path has fewer than two poses: " + record.query_id);
    }
    const auto & first = record.poses.front().pose.position;
    const auto & last = record.poses.back().pose.position;
    if (std::hypot(first.x - record.start_x, first.y - record.start_y) > 0.05 ||
      std::hypot(last.x - record.goal_x, last.y - record.goal_y) > 0.05)
    {
      throw std::runtime_error("path endpoints do not match frozen query: " + record.query_id);
    }
    records_.push_back(std::move(record));
  }
  if (records_.empty()) {
    throw std::runtime_error("path-bank index contains no paths");
  }
}

nav_msgs::msg::Path FrozenR1PathPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!active_) {
    throw nav2_core::PlannerException("FrozenR1PathPlanner is not active");
  }
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException("start/goal frame must equal frozen global frame " + global_frame_);
  }
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  const FrozenPathRecord * selected = nullptr;
  for (const auto & record : records_) {
    if (std::hypot(goal.pose.position.x - record.goal_x, goal.pose.position.y - record.goal_y) <=
      goal_position_tolerance_ && close_to(goal_yaw, record.goal_yaw, goal_yaw_tolerance_))
    {
      if (selected != nullptr) {
        throw nav2_core::PlannerException("ambiguous frozen goal matched multiple path-bank rows");
      }
      selected = &record;
    }
  }
  if (selected == nullptr) {
    throw nav2_core::PlannerException("goal does not exactly match any frozen audited query");
  }
  if (std::hypot(
      start.pose.position.x - selected->start_x,
      start.pose.position.y - selected->start_y) > start_position_tolerance_)
  {
    throw nav2_core::PlannerException(
            "current start is outside frozen-query start tolerance for " + selected->query_id);
  }

  nav_msgs::msg::Path plan;
  plan.header.frame_id = global_frame_;
  plan.header.stamp = clock_->now();
  plan.poses = selected->poses;
  for (auto & pose : plan.poses) {
    pose.header = plan.header;
  }

  std_msgs::msg::String trace;
  std::ostringstream json;
  json << "{\"architecture_id\":\"3D-V1-r1-nav2-teb-integration\""
       << ",\"query_id\":\"" << selected->query_id << "\""
       << ",\"l1\":\"deterministic_graph_astar\""
       << ",\"l2\":\"" << selected->l2_backend << "\""
       << ",\"l3\":\"smac_hybrid_astar\""
       << ",\"angle_bins\":48,\"motion_model\":\"DUBIN\""
       << ",\"route_edge_ids\":\"" << selected->route_edge_ids << "\""
       << ",\"l2_binding_hash\":\"" << selected->l2_binding_hash << "\""
       << ",\"canonical_path_hash\":\"" << selected->canonical_path_hash << "\""
       << ",\"canonical_path_audit_reused\":true}"
       ;
  trace.data = json.str();
  trace_pub_->publish(trace);
  RCLCPP_INFO(
    logger_, "Serving SHA-verified canonical 3D-V1-r1 L3 path for %s (%zu poses)",
    selected->query_id.c_str(), plan.poses.size());
  return plan;
}

}  // namespace three_d_v1_nav2

PLUGINLIB_EXPORT_CLASS(three_d_v1_nav2::FrozenR1PathPlanner, nav2_core::GlobalPlanner)
