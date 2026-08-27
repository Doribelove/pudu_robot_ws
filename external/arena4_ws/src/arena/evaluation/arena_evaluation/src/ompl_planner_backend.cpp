#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/config.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Console.h>
#include <yaml-cpp/yaml.h>
#include <Python.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

namespace
{
constexpr double kWheelbase = 0.50;
constexpr double kMinimumTurningRadius = 0.40;
constexpr double kMaximumSteering = std::atan(kWheelbase / kMinimumTurningRadius);
constexpr double kFootprintHalfLength = 0.255;
constexpr double kFootprintHalfWidth = 0.215;
constexpr double kPlanningMargin = 0.10;
constexpr double kSampleSpacing = 0.05;

std::string versionString()
{
  return std::to_string(OMPL_MAJOR_VERSION) + "." + std::to_string(OMPL_MINOR_VERSION) + "." +
         std::to_string(OMPL_PATCH_VERSION);
}

struct Arguments
{
  std::string algorithm;
  std::string map_yaml;
  std::string path_output;
  std::string summary_output;
  std::string allowed_mask;
  std::array<double, 3> start{};
  std::array<double, 3> goal{};
  unsigned int seed{20260821U};
  double timeout_s{5.0};
};

struct MapData
{
  double resolution{0.05};
  std::array<double, 3> origin{};
  int width{0};
  int height{0};
  double occupied_threshold{0.65};
  double free_threshold{0.196};
  bool negate{false};
  std::vector<std::uint8_t> pixels;
  std::vector<std::uint8_t> allowed;

  bool worldToCell(double x, double y, int & row, int & col) const
  {
    col = static_cast<int>(std::floor((x - origin[0]) / resolution));
    const int from_bottom = static_cast<int>(std::floor((y - origin[1]) / resolution));
    row = height - 1 - from_bottom;
    return row >= 0 && row < height && col >= 0 && col < width;
  }

  std::array<double, 2> cellCenter(int row, int col) const
  {
    return {origin[0] + (static_cast<double>(col) + 0.5) * resolution,
      origin[1] + (static_cast<double>(height - row) - 0.5) * resolution};
  }

  bool occupiedOrUnknown(int row, int col) const
  {
    if (row < 0 || row >= height || col < 0 || col >= width) {
      return true;
    }
    const double normalized = static_cast<double>(pixels[static_cast<std::size_t>(row * width + col)]) / 255.0;
    const double probability = negate ? normalized : 1.0 - normalized;
    return probability >= free_threshold;  // unknown is collision under this protocol
  }

  bool allowedAt(double x, double y) const
  {
    if (allowed.empty()) {return true;}
    int row = 0, col = 0;
    return worldToCell(x, y, row, col) && allowed[static_cast<std::size_t>(row * width + col)] > 0U;
  }
};

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity{0.0};
  double steering{0.0};
};

struct Result
{
  bool success{false};
  std::string failure_code{"NO_PATH"};
  std::string failure_detail;
  std::string backend;
  double planning_time_ms{0.0};
  double first_solution_time_ms{-1.0};
  long samples{-1L};
  std::size_t generated_states{0U};
  long rewires{-1L};
  unsigned int iterations{0U};
  std::vector<PathPoint> path;
};

double wrap(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle >= pi) {angle -= 2.0 * pi;}
  while (angle < -pi) {angle += 2.0 * pi;}
  return angle;
}

std::string nextToken(std::istream & stream)
{
  std::string token;
  while (stream >> token) {
    if (!token.empty() && token[0] == '#') {
      stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    return token;
  }
  throw std::runtime_error("unexpected end of PGM header");
}

std::vector<std::uint8_t> readPgm(const std::string & path, int & width, int & height)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {throw std::runtime_error("cannot open map image: " + path);}
  const std::string magic = nextToken(stream);
  width = std::stoi(nextToken(stream));
  height = std::stoi(nextToken(stream));
  const int maximum = std::stoi(nextToken(stream));
  if ((magic != "P5" && magic != "P2") || width <= 0 || height <= 0 || maximum <= 0 || maximum > 255) {
    throw std::runtime_error("unsupported PGM format");
  }
  std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width * height));
  if (magic == "P5") {
    stream.get();
    stream.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (stream.gcount() != static_cast<std::streamsize>(pixels.size())) {
      throw std::runtime_error("truncated PGM image");
    }
  } else {
    for (auto & pixel : pixels) {
      pixel = static_cast<std::uint8_t>(std::stoi(nextToken(stream)));
    }
  }
  return pixels;
}

MapData loadMap(const std::string & yaml_path, const std::string & allowed_mask)
{
  const YAML::Node config = YAML::LoadFile(yaml_path);
  MapData map;
  map.resolution = config["resolution"].as<double>();
  if (std::abs(map.resolution - 0.05) > 1.0e-12) {
    throw std::runtime_error("map resolution must be exactly 0.05 m/cell");
  }
  for (std::size_t i = 0; i < 3; ++i) {map.origin[i] = config["origin"][i].as<double>();}
  if (std::abs(map.origin[2]) > 1.0e-12) {throw std::runtime_error("rotated map origins are unsupported");}
  map.occupied_threshold = config["occupied_thresh"] ? config["occupied_thresh"].as<double>() : 0.65;
  map.free_threshold = config["free_thresh"] ? config["free_thresh"].as<double>() : 0.196;
  map.negate = config["negate"] ? config["negate"].as<int>() != 0 : false;
  std::string image = config["image"].as<std::string>();
  if (!image.empty() && image.front() != '/') {
    const std::size_t slash = yaml_path.find_last_of('/');
    image = (slash == std::string::npos ? std::string() : yaml_path.substr(0, slash + 1)) + image;
  }
  map.pixels = readPgm(image, map.width, map.height);
  if (!allowed_mask.empty()) {
    int mask_width = 0, mask_height = 0;
    map.allowed = readPgm(allowed_mask, mask_width, mask_height);
    if (mask_width != map.width || mask_height != map.height) {
      throw std::runtime_error("allowed-mask dimensions do not match the occupancy map");
    }
  }
  return map;
}

double pointSegmentDistance(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double denominator = dx * dx + dy * dy;
  const double t = denominator <= 1.0e-18 ? 0.0 : std::clamp(((px - ax) * dx + (py - ay) * dy) / denominator, 0.0, 1.0);
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

bool pointInPolygon(double x, double y, const std::array<std::array<double, 2>, 4> & polygon)
{
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[i];
    const auto & b = polygon[j];
    if (((a[1] > y) != (b[1] > y)) &&
      (x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]))
    {
      inside = !inside;
    }
  }
  return inside;
}

bool footprintValid(const MapData & map, double x, double y, double yaw)
{
  if (!map.allowedAt(x, y)) {return false;}
  const double half_length = kFootprintHalfLength + kPlanningMargin;
  const double half_width = kFootprintHalfWidth + kPlanningMargin;
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const std::array<std::array<double, 2>, 4> local{{
      {{half_length, half_width}}, {{half_length, -half_width}},
      {{-half_length, -half_width}}, {{-half_length, half_width}}
    }};
  std::array<std::array<double, 2>, 4> polygon{};
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -min_x;
  double min_y = min_x;
  double max_y = -min_x;
  for (std::size_t i = 0; i < local.size(); ++i) {
    polygon[i] = {{x + cosine * local[i][0] - sine * local[i][1],
        y + sine * local[i][0] + cosine * local[i][1]}};
    min_x = std::min(min_x, polygon[i][0]);
    max_x = std::max(max_x, polygon[i][0]);
    min_y = std::min(min_y, polygon[i][1]);
    max_y = std::max(max_y, polygon[i][1]);
  }
  int first_row = 0, first_col = 0, last_row = 0, last_col = 0;
  if (!map.worldToCell(min_x - map.resolution, min_y - map.resolution, first_row, first_col) ||
    !map.worldToCell(max_x + map.resolution, max_y + map.resolution, last_row, last_col))
  {
    return false;
  }
  const double half_diagonal = std::sqrt(2.0) * map.resolution / 2.0;
  for (int row = std::min(first_row, last_row); row <= std::max(first_row, last_row); ++row) {
    for (int col = std::min(first_col, last_col); col <= std::max(first_col, last_col); ++col) {
      if (!map.occupiedOrUnknown(row, col)) {continue;}
      const auto center = map.cellCenter(row, col);
      bool overlaps = pointInPolygon(center[0], center[1], polygon);
      for (std::size_t i = 0; !overlaps && i < polygon.size(); ++i) {
        const auto & a = polygon[i];
        const auto & b = polygon[(i + 1) % polygon.size()];
        overlaps = pointSegmentDistance(center[0], center[1], a[0], a[1], b[0], b[1]) <= half_diagonal;
      }
      if (overlaps) {return false;}
    }
  }
  return true;
}

bool diskValid(const MapData & map, double x, double y)
{
  if (!map.allowedAt(x, y)) {return false;}
  const double radius = std::hypot(kFootprintHalfLength, kFootprintHalfWidth) + kPlanningMargin;
  int first_row = 0, first_col = 0, last_row = 0, last_col = 0;
  if (!map.worldToCell(x - radius, y - radius, first_row, first_col) ||
    !map.worldToCell(x + radius, y + radius, last_row, last_col))
  {
    return false;
  }
  const double threshold = radius + std::sqrt(2.0) * map.resolution / 2.0;
  for (int row = std::min(first_row, last_row); row <= std::max(first_row, last_row); ++row) {
    for (int col = std::min(first_col, last_col); col <= std::max(first_col, last_col); ++col) {
      if (!map.occupiedOrUnknown(row, col)) {continue;}
      const auto center = map.cellCenter(row, col);
      if (std::hypot(center[0] - x, center[1] - y) <= threshold) {return false;}
    }
  }
  return true;
}

Arguments parseArguments(int argc, char ** argv)
{
  Arguments args;
  auto require = [&](int & index) -> std::string {
      if (++index >= argc) {throw std::runtime_error("missing command argument value");}
      return argv[index];
    };
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--version") {
      std::cout << "OMPL " << versionString() << '\n';
      std::exit(0);
    } else if (key == "--algorithm") {args.algorithm = require(i);
    } else if (key == "--map-yaml") {args.map_yaml = require(i);
    } else if (key == "--path-output") {args.path_output = require(i);
    } else if (key == "--summary-output") {args.summary_output = require(i);
    } else if (key == "--allowed-mask") {args.allowed_mask = require(i);
    } else if (key == "--seed") {args.seed = static_cast<unsigned int>(std::stoul(require(i)));
    } else if (key == "--timeout") {args.timeout_s = std::stod(require(i));
    } else if (key == "--start") {
      for (double & value : args.start) {value = std::stod(require(i));}
    } else if (key == "--goal") {
      for (double & value : args.goal) {value = std::stod(require(i));}
    } else {throw std::runtime_error("unknown argument: " + key);}
  }
  if ((args.algorithm != "rrt_star" && args.algorithm != "sst") || args.map_yaml.empty() ||
    args.path_output.empty() || args.summary_output.empty() || args.timeout_s <= 0.0)
  {
    throw std::runtime_error("required: --algorithm rrt_star|sst --map-yaml PATH --start X Y YAW --goal X Y YAW --path-output PATH --summary-output PATH");
  }
  return args;
}

void writePath(const std::string & path, const std::vector<PathPoint> & points)
{
  YAML::Emitter output;
  output << YAML::BeginSeq;
  for (const auto & point : points) {
    output << YAML::BeginMap << YAML::Key << "x" << YAML::Value << point.x
           << YAML::Key << "y" << YAML::Value << point.y
           << YAML::Key << "yaw" << YAML::Value << point.yaw
           << YAML::Key << "velocity" << YAML::Value << point.velocity
           << YAML::Key << "steering" << YAML::Value << point.steering << YAML::EndMap;
  }
  output << YAML::EndSeq;
  std::ofstream stream(path);
  stream << output.c_str() << '\n';
}

void writeSummary(const std::string & path, const Result & result)
{
  YAML::Emitter output;
  output << YAML::BeginMap
         << YAML::Key << "planner_success" << YAML::Value << result.success
         << YAML::Key << "failure_code" << YAML::Value << result.failure_code
         << YAML::Key << "failure_detail" << YAML::Value << result.failure_detail
         << YAML::Key << "planner_backend" << YAML::Value << result.backend
         << YAML::Key << "backend_version" << YAML::Value << versionString()
         << YAML::Key << "planning_time_ms" << YAML::Value << result.planning_time_ms
         << YAML::Key << "first_solution_time_ms" << YAML::Value;
  if (result.first_solution_time_ms < 0.0) {output << YAML::Null;} else {output << result.first_solution_time_ms;}
  output << YAML::Key << "samples" << YAML::Value;
  if (result.samples < 0) {output << YAML::Null;} else {output << result.samples;}
  output << YAML::Key << "generated_states" << YAML::Value << result.generated_states
         << YAML::Key << "rewires" << YAML::Value;
  if (result.rewires < 0) {output << YAML::Null;} else {output << result.rewires;}
  output << YAML::Key << "iterations" << YAML::Value << result.iterations
         << YAML::Key << "path_point_count" << YAML::Value << result.path.size()
         << YAML::EndMap;
  std::ofstream stream(path);
  stream << output.c_str() << '\n';
}

Result solveRrtStar(const MapData & map, const Arguments & args)
{
  Result result;
  result.backend = "OMPL geometric::RRTstar";
  auto space = std::make_shared<ob::RealVectorStateSpace>(2);
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0, map.origin[0]);
  bounds.setHigh(0, map.origin[0] + map.width * map.resolution);
  bounds.setLow(1, map.origin[1]);
  bounds.setHigh(1, map.origin[1] + map.height * map.resolution);
  space->setBounds(bounds);
  og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&map](const ob::State * state) {
      const auto * values = state->as<ob::RealVectorStateSpace::StateType>();
      return diskValid(map, values->values[0], values->values[1]);
    });
  setup.getSpaceInformation()->setStateValidityCheckingResolution(kSampleSpacing / space->getMaximumExtent());
  ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
  start[0] = args.start[0]; start[1] = args.start[1];
  goal[0] = args.goal[0]; goal[1] = args.goal[1];
  setup.setStartAndGoalStates(start, goal, 0.10);
  setup.getProblemDefinition()->setOptimizationObjective(
    std::make_shared<ob::PathLengthOptimizationObjective>(setup.getSpaceInformation()));
  auto planner = std::make_shared<og::RRTstar>(setup.getSpaceInformation());
  planner->setRange(1.0);
  planner->setGoalBias(0.10);
  planner->setRewireFactor(1.1);
  setup.setPlanner(planner);
  setup.setup();
  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + std::chrono::duration<double>(args.timeout_s);
  bool first_recorded = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    setup.solve(std::min(0.05, std::max(0.001, remaining)));
    if (!first_recorded && setup.haveExactSolutionPath()) {
      result.first_solution_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      first_recorded = true;
    }
  }
  result.planning_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  ob::PlannerData data(setup.getSpaceInformation());
  planner->getPlannerData(data);
  result.generated_states = data.numVertices();
  result.iterations = planner->numIterations();
  // RRTstar draws one random state per main-loop iteration. PlannerData
  // vertices are retained separately because rejected samples are not nodes.
  result.samples = static_cast<long>(result.iterations);
  result.failure_detail = "OMPL does not expose an exact RRTstar rewire counter; rewires is null";
  if (!setup.haveExactSolutionPath()) {
    result.failure_code = "NO_EXACT_SOLUTION";
    return result;
  }
  auto path = setup.getSolutionPath();
  const std::size_t count = std::max<std::size_t>(2U, static_cast<std::size_t>(std::ceil(path.length() / kSampleSpacing)) + 1U);
  path.interpolate(count);
  const auto & states = path.getStates();
  result.path.reserve(states.size());
  for (std::size_t index = 0; index < states.size(); ++index) {
    const auto * state = states[index]->as<ob::RealVectorStateSpace::StateType>();
    double yaw = args.start[2];
    if (index + 1 < states.size()) {
      const auto * next = states[index + 1]->as<ob::RealVectorStateSpace::StateType>();
      yaw = std::atan2(next->values[1] - state->values[1], next->values[0] - state->values[0]);
    } else if (index > 0) {
      const auto * previous = states[index - 1]->as<ob::RealVectorStateSpace::StateType>();
      yaw = std::atan2(state->values[1] - previous->values[1], state->values[0] - previous->values[0]);
    }
    result.path.push_back({state->values[0], state->values[1], wrap(yaw), 0.0, 0.0});
  }
  result.success = true;
  result.failure_code.clear();
  return result;
}

class PoseGoal : public ob::GoalRegion
{
public:
  PoseGoal(const ob::SpaceInformationPtr & si, const std::array<double, 3> & goal)
  : ob::GoalRegion(si), goal_(goal)
  {
    threshold_ = 1.0;
  }

  double distanceGoal(const ob::State * state) const override
  {
    const auto * compound = state->as<ob::CompoundStateSpace::StateType>();
    const auto * pose = compound->as<ob::SE2StateSpace::StateType>(0);
    const double position = std::hypot(pose->getX() - goal_[0], pose->getY() - goal_[1]) / 0.25;
    const double yaw = std::abs(wrap(pose->getYaw() - goal_[2])) / (10.0 * 3.14159265358979323846 / 180.0);
    return std::max(position, yaw);
  }

private:
  std::array<double, 3> goal_;
};

class BicyclePropagator : public oc::StatePropagator
{
public:
  explicit BicyclePropagator(oc::SpaceInformation * si) : oc::StatePropagator(si) {}

  void propagate(const ob::State * state, const oc::Control * control, double duration, ob::State * result) const override
  {
    si_->getStateSpace()->copyState(result, state);
    auto * compound = result->as<ob::CompoundStateSpace::StateType>();
    auto * pose = compound->as<ob::SE2StateSpace::StateType>(0);
    auto * dynamics = compound->as<ob::RealVectorStateSpace::StateType>(1);
    const auto * commands = control->as<oc::RealVectorControlSpace::ControlType>();
    double x = pose->getX(), y = pose->getY(), yaw = pose->getYaw();
    double velocity = dynamics->values[0], steering = dynamics->values[1];
    double elapsed = 0.0;
    while (elapsed < duration - 1.0e-12) {
      const double step = std::min(0.02, duration - elapsed);
      velocity = std::clamp(velocity + commands->values[0] * step, 0.05, 1.0);
      steering = std::clamp(steering + commands->values[1] * step, -kMaximumSteering, kMaximumSteering);
      x += velocity * std::cos(yaw) * step;
      y += velocity * std::sin(yaw) * step;
      yaw = wrap(yaw + velocity * std::tan(steering) / kWheelbase * step);
      elapsed += step;
    }
    pose->setX(x); pose->setY(y); pose->setYaw(yaw);
    dynamics->values[0] = velocity;
    dynamics->values[1] = steering;
    si_->getStateSpace()->enforceBounds(result);
  }

  bool canPropagateBackward() const override {return false;}
};

Result solveSst(const MapData & map, const Arguments & args)
{
  Result result;
  result.backend = "OMPL control::SST";
  auto pose_space = std::make_shared<ob::SE2StateSpace>();
  ob::RealVectorBounds pose_bounds(2);
  pose_bounds.setLow(0, map.origin[0]);
  pose_bounds.setHigh(0, map.origin[0] + map.width * map.resolution);
  pose_bounds.setLow(1, map.origin[1]);
  pose_bounds.setHigh(1, map.origin[1] + map.height * map.resolution);
  pose_space->setBounds(pose_bounds);
  auto dynamic_space = std::make_shared<ob::RealVectorStateSpace>(2);
  ob::RealVectorBounds dynamic_bounds(2);
  dynamic_bounds.setLow(0, 0.05); dynamic_bounds.setHigh(0, 1.0);
  dynamic_bounds.setLow(1, -kMaximumSteering); dynamic_bounds.setHigh(1, kMaximumSteering);
  dynamic_space->setBounds(dynamic_bounds);
  auto state_space = std::make_shared<ob::CompoundStateSpace>();
  state_space->addSubspace(pose_space, 1.0);
  state_space->addSubspace(dynamic_space, 0.2);
  auto control_space = std::make_shared<oc::RealVectorControlSpace>(state_space, 2);
  ob::RealVectorBounds control_bounds(2);
  control_bounds.setLow(0, -0.5); control_bounds.setHigh(0, 0.5);
  control_bounds.setLow(1, -0.6); control_bounds.setHigh(1, 0.6);
  control_space->setBounds(control_bounds);
  oc::SimpleSetup setup(control_space);
  setup.setStateValidityChecker([&map](const ob::State * state) {
      const auto * compound = state->as<ob::CompoundStateSpace::StateType>();
      const auto * pose = compound->as<ob::SE2StateSpace::StateType>(0);
      const auto * dynamics = compound->as<ob::RealVectorStateSpace::StateType>(1);
      return dynamics->values[0] >= 0.05 - 1.0e-9 && footprintValid(map, pose->getX(), pose->getY(), pose->getYaw());
    });
  auto si = setup.getSpaceInformation();
  si->setStatePropagator(std::make_shared<BicyclePropagator>(si.get()));
  si->setPropagationStepSize(0.05);
  si->setMinMaxControlDuration(2, 20);
  ob::ScopedState<> start(state_space);
  auto * start_compound = start->as<ob::CompoundStateSpace::StateType>();
  auto * start_pose = start_compound->as<ob::SE2StateSpace::StateType>(0);
  auto * start_dynamics = start_compound->as<ob::RealVectorStateSpace::StateType>(1);
  start_pose->setX(args.start[0]); start_pose->setY(args.start[1]); start_pose->setYaw(args.start[2]);
  start_dynamics->values[0] = 0.20; start_dynamics->values[1] = 0.0;
  setup.addStartState(start);
  setup.setGoal(std::make_shared<PoseGoal>(si, args.goal));
  auto planner = std::make_shared<oc::SST>(si);
  planner->setGoalBias(0.10);
  planner->setSelectionRadius(1.0);
  planner->setPruningRadius(0.20);
  setup.setPlanner(planner);
  setup.setup();
  const auto started = std::chrono::steady_clock::now();
  const auto deadline = started + std::chrono::duration<double>(args.timeout_s);
  bool first_recorded = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    setup.solve(std::min(0.05, std::max(0.001, remaining)));
    if (!first_recorded && setup.haveExactSolutionPath()) {
      result.first_solution_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      first_recorded = true;
    }
  }
  result.planning_time_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  ob::PlannerData data(si);
  planner->getPlannerData(data);
  result.generated_states = data.numVertices();
  result.failure_detail = "OMPL SST exposes neither an exact sample counter nor rewiring; samples and rewires are null";
  if (!setup.haveExactSolutionPath()) {
    result.failure_code = "NO_EXACT_SOLUTION";
    return result;
  }
  auto & path = setup.getSolutionPath();
  path.interpolate();
  result.path.reserve(path.getStateCount());
  for (const auto * state : path.getStates()) {
    const auto * compound = state->as<ob::CompoundStateSpace::StateType>();
    const auto * pose = compound->as<ob::SE2StateSpace::StateType>(0);
    const auto * dynamics = compound->as<ob::RealVectorStateSpace::StateType>(1);
    result.path.push_back({pose->getX(), pose->getY(), wrap(pose->getYaw()), dynamics->values[0], dynamics->values[1]});
  }
  result.success = true;
  result.failure_code.clear();
  return result;
}
}  // namespace

int runBackend(int argc, char ** argv)
{
  Result result;
  Arguments args;
  try {
    args = parseArguments(argc, argv);
    const MapData map = loadMap(args.map_yaml, args.allowed_mask);
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
    ompl::RNG::setSeed(args.seed);
    result = args.algorithm == "rrt_star" ? solveRrtStar(map, args) : solveSst(map, args);
  } catch (const std::exception & error) {
    result.failure_code = "BACKEND_EXCEPTION";
    result.failure_detail = error.what();
    result.backend = args.algorithm == "sst" ? "OMPL control::SST" : "OMPL geometric::RRTstar";
  }
  if (!args.path_output.empty()) {writePath(args.path_output, result.path);}
  if (!args.summary_output.empty()) {writeSummary(args.summary_output, result);}
  if (args.summary_output.empty()) {std::cerr << result.failure_code << ": " << result.failure_detail << '\n';}
  return result.failure_code == "BACKEND_EXCEPTION" ? 2 : 0;
}

PyObject * pythonRun(PyObject *, PyObject * args)
{
  PyObject * sequence_object = nullptr;
  if (!PyArg_ParseTuple(args, "O", &sequence_object)) {return nullptr;}
  PyObject * sequence = PySequence_Fast(sequence_object, "arguments must be a sequence of strings");
  if (sequence == nullptr) {return nullptr;}
  std::vector<std::string> values{"ompl_planner_backend"};
  const Py_ssize_t count = PySequence_Fast_GET_SIZE(sequence);
  values.reserve(static_cast<std::size_t>(count) + 1U);
  for (Py_ssize_t index = 0; index < count; ++index) {
    PyObject * item = PySequence_Fast_GET_ITEM(sequence, index);
    if (!PyUnicode_Check(item)) {
      Py_DECREF(sequence);
      PyErr_SetString(PyExc_TypeError, "all backend arguments must be strings");
      return nullptr;
    }
    values.emplace_back(PyUnicode_AsUTF8(item));
  }
  Py_DECREF(sequence);
  std::vector<char *> argv;
  argv.reserve(values.size());
  for (auto & value : values) {argv.push_back(value.data());}
  const int status = runBackend(static_cast<int>(argv.size()), argv.data());
  return PyLong_FromLong(status);
}

PyObject * pythonVersion(PyObject *, PyObject *)
{
  return PyUnicode_FromString(versionString().c_str());
}

PyMethodDef methods[] = {
  {"run", pythonRun, METH_VARARGS, "Run one isolated OMPL planner request."},
  {"version", pythonVersion, METH_NOARGS, "Return the linked OMPL version."},
  {nullptr, nullptr, 0, nullptr}
};

PyModuleDef module = {
  PyModuleDef_HEAD_INIT, "_ompl_planner_backend",
  "Mature OMPL adapters for the PLN-02 static benchmark.", -1, methods
};

PyMODINIT_FUNC PyInit__ompl_planner_backend()
{
  return PyModule_Create(&module);
}
