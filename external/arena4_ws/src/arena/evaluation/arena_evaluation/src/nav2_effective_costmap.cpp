#include <Python.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
constexpr std::uint8_t kFreeSpace = 0U;
constexpr std::uint8_t kInscribedInflatedObstacle = 253U;
constexpr std::uint8_t kLethalObstacle = 254U;
constexpr std::uint8_t kNoInformation = 255U;

struct CellData
{
  std::uint32_t index;
  std::uint32_t x;
  std::uint32_t y;
  std::uint32_t source_x;
  std::uint32_t source_y;
};

std::uint8_t computeCost(
  double distance_cells, double resolution, double scaling, double inscribed_radius)
{
  if (distance_cells == 0.0) {
    return kLethalObstacle;
  }
  if (distance_cells * resolution <= inscribed_radius) {
    return kInscribedInflatedObstacle;
  }
  const double factor = std::exp(
    -scaling * (distance_cells * resolution - inscribed_radius));
  return static_cast<std::uint8_t>(
    static_cast<double>(kInscribedInflatedObstacle - 1U) * factor);
}

std::vector<std::vector<int>> integerDistanceLevels(int radius, int & maximum_level)
{
  const int size = radius * 2 + 1;
  std::vector<std::pair<int, int>> points;
  points.reserve(static_cast<std::size_t>(size * size));
  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      if (x * x + y * y <= radius * radius) {
        points.emplace_back(x, y);
      }
    }
  }
  // Match InflationLayer::generateIntegerDistances exactly.  The comparator
  // deliberately has no secondary tie-break, just like the pinned source.
  std::sort(
    points.begin(), points.end(),
    [](const auto & left, const auto & right) {
      return left.first * left.first + left.second * left.second <
             right.first * right.first + right.second * right.second;
    });
  std::vector<std::vector<int>> levels(
    static_cast<std::size_t>(size), std::vector<int>(static_cast<std::size_t>(size), 0));
  std::pair<int, int> last{0, 0};
  int level = 0;
  for (const auto & point : points) {
    if (point.first * point.first + point.second * point.second !=
      last.first * last.first + last.second * last.second)
    {
      ++level;
    }
    levels[static_cast<std::size_t>(point.first + radius)]
      [static_cast<std::size_t>(point.second + radius)] = level;
    last = point;
  }
  maximum_level = level;
  return levels;
}

PyObject * inflate(PyObject *, PyObject * args)
{
  PyObject * source_object = nullptr;
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double inflation_radius = 0.0;
  double cost_scaling_factor = 0.0;
  double inscribed_radius = 0.0;
  if (!PyArg_ParseTuple(
      args, "Oiidddd",
      &source_object, &width, &height, &resolution, &inflation_radius,
      &cost_scaling_factor, &inscribed_radius))
  {
    return nullptr;
  }
  if (width <= 0 || height <= 0 || resolution <= 0.0 || inflation_radius < 0.0 ||
    cost_scaling_factor < 0.0 || inscribed_radius < 0.0)
  {
    PyErr_SetString(PyExc_ValueError, "invalid costmap dimensions or inflation parameters");
    return nullptr;
  }

  Py_buffer source{};
  if (PyObject_GetBuffer(source_object, &source, PyBUF_CONTIG_RO) != 0) {
    return nullptr;
  }
  const auto expected_size = static_cast<Py_ssize_t>(width) * static_cast<Py_ssize_t>(height);
  if (source.len != expected_size || source.itemsize != 1) {
    PyBuffer_Release(&source);
    PyErr_SetString(PyExc_ValueError, "static_cost must be a contiguous uint8 grid");
    return nullptr;
  }

  try {
    const auto * input = static_cast<const std::uint8_t *>(source.buf);
    std::vector<std::uint8_t> master(
      input, input + static_cast<std::size_t>(expected_size));
    const auto cell_radius = static_cast<unsigned int>(
      std::ceil(std::max(0.0, inflation_radius / resolution)));
    if (cell_radius > 0U) {
      const int matrix_radius = static_cast<int>(cell_radius) + 2;
      int maximum_level = 0;
      const auto distance_levels = integerDistanceLevels(matrix_radius, maximum_level);
      std::vector<std::vector<CellData>> bins(static_cast<std::size_t>(maximum_level + 1));
      for (auto & bin : bins) {
        bin.reserve(200U);
      }
      std::vector<std::uint8_t> seen(static_cast<std::size_t>(expected_size), 0U);

      const auto enqueue = [&] (
        std::uint32_t index, std::uint32_t x, std::uint32_t y,
        std::uint32_t source_x, std::uint32_t source_y)
        {
          if (seen[index] != 0U) {
            return;
          }
          const auto dx = static_cast<unsigned int>(
            x > source_x ? x - source_x : source_x - x);
          const auto dy = static_cast<unsigned int>(
            y > source_y ? y - source_y : source_y - y);
          const double distance = std::hypot(
            static_cast<double>(dx), static_cast<double>(dy));
          if (distance > static_cast<double>(cell_radius)) {
            return;
          }
          const int level = distance_levels
            [static_cast<std::size_t>(static_cast<int>(dx) + matrix_radius)]
            [static_cast<std::size_t>(static_cast<int>(dy) + matrix_radius)];
          bins[static_cast<std::size_t>(level)].push_back(
            CellData{index, x, y, source_x, source_y});
        };

      const auto process = [&] (const CellData & cell) {
          if (seen[cell.index] != 0U) {
            return;
          }
          seen[cell.index] = 1U;
          const auto dx = static_cast<unsigned int>(
            cell.x > cell.source_x ? cell.x - cell.source_x : cell.source_x - cell.x);
          const auto dy = static_cast<unsigned int>(
            cell.y > cell.source_y ? cell.y - cell.source_y : cell.source_y - cell.y);
          const auto cost = computeCost(
            std::hypot(static_cast<double>(dx), static_cast<double>(dy)),
            resolution, cost_scaling_factor, inscribed_radius);
          const auto old_cost = master[cell.index];
          if (old_cost == kNoInformation) {
            if (cost >= kInscribedInflatedObstacle) {
              master[cell.index] = cost;
            }
          } else {
            master[cell.index] = std::max(old_cost, cost);
          }
          if (cell.x > 0U) {
            enqueue(cell.index - 1U, cell.x - 1U, cell.y, cell.source_x, cell.source_y);
          }
          if (cell.y > 0U) {
            enqueue(
              cell.index - static_cast<std::uint32_t>(width), cell.x, cell.y - 1U,
              cell.source_x, cell.source_y);
          }
          if (cell.x + 1U < static_cast<std::uint32_t>(width)) {
            enqueue(cell.index + 1U, cell.x + 1U, cell.y, cell.source_x, cell.source_y);
          }
          if (cell.y + 1U < static_cast<std::uint32_t>(height)) {
            enqueue(
              cell.index + static_cast<std::uint32_t>(width), cell.x, cell.y + 1U,
              cell.source_x, cell.source_y);
          }
        };

      // InflationLayer initially appends every lethal cell to bin zero in
      // row-major order.  Processing that list directly is equivalent and
      // avoids retaining ~160 MB of redundant CellData on this map.
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const auto index = static_cast<std::uint32_t>(y * width + x);
          if (master[index] == kLethalObstacle) {
            process(CellData{
              index, static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
              static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y)});
          }
        }
      }
      for (std::size_t level = 1; level < bins.size(); ++level) {
        // The loop condition intentionally observes appended entries, matching
        // the pinned layer's vector iteration contract.
        for (std::size_t index = 0; index < bins[level].size(); ++index) {
          process(bins[level][index]);
        }
      }
    }
    PyBuffer_Release(&source);
    return PyBytes_FromStringAndSize(
      reinterpret_cast<const char *>(master.data()), expected_size);
  } catch (const std::exception & error) {
    PyBuffer_Release(&source);
    PyErr_SetString(PyExc_RuntimeError, error.what());
    return nullptr;
  }
}

PyMethodDef methods[] = {
  {"inflate", inflate, METH_VARARGS,
    "Reproduce the pinned Humble InflationLayer effective master grid."},
  {nullptr, nullptr, 0, nullptr},
};

PyModuleDef module = {
  PyModuleDef_HEAD_INIT,
  "_nav2_effective_costmap",
  "Pinned Nav2 effective-costmap oracle for 2A-V2 r2 ACK.",
  -1,
  methods,
  nullptr,
  nullptr,
  nullptr,
  nullptr,
};
}  // namespace

PyMODINIT_FUNC PyInit__nav2_effective_costmap()
{
  return PyModule_Create(&module);
}
