// Read-only path extraction on the same directed graph as the frozen D* loop.
// Match Python min((edge + g, row, col, state_id)) exactly; no epsilon ties.
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <unordered_set>

extern "C" int64_t extract_path(
    int64_t n, int64_t start, int64_t goal, int64_t limit,
    const int16_t* neighbors, const double* costs, const double* g,
    const uint8_t* blocked, const int64_t* sides,
    const int32_t* rows, const int32_t* cols, int64_t* output) {
  if (n <= 0 || start < 0 || goal < 0 || start >= n || goal >= n || limit < 1)
    return -1;
  if (!std::isfinite(g[start])) return 0;
  int64_t current = start, length = 1;
  output[0] = start;
  std::unordered_set<int64_t> visited{start};
  const auto absent = std::numeric_limits<int16_t>::min();
  while (current != goal && length < limit) {
    bool found = false;
    std::tuple<double, int32_t, int32_t, int64_t> best;
    for (int j = 0; j < 8; ++j) {
      const int delta = neighbors[current * 8 + j];
      if (delta == absent) continue;
      const int64_t v = current + delta;
      if (v < 0 || v >= n) return -1;
      if (blocked[current] || blocked[v]) continue;
      if (sides[j * 2] >= 0) {
        const int a = neighbors[current * 8 + sides[j * 2]];
        const int b = neighbors[current * 8 + sides[j * 2 + 1]];
        if (a == absent || b == absent) continue;
        if (current + a < 0 || current + a >= n || current + b < 0 || current + b >= n)
          return -1;
        if (blocked[current + a] || blocked[current + b]) continue;
      }
      const double value = costs[current * 8 + j] + g[v];
      if (!std::isfinite(value)) continue;
      const auto choice = std::make_tuple(value, rows[v], cols[v], v);
      if (!found || choice < best) { best = choice; found = true; }
    }
    if (!found) return 0;
    current = std::get<3>(best);
    if (!visited.insert(current).second) return 0;
    output[length++] = current;
  }
  return current == goal ? length : 0;
}
