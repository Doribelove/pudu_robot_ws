// Exact order-preserving counterpart of geometry.segment_cells.
// NumPy supplies linspace, preserving its endpoint and rounding behavior.
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

extern "C" int64_t segment_cells(const double* a, const double* b,
    const double* t, int64_t n, int64_t* out) {
  std::vector<std::pair<int32_t,int32_t>> points;
  points.reserve(n);
  for (int64_t i=0; i<n; ++i) {
    const int32_t row=static_cast<int32_t>(std::floor((a[0]+(b[0]-a[0])*t[i])+.5));
    const int32_t col=static_cast<int32_t>(std::floor((a[1]+(b[1]-a[1])*t[i])+.5));
    const auto p=std::make_pair(row,col);
    if (points.empty() || points.back()!=p) points.push_back(p);
  }
  std::unordered_set<uint64_t> seen;
  seen.reserve(points.size()*3);
  int64_t count=0;
  auto add=[&](int32_t r,int32_t c) {
    uint64_t key=(uint64_t(uint32_t(r))<<32)|uint32_t(c);
    if(seen.insert(key).second){out[count*2]=r;out[count*2+1]=c;++count;}
  };
  for (const auto& p:points) add(p.first,p.second);
  for (size_t i=1;i<points.size();++i) {
    const auto& p=points[i-1];const auto& q=points[i];
    if(p.first!=q.first && p.second!=q.second){add(p.first,q.second);add(q.first,p.second);}
  }
  return count;
}
