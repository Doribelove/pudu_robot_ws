#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>
#include <algorithm>

// Forward XY A*: preserve Python's (f,g,serial,id) lexicographic heap order,
// neighbor-slot order and precomputed original double edge/heuristic values.
extern "C" int tail_astar(int64_t n,int64_t start,int64_t goal,
 const int16_t* delta,const double* costs,const double* heuristic,
 double seconds,int64_t* path,uint64_t* stats,double* final_cost) {
 try {
  if(n<=0||start<0||start>=n||goal<0||goal>=n)return -1;
  const auto begin=std::chrono::steady_clock::now();
  const double inf=std::numeric_limits<double>::infinity();
  std::vector<double> g(n,inf);std::vector<int64_t> parent(n,-1);
  using Entry=std::tuple<double,double,uint64_t,int64_t>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> heap;
  uint64_t serial=0,expanded=0,generated=1;bool timeout=false,found=false;
  g[start]=0.;heap.emplace(heuristic[start],0.,0,start);
  while(!heap.empty()) {
   if(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>=seconds){timeout=true;break;}
   const auto item=heap.top();heap.pop();const double cost=std::get<1>(item);const int64_t u=std::get<3>(item);
   if(cost!=g[u])continue;
   if(u==goal){found=true;break;}
   ++expanded;
   for(int j=0;j<8;++j) {
    const int16_t d=delta[u*8+j];if(d==INT16_MIN)continue;
    const int64_t v=u+d;if(v<0||v>=n)return -2;
    const double candidate=cost+costs[u*8+j];
    if(candidate<g[v]) {
     g[v]=candidate;parent[v]=u;++serial;++generated;
     heap.emplace(candidate+heuristic[v],candidate,serial,v);
    }
   }
  }
  int64_t count=0;
  if(found){for(int64_t u=goal;;u=parent[u]){if(count>=n||u<0)return -3;path[count++]=u;if(u==start)break;}
   std::reverse(path,path+count);}
  stats[0]=expanded;stats[1]=generated;stats[2]=timeout;stats[3]=count;
  *final_cost=found?g[goal]:inf;return 0;
 } catch(...) {return -4;}
}
