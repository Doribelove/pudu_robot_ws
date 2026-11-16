// Sparse visited-state 2D fallback. No vehicle/heading search occurs here.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace {
struct Node { double g; int32_t parent; bool closed; };
struct Entry { double f,g; int32_t id; uint64_t serial; };
struct Later { bool operator()(const Entry&a,const Entry&b)const {
  return a.f!=b.f?a.f>b.f:a.serial>b.serial;
}};
}

extern "C" int v12_grid_astar(int32_t rows,int32_t cols,const uint8_t*free,
    const float*potential,double weight,int32_t start,int32_t goal,double seconds,
    uint64_t max_visited,int32_t*path,uint64_t capacity,uint64_t*stats) {
  using Clock=std::chrono::steady_clock;
  const auto begun=Clock::now();
  if(!stats)return 4;
  stats[0]=stats[1]=stats[2]=0;
  const int64_t n=int64_t(rows)*cols;
  if(!free||!path||rows<1||cols<1||n>INT32_MAX||start<0||goal<0||start>=n||goal>=n||
     !std::isfinite(seconds)||seconds<=0||!std::isfinite(weight)||weight<0||
     max_visited<2||capacity<2)return 4;
  if(!free[start]||!free[goal])return 1;
  try {
    std::unordered_map<int32_t,Node> best;
    best.reserve(std::min<uint64_t>(max_visited,65536));
    std::priority_queue<Entry,std::vector<Entry>,Later> queue;
    const int gr=goal/cols,gc=goal%cols;
    auto h=[&](int id){return std::hypot(double(id/cols-gr),double(id%cols-gc));};
    uint64_t serial=0;
    best.emplace(start,Node{0.,-1,false});queue.push({h(start),0.,start,serial++});
    constexpr int dr[8]={-1,0,1,0,-1,-1,1,1};
    constexpr int dc[8]={0,1,0,-1,-1,1,-1,1};
    while(!queue.empty()) {
      if((stats[0]&255u)==0 && std::chrono::duration<double>(Clock::now()-begun).count()>=seconds) {
        stats[1]=best.size();return 2;
      }
      Entry current=queue.top();queue.pop();
      auto it=best.find(current.id);
      if(it==best.end()||it->second.closed||it->second.g!=current.g)continue;
      it->second.closed=true;stats[0]++;
      if(current.id==goal) {
        int32_t id=goal;uint64_t count=0;
        while(id>=0) {
          if(count>=capacity)return 3;
          path[count++]=id;id=best.at(id).parent;
        }
        std::reverse(path,path+count);stats[1]=best.size();stats[2]=count;return 0;
      }
      const int r=current.id/cols,c=current.id%cols;
      for(int k=0;k<8;k++) {
        int rr=r+dr[k],cc=c+dc[k];
        if(rr<0||cc<0||rr>=rows||cc>=cols)continue;
        int32_t next=rr*cols+cc;
        if(!free[next]||(k>=4&&(!free[r*cols+cc]||!free[rr*cols+c])))continue;
        double cost=k<4?1.:std::sqrt(2.);
        if(potential) {
          double a=potential[current.id],b=potential[next];
          if(!std::isfinite(a)||!std::isfinite(b)||a<0||b<0)return 4;
          cost*=1.+weight*.5*(a+b);
        }
        double candidate=current.g+cost;
        auto found=best.find(next);
        if(found!=best.end()&&found->second.g<=candidate)continue;
        if(found==best.end()) {
          if(best.size()>=max_visited){stats[1]=best.size();return 3;}
          best.emplace(next,Node{candidate,current.id,false});
        } else found->second=Node{candidate,current.id,false};
        queue.push({candidate+h(next),candidate,next,serial++});
      }
    }
    stats[1]=best.size();return 1;
  } catch(...) {return 5;}
}
