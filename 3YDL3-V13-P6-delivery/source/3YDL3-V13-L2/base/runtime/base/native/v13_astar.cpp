// Fresh forward A*, identical directed weights and no diagonal corner cutting.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>
extern "C" int fresh_astar(int64_t n,int64_t start,int64_t goal,const int16_t* delta,const double* costs,const double* h,const uint8_t* blocked,const int64_t* sides,double seconds,int64_t* path,uint64_t* stats,double* final_cost,int64_t* trace,int64_t trace_cap){
 try {
  if(n<=0||start<0||goal<0||start>=n||goal>=n)return -1;
  auto begin=std::chrono::steady_clock::now();double inf=std::numeric_limits<double>::infinity();
  using E=std::tuple<double,double,uint64_t,int64_t>;
  std::priority_queue<E,std::vector<E>,std::greater<E>> open;
  std::vector<double> g(n,inf);std::vector<int64_t> parent(n,-1);std::vector<uint8_t> seen(n,0);
  uint64_t expanded=0,generated=1,pops=0,unique=0,peak=0;bool timeout=false,found=false;
  g[start]=0;if(!blocked[start]&&!blocked[goal])open.emplace(h[start],0,0,start);
  while(!open.empty()){
   peak=std::max(peak,uint64_t(open.size()));
   if(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>=seconds){timeout=true;break;}
   auto [f,cost,serial,u]=open.top();open.pop();++pops;if(cost!=g[u])continue;
   if(u==goal){found=true;break;}
   if(trace&&expanded<uint64_t(trace_cap))trace[expanded]=u;
   ++expanded;if(!seen[u]){seen[u]=1;++unique;}
   for(int j=0;j<8;++j){int d=delta[u*8+j];if(d==std::numeric_limits<int16_t>::min())continue;int64_t v=u+d;
    if(blocked[u]||blocked[v])continue;
    if(sides[j*2]>=0){int a=delta[u*8+sides[j*2]],b=delta[u*8+sides[j*2+1]];
     if(a==std::numeric_limits<int16_t>::min()||b==std::numeric_limits<int16_t>::min()||blocked[u+a]||blocked[u+b])continue;}
    double c=cost+costs[u*8+j];if(c<g[v]){g[v]=c;parent[v]=u;open.emplace(c+h[v],c,generated++,v);}
   }
  }
  int64_t count=0;if(found){for(int64_t u=goal;u!=-1;u=parent[u]){path[count++]=u;if(u==start)break;}std::reverse(path,path+count);}
  uint64_t s[]={expanded,generated,uint64_t(timeout),uint64_t(count),unique,peak,pops};std::copy(s,s+7,stats);*final_cost=found?g[goal]:inf;return 0;
 }catch(...){return -2;}
}
