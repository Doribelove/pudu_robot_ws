// V13 resumable D* loop; imports live g/rhs/OPEN and current hard blocks. No fast-math.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>
struct Entry {double a,b;uint64_t token;int64_t id;};
static bool later(const Entry& a,const Entry& b){return std::tie(a.a,a.b,a.token,a.id)>std::tie(b.a,b.b,b.token,b.id);}
struct Result {uint64_t expanded=0,generated=0,pops=0,pushes=0,updates=0,pred=0,visits=0,serial=1;int64_t timeout=0;std::vector<Entry> heap;std::vector<uint64_t> active;uint64_t unique=0;std::vector<uint8_t> seen;std::vector<int64_t> trace;};
extern "C" {
void* repair_dstar(int64_t n,int64_t start,int64_t goal,const int16_t* neighbors,const double* costs,const double* heuristic,double* g,double* rhs,double seconds,int64_t limit,
 const uint8_t* blocked,const int64_t* sides,const Entry* input,int64_t input_count,const uint64_t* tokens,uint64_t serial,double km,int capture){
 if(n<=0||start<0||goal<0||start>=n||goal>=n)return nullptr;
 auto* r=new Result();const auto absent=std::numeric_limits<uint64_t>::max();
 r->seen.assign(n,0);r->serial=serial;r->active.assign(tokens,tokens+n);r->heap.assign(input,input+input_count);
 std::make_heap(r->heap.begin(),r->heap.end(),later);
 auto edge=[&](int64_t u,int j,int64_t v){
  if(blocked[u]||blocked[v])return std::numeric_limits<double>::infinity();
  if(sides[j*2]>=0){int a=neighbors[u*8+sides[j*2]],b=neighbors[u*8+sides[j*2+1]];
   if(a==std::numeric_limits<int16_t>::min()||b==std::numeric_limits<int16_t>::min()||blocked[u+a]||blocked[u+b])return std::numeric_limits<double>::infinity();}
  return costs[u*8+j];
 };
 auto key=[&](int64_t u){double m=std::min(g[u],rhs[u]);return std::pair<double,double>{m+heuristic[u]+km,m};};
 auto push=[&](int64_t u){
  if(g[u]==rhs[u]){r->active[u]=absent;return;}
  const auto k=key(u);
  // All active entries keep their original keys, including after start shifts
  // (this cold API has no shift). A separate key table makes this O(1).
  r->heap.push_back({k.first,k.second,r->serial,u});r->active[u]=r->serial++;
  std::push_heap(r->heap.begin(),r->heap.end(),later);++r->pushes;
 };
 std::vector<double> qa(n,std::numeric_limits<double>::infinity()),qb(qa);for(const auto& e:r->heap){if(r->active[e.id]==e.token){qa[e.id]=e.a;qb[e.id]=e.b;}}
 auto update=[&](int64_t u){
  ++r->updates;
  if(u!=goal){double best=std::numeric_limits<double>::infinity();for(int j=0;j<8;++j){int delta=neighbors[u*8+j];if(delta==std::numeric_limits<int16_t>::min())continue;++r->visits;double v=edge(u,j,u+delta)+g[u+delta];if(v<best)best=v;}rhs[u]=best;}
  if(g[u]==rhs[u]){r->active[u]=absent;return;}
  auto k=key(u);if(r->active[u]!=absent&&qa[u]==k.first&&qb[u]==k.second)return;
  qa[u]=k.first;qb[u]=k.second;push(u);
 };
 auto begin=std::chrono::steady_clock::now();
 while(!r->heap.empty()){
  auto e=r->heap.front();if(!(std::pair<double,double>{e.a,e.b}<key(start)||rhs[start]!=g[start]))break;
  if((seconds>=0.&&std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>=seconds)||(limit>=0&&r->expanded>=uint64_t(limit))){r->timeout=1;break;}
  std::pop_heap(r->heap.begin(),r->heap.end(),later);r->heap.pop_back();++r->pops;
  if(r->active[e.id]!=e.token)continue;
  r->active[e.id]=absent;
  auto k=key(e.id);if(std::pair<double,double>{e.a,e.b}<k){qa[e.id]=k.first;qb[e.id]=k.second;push(e.id);continue;}
  if(g[e.id]>rhs[e.id])g[e.id]=rhs[e.id];else{g[e.id]=std::numeric_limits<double>::infinity();update(e.id);}
  if(!r->seen[e.id]){r->seen[e.id]=1;++r->unique;}++r->expanded;if(capture)r->trace.push_back(e.id);
  for(int j=0;j<8;++j){int delta=neighbors[e.id*8+j];if(delta==std::numeric_limits<int16_t>::min())continue;++r->pred;update(e.id+delta);}
  ++r->generated;
 }
 return r;
}
int64_t repair_size(void* p){return static_cast<Result*>(p)->heap.size();}
void repair_export(void* p,Entry* entries,uint64_t* tokens,uint64_t* stats){
 auto& r=*static_cast<Result*>(p);std::copy(r.heap.begin(),r.heap.end(),entries);std::copy(r.active.begin(),r.active.end(),tokens);
 uint64_t s[]={r.expanded,r.generated,r.pops,r.pushes,r.updates,r.pred,r.visits,r.serial,uint64_t(r.timeout),r.unique};std::copy(s,s+10,stats);
}
int64_t repair_trace_size(void* p){return static_cast<Result*>(p)->trace.size();}
void repair_trace_export(void* p,int64_t* dest){auto& v=static_cast<Result*>(p)->trace;std::copy(v.begin(),v.end(),dest);}
void repair_free(void* p){delete static_cast<Result*>(p);}
}
