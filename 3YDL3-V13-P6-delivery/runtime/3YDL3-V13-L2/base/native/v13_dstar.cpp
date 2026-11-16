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
struct Result {double setup_ms=0,kernel_ms=0;uint64_t rhs_scans=0,fast_relaxations=0,unaffected_predecessors=0,unchanged_queue_skips=0,queue_replacements=0,queue_erases=0,initial_stale_pruned=0;uint64_t expanded=0,generated=0,pops=0,pushes=0,updates=0,pred=0,visits=0,serial=1;int64_t timeout=0;std::vector<Entry> heap;std::vector<uint64_t> active;uint64_t unique=0;std::vector<uint8_t> seen;std::vector<int64_t> trace;};
struct IndexedHeap {
 std::vector<Entry>& h;std::vector<int64_t> position;
 IndexedHeap(std::vector<Entry>& heap,int64_t n,bool enabled):h(heap),position(enabled?n:0,-1){if(enabled)for(size_t i=0;i<h.size();++i)position[h[i].id]=i;}
 void swap_at(int64_t a,int64_t b){std::swap(h[a],h[b]);position[h[a].id]=a;position[h[b].id]=b;}
 void up(int64_t i){while(i>0){const auto p=(i-1)/2;if(!later(h[p],h[i]))break;swap_at(p,i);i=p;}}
 void down(int64_t i){for(;;){auto c=i*2+1;if(c>=int64_t(h.size()))break;if(c+1<int64_t(h.size())&&later(h[c],h[c+1]))++c;if(!later(h[i],h[c]))break;swap_at(i,c);i=c;}}
 void repair(int64_t i){if(i>0&&later(h[(i-1)/2],h[i]))up(i);else down(i);}
 void insert(Entry e){position[e.id]=h.size();h.push_back(e);up(h.size()-1);}
 void replace(int64_t i,Entry e){h[i]=e;position[e.id]=i;repair(i);}
 void erase(int64_t i){const auto removed=h[i].id;const auto last=h.back();h.pop_back();position[removed]=-1;if(i<int64_t(h.size())){h[i]=last;position[last.id]=i;repair(i);}}
};
extern "C" {
// Batch the independent rhs refreshes for a map-change notification. g does
// not change here; Python retains the original sorted _push order and tokens.
uint64_t update_affected_rhs(int64_t count,const int64_t* ids,int64_t goal,
 const int16_t* neighbors,const double* costs,const double* g,double* rhs,
 const uint8_t* blocked,const int64_t* sides){
 uint64_t visits=0;const auto absent=std::numeric_limits<int16_t>::min();
 const double inf=std::numeric_limits<double>::infinity();
 for(int64_t i=0;i<count;++i){
  const int64_t u=ids[i];if(u==goal)continue;double best=inf;
  for(int j=0;j<8;++j){
   const int delta=neighbors[u*8+j];if(delta==absent)continue;++visits;
   const int64_t v=u+delta;double cost=costs[u*8+j];
   if(blocked[u]||blocked[v])cost=inf;
   else if(sides[j*2]>=0){
    const int a=neighbors[u*8+sides[j*2]],b=neighbors[u*8+sides[j*2+1]];
    if(a==absent||b==absent||blocked[u+a]||blocked[u+b])cost=inf;
   }
   const double value=cost+g[v];if(value<best)best=value;
  }
  rhs[u]=best;
 }
 return visits;
}
void* repair_dstar(int64_t n,int64_t start,int64_t goal,const int16_t* neighbors,const double* costs,const double* heuristic,double* g,double* rhs,double seconds,int64_t limit,
 const uint8_t* blocked,const int64_t* sides,const Entry* input,int64_t input_count,const uint64_t* tokens,uint64_t serial,double km,int capture,int smart_rhs,int skip_unchanged,int indexed,const int64_t* opposite){
 if(n<=0||start<0||goal<0||start>=n||goal>=n)return nullptr;
 auto setup_begin=std::chrono::steady_clock::now();
 auto* r=new Result();const auto absent=std::numeric_limits<uint64_t>::max();
 r->seen.assign(n,0);r->serial=serial;r->active.assign(tokens,tokens+n);r->heap.assign(input,input+input_count);
 if(indexed){auto end=std::remove_if(r->heap.begin(),r->heap.end(),[&](const Entry& e){return r->active[e.id]!=e.token;});r->initial_stale_pruned=r->heap.end()-end;r->heap.erase(end,r->heap.end());}
 std::make_heap(r->heap.begin(),r->heap.end(),later);
 IndexedHeap indexed_heap(r->heap,n,indexed);
 auto edge=[&](int64_t u,int j,int64_t v){
  if(blocked[u]||blocked[v])return std::numeric_limits<double>::infinity();
  if(sides[j*2]>=0){int a=neighbors[u*8+sides[j*2]],b=neighbors[u*8+sides[j*2+1]];
   if(a==std::numeric_limits<int16_t>::min()||b==std::numeric_limits<int16_t>::min()||blocked[u+a]||blocked[u+b])return std::numeric_limits<double>::infinity();}
  return costs[u*8+j];
 };
 auto key=[&](int64_t u){double m=std::min(g[u],rhs[u]);return std::pair<double,double>{m+heuristic[u]+km,m};};
 auto invalidate=[&](int64_t u){r->active[u]=absent;if(indexed&&indexed_heap.position[u]>=0){indexed_heap.erase(indexed_heap.position[u]);++r->queue_erases;}};
 auto push=[&](int64_t u){
  if(g[u]==rhs[u]){invalidate(u);return;}
  const auto k=key(u);
  // All active entries keep their original keys, including after start shifts
  // (this cold API has no shift). A separate key table makes this O(1).
  Entry entry{k.first,k.second,r->serial,u};r->active[u]=r->serial++;
  if(indexed){const auto p=indexed_heap.position[u];if(p>=0){indexed_heap.replace(p,entry);++r->queue_replacements;}else indexed_heap.insert(entry);}
  else{r->heap.push_back(entry);std::push_heap(r->heap.begin(),r->heap.end(),later);}
  ++r->pushes;
 };
 std::vector<double> qa(indexed?0:n,std::numeric_limits<double>::infinity()),qb(qa);if(!indexed)for(const auto& e:r->heap){if(r->active[e.id]==e.token){qa[e.id]=e.a;qb[e.id]=e.b;}}
 auto refresh_rhs=[&](int64_t u,double unchanged_lower_bound=-std::numeric_limits<double>::infinity()){
  if(u==goal)return;
  ++r->rhs_scans;double best=std::numeric_limits<double>::infinity();
  for(int j=0;j<8;++j){int delta=neighbors[u*8+j];if(delta==std::numeric_limits<int16_t>::min())continue;++r->visits;double v=edge(u,j,u+delta)+g[u+delta];if(v<best)best=v;if(best==unchanged_lower_bound)break;}
  rhs[u]=best;
 };
 auto queue_vertex=[&](int64_t u){
  if(g[u]==rhs[u]){invalidate(u);return;}
  auto k=key(u);if(r->active[u]!=absent){if(indexed){const auto& e=r->heap[indexed_heap.position[u]];if(e.a==k.first&&e.b==k.second)return;}else if(qa[u]==k.first&&qb[u]==k.second)return;}
  if(!indexed){qa[u]=k.first;qb[u]=k.second;}push(u);
 };
 auto update=[&](int64_t u){++r->updates;refresh_rhs(u);queue_vertex(u);};
 auto notify_predecessor=[&](int64_t u,int64_t changed,double old_g,int reverse_slot){
  ++r->updates;const double previous_rhs=rhs[u];
  if(u!=goal){
   // Only g[changed] has changed in this step. On decrease the new rhs is
   // min(old rhs, changed successor contribution). On increase a full scan
   // is necessary only if the old minimum depended on that successor.
   const int delta=neighbors[u*8+reverse_slot];
   const double cost=delta==changed-u ? edge(u,reverse_slot,changed) : std::numeric_limits<double>::infinity();
   // An absent/infinite directed edge never contributes a finite minimum.
   if(!std::isfinite(cost)){++r->unaffected_predecessors;}
   else if(g[changed]<old_g){
    const double value=cost+g[changed];if(value<rhs[u])rhs[u]=value;
    ++r->fast_relaxations;
   }else if(rhs[u]==cost+old_g){
    // Other successor contributions did not change and cannot be below the
    // old minimum. An equal alternative proves the minimum is unchanged.
    refresh_rhs(u,rhs[u]);
   }
   else{++r->unaffected_predecessors;}
  }
  // g[u], heuristic and km did not change in this notification. If rhs
  // also stayed equal, the established queue entry already has the same key.
  if(skip_unchanged && rhs[u]==previous_rhs){++r->unchanged_queue_skips;}
  else queue_vertex(u);
 };
 auto begin=std::chrono::steady_clock::now();
 r->setup_ms=std::chrono::duration<double,std::milli>(begin-setup_begin).count();
 while(!r->heap.empty()){
  auto e=r->heap.front();if(!(std::pair<double,double>{e.a,e.b}<key(start)||rhs[start]!=g[start]))break;
  if((seconds>=0.&&std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>=seconds)||(limit>=0&&r->expanded>=uint64_t(limit))){r->timeout=1;break;}
  if(indexed)indexed_heap.erase(0);else{std::pop_heap(r->heap.begin(),r->heap.end(),later);r->heap.pop_back();}++r->pops;
  if(r->active[e.id]!=e.token)continue;
  r->active[e.id]=absent;
  auto k=key(e.id);if(std::pair<double,double>{e.a,e.b}<k){if(!indexed){qa[e.id]=k.first;qb[e.id]=k.second;}push(e.id);continue;}
  const double old_g=g[e.id];
  if(g[e.id]>rhs[e.id])g[e.id]=rhs[e.id];else{g[e.id]=std::numeric_limits<double>::infinity();update(e.id);}
  if(!r->seen[e.id]){r->seen[e.id]=1;++r->unique;}++r->expanded;if(capture)r->trace.push_back(e.id);
  for(int j=0;j<8;++j){int delta=neighbors[e.id*8+j];if(delta==std::numeric_limits<int16_t>::min())continue;++r->pred;if(smart_rhs)notify_predecessor(e.id+delta,e.id,old_g,opposite[j]);else update(e.id+delta);}
  ++r->generated;
 }
 r->kernel_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
 return r;
}
int64_t repair_size(void* p){return static_cast<Result*>(p)->heap.size();}
void repair_export(void* p,Entry* entries,uint64_t* tokens,uint64_t* stats){
 auto& r=*static_cast<Result*>(p);std::copy(r.heap.begin(),r.heap.end(),entries);std::copy(r.active.begin(),r.active.end(),tokens);
 uint64_t s[]={r.expanded,r.generated,r.pops,r.pushes,r.updates,r.pred,r.visits,r.serial,uint64_t(r.timeout),r.unique};std::copy(s,s+10,stats);
}
int64_t repair_trace_size(void* p){return static_cast<Result*>(p)->trace.size();}
void repair_trace_export(void* p,int64_t* dest){auto& v=static_cast<Result*>(p)->trace;std::copy(v.begin(),v.end(),dest);}
void repair_phase_export(void* p,double* out){auto& r=*static_cast<Result*>(p);out[0]=r.setup_ms;out[1]=r.kernel_ms;}
void repair_work_export(void* p,uint64_t* out){auto& r=*static_cast<Result*>(p);out[0]=r.rhs_scans;out[1]=r.fast_relaxations;out[2]=r.unaffected_predecessors;out[3]=r.unchanged_queue_skips;out[4]=r.queue_replacements;out[5]=r.queue_erases;out[6]=r.initial_stale_pruned;}
void repair_free(void* p){delete static_cast<Result*>(p);}
}
