// Bounded endpoint initializer; the caller must reoptimize and independently
// certify the full spline. Uses the unchanged original swept_rect callback.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>
using Pose=std::array<double,3>;
using Checker=long(*)(const unsigned char*,int,int,const double*,int,double,double,double,double,double);
static constexpr double pi=3.14159265358979323846;
static double mod(double x){x=std::fmod(x,2*pi);return x<0?x+2*pi:x;}
static double angle(double x){return mod(x+pi)-pi;}
struct Word{std::array<int,3> kind;std::array<double,3> p;double cost;};
static std::vector<Word> words(Pose a,Pose b,double r){
 double dx=b[0]-a[0],dy=b[1]-a[1],direction=std::atan2(dy,dx),A=mod(a[2]-direction),B=mod(b[2]-direction),d=std::hypot(dx,dy)/r;
 double sa=std::sin(A),sb=std::sin(B),ca=std::cos(A),cb=std::cos(B),cab=std::cos(A-B),p2,tmp,p,t;std::vector<Word> v;
 auto add=[&](std::array<int,3> k,double x,double y,double z){v.push_back({k,{x,y,z},r*(x+y+z)});};
 p2=2+d*d-2*cab+2*d*(sa-sb);if(p2>=-1e-12){tmp=std::atan2(cb-ca,d+sa-sb);add({1,0,1},mod(-A+tmp),std::sqrt(std::max(0.,p2)),mod(B-tmp));}
 p2=2+d*d-2*cab+2*d*(-sa+sb);if(p2>=-1e-12){tmp=std::atan2(ca-cb,d-sa+sb);add({-1,0,-1},mod(A-tmp),std::sqrt(std::max(0.,p2)),mod(-B+tmp));}
 p2=-2+d*d+2*cab+2*d*(sa+sb);if(p2>=-1e-12){p=std::sqrt(std::max(0.,p2));tmp=std::atan2(-ca-cb,d+sa+sb)-std::atan2(-2.,p);add({1,0,-1},mod(-A+tmp),p,mod(-B+tmp));}
 p2=d*d-2+2*cab-2*d*(sa+sb);if(p2>=-1e-12){p=std::sqrt(std::max(0.,p2));tmp=std::atan2(ca+cb,d-sa-sb)-std::atan2(2.,p);add({-1,0,1},mod(A-tmp),p,mod(B-tmp));}
 tmp=(6-d*d+2*cab+2*d*(sa-sb))/8;if(std::abs(tmp)<=1+1e-12){p=mod(2*pi-std::acos(std::clamp(tmp,-1.,1.)));t=mod(A-std::atan2(ca-cb,d-sa+sb)+p/2);add({-1,1,-1},t,p,mod(A-B-t+p));}
 tmp=(6-d*d+2*cab+2*d*(-sa+sb))/8;if(std::abs(tmp)<=1+1e-12){p=mod(2*pi-std::acos(std::clamp(tmp,-1.,1.)));t=mod(-A-std::atan2(ca-cb,d+sa-sb)+p/2);add({1,-1,1},t,p,mod(B-A-t+p));}
 std::stable_sort(v.begin(),v.end(),[](const Word&x,const Word&y){return x.cost<y.cost;});return v;
}
static Pose advance(Pose a,double k,double ds){if(std::abs(k)<1e-12)return {a[0]+ds*std::cos(a[2]),a[1]+ds*std::sin(a[2]),a[2]};double yaw=a[2]+k*ds;return {a[0]+(std::sin(yaw)-std::sin(a[2]))/k,a[1]-(std::cos(yaw)-std::cos(a[2]))/k,angle(yaw)};}
struct Key{int x,y,h;bool operator==(const Key&o)const{return x==o.x&&y==o.y&&h==o.h;}};
struct Hash{size_t operator()(const Key&k)const{return (uint64_t(uint32_t(k.x))*73856093)^(uint64_t(uint32_t(k.y))*19349663)^(uint64_t(k.h)*83492791);}};
struct Node{Pose p;int parent;double curvature,g;};
extern "C" int endpoint_seed_heading(std::uintptr_t address,const unsigned char* hard,int height,int width,const double* input_a,const double* input_b,double resolution,double ox,double oy,double half_length,double half_width,double radius,double budget,int max_nodes,double* output,int capacity,int* stats,const int16_t* labels,const double* dirs,int ndirs,double motion_sign){
 if(!address||!hard||!input_a||!input_b||!output||!stats||radius<=0||budget<=0||max_nodes<1||capacity<2)return -2;
 auto begin=std::chrono::steady_clock::now();auto expired=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>=budget;};
 Checker check=reinterpret_cast<Checker>(address);Pose a{input_a[0],input_a[1],input_a[2]},b{input_b[0],input_b[1],input_b[2]};int expanded=0,shots=0;const double step=.04;
 std::array<double,4> box{std::min(a[0],b[0])-2.,std::max(a[0],b[0])+2.,std::min(a[1],b[1])-2.,std::max(a[1],b[1])+2.};
 auto in=[&](Pose p){return p[0]>=box[0]&&p[0]<=box[1]&&p[1]>=box[2]&&p[1]<=box[3];};
 auto valid=[&](const std::vector<Pose>&v){for(auto p:v){if(!in(p))return false;
  int col=int(std::floor((p[0]-ox)/resolution)),row=height-1-int(std::floor((p[1]-oy)/resolution));
  if(row<0||row>=height||col<0||col>=width)return false;
  int rid=labels[row*width+col];if(rid>=0&&rid<ndirs){double dx=dirs[2*rid],dy=dirs[2*rid+1];
   if(std::hypot(dx,dy)>.5&&motion_sign*(dx*std::cos(p[2])+dy*std::sin(p[2]))<-.085)return false;}}
  return check(hard,height,width,v[0].data(),int(v.size()),resolution,ox,oy,half_length,half_width)==0;};
 if(!valid({a})||!valid({b})){stats[0]=0;stats[1]=0;stats[2]=3;return 0;}
 auto key=[&](Pose p){return Key{int(std::llround((p[0]-box[0])/.0125)),int(std::llround((p[1]-box[2])/.0125)),int(std::llround(mod(p[2])/(2*pi/288)))%288};};
 auto heuristic=[&](Pose p){return std::hypot(p[0]-b[0],p[1]-b[1])+.15*std::abs(angle(p[2]-b[2]));};
 std::vector<Node> nodes;nodes.reserve(max_nodes*2);nodes.push_back({a,-1,0,0});std::unordered_map<Key,double,Hash> costs;costs.reserve(max_nodes*3);costs[key(a)]=0.;
 using Q=std::pair<double,int>;std::priority_queue<Q,std::vector<Q>,std::greater<Q>>queue;queue.emplace(heuristic(a),0);
 while(!queue.empty()&&expanded<max_nodes&&!expired()){
  int id=queue.top().second;queue.pop();Node node=nodes[id];if(node.g>costs[key(node.p)]+1e-8)continue;++expanded;double distance=std::hypot(node.p[0]-b[0],node.p[1]-b[1]);
  if(expanded==1||expanded%40==0||(distance<.5&&std::abs(angle(node.p[2]-b[2]))<1.2)){
   for(double rad:{radius,radius*1.25,radius*1.6}){
    for(const auto& word:words(node.p,b,rad)){
     if(expired())break;
     if(word.cost>distance+3.||node.g+word.cost>std::hypot(a[0]-b[0],a[1]-b[1])+8.)continue;
     std::vector<Pose> curve{node.p};Pose current=node.p;
     for(int j=0;j<3;++j){double length=word.p[j]*rad;int count=std::max(1,int(std::ceil(length/.0249)));for(int i=1;i<=count;++i)curve.push_back(advance(current,word.kind[j]/rad,length*i/count));current=curve.back();}
     if(std::hypot(current[0]-b[0],current[1]-b[1])>1e-7||std::abs(angle(current[2]-b[2]))>1e-7)continue;
     curve.back()=b;++shots;if(!valid(curve))continue;
     std::vector<int> chain;for(int at=id;at>=0;at=nodes[at].parent)chain.push_back(at);std::reverse(chain.begin(),chain.end());std::vector<Pose> path{a};
     for(size_t j=1;j<chain.size();++j){const auto&next=nodes[chain[j]];Pose parent=nodes[chain[j-1]].p;for(int i=1;i<=6;++i)path.push_back(advance(parent,next.curvature,step*i/6));}
     path.insert(path.end(),curve.begin()+1,curve.end());if(int(path.size())>capacity){stats[2]=4;return -1;}
     for(size_t j=0;j<path.size();++j){std::copy(path[j].begin(),path[j].end(),output+3*j);}
     stats[0]=expanded;stats[1]=shots;stats[2]=0;return int(path.size());
    }
   }
  }
  for(double scaled:{0.,1.,-1.,.5,-.5,.75,-.75,.25,-.25}){
   double k=scaled/radius;Pose end=advance(node.p,k,step);if(!in(end))continue;double g=node.g+step*(1.+.015*std::abs(scaled));if(g>std::hypot(a[0]-b[0],a[1]-b[1])+8.)continue;Key ky=key(end);auto previous=costs.find(ky);if(previous!=costs.end()&&g>=previous->second-1e-9)continue;
   std::vector<Pose> edge{node.p};for(int i=1;i<=6;++i)edge.push_back(advance(node.p,k,step*i/6));if(!valid(edge))continue;
   costs[ky]=g;nodes.push_back({end,id,k,g});queue.emplace(g+1.5*heuristic(end),int(nodes.size()-1));
  }
 }
 stats[0]=expanded;stats[1]=shots;stats[2]=expired()?1:(queue.empty()?2:5);return 0;
}
