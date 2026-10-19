// Query-local continuous-pose search on the ORIGINAL occupancy grid.
// No footprint reduction, no heading-only smoothing, no permanent graph edit.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>
extern "C" long swept_rect(const uint8_t*,int,int,const double*,int,double,double,double,double,double);
namespace {
constexpr double PI=3.141592653589793;
double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
double curvature_at(int k){return std::abs(k)==7?(k>0?2.5:-2.5):k*.4;}
struct Node {double x,y,t,g;int k,phase,parent;double opposing;};
struct Open {double f;int id;bool operator<(const Open&o)const{return f>o.f;}};
}
extern "C" int vehicle_search(const uint8_t* hard,int H,int W,double res,double ox,double oy,
 const int16_t* labels,const float* preference,const float* clearance,const uint8_t* maneuver,int r0,int c0,int h,int w,
 const int16_t* phases,const double* directions,int nphase,const double* start,const double* goal,
 double half_length,double half_width,double min_radius,double budget,int max_nodes,
 double* output,int output_capacity,double* stats,const float* fine_heuristic,int factor){
 const auto tick=std::chrono::steady_clock::now();
 auto elapsed=[&](){return std::chrono::duration<double>(std::chrono::steady_clock::now()-tick).count();};
 auto cell=[&](double x,double y){int c=int(std::floor((x-ox)/res))-c0,r=H-1-int(std::floor((y-oy)/res))-r0;return r>=0&&c>=0&&r<h&&c<w?r*w+c:-1;};
 auto allowed=[&](int p){if(p<0)return false;int lab=labels[p];for(int k=0;k<nphase;k++)if(phases[k]==lab)return !hard[(p/w+r0)*W+p%w+c0];return false;};
 auto direction_ok=[&](int p,double dx,double dy){
  if(maneuver[p])return true;
  for(int k=0;k<nphase;k++)if(phases[k]==labels[p]){
   if(dx*directions[2*k]+dy*directions[2*k+1]>=-std::sin(PI/36)*std::hypot(dx,dy)-1e-6)return true;
  }return false;
 };
 int sc=cell(start[0],start[1]),gc=cell(goal[0],goal[1]);
 if(!allowed(sc)||!allowed(gc))return -1;
 auto fine_h=[&](double x,double y){double ds=res/factor;int cc=int(std::floor((x-ox-c0*res)/ds)),rr=int(std::floor((oy+(H-r0)*res-y)/ds));return rr>=0&&cc>=0&&rr<h*factor&&cc<w*factor?fine_heuristic[rr*w*factor+cc]:std::numeric_limits<float>::infinity();};
 // Grid geodesic guides continuous poses; it is not a collision certificate.
 std::vector<float> heuristic(h*w,std::numeric_limits<float>::infinity());
 for(int heuristic_trial=0;heuristic_trial<3;heuristic_trial++){
 std::fill(heuristic.begin(),heuristic.end(),std::numeric_limits<float>::infinity());
 std::priority_queue<std::pair<float,int>,std::vector<std::pair<float,int>>,std::greater<std::pair<float,int>>> heap;
 heuristic[gc]=0;heap.emplace(0,gc);int dr[8]={-1,-1,-1,0,0,1,1,1},dc[8]={-1,0,1,-1,1,-1,0,1};
 while(!heap.empty()){
  auto [g,p]=heap.top();heap.pop();if(g!=heuristic[p])continue;
  for(int j=0;j<8;j++){int rr=p/w+dr[j],cc=p%w+dc[j];if(rr<0||cc<0||rr>=h||cc>=w)continue;int q=rr*w+cc;
   if(!allowed(q)||(heuristic_trial<2&&clearance[q]<(heuristic_trial==0?std::hypot(half_length,half_width):half_width)))continue;
   // Reverse heuristic propagation tests the forward q -> p transition.
   // A goal-directed point heuristic must not attract a vehicle backwards
   // along its explicitly selected traffic side.
   double dx=-dc[j]*res,dy=dr[j]*res;
   if(!direction_ok(q,dx,dy)||!direction_ok(p,dx,dy))continue;
   // Strongly discourage point-sized gaps in the heuristic rather than
   // assuming a point geodesic is a suitable vehicle route. This is guidance;
   // the exact original rectangle checker is still authoritative.
   float penalty=clearance[q]<half_width?100.f:1.f+std::max(0.,.35-double(clearance[q]))*8.;
   float ng=g+res*(dr[j]&&dc[j]?1.41421356237:1.)*penalty;
   if(ng<heuristic[q]){heuristic[q]=ng;heap.emplace(ng,q);}}
 }
 if(std::isfinite(heuristic[sc])){stats[10]=heuristic_trial;break;}
 }
 stats[0]=elapsed();if(!std::isfinite(fine_h(start[0],start[1])))return -2;
 auto key=[&](const Node&a){
  int pc=cell(a.x,a.y);bool broad=pc>=0&&clearance[pc]>.5;
  double q=broad?2.*res:.5*res;
  int ix=int(std::floor((a.x-ox-c0*res)/q));
  int iy=int(std::floor((a.y-oy-(H-r0-h)*res)/q));
  int bins=broad?72:144;
  int it=int(std::floor((wrap(a.t)+PI)*bins/(2*PI)))%bins;
  return (((((uint64_t(a.phase)*(2*h+2)+iy)*(2*w+2)+ix)*144+it)*15+(a.k+7))*2+broad);
 };
 std::vector<Node> nodes;nodes.reserve(std::min(max_nodes,500000));
 std::unordered_map<uint64_t,int> best;best.reserve(std::min(max_nodes,500000));
 std::priority_queue<Open> open;
 nodes.push_back({start[0],start[1],start[2],0,0,0,-1,0.});best[key(nodes[0])]=0;open.push({heuristic[sc],0});
 int found=-1,expanded=0,collisions=0;double step=.15;double closest=1e100;
 while(!open.empty()){
  if((expanded&255)==0&&(elapsed()>budget||int(nodes.size())>=max_nodes))break;
  int id=open.top().id;open.pop();Node a=nodes[id];if(best[key(a)]!=id)continue;expanded++;
  double remain=std::hypot(a.x-goal[0],a.y-goal[1]);
  if(remain<closest){closest=remain;stats[6]=a.x;stats[7]=a.y;stats[8]=a.t;stats[9]=remain;}
  if(a.phase==nphase-1&&remain<.085&&std::abs(wrap(a.t-goal[2]))<PI/45){found=id;break;}
  for(int dk=-2;dk<=2;dk++){
   int k=a.k+dk;if(k< -7||k>7)continue;double curvature=curvature_at(k);
   if(std::abs(curvature)>1/min_radius)continue;
   if(std::abs(std::atan(.5*curvature)-std::atan(.5*curvature_at(a.k)))>PI/12-1e-6)continue;
   double length=(k==0&&a.k==0&&clearance[cell(a.x,a.y)]>.5&&remain>.5)?step:.05;
   int count=int(std::ceil(length/.025));
   double poses[27];poses[0]=a.x;poses[1]=a.y;poses[2]=a.t;int phase=a.phase;bool valid=true;double opposing=a.opposing;
   for(int s=1;s<=count;s++){
    double d=length*s/count,theta=a.t+curvature*d;
    double x=a.x+(k?(std::sin(theta)-std::sin(a.t))/curvature:d*std::cos(a.t));
    double y=a.y+(k?(-std::cos(theta)+std::cos(a.t))/curvature:d*std::sin(a.t));
    poses[3*s]=x;poses[3*s+1]=y;poses[3*s+2]=wrap(theta);
    int p=cell(x,y);if(p<0){valid=false;break;}int lab=labels[p];
    if(lab!=phases[phase]){if(phase+1<nphase&&lab==phases[phase+1])phase++;else {valid=false;break;}}
    double dx=x-poses[3*(s-1)],dy=y-poses[3*(s-1)+1];
    double rx=directions[2*phase],ry=directions[2*phase+1];
    if(dx*rx+dy*ry< -std::sin(PI/36)*std::hypot(dx,dy)-1e-6){
     int previous=cell(poses[3*(s-1)],poses[3*(s-1)+1]);
     if(!maneuver[p]||previous<0||!maneuver[previous]){valid=false;break;}
     opposing+=std::hypot(dx,dy);if(opposing>2.){valid=false;break;}
    }
   }
   if(!valid)continue;
   if(swept_rect(hard,H,W,poses,count+1,res,ox,oy,half_length,half_width)!=0){collisions++;continue;}
   int p=cell(poses[3*count],poses[3*count+1]);if(!std::isfinite(fine_h(poses[3*count],poses[3*count+1])))continue;
   Node b{poses[3*count],poses[3*count+1],poses[3*count+2],a.g+length*(1.+.08*std::abs(curvature)+.10*preference[p])+.006*std::abs(dk),k,phase,id,opposing};
   uint64_t hash=key(b);auto it=best.find(hash);if(it!=best.end()&&nodes[it->second].g<=b.g)continue;
   int nb=nodes.size();nodes.push_back(b);best[hash]=nb;
   double lower=std::max(double(fine_h(b.x,b.y)),std::hypot(b.x-goal[0],b.y-goal[1]));
   open.push({b.g+8.0*lower,nb});
  }
 }
 stats[1]=expanded;stats[2]=nodes.size();stats[3]=collisions;stats[4]=elapsed();
 if(found<0)return elapsed()>=budget?-3:(int(nodes.size())>=max_nodes?-4:-5);
 std::vector<int> chain;for(int id=found;id>=0;id=nodes[id].parent)chain.push_back(id);std::reverse(chain.begin(),chain.end());
 int used=0;auto emit=[&](double x,double y,double t){if(used>=output_capacity)return false;output[3*used]=x;output[3*used+1]=y;output[3*used+2]=wrap(t);used++;return true;};
 emit(start[0],start[1],start[2]);
 for(size_t j=1;j<chain.size();j++){
  const Node&a=nodes[chain[j-1]],&b=nodes[chain[j]];double k=curvature_at(b.k);
  double chord=std::hypot(b.x-a.x,b.y-a.y),length=k?std::abs(wrap(b.t-a.t)/k):chord;
  int n=std::max(1,int(std::ceil(length/.025)));
  for(int s=1;s<=n;s++){
   double d=length*s/n,theta=a.t+k*d;
   if(!emit(a.x+(k?(std::sin(theta)-std::sin(a.t))/k:d*std::cos(a.t)),
            a.y+(k?(-std::cos(theta)+std::cos(a.t))/k:d*std::sin(a.t)),theta))return -6;
  }
 }
 stats[5]=nodes[found].g;return used;
}

extern "C" int grid_distance(const uint8_t* free,const float* cost,int h,int w,int goal,float ds,float* distance){
 std::fill(distance,distance+h*w,std::numeric_limits<float>::infinity());
 if(goal<0||goal>=h*w||!free[goal])return -1;
 std::priority_queue<std::pair<float,int>,std::vector<std::pair<float,int>>,std::greater<std::pair<float,int>>> heap;
 distance[goal]=0;heap.emplace(0,goal);int dr[8]={-1,-1,-1,0,0,1,1,1},dc[8]={-1,0,1,-1,1,-1,0,1};int expanded=0;
 while(!heap.empty()){
  auto [g,p]=heap.top();heap.pop();if(g!=distance[p])continue;expanded++;
  for(int j=0;j<8;j++){int rr=p/w+dr[j],cc=p%w+dc[j];if(rr<0||cc<0||rr>=h||cc>=w)continue;int q=rr*w+cc;if(!free[q])continue;
   float ng=g+ds*(dr[j]&&dc[j]?1.41421356237f:1.f)*(.5f*(cost[p]+cost[q]));
   if(ng<distance[q]){distance[q]=ng;heap.emplace(ng,q);}}
 }return expanded;
}
