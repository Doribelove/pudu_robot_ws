// SPDX-License-Identifier: Apache-2.0
#include <nav2_core/global_planner.hpp>
#include <nav2_core/exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <openssl/evp.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <set>

namespace trilayer_nav2_planner {
using json=nlohmann::json;
using Clock=std::chrono::steady_clock;
static void fail(const std::string &s){throw nav2_core::PlannerException("TriLayer: "+s);}
static std::string hash(const std::vector<unsigned char>&v){
 unsigned char out[SHA256_DIGEST_LENGTH];SHA256(v.data(),v.size(),out);
 std::ostringstream s;for(auto c:out)s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(c);return s.str();
}
class Planner:public nav2_core::GlobalPlanner {
 rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
 std::atomic<bool> active_{false}; std::mutex operation_; std::string socket_;
 double timeout_=60;uint64_t sequence_=0;
 struct Snapshot {json context;std::string all_hash;std::string encoded;};
 Snapshot snapshot(){
  if(!map_ || map_->getGlobalFrameID()!="map")fail("only map frame supported");
  auto footprint=map_->getRobotFootprint();if(footprint.size()!=4)fail("fixed rectangular footprint required");
  std::set<std::pair<int,int>> corners;for(auto&p:footprint){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||std::abs(p.z)>1e-8||std::abs(std::abs(p.x)-.265)>1e-6||std::abs(std::abs(p.y)-.225)>1e-6)fail("footprint must match P6 half lengths .265/.225 including padding");corners.emplace(p.x>0?1:-1,p.y>0?1:-1);}if(corners.size()!=4)fail("invalid rectangle");
  double area=0;for(size_t i=0;i<4;++i){auto&a=footprint[i];auto&b=footprint[(i+1)%4];area+=a.x*b.y-b.x*a.y;}if(std::abs(std::abs(area)-8*.265*.225)>1e-6)fail("crossed footprint polygon");
  auto m=map_->getCostmap();std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*m->getMutex());
  size_t w=m->getSizeInCellsX(),h=m->getSizeInCellsY();
  if(!w||!h||w>1000000/h||std::abs(m->getResolution()-.05)>1e-8||!std::isfinite(m->getOriginX())||!std::isfinite(m->getOriginY()))fail("unsupported grid geometry");
  std::vector<unsigned char> raw(m->getCharMap(),m->getCharMap()+w*h),hard(raw);
  for(auto &v:hard)v=(v>=253)?1:0;
  std::string encoded(4*((raw.size()+2)/3)+1,' ' );
  EVP_EncodeBlock(reinterpret_cast<unsigned char*>(encoded.data()),raw.data(),raw.size());encoded.pop_back();
  return {{{"frame_id","map"},{"width",w},{"height",h},{"resolution",.05},{"origin",{m->getOriginX(),m->getOriginY()}},{"hard_mask_sha256",hash(hard)},{"costmap_sha256",hash(raw)}},hash(raw),encoded};
 }
 static json pose(const geometry_msgs::msg::PoseStamped&p){
  const auto&a=p.pose.position;const auto&q=p.pose.orientation;
  if(p.header.frame_id!="map"||!std::isfinite(a.x)||!std::isfinite(a.y)||!std::isfinite(a.z)||std::abs(a.z)>1e-8||!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)||!std::isfinite(q.w)||std::abs(q.x)>1e-8||std::abs(q.y)>1e-8||std::abs(q.z*q.z+q.w*q.w-1)>1e-6)fail("requires finite planar normalized map poses");
  return {a.x,a.y,2*std::atan2(q.z,q.w)};
 }
 json exchange(const json&request){
  int fd=::socket(AF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(fd<0)fail("socket creation failed");
  struct Close{int fd;~Close(){::close(fd);}}close{fd};
  auto deadline=Clock::now()+std::chrono::milliseconds(static_cast<int>(timeout_*1000));
  auto wait=[&](short events){while(true){
   if(!active_.load())fail("deactivated during request");
   auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();if(remaining<=0)fail("worker deadline exceeded");
   pollfd p{fd,events,0};int r=::poll(&p,1,static_cast<int>(std::min<int64_t>(remaining,50)));
   if(r<0&&errno==EINTR)continue;if(r<0)fail("poll failed");if(r>0){if(p.revents&events)return;if(p.revents&(POLLERR|POLLHUP|POLLNVAL))fail("worker disconnected");}
  }};
  sockaddr_un addr{};addr.sun_family=AF_UNIX;std::memcpy(addr.sun_path,socket_.c_str(),socket_.size()+1);
  if(::connect(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0){if(errno!=EINPROGRESS)fail("worker unavailable");wait(POLLOUT);int e=0;socklen_t n=sizeof(e);if(getsockopt(fd,SOL_SOCKET,SO_ERROR,&e,&n)||e)fail("worker connection failed");}
  auto data=request.dump()+"\n";size_t offset=0;
  while(offset<data.size()){wait(POLLOUT);auto n=send(fd,data.data()+offset,data.size()-offset,MSG_NOSIGNAL);if(n<0&&(errno==EAGAIN||errno==EINTR))continue;if(n<=0)fail("worker write failed");offset+=n;}
  std::string result;char buf[8192];while(true){wait(POLLIN);auto n=recv(fd,buf,sizeof(buf),0);if(n<0&&(errno==EAGAIN||errno==EINTR))continue;if(n<=0)fail("incomplete worker response");result.append(buf,n);if(result.size()>8*1024*1024)fail("oversized worker response");auto end=result.find('\n');if(end!=std::string::npos){if(end!=result.size()-1)fail("trailing worker response");return json::parse(result);}}
 }
public:
 void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr&parent,std::string name,std::shared_ptr<tf2_ros::Buffer>,std::shared_ptr<nav2_costmap_2d::Costmap2DROS>costmap)override{
  std::lock_guard<std::mutex> lock(operation_);auto node=parent.lock();if(!node||!costmap)fail("missing parent/costmap");node_=parent;map_=costmap;
  if(!node->has_parameter(name+".worker_socket"))node->declare_parameter(name+".worker_socket",std::string(""));
  if(!node->has_parameter(name+".timeout_seconds"))node->declare_parameter(name+".timeout_seconds",60.);
  if(!node->has_parameter(name+".allow_unknown"))node->declare_parameter(name+".allow_unknown",false);
  if(node->get_parameter(name+".allow_unknown").as_bool())fail("allow_unknown must be false");
  socket_=node->get_parameter(name+".worker_socket").as_string();timeout_=node->get_parameter(name+".timeout_seconds").as_double();
  if(socket_.empty()||socket_[0]!='/'||socket_.size()>=sizeof(sockaddr_un::sun_path)||!std::isfinite(timeout_)||timeout_<=0||timeout_>60)fail("explicit absolute socket and timeout (0,60] required");
  if(map_->getLayeredCostmap()->isRolling())fail("rolling costmap unsupported");
  snapshot();
 }
 void activate()override{active_=true;}
 void deactivate()override{active_=false;}
 void cleanup()override{active_=false;std::lock_guard<std::mutex> lock(operation_);map_.reset();node_.reset();}
 nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped&start,const geometry_msgs::msg::PoseStamped&goal)override{
  std::unique_lock<std::mutex> lock(operation_,std::try_to_lock);if(!lock.owns_lock())fail("concurrent request rejected");
  try{
   if(!active_)fail("inactive planner");auto before=snapshot();auto id=std::to_string(::getpid())+"-"+std::to_string(++sequence_);
   auto response=exchange({{"v",1},{"id",id},{"op","plan"},{"start",pose(start)},{"goal",pose(goal)},{"context",before.context},{"snapshot",before.encoded}});
   if(response.at("v")!=1||response.at("id")!=id||!response.at("ok").get<bool>())fail("worker refusal: "+response.value("failure_code",std::string("invalid_response")));
   if(!response.at("audit_valid").get<bool>()||!response.at("dynamic_version").is_number_integer()||response.at("dynamic_version")!=response.at("trajectory_version"))fail("uncertified or stale trajectory");
   if(response.at("frame_id")!="map"||response.at("context")!=before.context)fail("worker context mismatch");
   auto points=response.at("path");if(!points.is_array()||points.size()<2||points.size()>100000)fail("invalid path size");
   auto node=node_.lock();if(!node)fail("parent expired");nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=node->now();
   for(auto&p:points){if(!p.is_array()||p.size()!=3)fail("invalid path point");double x=p[0].get<double>(),y=p[1].get<double>(),a=p[2].get<double>();if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(a))fail("nonfinite path");geometry_msgs::msg::PoseStamped out;out.header=path.header;out.pose.position.x=x;out.pose.position.y=y;out.pose.orientation.z=std::sin(a/2);out.pose.orientation.w=std::cos(a/2);path.poses.push_back(out);}
   auto endpoint_ok=[](const json&p,const json&q){double dx=p[0].get<double>()-q[0].get<double>(),dy=p[1].get<double>()-q[1].get<double>(),da=p[2].get<double>()-q[2].get<double>();return std::hypot(dx,dy)<=.2+1e-9&&std::abs(std::atan2(std::sin(da),std::cos(da)))<=std::acos(-1.)/6+1e-9;};
   if(!endpoint_ok(points.front(),pose(start))||!endpoint_ok(points.back(),pose(goal)))fail("path exceeds original P6 endpoint tolerance .2m/30deg");
   auto after=snapshot();if(!active_||after.context!=before.context||after.all_hash!=before.all_hash)fail("caller costmap changed while planning");return path;
  }catch(const nav2_core::PlannerException&){throw;}catch(const std::exception&e){fail(e.what());}throw nav2_core::PlannerException("unreachable");
 }
};
}
PLUGINLIB_EXPORT_CLASS(trilayer_nav2_planner::Planner,nav2_core::GlobalPlanner)
