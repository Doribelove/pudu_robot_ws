// SPDX-License-Identifier: Apache-2.0
#include "delta_snapshot.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <sstream>
#include <thread>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace trilayer_master_snapshot {
using GetCostmap = nav2_msgs::srv::GetCostmap;
static uint64_t generation() {
  static std::atomic<uint64_t> counter{static_cast<uint64_t>(
    std::chrono::system_clock::now().time_since_epoch().count()) | 1ULL};
  return counter.fetch_add(1);
}
struct Context {
  // gate always precedes master mutex. updateCosts/Bounds never acquire gate.
  std::mutex gate;
  bool active=false;
  std::atomic<bool> reset_requested{false};
  nav2_costmap_2d::Costmap2D* master=nullptr;
  std::string frame;
  rclcpp::Clock::SharedPtr clock;
  std::unique_ptr<three_yd::DeltaSnapshot> codec;
};
class MasterSnapshotLayer : public nav2_costmap_2d::Layer {
 public:
  ~MasterSnapshotLayer() override {
    deactivate();
    if(executor_)executor_->cancel();
    if(worker_.joinable())worker_.join();
    service_.reset();service_group_.reset();executor_.reset();
  }
  void onInitialize() override {
    auto node=node_.lock();
    if(!node) throw std::runtime_error("snapshot parent node expired");
    declareParameter("service_name", rclcpp::ParameterValue("trilayer_get_costmap"));
    std::string name;node->get_parameter(name_+".service_name",name);
    if(name.empty()||name[0]=='/')throw std::invalid_argument("snapshot service_name must be relative");
    context_=std::make_shared<Context>();
    context_->master=layered_costmap_->getCostmap();
    context_->frame=layered_costmap_->getGlobalFrameID();
    context_->clock=node->get_clock();
    context_->codec=std::make_unique<three_yd::DeltaSnapshot>(generation());
    // Own callback group: destroy its service weak references before plugin unload.
    service_group_=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto context=context_;
    service_=node->create_service<GetCostmap>(name,
      [context](const std::shared_ptr<GetCostmap::Request> request,
                std::shared_ptr<GetCostmap::Response> response) {
        std::lock_guard<std::mutex> gate(context->gate);
        if(!context->active){response->map.metadata.layer="trilayer_snapshot_inactive";return;}
        // The entire metadata/data observation and delta encoding occur under
        // the same mutex used by LayeredCostmap::updateMap, never a topic cache.
        auto* master=context->master;
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*master->getMutex());
        if(context->reset_requested.exchange(false))
          context->codec=std::make_unique<three_yd::DeltaSnapshot>(generation());
        auto w=master->getSizeInCellsX(),h=master->getSizeInCellsY();
        auto* bytes=master->getCharMap();
        if(!w||!h||!bytes||uint64_t(w)*h>400000000){response->map.metadata.layer="trilayer_snapshot_unavailable";return;}
        auto& map=response->map;
        auto now=context->clock->now();
        map.header.stamp=now;map.header.frame_id=context->frame;
        map.metadata.map_load_time=now;map.metadata.update_time=now;
        map.metadata.size_x=w;map.metadata.size_y=h;
        map.metadata.resolution=master->getResolution();
        map.metadata.origin.position.x=master->getOriginX();
        map.metadata.origin.position.y=master->getOriginY();
        map.metadata.origin.orientation.w=1.;map.metadata.layer="master";
        const auto& layer=request->specs.layer;
        const bool v2=layer.rfind("3yd_delta_v2|",0)==0;
        if(v2||layer.rfind("3yd_delta_v1|",0)==0) {
          std::istringstream stream(layer.substr(13));
          std::string token,epoch_text,sequence_text;
          std::getline(stream,token,'|');std::getline(stream,epoch_text,'|');std::getline(stream,sequence_text,'|');
          try {
            size_t a=0,b=0;auto epoch=std::stoull(epoch_text,&a),base=std::stoull(sequence_text,&b);
            if(a!=epoch_text.size()||b!=sequence_text.size())throw std::invalid_argument("invalid sequence");
            map.data=context->codec->encode(bytes,w,h,reinterpret_cast<uintptr_t>(master),token,epoch,base,v2);
            map.metadata.layer=std::string(v2?"3yd_delta_v2|":"3yd_delta_v1|")+token;
          }catch(const std::exception&){map.metadata.layer="3yd_delta_error";map.data.clear();}
        }else map.data.assign(bytes,bytes+size_t(w)*h);
      },rmw_qos_profile_services_default,service_group_);
    executor_=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(service_group_,node->get_node_base_interface());
    worker_=std::thread([executor=executor_]{executor->spin();});
    enabled_=true;current_=true;
  }
  void activate() override {
    if(!context_)return;
    std::lock_guard<std::mutex> gate(context_->gate);
    context_->codec=std::make_unique<three_yd::DeltaSnapshot>(generation());
    context_->active=true;
  }
  void deactivate() override {
    if(!context_)return;
    std::lock_guard<std::mutex> gate(context_->gate);
    context_->active=false; // waits for any reader before parent can clean up
  }
  void reset() override {
    // May run while the caller owns master mutex: do not acquire gate here.
    if(context_)context_->reset_requested.store(true);
  }
  bool isClearable() override {return false;}
  void updateBounds(double,double,double,double*,double*,double*,double*) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D&,int,int,int,int) override {}
 private:
  std::shared_ptr<Context> context_;
  rclcpp::Service<GetCostmap>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread worker_;
};
}
PLUGINLIB_EXPORT_CLASS(trilayer_master_snapshot::MasterSnapshotLayer,nav2_costmap_2d::Layer)
