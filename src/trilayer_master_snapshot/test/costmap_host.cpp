// SPDX-License-Identifier: Apache-2.0
// Test-only stock Costmap2DROS host. No controller/action server/robot driver.
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  const char* value=std::getenv("TRILAYER_TEST_NAMESPACE");
  std::string ns=value?value:"/trilayer_master_test";
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args","-r","__ns:="+ns});
  options.parameter_overrides({
    {"global_frame","map"},{"robot_base_frame","map"},
    {"plugins",std::getenv("TRILAYER_TEST_NO_SNAPSHOT")?std::vector<std::string>{"static_layer"}:std::vector<std::string>{"static_layer","snapshot"}},
    {"static_layer.plugin","nav2_costmap_2d::StaticLayer"},
    {"static_layer.map_topic",ns+"/input_map"},
    {"static_layer.subscribe_to_updates",true},
    {"static_layer.map_subscribe_transient_local",true},
    {"snapshot.plugin","trilayer_master_snapshot::MasterSnapshotLayer"},
    {"snapshot.service_name","trilayer_get_costmap"},
    {"update_frequency",20.0},{"publish_frequency",0.0},
    {"track_unknown_space",true},{"always_send_full_costmap",true},
    {"resolution",1.0},{"width",4},{"height",4},{"origin_x",-1.0},{"origin_y",-1.0}
  });
  // Keep stock layer code loaded until NodeBase releases callback weak refs.
  // A stock-StaticLayer-only control reproduces a late CallbackGroup destructor
  // crash without this supported pluginlib lifetime guard on this installation.
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> stock_lifetime("nav2_costmap_2d","nav2_costmap_2d::Layer");
  stock_lifetime.loadLibraryForClass("nav2_costmap_2d::StaticLayer");
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  auto helper=std::make_shared<rclcpp::Node>("snapshot_test_helper",ns);
  auto locked=helper->create_publisher<std_msgs::msg::Bool>("master_locked",10);
  auto hold=helper->create_service<std_srvs::srv::Trigger>("hold_master",
    [map,locked](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){
      auto* costmap=map->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> guard(*costmap->getMutex());
      std_msgs::msg::Bool msg;msg.data=true;locked->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(350));
      msg.data=false;locked->publish(msg);res->success=true;
    });
  auto executor=std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(),2);
  executor->add_node(map->get_node_base_interface());executor->add_node(helper);
  std::thread spin([&]{executor->spin();});
  int result=0;
  try {
    if(map->configure().id()!=2)throw std::runtime_error("configure failed");
    if(map->activate().id()!=3)throw std::runtime_error("activate failed");
    std::cout<<"HOST_ACTIVE "<<ns<<std::endl;
    int seconds=40;if(const char* n=std::getenv("TRILAYER_TEST_SECONDS"))seconds=std::stoi(n);
    if(seconds<1||seconds>120)throw std::runtime_error("test duration outside 1..120");
    for(int i=0;i<seconds*10&&rclcpp::ok();++i)std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(rclcpp::ok())map->deactivate();
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;result=1;}
  executor->cancel();spin.join();executor.reset();
  // Do not destroy plugin services concurrently with an executor wait set.
  if(rclcpp::ok()&&map->get_current_state().id()==2)map->cleanup();
  rclcpp::shutdown();return result;
}
