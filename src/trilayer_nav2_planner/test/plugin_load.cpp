// SPDX-License-Identifier: Apache-2.0
#include <pluginlib/class_loader.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/exceptions.hpp>
#include <iostream>
#include <cstdlib>
int main(){
 try{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  auto plugin=loader.createSharedInstance("trilayer_nav2_planner/Planner");
  auto actual=loader.getClassLibraryPath("trilayer_nav2_planner/Planner");
  const char * expected=std::getenv("TRILAYER_TEST_PREFIX");
  if(expected && actual.rfind(std::string(expected)+"/",0)!=0){std::cerr<<"wrong plugin origin: "<<actual<<"\n";return 1;}
  std::cout<<"PLUGIN_ORIGIN "<<actual<<"\n";
  plugin->deactivate();
  try{geometry_msgs::msg::PoseStamped p;plugin->createPlan(p,p);std::cerr<<"inactive planner accepted request\n";return 1;}
  catch(const nav2_core::PlannerException&){std::cout<<"PASS real pluginlib load; inactive request rejected, no worker invoked\n";}
  plugin->cleanup();return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
