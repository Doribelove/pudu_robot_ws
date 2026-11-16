#include <nav2_costmap_2d/layered_costmap.hpp>
#include <malloc.h>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
struct Pattern : nav2_costmap_2d::Layer {
 bool filter;explicit Pattern(bool f):filter(f){enabled_=true;current_=true;}
 void reset() override {} bool isClearable() override{return true;}
 void updateBounds(double,double,double,double* a,double* b,double* c,double* d)override{*a=-10000;*b=-10000;*c=10000;*d=10000;}
 void updateCosts(nav2_costmap_2d::Costmap2D& m,int a,int b,int c,int d)override{
  for(int y=b;y<d;++y)for(int x=a;x<c;++x)m.setCost(x,y,filter?m.getCost(x,y)+2:(x+3*y)%250);
 }
};
size_t allocated(){auto m=mallinfo2();return m.uordblks+m.hblkhd;}
int main(int argc,char** argv){
 bool compact=argc>1&&std::string(argv[1])=="1";setenv("THREEYD_V12_NATIVE_COMPACT",compact?"1":"0",1);
 nav2_costmap_2d::LayeredCostmap m("map",false,true);m.addPlugin(std::make_shared<Pattern>(false));
 auto before=allocated();m.resizeMap(3000,3000,.05,-2.,-3.);auto used=allocated()-before;
 if(compact?(used>10000000):(used<17000000))throw std::runtime_error("unused primary allocation");
 auto check=[&](bool filter){m.updateMap(0,0,0);auto* c=m.getCostmap();for(unsigned y=0;y<c->getSizeInCellsY();++y)for(unsigned x=0;x<c->getSizeInCellsX();++x)if(c->getCost(x,y)!=(x+3*y)%250+(filter?2:0))throw std::runtime_error("cost mismatch");};
 check(false);m.resizeMap(129,257,.1,1,2);check(false);
 m.addFilter(std::make_shared<Pattern>(true));check(true); // late activation after two geometries
 m.resizeMap(301,193,.2,-4,6);check(true);
 nav2_costmap_2d::LayeredCostmap r("map",true,true);r.addPlugin(std::make_shared<Pattern>(false));r.resizeMap(129,257,.1,0,0);r.updateMap(10,12,0);r.addFilter(std::make_shared<Pattern>(true));r.updateMap(11,13,0);
 std::cout<<"{\"compact\":"<<compact<<",\"allocation_bytes\":"<<used<<",\"verified\":true}"<<std::endl;
}
