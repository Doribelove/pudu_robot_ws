#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <random>
#include <iostream>
#include <cstring>
#include <cstdlib>
class Probe: public nav2_costmap_2d::InflationLayer {
public:
 Probe(bool unknown, bool around) {enabled_=true; resolution_=0.05;
 inflation_radius_=0.55; inscribed_radius_=0.225; cost_scaling_factor_=3.;
 inflate_unknown_=unknown;inflate_around_unknown_=around;cell_inflation_radius_=11;
 computeCaches();}
 size_t retained() {size_t n=0;for(auto & b:inflation_cells_)n+=b.capacity()*sizeof(nav2_costmap_2d::CellData);return n;}
};
int main(int argc,char **argv) {
 std::mt19937 rng(611);size_t tests=0,peak_old=0,peak_new=0;
 for(int flags=0;flags<4;++flags) {
  Probe old(flags&1,flags&2),compact(flags&1,flags&2);
  for(int run=0;run<200;++run) {
   unsigned int w=33+rng()%130,h=31+rng()%110;
   nav2_costmap_2d::Costmap2D a(w,h,.05,-10.,-10.,0),b(w,h,.05,-10.,-10.,0);
   auto *p=a.getCharMap();
   for(unsigned i=0;i<w*h;++i) {auto r=rng()%100;p[i]=run%5==0?254:(r<35?254:r<45?255:r<70?0:rng()%254);}
   std::memcpy(b.getCharMap(),p,w*h);
   int x0=run%3?int(rng()%(w/2)):0,y0=run%3?int(rng()%(h/2)):0;
   int x1=run%3?w/2+rng()%(w/2):w,y1=run%3?h/2+rng()%(h/2):h;
   setenv("THREEYD_R11_MODE","control",1);old.updateCosts(a,x0,y0,x1,y1);
   setenv("THREEYD_R11_MODE","A",1);compact.updateCosts(b,x0,y0,x1,y1);
   if(std::memcmp(a.getCharMap(),b.getCharMap(),w*h)) {std::cerr<<"FAIL "<<flags<<" "<<run<<"\n";return 1;}
   peak_old=std::max(peak_old,old.retained());peak_new=std::max(peak_new,compact.retained());++tests;
  }
 }
 // Size of the actual map is supplied optionally for a cold all-lethal probe.
 if(argc==3) {
  unsigned w=std::stoul(argv[1]),h=std::stoul(argv[2]);
  Probe old(false,false),compact(false,false);
  nav2_costmap_2d::Costmap2D a(w,h,.05,0.,0.,254),b(w,h,.05,0.,0.,254);
  setenv("THREEYD_R11_MODE","control",1);old.updateCosts(a,0,0,w,h);
  setenv("THREEYD_R11_MODE","A",1);compact.updateCosts(b,0,0,w,h);
  if(std::memcmp(a.getCharMap(),b.getCharMap(),w*h))return 2;
  std::cout<<"cold_queue_bytes "<<old.retained()<<" "<<compact.retained()<<"\n";
 }
 std::cout<<"PASS "<<tests<<" random_full_partial_unknown_cases; retained_bytes "<<peak_old<<" "<<peak_new<<"\n";
}
