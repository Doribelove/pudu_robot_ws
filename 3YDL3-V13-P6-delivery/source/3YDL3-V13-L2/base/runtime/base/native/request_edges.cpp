// Exact float64 operations from PrecomputedEdgeState._rebuild_costs.
// No reassociation, fused multiply-add, fast-math or approximate heuristics.
#include <cstdint>
#include <limits>
extern "C" int request_edges(int64_t n,int64_t width,const int16_t* delta,
 const double* direction,const double* potential,double weight,const int64_t* stage,
 const int64_t* offsets,const int64_t* sides,const double* thresholds,
 const double* lengths,int16_t sentinel,double* out) {
 if(n<0||width<1||!delta||!direction||!potential||!offsets||!sides||!thresholds||!lengths||!out)return 1;
 for(int64_t a=0;a<n;++a)for(int64_t j=0;j<width;++j){
  double cost=std::numeric_limits<double>::infinity();
  int16_t d=delta[a*width+j];
  if(d!=sentinel){
   int64_t b=a+d;if(b<0||b>=n)return 2;
   double dr=double(offsets[2*j]),dc=double(offsets[2*j+1]);
   bool legal=(dc*direction[2*a]-dr*direction[2*a+1]>=thresholds[j]) &&
              (dc*direction[2*b]-dr*direction[2*b+1]>=thresholds[j]);
   if(stage)legal=legal&&stage[b]>=stage[a];
   if(dr!=0&&dc!=0)legal=legal&&delta[a*width+sides[2*j]]!=sentinel&&delta[a*width+sides[2*j+1]]!=sentinel;
   if(legal)cost=lengths[j]*(1.+(weight*(potential[a]+potential[b]))*.5);
  }
  out[a*width+j]=cost;
 }
 return 0;
}
