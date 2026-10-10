// Independent closed-cell SAT check. Every continuous interpolated segment is
// enclosed by a midpoint oriented rectangle with a proven displacement bound.
#include <cmath>
#include <algorithm>
#include <cstdint>
static double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
extern "C" long swept_rect(const uint8_t* hard,int h,int w,const double* p,int n,
 double res,double ox,double oy,double hl,double hw) {
 if(n<1 || res<=0) return -1;
 const double radius=std::hypot(hl,hw), cell=res*.5;
 long checks=0;
 for(int i=0;i<std::max(1,n-1);++i) {
  const double *a=p+3*i,*b=p+3*std::min(n-1,i+1);
  const double da=wrap(b[2]-a[2]);
  const double x=(a[0]+b[0])*.5,y=(a[1]+b[1])*.5,t=a[2]+da*.5;
  if(!std::isfinite(x+y+t)) return -1;
  const double e=std::hypot(b[0]-a[0],b[1]-a[1])*.5+2*radius*std::sin(std::abs(da)*.25)+1e-10;
  const double l=hl+e,v=hw+e,c=std::cos(t),s=std::sin(t),ac=std::abs(c),as=std::abs(s);
  const double ex=l*ac+v*as,ey=l*as+v*ac;
  int x0=std::floor((x-ex-ox)/res),x1=std::floor((x+ex-ox)/res);
  int y0=std::floor((y-ey-oy)/res),y1=std::floor((y+ey-oy)/res);
  if(x0<0||y0<0||x1>=w||y1>=h) return i+1;
  for(int yy=y0;yy<=y1;++yy)for(int xx=x0;xx<=x1;++xx) {
   if(!hard[(h-1-yy)*w+xx])continue;
   ++checks;
   double dx=ox+(xx+.5)*res-x,dy=oy+(yy+.5)*res-y;
   if(std::abs(dx)<=ex+cell && std::abs(dy)<=ey+cell &&
      std::abs(dx*c+dy*s)<=l+cell*(ac+as) &&
      std::abs(-dx*s+dy*c)<=v+cell*(ac+as)) return i+1;
  }
 }
 return 0;
}
