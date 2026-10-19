#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

// Same four-axis rectangle/cell SAT and growth order as V0, using doubles.
// No approximate obstacle filtering or fast-math is used.
extern "C" int curve_rectangles_center(const uint8_t* hard, int h, int w,
 double res, double ox, double oy, const double* points,
 const double* tangents, int count, const double* offsets, int noff,
 double seconds, double* bounds, double* axes) {
 const auto deadline=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
 const double cell=res/2;
 for(int i=0;i<count;++i) {
  if(i%32==0 && std::chrono::steady_clock::now()>=deadline)return 1;
  const double px=points[2*i],py=points[2*i+1];
  double tx=tangents[2*i],ty=tangents[2*i+1],norm=std::hypot(tx,ty);
  if(norm<1e-10){tx=1;ty=0;norm=1;}
  tx/=norm;ty/=norm;const double nx=-ty,ny=tx;
  int col=std::floor((px-ox)/res),row=h-1-std::floor((py-oy)/res);
  if(row<0||row>=h||col<0||col>=w||hard[row*w+col])return 2;
  const int rad=std::ceil(3./res);
  struct Cell {double x,y; int row,col;};
  std::vector<Cell> obs;
  std::vector<size_t> rowbegin(2*rad+2,0);
  const int roirow0=std::max(0,row-rad),roirow1=std::min(h,row+rad+1);
  for(int r=roirow0;r<roirow1;++r){
   rowbegin[r-roirow0]=obs.size();
   for(int c=std::max(0,col-rad);c<std::min(w,col+rad+1);++c)
    if(hard[r*w+c])obs.push_back({ox+(c+.5)*res-px,oy+(h-r-.5)*res-py,r,c});
  }
  rowbegin[roirow1-roirow0]=obs.size();
  const double projected=cell*(std::abs(tx)+std::abs(ty));
  double dx=0,dy=0;
  auto clear=[&](const std::array<double,4>& e){
   double mx=((e[1]-e[0])*tx+(e[3]-e[2])*nx)/2;
   double my=((e[1]-e[0])*ty+(e[3]-e[2])*ny)/2;
   double ht=(e[0]+e[1])/2,hn=(e[2]+e[3])/2;
   double wx=ht*std::abs(tx)+hn*std::abs(nx),wy=ht*std::abs(ty)+hn*std::abs(ny);
   double cx=px+dx+mx,cy=py+dy+my;
   if(cx-wx<ox||cx+wx>ox+w*res||cy-wy<oy||cy+wy>oy+h*res)return false;
   // Conservative integer bounds only skip cells strictly outside the
   // original world-axis SAT. All retained cells use the original arithmetic.
   const int cr0=std::max(roirow0,int(std::floor(h-.5-(cy+wy+cell-oy)/res))-1);
   const int cr1=std::min(roirow1,int(std::ceil(h-.5-(cy-wy-cell-oy)/res))+2);
   const int cc0=int(std::floor((cx-wx-cell-ox)/res-.5))-1;
   const int cc1=int(std::ceil((cx+wx+cell-ox)/res-.5))+1;
   for(int rr=cr0;rr<cr1;++rr){
    auto first=obs.begin()+rowbegin[rr-roirow0],last=obs.begin()+rowbegin[rr-roirow0+1];
    auto begin=std::lower_bound(first,last,cc0,[](const Cell& c,int x){return c.col<x;});
    for(auto it=begin;it!=last&&it->col<=cc1;++it){
    const auto& o=*it;
    double x=o.x-dx,y=o.y-dy;
    if(std::abs(x-mx)>wx+cell||std::abs(y-my)>wy+cell)continue;
    double ot=x*tx+y*ty,on=x*nx+y*ny;
    if(ot>=-e[0]-projected&&ot<=e[1]+projected&&on>=-e[2]-projected&&on<=e[3]+projected)return false;
    }
   }
   return true;
  };
  std::array<double,4> ext{.001,.001,.001,.001};
  if(!clear(ext)){
   bool found=false;
   for(int k=0;k<noff;++k){dx=offsets[2*k];dy=offsets[2*k+1];if(clear(ext)){found=true;break;}}
   if(!found){dx=dy=0;ext={.001,.001,.001,.001};}
  }
  for(double step:{.1,.025}){
   for(int pass=0;pass<int(1.4/step)+1;++pass){
    bool changed=false;
    for(int k:{2,3,0,1}){auto test=ext;test[k]+=step;if(test[k]<=1.401&&clear(test)){ext=test;changed=true;}}
    if(!changed)break;
   }
  }
  double ct=(px+dx)*tx+(py+dy)*ty,cn=(px+dx)*nx+(py+dy)*ny;
  bounds[4*i]=ct-ext[0];bounds[4*i+1]=ct+ext[1];bounds[4*i+2]=cn-ext[2];bounds[4*i+3]=cn+ext[3];
  axes[4*i]=tx;axes[4*i+1]=ty;axes[4*i+2]=nx;axes[4*i+3]=ny;
 }
 return 0;
}
