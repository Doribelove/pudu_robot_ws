// Dispatch independent alternatives to the unchanged original footprint check.
// There is no sweep between candidates, cells, or groups.
#include <cstdint>
using Checker=long(*)(const unsigned char*,int,int,const double*,int,
                     double,double,double,double,double);
extern "C" int first_clear_pose_groups(std::uintptr_t address,
    const unsigned char* hard,int height,int width,const double* centers,
    const double* yaws,int groups,int yaw_count,const double* offsets,int offset_count,
    double resolution,double ox,double oy,double half_length,double half_width,
    int* indices,int* checked) {
    if(!address||!hard||!centers||!yaws||!offsets||!indices||!checked||
       height<=0||width<=0||groups<0||yaw_count<=0||offset_count<=0)return -2;
    auto check=reinterpret_cast<Checker>(address);
    for(int group=0;group<groups;++group){
        indices[group]=-1;checked[group]=0;
        for(int offset=0;offset<offset_count && indices[group]<0;++offset){
            const double x=centers[2*group]+offsets[2*offset];
            const double y=centers[2*group+1]+offsets[2*offset+1];
            for(int yaw=0;yaw<yaw_count;++yaw){
                const double pose[3]={x,y,yaws[group*yaw_count+yaw]};
                ++checked[group];
                if(check(hard,height,width,pose,1,resolution,ox,oy,half_length,half_width)==0){
                    indices[group]=offset*yaw_count+yaw;break;
                }
            }
        }
    }
    return 0;
}
