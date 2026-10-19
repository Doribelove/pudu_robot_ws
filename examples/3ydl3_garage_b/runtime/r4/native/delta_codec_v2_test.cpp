#include "delta_snapshot.hpp"
struct Holder {three_yd::DeltaSnapshot state;std::vector<uint8_t> out;explicit Holder(uint64_t e):state(e){}};
extern "C" {
void* codec_create(uint64_t epoch){return new Holder(epoch);}
const uint8_t* codec_encode(void* ptr,const uint8_t* data,uint32_t w,uint32_t h,uint64_t id,const char* owner,uint64_t epoch,uint64_t base,uint64_t* size){auto& v=*static_cast<Holder*>(ptr);v.out=v.state.encode(data,w,h,id,owner,epoch,base,true);*size=v.out.size();return v.out.data();}
void codec_destroy(void* ptr){delete static_cast<Holder*>(ptr);}
}
