#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <openssl/sha.h>
namespace three_yd {
inline void u32(std::vector<uint8_t>& out,uint32_t n){for(int i=0;i<4;++i)out.push_back(uint8_t(n>>(8*i)));}
inline void u64(std::vector<uint8_t>& out,uint64_t n){for(int i=0;i<8;++i)out.push_back(uint8_t(n>>(8*i)));}
struct DeltaSnapshot {
 uint64_t epoch,sequence=0;std::string owner;uintptr_t identity=0;uint32_t width=0,height=0;std::vector<uint8_t> previous;
 explicit DeltaSnapshot(uint64_t generation):epoch(generation){}
 std::vector<uint8_t> encode(const uint8_t* current,uint32_t w,uint32_t h,uintptr_t id,const std::string& token,uint64_t want_epoch,uint64_t base){
  if(!w||!h||uint64_t(w)*h>400000000||token.size()!=32||token.find_first_not_of("0123456789abcdef")!=std::string::npos)throw std::invalid_argument("delta snapshot input");
  const size_t n=size_t(w)*h;bool full=owner!=token||identity!=id||width!=w||height!=h||previous.size()!=n||want_epoch!=epoch||base!=sequence;
  std::vector<uint8_t> payload;uint32_t count=0;
  if(!full){
   constexpr uint32_t tile=128;
   for(uint32_t y=0;y<h;y+=tile)for(uint32_t x=0;x<w;x+=tile){
    uint32_t dy=std::min(tile,h-y),dx=std::min(tile,w-x);bool changed=false;
    for(uint32_t j=0;j<dy&&!changed;++j)changed=std::memcmp(current+size_t(y+j)*w+x,previous.data()+size_t(y+j)*w+x,dx)!=0;
    if(!changed)continue;
    ++count;u32(payload,x);u32(payload,y);u32(payload,dx);u32(payload,dy);
    for(uint32_t j=0;j<dy;++j){const auto* p=current+size_t(y+j)*w+x;payload.insert(payload.end(),p,p+dx);}
   }
   // A changed map may be cheaper as a full payload. Still one bounded state.
   if(payload.size()>=n){full=true;payload.clear();count=0;}
  }
  std::vector<uint8_t> out;out.reserve(80+(full?n:payload.size()));
  const char magic[]="3YDACK06";for(int i=0;i<8;++i)out.push_back(static_cast<uint8_t>(magic[i]));
  u64(out,epoch);u64(out,sequence+1);u64(out,full?0:sequence);u32(out,w);u32(out,h);u32(out,count);u32(out,full?1:0);
  uint8_t hash[32];SHA256(current,n,hash);out.insert(out.end(),hash,hash+32);
  if(full)out.insert(out.end(),current,current+n);else out.insert(out.end(),payload.begin(),payload.end());
  // Commit after the complete packet exists. Lost replies force full resync.
  previous.assign(current,current+n);owner=token;identity=id;width=w;height=h;++sequence;
  return out;
 }
};
}
