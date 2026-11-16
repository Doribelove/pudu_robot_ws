#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <openssl/sha.h>
namespace three_yd {
inline void u32(std::vector<uint8_t>& out,uint32_t n){for(int i=0;i<4;++i)out.push_back(uint8_t(n>>(8*i)));}
inline void u64(std::vector<uint8_t>& out,uint64_t n){for(int i=0;i<8;++i)out.push_back(uint8_t(n>>(8*i)));}
// A constant tile owns one observed byte. Varying tiles own every original byte.
// Shared immutable blocks make unchanged snapshots copy only their small index.
struct SnapshotTile {
 uint8_t constant=0;
 std::shared_ptr<const std::vector<uint8_t>> bytes;
 bool matches(const uint8_t* current,uint32_t stride,uint32_t dx,uint32_t dy) const {
  for(uint32_t j=0;j<dy;++j){const auto* p=current+size_t(j)*stride;
   if(bytes){if(std::memcmp(p,bytes->data()+size_t(j)*dx,dx))return false;}
   else if(!std::all_of(p,p+dx,[this](uint8_t v){return v==constant;}))return false;
  }return true;
 }
 static SnapshotTile read(const uint8_t* current,uint32_t stride,uint32_t dx,uint32_t dy){
  SnapshotTile result;result.constant=*current;bool uniform=true;
  for(uint32_t j=0;j<dy&&uniform;++j){const auto* p=current+size_t(j)*stride;
   uniform=std::all_of(p,p+dx,[&](uint8_t v){return v==result.constant;});}
  if(!uniform){auto data=std::make_shared<std::vector<uint8_t>>(size_t(dx)*dy);
   for(uint32_t j=0;j<dy;++j)std::memcpy(data->data()+size_t(j)*dx,current+size_t(j)*stride,dx);
   result.bytes=std::move(data);
  }return result;
 }
};
struct DeltaSnapshot {
 uint64_t epoch,sequence=0;std::string owner;uintptr_t identity=0;uint32_t width=0,height=0;
 std::vector<SnapshotTile> previous;
 explicit DeltaSnapshot(uint64_t generation):epoch(generation){}
 size_t stored_bytes()const{size_t n=previous.capacity()*sizeof(SnapshotTile);for(const auto& t:previous)if(t.bytes)n+=t.bytes->capacity();return n;}
 std::vector<uint8_t> encode(const uint8_t* current,uint32_t w,uint32_t h,uintptr_t id,const std::string& token,uint64_t want_epoch,uint64_t base,bool sparse_full=false){
  if(!w||!h||uint64_t(w)*h>400000000||token.size()!=32||token.find_first_not_of("0123456789abcdef")!=std::string::npos)throw std::invalid_argument("delta snapshot input");
  constexpr uint32_t tile=128;const size_t n=size_t(w)*h,nt=size_t((w+tile-1)/tile)*((h+tile-1)/tile);
  bool full=owner!=token||identity!=id||width!=w||height!=h||previous.size()!=nt||want_epoch!=epoch||base!=sequence;
  std::vector<SnapshotTile> next;next.reserve(nt);std::vector<uint8_t> payload;uint32_t count=0;size_t k=0;
  for(uint32_t y=0;y<h;y+=tile)for(uint32_t x=0;x<w;x+=tile,++k){
   uint32_t dy=std::min(tile,h-y),dx=std::min(tile,w-x);const auto* p=current+size_t(y)*w+x;
   const bool changed=full||!previous[k].matches(p,w,dx,dy);
   next.push_back(changed?SnapshotTile::read(p,w,dx,dy):previous[k]);
   if(!changed || (full&&(!sparse_full||(!next.back().bytes&&next.back().constant==254))))continue;
   ++count;u32(payload,x);u32(payload,y);u32(payload,dx);u32(payload,dy);
   for(uint32_t j=0;j<dy;++j){const auto* row=p+size_t(j)*w;payload.insert(payload.end(),row,row+dx);}
  }
  // A delta larger than a full frame follows the original dense-full fallback.
  // v2 full frames omit only tiles actually observed to equal byte 254.
  if(!full&&payload.size()>=n){
   full=true;payload.clear();count=0;k=0;
   if(sparse_full)for(uint32_t y=0;y<h;y+=tile)for(uint32_t x=0;x<w;x+=tile,++k){
    if(!next[k].bytes&&next[k].constant==254)continue;
    uint32_t dy=std::min(tile,h-y),dx=std::min(tile,w-x);
    ++count;u32(payload,x);u32(payload,y);u32(payload,dx);u32(payload,dy);
    for(uint32_t j=0;j<dy;++j){const auto* row=current+size_t(y+j)*w+x;payload.insert(payload.end(),row,row+dx);}
   }
  }
  bool closed_full=full&&sparse_full&&payload.size()<n;
  if((!full&&payload.size()>=n)||(full&&!closed_full)){full=true;closed_full=false;payload.clear();count=0;}
  std::vector<uint8_t> out;out.reserve(80+((full&&!closed_full)?n:payload.size()));
  const char* magic=sparse_full?"3YDACK07":"3YDACK06";for(int i=0;i<8;++i)out.push_back(static_cast<uint8_t>(magic[i]));
  u64(out,epoch);u64(out,sequence+1);u64(out,full?0:sequence);u32(out,w);u32(out,h);u32(out,count);u32(out,closed_full?2:(full?1:0));
  uint8_t hash[32];SHA256(current,n,hash);out.insert(out.end(),hash,hash+32);
  if(full&&!closed_full)out.insert(out.end(),current,current+n);else out.insert(out.end(),payload.begin(),payload.end());
  // Commit only after packet construction and the actual full-frame SHA succeed.
  previous.swap(next);owner=token;identity=id;width=w;height=h;++sequence;return out;
 }
};
}
