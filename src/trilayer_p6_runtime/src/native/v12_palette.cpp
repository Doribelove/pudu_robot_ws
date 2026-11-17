#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>
#include <array>
template<int Bits> static inline int decode_row(const uint8_t* raw,size_t first,size_t n,
 const uint8_t* values,size_t colors,uint8_t* out){
 constexpr size_t Factor=8/Bits;constexpr int Shift=Bits==1?3:Bits==2?2:Bits==4?1:0;
 for(size_t x=0;x<n;x++){
  size_t at=first+x;unsigned code=(raw[at>>Shift]>>((Factor-1-(at&(Factor-1)))*Bits))&((1u<<Bits)-1);
  if(code>=colors)return 2;out[x]=values[code];
 }
 return 0;
}
extern "C" int v12_palette_mask(const uint8_t* packed,size_t packed_bytes,size_t cells,int bits,
 const uint8_t* values,size_t colors,uint8_t* out){
 if(!packed||!values||!out||!colors||(bits!=1&&bits!=2&&bits!=4&&bits!=8))return 1;
 const size_t factor=8/bits;const uint8_t mask=(1u<<bits)-1;
 if((cells+factor-1)/factor>packed_bytes)return 3;
 for(size_t i=0;i<cells;i++){
  const unsigned code=(packed[i/factor]>>((factor-1-i%factor)*bits))&mask;
  if(code>=colors)return 2;out[i]=values[code];
 }
 return 0;
}
extern "C" int v12_palette_points(const uint8_t* packed,size_t packed_bytes,size_t cells,const int64_t* indices,
 size_t n,int bits,const uint8_t* palette,size_t colors,int item,uint8_t* out){
 if(!packed||!indices||!palette||!out||!colors||(item!=1&&item!=2)||(bits!=1&&bits!=2&&bits!=4&&bits!=8))return 1;
 const size_t factor=8/bits;const uint8_t mask=(1u<<bits)-1;
 if((cells+factor-1)/factor>packed_bytes)return 3;
 for(size_t i=0;i<n;i++){
  if(indices[i]<0||(size_t)indices[i]>=cells)return 3;
  const size_t index=indices[i];const int shift=bits==1?3:bits==2?2:bits==4?1:0;
  const unsigned code=(packed[index>>shift]>>((factor-1-(index&(factor-1)))*bits))&mask;
  if(code>=colors)return 2;std::memcpy(out+i*item,palette+code*item,item);
 }
 return 0;
}

// Descriptors: kind,r,c,h,w,bits,colors,codes,palette,constant,code_bytes.
static int uniform(const uint64_t* d,const uint8_t* table,uint8_t& value){
 if(d[0]==0){if(d[9]>65535)return -1;value=table[d[9]];return 1;}
 if(d[0]==2)return 0;
 if(d[0]!=1||!d[8]||!d[6]||d[6]>256)return -1;
 const auto* palette=reinterpret_cast<const uint16_t*>(d[8]);value=table[palette[0]];
 for(size_t i=1;i<d[6];i++)if(table[palette[i]]!=value)return 0;
 return 1;
}
extern "C" int v12_mask_needed(const uint64_t* desc,size_t n,const uint8_t* table,uint8_t* needed){
 if(!desc||!table||!needed)return 1;
 for(size_t i=0;i<n;i++){uint8_t v=0;int result=uniform(desc+i*11,table,v);if(result<0)return 2;needed[i]=result==0;}
 return 0;
}
extern "C" int v12_full_mask(const uint64_t* desc,size_t n,const uint8_t* table,size_t h,size_t w,uint8_t* out){
 if(!desc||!table||!out)return 1;
 std::vector<uint8_t> all_same(n),values(n);
 std::vector<std::array<uint8_t,256>> palette_values(n);
 for(size_t i=0;i<n;i++){
  const auto* d=desc+i*11;int all=uniform(d,table,values[i]);
  size_t r=d[1],c=d[2],dh=d[3],dw=d[4],cells=dh*dw;
  if(all<0||!dh||!dw||r+dh>h||c+dw>w)return 2;
  all_same[i]=all;if(all)continue;
  if(!d[7])return 3;
  if(d[0]==2){if(cells*2>d[10])return 3;}else{
   int bits=d[5];if(bits!=1&&bits!=2&&bits!=4&&bits!=8)return 3;
   size_t factor=8/bits;if((cells+factor-1)/factor>d[10])return 3;
   const auto* palette=reinterpret_cast<const uint16_t*>(d[8]);
   for(size_t k=0;k<d[6];k++)palette_values[i][k]=table[palette[k]];
  }
 }
 // Descriptor bands are the complete row-major tile partition. Traverse
 // output rows contiguously instead of revisiting every page once per tile.
 for(size_t begin=0;begin<n;){
  size_t end=begin+1;const auto* first=desc+begin*11;
  while(end<n&&desc[end*11+1]==first[1])end++;
  for(size_t y=0;y<first[3];y++)for(size_t i=begin;i<end;i++){
   const auto* d=desc+i*11;if(d[3]!=first[3])return 2;
   auto* target=out+(d[1]+y)*w+d[2];size_t dw=d[4];
   if(all_same[i]){std::memset(target,values[i],dw);continue;}
   if(d[0]==2){const auto* raw=reinterpret_cast<const uint16_t*>(d[7])+y*dw;for(size_t x=0;x<dw;x++)target[x]=table[raw[x]];}
   else{
    const auto* raw=reinterpret_cast<const uint8_t*>(d[7]);int code=0;
    switch(d[5]){
     case 1:code=decode_row<1>(raw,y*dw,dw,palette_values[i].data(),d[6],target);break;
     case 2:code=decode_row<2>(raw,y*dw,dw,palette_values[i].data(),d[6],target);break;
     case 4:code=decode_row<4>(raw,y*dw,dw,palette_values[i].data(),d[6],target);break;
     case 8:code=decode_row<8>(raw,y*dw,dw,palette_values[i].data(),d[6],target);break;
     default:return 3;
    }
    if(code)return code;
   }
  }
  begin=end;
 }
 return 0;
}
