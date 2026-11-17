// SPDX-License-Identifier: Apache-2.0
#include "delta_snapshot.hpp"
#include <limits>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
static std::mutex guard;
static std::unordered_map<uint64_t,std::unique_ptr<three_yd::DeltaSnapshot>> states;
static std::unordered_set<void*> allocations;
static uint64_t next_handle=1;
static thread_local std::string error;
extern "C" {
const char* tl_delta_error() noexcept { return error.c_str(); }
int tl_delta_create(uint64_t epoch,uint64_t* handle) noexcept {
 try { std::lock_guard<std::mutex> lock(guard); if(!handle) throw std::invalid_argument("null handle output");
  if(!epoch) throw std::invalid_argument("epoch must be nonzero");
  if(!next_handle) throw std::overflow_error("handle space exhausted");
  auto p=std::make_unique<three_yd::DeltaSnapshot>(epoch); auto h=next_handle++;
  states.emplace(h,std::move(p));*handle=h;error.clear();return 0;
 } catch(const std::exception& e){error=e.what();return 1;}catch(...){error="unknown native error";return 2;}
}
int tl_delta_close(uint64_t handle) noexcept {
 try {std::lock_guard<std::mutex> lock(guard);states.erase(handle);error.clear();return 0;}
 catch(const std::exception& e){error=e.what();return 1;}catch(...){error="unknown native error";return 2;}
}
int tl_delta_free(void* ptr) noexcept {
 try {std::lock_guard<std::mutex> lock(guard);if(!ptr)return 0;
  if(!allocations.erase(ptr))throw std::invalid_argument("unknown allocation");
  std::free(ptr);error.clear();return 0;
 }catch(const std::exception& e){error=e.what();return 1;}catch(...){error="unknown native error";return 2;}
}
int tl_delta_encode(uint64_t handle,const uint8_t* data,size_t length,uint32_t w,uint32_t h,
 uint64_t identity,const char* token,size_t token_length,uint64_t epoch,uint64_t base,int sparse,
 void** output,size_t* output_length) noexcept {
 if(output) *output=nullptr;
 if(output_length) *output_length=0;
 try {std::lock_guard<std::mutex> lock(guard);
  if(!output||!output_length||!data||!token)throw std::invalid_argument("null pointer");
  if(!w||!h||uint64_t(w)*h>400000000||uint64_t(w)*h!=length)throw std::invalid_argument("invalid dimensions/data length");
  if(token_length!=32||(sparse!=0&&sparse!=1))throw std::invalid_argument("invalid token length or mode");
  if(identity>std::numeric_limits<uintptr_t>::max())throw std::invalid_argument("identity overflow");
  auto it=states.find(handle);if(it==states.end())throw std::invalid_argument("closed or unknown state");
  if(it->second->sequence==std::numeric_limits<uint64_t>::max())throw std::overflow_error("sequence exhausted");
  // Work on a copy so allocation failure cannot silently advance the live state.
  auto trial=*it->second;
  auto packet=trial.encode(data,w,h,static_cast<uintptr_t>(identity),std::string(token,token_length),epoch,base,sparse!=0);
  void* memory=std::malloc(packet.size());if(!memory)throw std::bad_alloc();
  try {allocations.insert(memory);}catch(...){std::free(memory);throw;}
  std::memcpy(memory,packet.data(),packet.size());
  *it->second=std::move(trial);*output=memory;*output_length=packet.size();error.clear();return 0;
 }catch(const std::exception& e){error=e.what();return 1;}catch(...){error="unknown native error";return 2;}
}
}
