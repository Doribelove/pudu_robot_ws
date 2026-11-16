#include <cstddef>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <immintrin.h>

struct ByteView {
    const std::uint8_t* data;
    std::ptrdiff_t row_stride;
    std::ptrdiff_t col_stride;
    std::uint8_t at(std::ptrdiff_t r, std::ptrdiff_t c) const {
        return data[r * row_stride + c * col_stride];
    }
};

// Local counters avoid aliasing flags with the exported count buffer.
// Contiguous columns are the production path; other strides stay exact.
template<bool contiguous, bool previous>
static void classify(const ByteView* v, std::uint8_t* __restrict__ flags,
    std::ptrdiff_t rows, std::ptrdiff_t cols,
    std::uint64_t* counts, std::int64_t* bbox) {
    std::uint64_t n0=0,n1=0,n2=0,n3=0,n4=0;
    std::int64_t x0=cols,y0=rows,x1=0,y1=0;
    for (std::ptrdiff_t y=0;y<rows;++y) {
        const std::uint8_t* p[6];
        for (int j=0;j<6;++j) p[j]=v[j].data+y*v[j].row_stride;
        std::ptrdiff_t first=cols,last=-1;
        for (std::ptrdiff_t x=0;x<cols;++x) {
            auto at=[&](int j) { return p[j][contiguous ? x : x*v[j].col_stride]; };
            const bool dirty=!previous || at(5)!=at(2);
            const bool h=at(3)!=0,s=at(4)!=0;
            const bool affected=(at(1)!=0) | dirty | h | s;
            const bool hard=affected & (h | (at(0)==100));
            const bool soft=affected & s & !hard;
            const bool ordinary=affected & !hard & !soft;
            flags[y*cols+x]=affected|(hard<<1)|(soft<<2)|(dirty<<3)|(ordinary<<4);
            n0+=affected;n1+=hard;n2+=soft;n3+=dirty;n4+=ordinary;
            if (dirty) { first=std::min(first,x);last=x; }
        }
        if (last>=0) {
            x0=std::min<std::int64_t>(x0,first);x1=std::max<std::int64_t>(x1,last+1);
            y0=std::min<std::int64_t>(y0,y);y1=y+1;
        }
    }
    counts[0]=n0;counts[1]=n1;counts[2]=n2;counts[3]=n3;counts[4]=n4;
    bbox[0]=x0;bbox[1]=y0;bbox[2]=x1;bbox[3]=y1;
}

// Runtime-dispatched AVX2 byte operations. No approximate arithmetic.
__attribute__((target("avx2,popcnt")))
static void classify_vector(const ByteView* v,std::uint8_t* flags,
    std::ptrdiff_t rows,std::ptrdiff_t cols,bool previous,
    std::uint64_t* counts,std::int64_t* bbox) {
    std::uint64_t n[5]={0,0,0,0,0};
    std::int64_t x0=cols,y0=rows,x1=0,y1=0;
    const auto zero=_mm256_setzero_si256(),ones=_mm256_set1_epi8(-1);
    for (std::ptrdiff_t y=0;y<rows;++y) {
        const std::uint8_t* p[6];
        for(int j=0;j<6;++j)p[j]=v[j].data+y*v[j].row_stride;
        std::ptrdiff_t first=cols,last=-1,x=0;
        for (;x+32<=cols;x+=32) {
            auto load=[&](int j) __attribute__((target("avx2"))) {
                return _mm256_loadu_si256(reinterpret_cast<const __m256i*>(p[j]+x));
            };
            auto truth=[&](__m256i a) __attribute__((target("avx2"))) {
                return _mm256_xor_si256(_mm256_cmpeq_epi8(a,zero),ones);
            };
            auto h=truth(load(3)),soft_source=truth(load(4));
            auto dirty=previous ? _mm256_xor_si256(_mm256_cmpeq_epi8(load(5),load(2)),ones):ones;
            auto affected=_mm256_or_si256(_mm256_or_si256(truth(load(1)),dirty),_mm256_or_si256(h,soft_source));
            auto hard=_mm256_and_si256(affected,_mm256_or_si256(h,_mm256_cmpeq_epi8(load(0),_mm256_set1_epi8(100))));
            auto soft=_mm256_andnot_si256(hard,_mm256_and_si256(affected,soft_source));
            auto ordinary=_mm256_andnot_si256(_mm256_or_si256(hard,soft),affected);
            __m256i masks[5]={affected,hard,soft,dirty,ordinary};
            unsigned dm=0;auto packed=zero;
            for(int j=0;j<5;++j){
                unsigned bits=static_cast<unsigned>(_mm256_movemask_epi8(masks[j]));
                n[j]+=__builtin_popcount(bits);
                if(j==3)dm=bits;
                packed=_mm256_or_si256(packed,_mm256_and_si256(masks[j],_mm256_set1_epi8(1<<j)));
            }
            _mm256_storeu_si256(reinterpret_cast<__m256i*>(flags+y*cols+x),packed);
            if(dm){first=std::min(first,x+__builtin_ctz(dm));last=x+31-__builtin_clz(dm);}
        }
        for(;x<cols;++x){
            const bool dirty=!previous || p[5][x]!=p[2][x];
            const bool h=p[3][x]!=0,ss=p[4][x]!=0;
            const bool affected=(p[1][x]!=0)|dirty|h|ss;
            const bool hard=affected&(h|(p[0][x]==100));
            const bool soft=affected&ss&!hard,ordinary=affected&!hard&!soft;
            flags[y*cols+x]=affected|(hard<<1)|(soft<<2)|(dirty<<3)|(ordinary<<4);
            n[0]+=affected;n[1]+=hard;n[2]+=soft;n[3]+=dirty;n[4]+=ordinary;
            if(dirty){first=std::min(first,x);last=x;}
        }
        if(last>=0){x0=std::min<std::int64_t>(x0,first);x1=std::max<std::int64_t>(x1,last+1);y0=std::min<std::int64_t>(y0,y);y1=y+1;}
    }
    std::copy(n,n+5,counts);bbox[0]=x0;bbox[1]=y0;bbox[2]=x1;bbox[3]=y1;
}

extern "C" int vector_ack_available(){
    return __builtin_cpu_supports("avx2") && __builtin_cpu_supports("popcnt");
}

extern "C" void classify_ack(const ByteView* v,std::uint8_t* flags,
    std::ptrdiff_t rows,std::ptrdiff_t cols,int has_previous,
    std::uint64_t* counts,std::int64_t* bbox) {
    bool contiguous=true;
    for (int j=0;j<6;++j) contiguous=contiguous && v[j].col_stride==1;
    if (contiguous && vector_ack_available()) {
        classify_vector(v,flags,rows,cols,has_previous,counts,bbox);
    } else if (contiguous) {
        if (has_previous) classify<true,true>(v,flags,rows,cols,counts,bbox);
        else classify<true,false>(v,flags,rows,cols,counts,bbox);
    } else {
        if (has_previous) classify<false,true>(v,flags,rows,cols,counts,bbox);
        else classify<false,false>(v,flags,rows,cols,counts,bbox);
    }
}

extern "C" int equal_ack(const ByteView* a, const ByteView* b,
    std::ptrdiff_t rows, std::ptrdiff_t cols) {
    for (std::ptrdiff_t y = 0; y < rows; ++y) {
        if (a->col_stride == 1 && b->col_stride == 1) {
            if (std::memcmp(a->data+y*a->row_stride, b->data+y*b->row_stride, cols)) return 0;
        } else {
            for (std::ptrdiff_t x = 0; x < cols; ++x)
                if (a->at(y,x) != b->at(y,x)) return 0;
        }
    }
    return 1;
}
