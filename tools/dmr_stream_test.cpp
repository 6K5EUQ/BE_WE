// DmrDecoder 스트리밍 클래스 검증 — 캡처 샘플을 feed() 로 흘려 CRC-valid 레코드 확인.
//   build: g++ -O2 -std=c++17 -I src tools/dmr_stream_test.cpp -lm -o /tmp/dmr_stream && /tmp/dmr_stream <file> <sr>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <complex>
#include <cmath>
#include <map>
#include "modules/dmr/dmr_decode.hpp"
using cf=std::complex<float>;
static constexpr double PI=3.14159265358979323846;

static std::vector<cf> load_iq(const char* fn,size_t maxc){
    FILE* f=fopen(fn,"rb"); if(!f){perror("open");exit(1);}
    std::vector<cf> v; v.reserve(maxc); const char* dot=strrchr(fn,'.');
    bool ci16 = dot && !strcmp(dot,".sigmf-data");
    if(ci16){ int16_t b[4096]; while(v.size()<maxc){ size_t n=fread(b,2,4096,f); if(n<2)break;
        for(size_t i=0;i+1<n&&v.size()<maxc;i+=2) v.push_back(cf(b[i]/2048.f,b[i+1]/2048.f)); } }
    else { float b[4096]; while(v.size()<maxc){ size_t n=fread(b,4,4096,f); if(n<2)break;
        for(size_t i=0;i+1<n&&v.size()<maxc;i+=2) v.push_back(cf(b[i],b[i+1])); } }
    fclose(f); return v;
}
static std::vector<cf> resample_to(const std::vector<cf>& in,double isr,double osr){
    if(fabs(isr-osr)<1.0) return in; double step=isr/osr; if(in.size()<64)return in;
    size_t nout=(size_t)(in.size()/step)-32; std::vector<cf> out; out.reserve(nout); const int T=16;
    for(size_t m=0;m<nout;m++){ double tin=m*step; long c=(long)floor(tin); double fr=tin-c;
        cf acc(0,0); double ws=0;
        for(int k=-T+1;k<=T;k++){ long idx=c+k; if(idx<0||idx>=(long)in.size())continue;
            double xx=PI*(k-fr); double s=(fabs(xx)<1e-6)?1.0:sin(xx)/xx;
            double a=(double)(k-fr)/T,w=0.42+0.5*cos(PI*a)+0.08*cos(2*PI*a),h=s*w; acc+=in[idx]*(float)h; ws+=h; }
        out.push_back(ws>1e-9?acc/(float)ws:acc); }
    return out;
}

int main(int argc,char**argv){
    if(argc<2){ fprintf(stderr,"usage: %s file [sr]\n",argv[0]); return 1; }
    double isr=argc>2?atof(argv[2]):33684.0; const double OSR=48000.0;
    auto raw=load_iq(argv[1],200'000'000);
    auto x=resample_to(raw,isr,OSR);
    printf("resampled %zu @ %.0f Hz\n",x.size(),OSR);
    // 판별기 → boxcar 정합필터 (워커가 할 일을 테스트에서 재현)
    std::vector<float> dr(x.size(),0.f);
    for(size_t n=1;n<x.size();n++){ float oi=x[n].real(),oq=x[n].imag(),pi=x[n-1].real(),pq=x[n-1].imag();
        dr[n]=atan2f(oq*pi-oi*pq, oi*pi+oq*pq+1e-20f); }
    std::vector<float> mf(dr.size(),0.f);
    { std::vector<double> ps(dr.size()+1,0.0); for(size_t n=0;n<dr.size();n++) ps[n+1]=ps[n]+dr[n];
      int W=(int)llround(OSR/4800.0),H=W/2;
      for(long n=0;n<(long)dr.size();n++){ long lo=n-H<0?0:n-H,hi=n+H+1>(long)dr.size()?(long)dr.size():n+H+1;
        mf[n]=(float)((ps[hi]-ps[lo])/(hi-lo)); } }

    DmrDecoder dec; dec.configure(OSR);
    int nrec=0,ncrc=0,nvoice=0; std::map<uint64_t,int> csbk;
    dec.on_record=[&](const DmrRecord& m){
        nrec++; if(m.crc_ok)ncrc++; if(m.is_voice)nvoice++;
        if(m.csbko>=0 && m.crc_ok){ uint64_t k=((uint64_t)m.dst_id<<24)|m.src_id; csbk[k]++; }
        if(nrec<=14) printf("  rec: %s CC=%d dt=%d csbko=%d flco=%d dst=%u src=%u crc=%d voice=%d\n",
            m.csbko>=0?"CSBK":(m.flco>=0?"LC":"?"),m.color_code,m.data_type,m.csbko,m.flco,m.dst_id,m.src_id,m.crc_ok,m.is_voice);
    };
    for(float s: mf) dec.feed(s);
    printf("\nrecords=%d  crc_ok=%d  voice=%d  distinct CRC-CSBK(dst,src)=%zu\n",nrec,ncrc,nvoice,csbk.size());
    int sh=0; for(auto&kv:csbk){ if(sh++<8) printf("  CSBK dst=%llu src=%llu x%d\n",
        (unsigned long long)(kv.first>>24),(unsigned long long)(kv.first&0xFFFFFF),kv.second); }
    return ncrc>0?0:2;
}
