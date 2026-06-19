// DMR 음성 → WAV 검증 하니스 (mbelib AMBE 디코드).
//   build: g++ -O2 -std=c++17 -I src -I libs/mbelib tools/dmr_voice_wav.cpp libs/mbelib/*.c -lm -o /tmp/dmr_voice
//   run:   /tmp/dmr_voice <file.sigmf-data> <in_sr_hz> [out.wav]
// 캡처 → 판별기 → boxcar 정합필터 → DmrDecoder(on_voice) → mbelib → 8kHz WAV.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <complex>
#include <cmath>
#include "modules/dmr/dmr_decode.hpp"
#include "modules/dmr/dmr_ambe.hpp"
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
static void write_wav(const char* fn, const std::vector<short>& pcm, int sr){
    FILE* f=fopen(fn,"wb"); if(!f){perror("wav");return;}
    uint32_t dlen=(uint32_t)(pcm.size()*2), rlen=36+dlen, brate=sr*2;
    fwrite("RIFF",1,4,f); fwrite(&rlen,4,1,f); fwrite("WAVE",1,4,f);
    fwrite("fmt ",1,4,f); uint32_t f16=16; uint16_t pcm1=1,ch1=1,bps=16,ba=2; uint32_t srr=sr;
    fwrite(&f16,4,1,f); fwrite(&pcm1,2,1,f); fwrite(&ch1,2,1,f); fwrite(&srr,4,1,f);
    fwrite(&brate,4,1,f); fwrite(&ba,2,1,f); fwrite(&bps,2,1,f);
    fwrite("data",1,4,f); fwrite(&dlen,4,1,f); fwrite(pcm.data(),2,pcm.size(),f);
    fclose(f);
}

int main(int argc,char**argv){
    if(argc<3){ fprintf(stderr,"usage: %s file.sigmf-data in_sr_hz [out.wav]\n",argv[0]); return 1; }
    double isr=atof(argv[2]); const double OSR=48000.0;
    const char* wav=argc>3?argv[3]:"/tmp/dmr_voice.wav";
    auto raw=load_iq(argv[1],200'000'000);
    auto x=resample_to(raw,isr,OSR);
    printf("resampled %zu @ %.0f Hz\n",x.size(),OSR);
    // 채널 LPF (선형위상 FIR, 윈도드싱크) — DMR ~±6.5kHz 만 통과, 인접노이즈 제거.
    // IIR(위상왜곡) 대신 FIR 로 ISI 없이. (워커 DDC 가 할 일을 하니스에서 재현)
    { const int N=63; const double fc=6500.0/OSR; std::vector<double> h(N); double hs=0;
      for(int i=0;i<N;i++){ int k=i-N/2; double s=(k==0)?2*fc:sin(2*PI*fc*k)/(PI*k);
        double w=0.54-0.46*cos(2*PI*i/(N-1)); h[i]=s*w; hs+=h[i]; }
      for(int i=0;i<N;i++) h[i]/=hs;
      std::vector<cf> y(x.size());
      for(long n=0;n<(long)x.size();n++){ cf acc(0,0);
        for(int i=0;i<N;i++){ long idx=n-N/2+i; if(idx>=0&&idx<(long)x.size()) acc+=x[idx]*(float)h[i]; }
        y[n]=acc; }
      x.swap(y); }
    std::vector<float> dr(x.size(),0.f);
    for(size_t n=1;n<x.size();n++){ float oi=x[n].real(),oq=x[n].imag(),pi=x[n-1].real(),pq=x[n-1].imag();
        dr[n]=atan2f(oq*pi-oi*pq, oi*pi+oq*pq+1e-20f); }
    std::vector<float> mf(dr.size(),0.f);
    { std::vector<double> ps(dr.size()+1,0.0); for(size_t n=0;n<dr.size();n++) ps[n+1]=ps[n]+dr[n];
      int W=(int)llround(OSR/4800.0),H=W/2;
      for(long n=0;n<(long)dr.size();n++){ long lo=n-H<0?0:n-H,hi=n+H+1>(long)dr.size()?(long)dr.size():n+H+1;
        mf[n]=(float)((ps[hi]-ps[lo])/(hi-lo)); } }

    // omega 측정 (파일 declared SR 반올림 흡수 — 라이브는 SDR SR 정확해 불필요).
    // sync 부호상관 최대가 되는 omega → 그 값으로 configure (strobe 정렬 → 전 sync 검출).
    double bestOm=OSR/4800.0;
    { int8_t ps[dmr::N_SYNC][24];
      for(int p=0;p<dmr::N_SYNC;p++){ int8_t s[24]; dmr::sync_to_syms(dmr::SYNC_PATTERNS[p].bits48,s);
        for(int k=0;k<24;k++) ps[p][k]=(s[k]<0)?-1:1; }
      auto sc=[&](double st,double om)->int{ float w[24];
        for(int k=0;k<24;k++){ double pp=st+k*om; long i=(long)pp; float f=(float)(pp-i);
          w[k]=(i>=0&&i+1<(long)mf.size())?mf[i]*(1-f)+mf[i+1]*f:0.f; }
        int best=-1; for(int p=0;p<dmr::N_SYNC;p++){ int m0=0,m1=0;
          for(int k=0;k<24;k++){ int s=(w[k]<0)?-1:1; if(s==ps[p][k])m0++;else m1++; }
          if(m0>best)best=m0; if(m1>best)best=m1; } return best; };
      int bestScore=-1; double lim=std::min((double)mf.size()-300.0, 500000.0);
      for(double om=OSR/4860.0; om<=OSR/4790.0; om+=0.01)
        for(double st=1; st+24*om<lim; st+=1){ int s=sc(st,om); if(s>bestScore){bestScore=s;bestOm=om;} }
      printf("measured omega=%.3f (nominal %.3f, score %d/24)\n", bestOm, OSR/4800.0, bestScore);
    }
    DmrDecoder dec; dec.configure(4800.0*bestOm);
    dmr::DmrAmbeDecoder ambe;
    std::vector<short> pcm; int voiceFrames=0, calls=0; long errSum=0, errHi=0;
    dec.on_voice=[&](const uint8_t* frames,int nf,bool nc){
        if(nc){ ambe.reset(); calls++; }
        for(int f=0;f<nf;f++){ short out[160]; int e=ambe.decode(frames+f*36,out);
            errSum+=e; if(e>=4) errHi++;
            pcm.insert(pcm.end(),out,out+160); voiceFrames++; }
    };
    int recs=0; dec.on_record=[&](const DmrRecord&){ recs++; };
    for(float s: mf) dec.feed(s);

    printf("DBG detected=%ld extracted=%ld voice=%ld data=%ld\n",dec.dbgDet,dec.dbgExtract,dec.dbgVoice,dec.dbgData);
    printf("voice frames=%d (calls=%d)  audio=%.2fs  data records=%d\n",
        voiceFrames, calls, pcm.size()/8000.0, recs);
    if(voiceFrames) printf("AMBE FEC errs: avg=%.2f/frame  high-err(>=4)=%ld (%.0f%%) → %s\n",
        (double)errSum/voiceFrames, errHi, 100.0*errHi/voiceFrames,
        errHi*3<voiceFrames?"양호(intelligible 기대)":"오류많음(약신호/슬라이싱)");
    if(!pcm.empty()){ write_wav(wav, pcm, 8000); printf("WAV: %s (%zu samples @ 8kHz)\n",wav,pcm.size()); }
    else printf("음성 프레임 없음 — 음성버스트 미검출(제어/idle 캡처?)\n");
    return pcm.empty()?2:0;
}
