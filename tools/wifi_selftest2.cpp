// Front-end 셀프테스트: 합성 LTF+SIGNAL(20.0MSPS) 생성 → 채널추정+등화+L-SIG 디코드.
//   front-end DSP(FFT bin / 채널추정 / MRC 등화 / pilot / deinterleave) 격리 검증.
//   build: g++ -O2 -I src tools/wifi_selftest2.cpp -lfftw3f -lm -o /tmp/st2
#include <cstdio>
#include <vector>
#include <complex>
#include <cmath>
#include <fftw3.h>
#include "modules/wifi/wifi_ofdm.hpp"
using cf=std::complex<float>;
using namespace wifi_ofdm;
static constexpr double PI=3.14159265358979323846;
static inline int bin_of(int sc){ return (sc+64)%64; }

static void ifft64(const cf* f, cf* t){
    fftwf_complex *i=fftwf_alloc_complex(64),*o=fftwf_alloc_complex(64);
    fftwf_plan p=fftwf_plan_dft_1d(64,i,o,FFTW_BACKWARD,FFTW_ESTIMATE);
    for(int k=0;k<64;k++){i[k][0]=f[k].real();i[k][1]=f[k].imag();}
    fftwf_execute(p); for(int k=0;k<64;k++) t[k]=cf(o[k][0],o[k][1])/8.0f; // 임의 스케일
    fftwf_destroy_plan(p);fftwf_free(i);fftwf_free(o);
}
static void fft64(const cf* t, cf* f){
    fftwf_complex *i=fftwf_alloc_complex(64),*o=fftwf_alloc_complex(64);
    fftwf_plan p=fftwf_plan_dft_1d(64,i,o,FFTW_FORWARD,FFTW_ESTIMATE);
    for(int k=0;k<64;k++){i[k][0]=t[k].real();i[k][1]=t[k].imag();}
    fftwf_execute(p); for(int k=0;k<64;k++) f[k]=cf(o[k][0],o[k][1]);
    fftwf_destroy_plan(p);fftwf_free(i);fftwf_free(o);
}
static void conv_encode(const std::vector<uint8_t>& in, std::vector<uint8_t>& out){
    int reg=0; out.clear(); auto par=[&](int g){int x=reg&g,c=0;while(x){c^=x&1;x>>=1;}return c;};
    for(uint8_t b:in){ reg=((reg<<1)|b)&0x7f; out.push_back(par(0133)); out.push_back(par(0171)); }
}
static std::vector<int> data_scs(){ std::vector<int> v; for(int sc=-26;sc<=26;sc++){if(sc==0||is_pilot(sc))continue;v.push_back(sc);} return v; }
static const int PSC[4]={-21,-7,7,21}; static const int PVAL[4]={1,1,1,-1};

int main(int argc,char**argv){
    double cfo_hz = argc>1?atof(argv[1]):0.0;
    auto DSC=data_scs();
    // SIGNAL 24비트: 6Mbps(1101)|rsv0|len=100|parity|tail0
    std::vector<uint8_t> sig(24,0); int rb[4]={1,1,0,1}; for(int i=0;i<4;i++)sig[i]=rb[i];
    int len=100; for(int i=0;i<12;i++) sig[5+i]=(len>>i)&1;
    int par=0; for(int i=0;i<17;i++)par^=sig[i]; sig[17]=par;
    std::vector<uint8_t> coded; conv_encode(sig,coded);
    uint8_t inter[48]; for(int k=0;k<48;k++){int i=3*(k%16)+k/16; inter[i]=coded[k];}
    // SIGNAL freq: data BPSK(bit0→-1,bit1→+1) + pilots(p0=1) + DC 0
    cf sigf[64]={}; for(int b=0;b<64;b++) sigf[b]=0;
    for(int k=0;k<48;k++) sigf[bin_of(DSC[k])] = inter[k]? cf(1,0):cf(-1,0);
    for(int pi=0;pi<4;pi++) sigf[bin_of(PSC[pi])] = cf((float)(PVAL[pi]*PILOT_POL[0]),0);
    cf sigt[64]; ifft64(sigf,sigt);
    // LTF time
    cf ltff[64]={}; for(int sc=-26;sc<=26;sc++) ltff[bin_of(sc)]=cf(LTF_FREQ[sc+26],0);
    cf ltft[64]; ifft64(ltff,ltft);
    // 랜덤 per-subcarrier 채널 적용 (LTF+SIGNAL 동일) → 등화 검증. arg[2]=="chan"
    if(argc>2 && !strcmp(argv[2],"chan")){
        unsigned seed=12345; auto rnd=[&](){ seed=seed*1103515245+12345; return ((seed>>16)&0x7fff)/32768.0f; };
        cf Htrue[64]; for(int b=0;b<64;b++){ float mag=0.3f+1.4f*rnd(); float ph=(rnd()-0.5f)*6.28f; Htrue[b]=std::polar(mag,ph); }
        cf lf2[64],sf2[64]; for(int b=0;b<64;b++){ lf2[b]=ltff[b]*Htrue[b]; sf2[b]=sigf[b]*Htrue[b]; }
        ifft64(lf2,ltft); ifft64(sf2,sigt);
        printf("[랜덤 채널 적용]\n");
    }
    // 패킷 조립: [GI2 32 = ltft[32..63]][T1 64][T2 64][SIG GI 16 = sigt[48..63]][SIG 64]
    std::vector<cf> x;
    for(int i=32;i<64;i++) x.push_back(ltft[i]);
    for(int i=0;i<64;i++) x.push_back(ltft[i]);
    for(int i=0;i<64;i++) x.push_back(ltft[i]);
    for(int i=48;i<64;i++) x.push_back(sigt[i]);
    for(int i=0;i<64;i++) x.push_back(sigt[i]);
    // CFO 주입
    for(size_t n=0;n<x.size();n++) x[n]*=std::polar(1.0f,(float)(2*PI*cfo_hz*n/20e6));
    // 리샘플 왕복 테스트: 20.0→30.72→20.0 (실제 30.72 SDR 캡처 시뮬)
    if(argc>2 && !strcmp(argv[2],"resamp")){
        auto rs=[](const std::vector<cf>& in,double isr,double osr){
            double step=isr/osr; size_t nout=(size_t)(in.size()/step)-32; std::vector<cf> o; o.reserve(nout); int T=16;
            for(size_t m=0;m<nout;m++){ double tin=m*step; long c=(long)floor(tin); double fr=tin-c; cf a(0,0); double ws=0;
                for(int k=-T+1;k<=T;k++){ long id=c+k; if(id<0||id>=(long)in.size())continue; double xx=PI*(k-fr); double s=(fabs(xx)<1e-6)?1.0:sin(xx)/xx;
                    double aa=(double)(k-fr)/T; double w=0.42+0.5*cos(PI*aa)+0.08*cos(2*PI*aa); double h=s*w; a+=in[id]*(float)h; ws+=h; }
                o.push_back(ws>1e-9?a/(float)ws:a); } return o; };
        auto up=rs(x,20.0e6,30.72e6); x=rs(up,30.72e6,20.0e6);
        printf("[리샘플 왕복 20→30.72→20]\n");
    }

    // ── 디코드 (wifi_offline 과 동일) ──
    long ltf1=32, ltf2=96, sigsym=160+16;
    cf ac=0; for(int i=0;i<64;i++) ac+=x[ltf2+i]*std::conj(x[ltf1+i]);
    double cfo=std::arg(ac)/(2*PI*64/20e6);
    auto decfo=[&](long n){ return std::polar(1.0f,(float)(-2*PI*cfo*(n-ltf1)/20e6)); };
    cf y1[64],y2[64],t1[64],t2[64];
    for(int i=0;i<64;i++){t1[i]=x[ltf1+i]*decfo(ltf1+i); t2[i]=x[ltf2+i]*decfo(ltf2+i);}
    fft64(t1,y1); fft64(t2,y2);
    cf H[64]; for(int sc=-26;sc<=26;sc++){int b=bin_of(sc); cf avg=(y1[b]+y2[b])*0.5f; float r=LTF_FREQ[sc+26]; H[b]= r!=0? avg/r : cf(0,0);}
    cf st[64],sf[64]; for(int i=0;i<64;i++) st[i]=x[sigsym+i]*decfo(sigsym+i);
    fft64(st,sf);
    double sph=0,sscph=0;
    for(int pi=0;pi<4;pi++){int b=bin_of(PSC[pi]); cf m=sf[b]*std::conj(H[b])*(float)(PVAL[pi]*PILOT_POL[0]); sph+=std::arg(m); sscph+=std::arg(m)*PSC[pi];}
    double a=sph/4, bs=sscph/980.0;
    float subc[48]; double isum=0,rsum=0;
    for(int k=0;k<48;k++){int b=bin_of(DSC[k]); cf cor=std::polar(1.0f,(float)(-(a+bs*DSC[k]))); cf m=sf[b]*std::conj(H[b])*cor; subc[k]=-m.real(); isum+=fabs(m.imag()); rsum+=fabs(m.real());}
    float codeds[48]; for(int k=0;k<48;k++){int i=3*(k%16)+k/16; codeds[k]=subc[i];}
    Viterbi vit; std::vector<uint8_t> bits; vit.decode(codeds,24,bits);
    int rr=0; for(int i=0;i<4;i++)rr|=bits[i]<<i;
    int rl=0; for(int i=0;i<12;i++)rl|=bits[5+i]<<i;
    int rp=0; for(int i=0;i<17;i++)rp^=bits[i];
    printf("cfo_inject=%.0fkHz cfo_est=%.1fkHz |imag|/|real|=%.3f\n",cfo_hz/1e3,cfo/1e3,rsum>1e-9?isum/rsum:0);
    printf("decoded rate=0x%X(기대0xB) len=%d(기대100) parity=%d tail0=%d → %s\n",
        rr,rl,rp==bits[17],(bits[18]|bits[19]|bits[20]|bits[21]|bits[22]|bits[23])==0,
        (rr==0xB&&rl==100)?"FRONT-END OK":"FRONT-END BUG");
    return 0;
}
