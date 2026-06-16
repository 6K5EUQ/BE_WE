// 오프라인 802.11 OFDM 비콘 디코더 하니스 — 캡처 .cf32 에 대해 개발/검증.
//   build: g++ -O2 -I src tools/wifi_offline.cpp -lfftw3f -lm -o /tmp/wifi_offline
//   run:   /tmp/wifi_offline <file.cf32> <in_sr_hz>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <complex>
#include <cmath>
#include <fftw3.h>
#include "modules/wifi/wifi_ofdm.hpp"
using cf=std::complex<float>;
using namespace wifi_ofdm;
static constexpr double PI=3.14159265358979323846;

static std::vector<cf> load_cf32(const char* fn, size_t maxc){
    FILE* f=fopen(fn,"rb"); if(!f){perror("open");exit(1);}
    std::vector<cf> v; v.reserve(maxc);
    const char* dot=strrchr(fn,'.');
    bool ci16 = dot && !strcmp(dot,".sigmf-data");   // BladeRF SigMF = ci16_le
    if(ci16){ int16_t buf[4096];
        while(v.size()<maxc){ size_t n=fread(buf,sizeof(int16_t),4096,f); if(n<2)break;
            for(size_t i=0;i+1<n&&v.size()<maxc;i+=2) v.push_back(cf(buf[i]/2048.f,buf[i+1]/2048.f)); }
        printf("[ci16_le 입력]\n");
    } else { float buf[4096];
        while(v.size()<maxc){ size_t n=fread(buf,sizeof(float),4096,f); if(n<2)break;
            for(size_t i=0;i+1<n&&v.size()<maxc;i+=2) v.push_back(cf(buf[i],buf[i+1])); } }
    fclose(f); return v;
}
static std::vector<cf> resample_to(const std::vector<cf>& in,double in_sr,double out_sr){
    if(fabs(in_sr-out_sr)<1.0) return in;
    double step=in_sr/out_sr; size_t nout=(size_t)(in.size()/step)-32;
    std::vector<cf> out; out.reserve(nout); const int T=16;  // ±16 = 32탭 (위상 일관성↑)
    for(size_t m=0;m<nout;m++){
        double tin=m*step; long c=(long)floor(tin); double frac=tin-c;
        cf acc(0,0); double wsum=0;
        for(int k=-T+1;k<=T;k++){ long idx=c+k; if(idx<0||idx>=(long)in.size())continue;
            double x=PI*(k-frac); double s=(fabs(x)<1e-6)?1.0:sin(x)/x;
            double a=(double)(k-frac)/T; // Blackman
            double w=0.42+0.5*cos(PI*a)+0.08*cos(2*PI*a);
            double h=s*w; acc+=in[idx]*(float)h; wsum+=h; }
        out.push_back(wsum>1e-9?acc/(float)wsum:acc);
    }
    return out;
}
struct Pkt{ size_t pos; float m; };
static std::vector<Pkt> detect(const std::vector<cf>& x){
    size_t N=x.size(); std::vector<Pkt> out; const int L=16;
    auto win=[&](size_t n)->float{ cf C=0; double P=0;
        for(int k=0;k<L;k++){ if(n+k+L>=N)break; C+=x[n+k]*std::conj(x[n+k+L]); P+=std::norm(x[n+k+L]); }
        return P>1e-9?std::abs(C)/(float)P:0.f; };
    bool inb=false; size_t pstart=0; float pmax=0;
    for(size_t n=0;n+2*L<N;n+=4){ float m=win(n);
        if(m>0.6f){ if(!inb){inb=true;pstart=n;pmax=m;}else pmax=std::max(pmax,m);}
        else if(inb){ inb=false; if(n-pstart>=80) out.push_back({pstart,pmax}); } }
    return out;
}

// ── FFT 64 (FFTW) ──
struct FFT64{
    fftwf_complex *in,*out; fftwf_plan p;
    FFT64(){ in=fftwf_alloc_complex(64); out=fftwf_alloc_complex(64);
        p=fftwf_plan_dft_1d(64,in,out,FFTW_FORWARD,FFTW_ESTIMATE); }
    ~FFT64(){ fftwf_destroy_plan(p); fftwf_free(in); fftwf_free(out); }
    void run(const cf* t,cf* f){ for(int i=0;i<64;i++){in[i][0]=t[i].real();in[i][1]=t[i].imag();}
        fftwf_execute(p); for(int i=0;i<64;i++) f[i]=cf(out[i][0],out[i][1]); }
};
// subcarrier(-26..26) → FFT bin
static inline int bin_of(int sc){ return (sc+64)%64; }
// data subcarrier 순서 리스트(48): -26..-1,1..26 (DC + pilot 제외)
static std::vector<int> data_scs(){
    std::vector<int> v; for(int sc=-26;sc<=26;sc++){ if(sc==0)continue; if(is_pilot(sc))continue; v.push_back(sc);} return v;
}
static const int PILOT_SC[4]={-21,-7,7,21};
static const int PILOT_VAL[4]={1,1,1,-1};
static void conv_enc(const uint8_t* in,int n,uint8_t* out){
    int reg=0; auto par=[&](int g){int x=reg&g,c=0;while(x){c^=x&1;x>>=1;}return c;};
    for(int i=0;i<n;i++){ reg=((reg<<1)|in[i])&0x7f; out[2*i]=par(0133); out[2*i+1]=par(0171);} }

int main(int argc,char**argv){
    if(argc<2){ fprintf(stderr,"usage: %s file.cf32 [in_sr_hz]\n",argv[0]); return 1; }
    double in_sr=argc>2?atof(argv[2]):20.48e6;
    auto raw=load_cf32(argv[1],40'000'000);
    printf("loaded %zu @ %.4f MSPS\n",raw.size(),in_sr/1e6);
    auto x=resample_to(raw,in_sr,20.0e6);
    if(argc>3 && !strcmp(argv[3],"conj")){ for(auto& v:x) v=std::conj(v); printf("[spectral inversion test: conjugated]\n"); }
    printf("resampled %zu @ 20.0 MSPS\n",x.size());
    auto pk=detect(x);
    printf("detected %zu preambles\n",pk.size());

    // L-LTF 시간영역 기준 (freq→IFFT)
    cf ltf_t[64];
    { fftwf_complex *fi=fftwf_alloc_complex(64),*fo=fftwf_alloc_complex(64);
      fftwf_plan ip=fftwf_plan_dft_1d(64,fi,fo,FFTW_BACKWARD,FFTW_ESTIMATE);
      for(int i=0;i<64;i++){fi[i][0]=0;fi[i][1]=0;}
      for(int sc=-26;sc<=26;sc++){ fi[bin_of(sc)][0]=LTF_FREQ[sc+26]; }
      fftwf_execute(ip); for(int i=0;i<64;i++) ltf_t[i]=cf(fo[i][0],fo[i][1])/64.0f;
      fftwf_destroy_plan(ip); fftwf_free(fi); fftwf_free(fo); }

    FFT64 fft; Viterbi vit; auto DSC=data_scs();
    int tried=0,lsig_ok=0,beacons_ok=0; int rate_hist[16]={0};

    for(auto& P : pk){
        if(tried>=400) break;
        // 후보 LTF 시작 = pstart 부근. STF plateau 끝~LTF. ±48 윈도우 정밀 동기.
        long base=(long)P.pos;
        // ── coarse CFO: STF lag-16 자기상관 위상 (±625kHz 모호성) ──
        cf cstf=0; for(int i=0;i<128 && base+i+16<(long)x.size();i++) cstf+=x[base+i+16]*std::conj(x[base+i]);
        double cfo_c=std::arg(cstf)/(2*PI*16/20e6);  // later·conj(earlier) → +부호 (fine 과 일치)
        auto derot=[&](long n)->cf{ return std::polar(1.0f,(float)(-2*PI*cfo_c*n/20e6)); };
        (void)derot;
        // L-LTF 위치: lag-64 자기상관 (두 LTF 심볼 동일 → CFO-robust). 피크=첫 LTF 시작.
        long best=-1; float bestc=0;
        for(long off=base+150; off<base+270 && off+128<(long)x.size(); off++){
            cf c=0; double e1=0,e2=0;
            for(int i=0;i<64;i++){ c+=x[off+i]*std::conj(x[off+i+64]); e1+=std::norm(x[off+i]); e2+=std::norm(x[off+i+64]); }
            float cc=(e1>1e-9&&e2>1e-9)?std::abs(c)/(float)sqrt(e1*e2):0; if(cc>bestc){bestc=cc;best=off;}
        }
        if(tried<15) printf("  [dbg] pkt@%zu LTF@base+%ld autoc=%.3f cfoC=%.1fkHz\n",P.pos,best-base,bestc,cfo_c/1e3);
        if(best<0||bestc<0.7f) continue;
        tried++;
        if(best+128>=(long)x.size()) continue;
        // ── CFO: 강한 LTF(lag-64). best 가 plateau 어디든 64-간격 위상은 CFO 그대로 ──
        cf ac=0; for(int i=0;i<64;i++) ac+=x[best+64+i]*std::conj(x[best+i]);
        double cfo_f=std::arg(ac)/(2*PI*64/20e6);                  // fine ±156kHz (lag-64)
        double nwrap=std::round((cfo_c-cfo_f)/312500.0);           // coarse 로 lag-64 ±312.5kHz 모호성 해소
        double cfo=cfo_f+nwrap*312500.0;
        auto decfo=[&](long n)->cf{ return std::polar(1.0f,(float)(-2*PI*cfo*(n-best)/20e6)); };
        // ── 정밀 타이밍: CFO 보정 후 LTF 시간기준 matched-filter (plateau 32샘플 모호성 해소) ──
        double eref=0; for(int i=0;i<64;i++) eref+=std::norm(ltf_t[i]);
        long ltf1=best; float mc=0;
        for(long off=best-40; off<=best+40 && off+64<(long)x.size(); off++){ if(off<0)continue;
            cf c=0; double e=0; for(int i=0;i<64;i++){ cf s=x[off+i]*decfo(off+i); c+=s*std::conj(ltf_t[i]); e+=std::norm(s); }
            float cc=(e>1e-9&&eref>1e-9)?std::abs(c)/(float)sqrt(e*eref):0; if(cc>mc){mc=cc;ltf1=off;} }
        long ltf2=ltf1+64;
        if(ltf2+64>=(long)x.size()) continue;
        // 채널추정: 두 LTF FFT 평균 / 기준
        cf y1[64],y2[64],t1[64],t2[64];
        for(int i=0;i<64;i++){ t1[i]=x[ltf1+i]*decfo(ltf1+i); t2[i]=x[ltf2+i]*decfo(ltf2+i); }
        fft.run(t1,y1); fft.run(t2,y2);
        cf H[64]; for(int sc=-26;sc<=26;sc++){ int b=bin_of(sc); cf avg=(y1[b]+y2[b])*0.5f;
            float ref=LTF_FREQ[sc+26]; H[b]= ref!=0? avg/(float)ref : cf(0,0); }
        // SIGNAL 심볼: LTF2 끝 + GI(16) 후 64. (ltf2+64 = SIGNAL CP 시작)
        long sigsym=ltf2+64+16;
        if(sigsym+64>=(long)x.size()) continue;
        cf st[64],sf[64]; for(int i=0;i<64;i++) st[i]=x[sigsym+i]*decfo(sigsym+i);
        fft.run(st,sf);
        static int dbg1=0;
        if(!dbg1 && bestc>0.78f){ dbg1=1;
            double einb=0,eoob=0;
            for(int b=0;b<64;b++){ int sc=(b<33)?b:b-64; double e=std::norm(sf[b]);
                if(sc>=-26&&sc<=26&&sc!=0) einb+=e; else eoob+=e; }
            printf("    [DBG pkt autoc=%.2f] in-band/oob energy = %.1f (높을수록 타이밍 OK)\n",bestc,einb/(eoob+1e-9));
            printf("    H|.| @sc-26,-13,-1,1,13,26: ");
            for(int sc: {-26,-13,-1,1,13,26}) printf("%.2f ",std::abs(H[bin_of(sc)]));
            printf("\n    eq(sf/H) sc=-26..-19 (re,im): ");
            for(int sc=-26;sc<=-19;sc++){ cf eq=sf[bin_of(sc)]/(H[bin_of(sc)]+cf(1e-6f,0)); printf("(%.1f,%.1f) ",eq.real(),eq.imag()); }
            printf("\n");
        }
        // pilot 위상 선형 fit: phase = a + b·sc (a=잔류CFO위상, b=잔류타이밍 ramp). sum_sc=0, sum_sc2=980.
        double sph=0, sscph=0;
        for(int pi=0;pi<4;pi++){ int b=bin_of(PILOT_SC[pi]); cf m=sf[b]*std::conj(H[b])*(float)(PILOT_VAL[pi]*PILOT_POL[0]);
            double ph=std::arg(m); sph+=ph; sscph+=ph*PILOT_SC[pi]; }
        double a=sph/4.0, bslope=sscph/980.0; if(getenv("NOPILOT")){a=0;bslope=0;} if(getenv("NOSLOPE"))bslope=0;
        // 48 data subcarrier soft — MRC + 선형위상 제거. 802.11 BPSK bit0→-1 → soft=-real.
        float subc[48]; double isum=0,rsum=0;
        for(int k=0;k<48;k++){ int b=bin_of(DSC[k]); cf cor=std::polar(1.0f,(float)(-(a+bslope*DSC[k])));
            cf m=sf[b]*std::conj(H[b])*cor; subc[k]=(getenv("FLIP")?1.f:-1.f)*m.real(); isum+=fabs(m.imag()); rsum+=fabs(m.real()); }
        if(tried<=8) printf("    [sig] |imag|/|real|=%.2f slope=%.3f (작을수록 clean BPSK)\n", rsum>1e-9?isum/rsum:0, bslope);
        static int dmp=0;
        if(!dmp && mc>0.85f){ dmp=1;
            printf("    [DUMP mc=%.2f] 48 등화 real (clean BPSK 면 ±일정값):\n      ",mc);
            for(int k=0;k<48;k++){ printf("%+.1f ",subc[k]); if(k%16==15)printf("\n      "); }
            printf("\n");
        }
        // deinterleave: coded[k]=subc[ i(k) ], i(k)=3*(k%16)+k/16
        float coded[48]; for(int k=0;k<48;k++){ int i=3*(k%16)+k/16; coded[k]=subc[i]; }
        // Viterbi r=1/2 → 24 bits
        std::vector<uint8_t> bits; vit.decode(coded,24,bits);
        // SIGNAL: RATE(4 LSB..) | reserved(1) | LENGTH(12, LSB first) | parity(1) | tail(6)
        int rate=0; for(int i=0;i<4;i++) rate|=bits[i]<<i;
        int len=0;  for(int i=0;i<12;i++) len|=bits[5+i]<<i;
        int par=0;  for(int i=0;i<17;i++) par^=bits[i];
        bool pok=(par==bits[17]);
        bool tail0=true; for(int i=18;i<24;i++) if(bits[i])tail0=false;
        rate_hist[rate&0xF]++;
        // reencode 거리 = 진짜 디코드 판별 (parity 보다 강함). 모든 패킷 계산 + 최소 추적.
        uint8_t re[48]; conv_enc(bits.data(),24,re);
        int dist=0; for(int k=0;k<48;k++){ int hard=(coded[k]<0)?1:0; if(hard!=re[k])dist++; }
        { static int mind=99; if(dist<mind){ mind=dist;
            printf("  [MIN dist=%d/48] rate=0x%X len=%d parity=%d tail0=%d ratio=%.2f\n",dist,rate,len,pok,tail0,rsum>1e-9?isum/rsum:9); } }
        if(dist>4) continue;                      // dist≤4 만 진짜 디코드로 인정
        lsig_ok++;
        printf("  *** LSIG decoded(dist=%d) @%.3fms cfo=%.1fkHz rate=0x%X len=%d ***\n",dist,ltf1/20e6*1e3,cfo/1e3,rate,len);
        // ── M4: 6Mbps(0xB) beacon DATA 디코드 → descramble → MAC/IE → SSID ──
        if(rate!=0xB) continue;                       // MVP: 6Mbps OFDM 만
        int NDBPS=24;                                  // 6Mbps
        int nsym=(16+8*len+6+NDBPS-1)/NDBPS;           // SERVICE16+8len+tail6
        if(sigsym+80+(long)nsym*80 >= (long)x.size()) continue;
        std::vector<float> cs; cs.reserve(nsym*48);
        for(int s=0;s<nsym;s++){
            long ss=sigsym+80*(s+1);
            cf dt[64],dff[64]; for(int i=0;i<64;i++) dt[i]=x[ss+i]*decfo(ss+i);
            fft.run(dt,dff);
            int pol=PILOT_POL[(s+1)%127];
            double sp=0,ssc=0;
            for(int pi=0;pi<4;pi++){ int b=bin_of(PILOT_SC[pi]); cf m=dff[b]*std::conj(H[b])*(float)(PILOT_VAL[pi]*pol);
                sp+=std::arg(m); ssc+=std::arg(m)*PILOT_SC[pi]; }
            double aa=sp/4, bb=ssc/980.0;
            float sub[48]; for(int k=0;k<48;k++){ int b=bin_of(DSC[k]); cf cor=std::polar(1.0f,(float)(-(aa+bb*DSC[k])));
                cf m=dff[b]*std::conj(H[b])*cor; sub[k]=(getenv("FLIP")?1.f:-1.f)*m.real(); }
            for(int k=0;k<48;k++){ int i=3*(k%16)+k/16; cs.push_back(sub[i]); }
        }
        int nbits=16+8*len+6;
        std::vector<uint8_t> db; vit.decode(cs.data(), nbits, db);
        // descramble (self-sync LFSR x^7+x^4+1): SERVICE[0:6]=0 → seed = 첫 7 비트
        int scr=0; for(int i=0;i<7;i++) scr|=db[i]<<(6-i);
        std::vector<uint8_t> ds(nbits);
        for(int i=0;i<nbits;i++){ int fb=((scr>>6)^(scr>>3))&1; ds[i]=db[i]^fb; scr=((scr<<1)|fb)&0x7f; }
        // PSDU 바이트: SERVICE 2바이트 스킵 → MPDU len 바이트 (LSB-first)
        std::vector<uint8_t> mpdu(len);
        for(int B=0;B<len;B++){ int v=0; for(int b=0;b<8;b++) v|=ds[16+B*8+b]<<b; mpdu[B]=(uint8_t)v; }
        // FCS
        uint32_t fcs=(mpdu[len-4])|(mpdu[len-3]<<8)|(mpdu[len-2]<<16)|((uint32_t)mpdu[len-1]<<24);
        uint32_t calc=crc32_802(mpdu.data(), len-4);
        bool fcsok=(fcs==calc);
        bool beacon=(mpdu[0]==0x80);
        printf("    [DATA] nsym=%d len=%d FCS=%s frame=0x%02X %s\n",nsym,len,fcsok?"OK":"bad",mpdu[0],beacon?"(BEACON)":"");
        if(fcsok && beacon){
            char bssid[20]; snprintf(bssid,sizeof(bssid),"%02x:%02x:%02x:%02x:%02x:%02x",mpdu[16],mpdu[17],mpdu[18],mpdu[19],mpdu[20],mpdu[21]);
            char ssid[33]="(hidden)"; int chn=0; const char* sec="Open"; char phy[8]="g";
            int p=36;  // beacon body: ts8+int2+cap2=12 → IE start 24+12=36
            while(p+2<=len-4){ int id=mpdu[p],l=mpdu[p+1]; if(p+2+l>len-4)break;
                if(id==0){ int n=l<32?l:32; memcpy(ssid,&mpdu[p+2],n); ssid[n]=0; if(n==0)strcpy(ssid,"(hidden)"); }
                else if(id==3 && l>=1) chn=mpdu[p+2];
                else if(id==48) sec="WPA2";
                else if(id==45) strcpy(phy,"n");
                else if(id==191) strcpy(phy,"ac");
                else if(id==221 && l>=4 && mpdu[p+2]==0x00&&mpdu[p+3]==0x50&&mpdu[p+4]==0xf2&&mpdu[p+5]==0x01 && !strcmp(sec,"Open")) sec="WPA";
                else if(id==255 && l>=1 && mpdu[p+2]==35) strcpy(phy,"ax");  // ext IE: HE Capabilities
                p+=2+l;
            }
            printf("    *** BEACON DECODED: SSID=\"%s\"  ch=%d  sec=%s  phy=802.11%s  BSSID=%s ***\n",ssid,chn,sec,phy,bssid);
            beacons_ok++;
        }
    }
    printf("\nL-SIG: tried=%d  parity+tail OK=%d (%.0f%%)  BEACONS decoded(FCS ok)=%d\n",tried,lsig_ok,tried?100.0*lsig_ok/tried:0,beacons_ok);
    printf("rate-nibble histogram (0xB=6Mbps,0xF=9,0xA=12,...): ");
    for(int i=0;i<16;i++) if(rate_hist[i]) printf("0x%X:%d ",i,rate_hist[i]);
    printf("\n");
    return 0;
}
