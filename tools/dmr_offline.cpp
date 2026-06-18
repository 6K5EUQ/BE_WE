// 오프라인 DMR 복조 하니스 — 캡처 .sigmf-data/.cf32 로 phys+sync 검증 (게이트2).
//   build: g++ -O2 -std=c++17 -I src tools/dmr_offline.cpp -lm -o /tmp/dmr_offline
//   run:   /tmp/dmr_offline <file.sigmf-data> [in_sr_hz]
// 단계: load(ci16_le /2048) → 48kHz 리샘플 → FM 판별기 → RRC-ish LP →
//       Gardner 심볼동기(10 sps) → 적응형 4레벨 슬라이서 → 24심볼 SYNC 상관.
// 1차 성공기준: DMR SYNC lock 발생(상관 hit) + 4레벨 히스토그램 분리.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <complex>
#include <cmath>
#include "modules/dmr/dmr_sync.hpp"
#include "modules/dmr/dmr_fec.hpp"
using cf=std::complex<float>;
static constexpr double PI=3.14159265358979323846;

// ── ci16_le(.sigmf-data) / cf32 로더 (wifi_offline.cpp 패턴) ──
static std::vector<cf> load_iq(const char* fn,size_t maxc){
    FILE* f=fopen(fn,"rb"); if(!f){perror("open");exit(1);}
    std::vector<cf> v; v.reserve(maxc);
    const char* dot=strrchr(fn,'.');
    bool ci16 = dot && !strcmp(dot,".sigmf-data");
    if(ci16){ int16_t buf[4096];
        while(v.size()<maxc){ size_t n=fread(buf,sizeof(int16_t),4096,f); if(n<2)break;
            for(size_t i=0;i+1<n&&v.size()<maxc;i+=2) v.push_back(cf(buf[i]/2048.f,buf[i+1]/2048.f)); }
        printf("[ci16_le 입력]\n");
    } else { float buf[4096];
        while(v.size()<maxc){ size_t n=fread(buf,sizeof(float),4096,f); if(n<2)break;
            for(size_t i=0;i+1<n&&v.size()<maxc;i+=2) v.push_back(cf(buf[i],buf[i+1])); } }
    fclose(f); return v;
}
// 윈도우드-싱크 리샘플 (wifi_offline.cpp:30-45)
static std::vector<cf> resample_to(const std::vector<cf>& in,double in_sr,double out_sr){
    if(fabs(in_sr-out_sr)<1.0) return in;
    double step=in_sr/out_sr; if(in.size()<64) return in;
    size_t nout=(size_t)(in.size()/step)-32;
    std::vector<cf> out; out.reserve(nout); const int T=16;
    for(size_t m=0;m<nout;m++){
        double tin=m*step; long c=(long)floor(tin); double frac=tin-c;
        cf acc(0,0); double wsum=0;
        for(int k=-T+1;k<=T;k++){ long idx=c+k; if(idx<0||idx>=(long)in.size())continue;
            double x=PI*(k-frac); double s=(fabs(x)<1e-6)?1.0:sin(x)/x;
            double a=(double)(k-frac)/T; double w=0.42+0.5*cos(PI*a)+0.08*cos(2*PI*a);
            double h=s*w; acc+=in[idx]*(float)h; wsum+=h; }
        out.push_back(wsum>1e-9?acc/(float)wsum:acc);
    }
    return out;
}

static inline float fm_discrim(cf z,cf zp){
    float oi=z.real(),oq=z.imag(),pi=zp.real(),pq=zp.imag();
    return atan2f(oq*pi - oi*pq, oi*pi + oq*pq + 1e-20f);
}
static inline double interp(const std::vector<float>& d,double p){
    long i=(long)floor(p); double f=p-i;
    if(i<0||i+1>=(long)d.size()) return 0.0;
    return d[i]*(1.0-f)+d[i+1]*f;
}

// SYNC 들의 기대 부호열 (모든 SYNC 심볼은 ±3 → 부호만으로 AGC-독립 상관)
struct SignPat{ dmr::SyncType type; int8_t sgn[dmr::SYNC_SYMS]; };
static std::vector<SignPat> build_sign_pats(){
    std::vector<SignPat> v;
    for(int p=0;p<dmr::N_SYNC;p++){
        SignPat sp; sp.type=dmr::SYNC_PATTERNS[p].type;
        int8_t s[dmr::SYNC_SYMS]; dmr::sync_to_syms(dmr::SYNC_PATTERNS[p].bits48,s);
        for(int k=0;k<dmr::SYNC_SYMS;k++) sp.sgn[k]=(s[k]<0)?-1:1;
        v.push_back(sp);
    }
    return v;
}
// soft 심볼 윈도우 24개에 대해 부호상관 최대일치(0..24) + 타입/극성
static int sign_correlate(const float* w,const std::vector<SignPat>& pats,
                          dmr::SyncType& best,bool& inv){
    int bestScore=-1; best=dmr::SyncType::NONE; inv=false;
    for(auto& sp:pats){
        int m0=0,m1=0;
        for(int k=0;k<dmr::SYNC_SYMS;k++){
            int s=(w[k]<0)?-1:1;
            if(s==sp.sgn[k]) m0++; else m1++;       // m1 = 반전 일치수
        }
        if(m0>bestScore){bestScore=m0;best=sp.type;inv=false;}
        if(m1>bestScore){bestScore=m1;best=sp.type;inv=true;}
    }
    return bestScore;
}

int main(int argc,char**argv){
    if(argc<2){ fprintf(stderr,"usage: %s file.sigmf-data [in_sr_hz]\n",argv[0]); return 1; }
    double in_sr = argc>2?atof(argv[2]):111111.0;
    const double OUT_SR=48000.0, BAUD=4800.0;
    const double SPS=OUT_SR/BAUD;                 // 10.0

    auto raw=load_iq(argv[1],200'000'000);
    printf("loaded %zu samples @ %.4f kSPS (%.2f s)\n",raw.size(),in_sr/1e3,raw.size()/in_sr);
    auto x=resample_to(raw,in_sr,OUT_SR);
    printf("resampled %zu @ %.1f kSPS (sps=%.3f)\n",x.size(),OUT_SR/1e3,SPS);

    // ── FM 판별기 → integrate-and-dump 정합필터(1심볼 boxcar) ──
    // 4FSK 심볼은 1심볼동안 일정 주파수 → boxcar 평균이 최적 정합(±3/±1 분리 최대).
    // (좁은 LP 는 ISI 로 외곽심볼을 안쪽으로 끌어 magnitude 오분류 → 폐기)
    std::vector<float> draw(x.size(),0.f);
    for(size_t n=1;n<x.size();n++) draw[n]=fm_discrim(x[n],x[n-1]);
    std::vector<float> d(draw.size(),0.f);
    { std::vector<double> ps(draw.size()+1,0.0);
      for(size_t n=0;n<draw.size();n++) ps[n+1]=ps[n]+draw[n];
      int W=(int)llround(SPS), H=W/2;
      for(long n=0;n<(long)draw.size();n++){
        long lo=n-H<0?0:n-H, hi=n+H+1>(long)draw.size()?(long)draw.size():n+H+1;
        d[n]=(float)((ps[hi]-ps[lo])/(hi-lo)); } }

    // 전력기반 신호구간 (진단용): |x|^2 평균 대비
    { double pmean=0; for(auto&z:x) pmean+=std::norm(z); pmean/=x.size();
      double psig=0; size_t nsig=0; for(auto&z:x){ if(std::norm(z)>pmean){psig+=std::norm(z);nsig++;} }
      printf("power: mean|x|^2=%.4g  active≈%.0f%%\n",pmean,100.0*nsig/x.size()); }

    // ── 슬라이딩 부호상관 (AGC-독립): omega 스윕으로 클럭드리프트 흡수 ──
    auto pats=build_sign_pats();
    double bestOmega=SPS; size_t bestPos=0; int bestScore=-1;
    dmr::SyncType bestType=dmr::SyncType::NONE; bool bestInv=false;
    int scoreHist[25]={0};
    float w[dmr::SYNC_SYMS];
    for(double omega=SPS-0.10; omega<=SPS+0.101; omega+=0.02){
        double span=(dmr::SYNC_SYMS-1)*omega;
        for(double start=1.0; start+span<(double)d.size()-2; start+=1.0){
            for(int k=0;k<dmr::SYNC_SYMS;k++) w[k]=(float)interp(d,start+k*omega);
            dmr::SyncType t; bool inv;
            int sc=sign_correlate(w,pats,t,inv);
            if(omega==SPS && sc<25) scoreHist[sc]++;   // 대표 omega 분포만
            if(sc>bestScore){ bestScore=sc; bestOmega=omega; bestPos=(size_t)start;
                              bestType=t; bestInv=inv; }
        }
    }
    printf("\nbest SIGN-corr: score=%d/24  omega=%.3f  pos=%zu (%.3fs)  %-9s %s\n",
           bestScore,bestOmega,bestPos, bestPos/OUT_SR,
           dmr::sync_name(bestType), bestInv?"(INVERTED)":"");
    printf("sign-score hist @omega=10 [16..24]: ");
    for(int k=16;k<=24;k++) printf("%d:%d ",k,scoreHist[k]);
    printf("\n");

    // best omega/phase 로 전체 심볼 strobe → SYNC hit 스캔 + 4레벨 히스토그램
    if(bestScore>=20){
        double omega=bestOmega;
        double phase=fmod((double)bestPos,omega);
        std::vector<float> sf;
        for(double pp=phase; pp<(double)d.size()-2; pp+=omega) sf.push_back((float)interp(d,pp));
        printf("strobed %zu symbols @phase=%.2f omega=%.3f\n",sf.size(),phase,omega);
        int hits=0; long byType[16]={0};
        std::vector<size_t> spos; std::vector<int> stype;
        for(size_t i=0;i+dmr::SYNC_SYMS<=sf.size();i++){
            dmr::SyncType t; bool inv;
            int sc=sign_correlate(&sf[i],pats,t,inv);
            if(sc>=22){ hits++; byType[(int)t]++; spos.push_back(i); stype.push_back((int)t); }
        }
        printf("SYNC hits(score>=20): %d\n",hits);
        for(int t=1;t<=dmr::N_SYNC;t++) if(byType[t])
            printf("  %-9s : %ld\n",dmr::sync_name((dmr::SyncType)t),byType[t]);
        // 연속 sync 간격(심볼) 히스토그램 — 버스트/슈퍼프레임 구조 진단
        printf("\nsync 간격(심볼) 분포:\n");
        int dh[600]={0};
        for(size_t i=1;i<spos.size();i++){ size_t dd=spos[i]-spos[i-1]; if(dd<600) dh[dd]++; }
        for(int dd=0;dd<600;dd++) if(dh[dd]) printf("  Δ%3d sym (%.1f ms): %d\n",dd,dd*omega/OUT_SR*1000,dh[dd]);
        // ── 버스트 디코드: Slot Type(Golay)→CC/DataType, BPTC(196,96)→LC/CSBK ──
        // sync 부호열은 INV(voice)==data(normal) 이라 극성/voice·data 모호 →
        // 두 극성 모두 시도, Golay 비트오류 적은 쪽 채택.
        // DC/레벨 보정: sync(전부 ±3) 로 outer 레벨 추정.
        double dcm=0; for(float v:sf) dcm+=v; dcm/=sf.size();
        double lvl=0; long lc=0;
        for(size_t si=0;si<spos.size();si++){ size_t sp=spos[si];
            for(int k=0;k<dmr::SYNC_SYMS;k++){ lvl+=fabs(sf[sp+k]-dcm); lc++; } }
        lvl/=(lc?lc:1);
        double thi=dcm+2*lvl/3, tlo=dcm-2*lvl/3;
        printf("\nslicer: dc=%.4f outer=%.4f\n",dcm,lvl);

        static const char* DT[16]={"PI_Hdr","VoiceLCHdr","TermLC","CSBK","MBC_Hdr",
            "MBC_Cont","Data_Hdr","Rate12","Rate34","Idle","Rate1","USBD","?12","?13","?14","?15"};

        // 정밀 타이밍: 버스트별 2-파라미터(S0, 국소 omega) 최적화.
        // INFO 는 sync(burst-sym54)에서 ±54~77 심볼 떨어져 → 전체 132심볼 span 으로
        // omega 를 핀고정해야 INFO 드리프트 제거. 목적함수 = Σ|d| (정타이밍=피크).
        int decoded=0; long ccCount[16]={0};
        for(size_t si=0;si<spos.size();si++){
            size_t sp=spos[si];
            double approx = phase + (double)sp*omega;        // sync sym0 ≈ sample
            if(approx < 60*omega || approx+82*omega >= (double)d.size()-2) continue;
            // 2D 탐색: dl(위상) × ol(국소 omega), burst-sym 0..131 에서 Σ|d| 최대
            double S0=approx, ol=omega, bsc=-1;
            for(double cand_ol=omega-0.20; cand_ol<=omega+0.20+1e-9; cand_ol+=0.01)
              for(double dl=-omega; dl<=omega+1e-9; dl+=0.25){
                double base=approx+dl, sc=0;
                for(int b=0;b<132;b+=2) sc+=fabs(interp(d, base+(b-54)*cand_ol));
                if(sc>bsc){ bsc=sc; S0=base; ol=cand_ol; }
              }
            // 국소 dc/outer (sync 24심볼 = 전부 ±3)
            double sdc=0; for(int k=0;k<dmr::SYNC_SYMS;k++) sdc+=interp(d,S0+k*ol); sdc/=dmr::SYNC_SYMS;
            double slvl=0; for(int k=0;k<dmr::SYNC_SYMS;k++) slvl+=fabs(interp(d,S0+k*ol)-sdc); slvl/=dmr::SYNC_SYMS;
            auto dibitAt=[&](double bsym,int pol)->int{           // bsym = burst-sym index
                double v=interp(d, S0+(bsym-54.0)*ol); v=pol?(2*sdc-v):v;
                int sym=(v>=sdc+2*slvl/3)?3:(v>=sdc)?1:(v>=sdc-2*slvl/3)?-1:-3;
                return dmr::sym_to_dibit(sym);
            };
            auto bitsAt=[&](double b0,int nsym,int pol,uint8_t* out){
                for(int s=0;s<nsym;s++){ int db=dibitAt(b0+s,pol); out[2*s]=(db>>1)&1; out[2*s+1]=db&1; }
            };
            // 극성: sync 24심볼 재구성 → 매칭 sync word biterr 최소 극성 채택
            int pol=0,syncErr=99; dmr::SyncType stype=dmr::SyncType::NONE;
            for(int pp=0;pp<2;pp++){
                uint8_t sb[48]; bitsAt(54,24,pp,sb);
                uint64_t got=0; for(int i=0;i<48;i++) got=(got<<1)|sb[i];
                for(int p=0;p<dmr::N_SYNC;p++){
                    int e=__builtin_popcountll(got^dmr::SYNC_PATTERNS[p].bits48);
                    if(e<syncErr){syncErr=e;pol=pp;stype=dmr::SYNC_PATTERNS[p].type;}
                }
            }
            // Slot Type: burst-sym 49..53 (a) + 78..82 (b) → Golay(20,8)
            uint8_t ab[10],bb[10]; bitsAt(49,5,pol,ab); bitsAt(78,5,pol,bb);
            uint32_t cw=0; for(int i=0;i<10;i++)cw=(cw<<1)|ab[i]; for(int i=0;i<10;i++)cw=(cw<<1)|bb[i];
            uint8_t d8; int gerr=dmr::golay20_8_decode(cw,d8);
            int cc=(d8>>4)&0xF, dt=d8&0xF;
            bool isData = (syncErr<=4) && (stype==dmr::SyncType::BS_DATA||stype==dmr::SyncType::MS_DATA||
                          stype==dmr::SyncType::DIRECT_DATA_TS1||stype==dmr::SyncType::DIRECT_DATA_TS2);
            printf("  %.3fs %-9s syncErr=%d pol=%d %s",
                   S0/OUT_SR, dmr::sync_name(stype), syncErr, pol, isData?"[DATA]":"[VOICE]");
            if(gerr<=3){ printf(" Golay_err=%d CC=%d %s",gerr,cc,DT[dt]); if(gerr<=2){decoded++;ccCount[cc]++;} }
            printf("\n");
            if(gerr<=3 && (dt==1||dt==2||dt==3)){     // VoiceLCHdr/TermLC/CSBK → BPTC196
                uint8_t info[196]; bitsAt(0,49,pol,info); bitsAt(83,49,pol,info+98);
                uint8_t p96[96]; dmr::bptc196_96(info,p96);
                int flco=(int)dmr::bits_to_uint(p96+2,6), fid=(int)dmr::bits_to_uint(p96+8,8);
                uint32_t dst=dmr::bits_to_uint(p96+24,24), src=dmr::bits_to_uint(p96+48,24);
                if(dt==3){
                    uint16_t comp=dmr::crc16_ccitt(p96,80), stored=(uint16_t)dmr::bits_to_uint(p96+80,16);
                    uint16_t resid=comp^stored;        // ==0xA5A5 면 CRC OK
                    if(decoded<=60) printf("      CSBK op=%d FID=%d dst=%u src=%u crcXor=%04x%s\n",
                        (int)dmr::bits_to_uint(p96+2,6),fid,dmr::bits_to_uint(p96+16,24),dmr::bits_to_uint(p96+40,24),
                        resid, resid==0x5A5A?" OK":"");
                } else if(decoded<=60) printf("      LC FLCO=%d FID=%d dst(TG)=%u src=%u\n",flco,fid,dst,src);
            }
        }
        printf("\nGolay-decoded bursts(err<=2): %d / %d sync\n",decoded,(int)spos.size());
        printf("CC 분포: "); for(int c=0;c<16;c++) if(ccCount[c]) printf("CC%d:%ld ",c,ccCount[c]); printf("\n");
        return decoded>0?0:2;
    }
    printf("NO LOCK (best sign-score %d < 20)\n",bestScore);
    return 2;
}
