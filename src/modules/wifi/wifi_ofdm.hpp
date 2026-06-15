#pragma once
// ── 802.11a/g OFDM 비콘 수신기 (20 MHz, 입력=20.0 MSPS 채널 baseband 복소) ──
// 표준: IEEE 802.11-2016 Clause 17. MVP = 6 Mbps(BPSK r=1/2) 비콘.
// 단계: 패킷검출(L-STF autocorr) → CFO → L-LTF 동기/채널추정 → 심볼 FFT/등화
//       → L-SIG(Viterbi) → DATA demap/deinterleave/Viterbi/descramble → MPDU/FCS → IE.
// 헤더 전용 — 오프라인 하니스 + (추후)스트리밍 워커 공용. FFTW3f 필요.
#include <vector>
#include <complex>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <functional>
#include <fftw3.h>
#include "wifi_meta.hpp"

namespace wifi_ofdm {

using cf = std::complex<float>;
static constexpr int FFT=64, CP=16, SYM=80;

// ── L-LTF 주파수영역 시퀀스 (subcarrier -26..+26, DC=0) ────────────────────
static const int LTF_FREQ[53] = {
 1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,
 0,
 1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1 };

// pilot subcarrier 인덱스(-26 기준 배열 0..52 에서) 및 기준값
// 실제 subcarrier: -21,-7,+7,+21
static inline bool is_pilot(int sc){ return sc==-21||sc==-7||sc==7||sc==21; }

// pilot polarity 시퀀스 p_0..p_126 (scrambler all-ones init). 표준 Table L-1.
static const int8_t PILOT_POL[127] = {
 1,1,1,1,-1,-1,-1,1,-1,-1,-1,-1,1,1,-1,1,-1,-1,1,1,-1,1,1,-1,1,1,1,1,1,1,-1,1,
 1,1,-1,1,1,-1,-1,1,1,1,-1,1,-1,-1,-1,1,-1,1,-1,-1,1,-1,-1,1,1,1,1,1,-1,-1,1,1,
 -1,-1,1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,-1,-1,-1,1,-1,-1,1,-1,1,1,1,1,-1,1,-1,1,-1,1,
 -1,-1,-1,-1,-1,1,-1,1,1,-1,1,-1,1,1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,-1,-1,-1 };

// ── 길쌈부호 Viterbi (K=7, r=1/2, g0=133, g1=171 octal) — soft input ───────
struct Viterbi {
    static constexpr int NS=64; // 2^(K-1)
    // 출력 비트 룩업: out0/out1 for (state,input)
    int8_t o0[NS][2], o1[NS][2], ns[NS][2];
    Viterbi(){
        for(int s=0;s<NS;s++) for(int in=0;in<2;in++){
            int reg=(s<<1)|in;                       // 7-bit reg (msb=new bit)
            auto par=[&](int g){ int x=reg&g,c=0; while(x){c^=x&1;x>>=1;} return c; };
            o0[s][in]=par(0133); o1[s][in]=par(0171);
            ns[s][in]=reg&0x3f;                      // next state (low 6 bits)
        }
    }
    // soft bits: +1=강한0, -1=강한1 (LLR 부호). 코드길이 = 2*nbits.
    void decode(const float* soft, int nbits, std::vector<uint8_t>& out){
        const float INF=1e9f;
        std::vector<float> pm(NS, INF), npm(NS);
        std::vector<std::vector<uint8_t>> tb(nbits, std::vector<uint8_t>(NS));
        pm[0]=0;
        for(int t=0;t<nbits;t++){
            float a=soft[2*t], b=soft[2*t+1];
            for(int s=0;s<NS;s++) npm[s]=INF;
            for(int s=0;s<NS;s++){
                if(pm[s]>=INF) continue;
                for(int in=0;in<2;in++){
                    // BPSK soft metric: 기대비트 0→+, 1→-. 거리 = (a - sgn)^2 류 대신 상관.
                    float e0 = (o0[s][in]? -a : a);
                    float e1 = (o1[s][in]? -b : b);
                    float m = pm[s] - e0 - e1;        // 상관 클수록 metric 감소(=좋음)
                    int n=ns[s][in];
                    if(m<npm[n]){ npm[n]=m; tb[t][n]=(uint8_t)s; }
                }
            }
            pm.swap(npm);
        }
        // 종단: tail-terminated 코드 → 종단 상태 = 0 강제 (6 tail bit 가 인코더 flush)
        int s=0;
        std::vector<uint8_t> bits(nbits);
        // 디코드 비트 = 그 스텝 후 상태의 LSB (reg = (prev<<1)|in → ns&1 = in)
        for(int t=nbits-1;t>=0;t--){ int ps=tb[t][s]; bits[t]=(uint8_t)(s&1); s=ps; }
        out.assign(bits.begin(),bits.end());
    }
};

// ── 디인터리버 (N_CBPS=48, BPSK) ──────────────────────────────────────────
static inline void deinterleave48(const float* in, float* out){
    const int N=48;
    for(int k=0;k<N;k++){
        int i=3*(k%16)+k/16;       // 1st perm (BPSK 는 2nd perm 항등)
        out[k]=in[i];
    }
    // 위는 interleave 매핑(k→i). 디인터리브는 역: out[i_perm(k)] 정렬.
    // 사용처에서 역매핑으로 처리하므로 여기선 항등배치 후 호출측 정의에 맞춤.
}

// subcarrier(-26..26) → FFT bin
static inline int ofdm_bin(int sc){ return (sc+64)%64; }
static const int DSC48[48]={-26,-25,-24,-23,-22,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,
    -6,-5,-4,-3,-2,-1,1,2,3,4,5,6,8,9,10,11,12,13,14,15,16,17,18,19,20,22,23,24,25,26};
static const int PSC4[4]={-21,-7,7,21};
static const int PVAL4[4]={1,1,1,-1};

// CRC-32 (802.11 FCS): poly 0x04C11DB7, reflected, init/xor 0xFFFFFFFF
static inline uint32_t crc32_802(const uint8_t* d, size_t n){
    static uint32_t T[256]; static bool init=false;
    if(!init){ for(uint32_t i=0;i<256;i++){ uint32_t c=i; for(int k=0;k<8;k++) c=(c&1)?(0xEDB88320u^(c>>1)):(c>>1); T[i]=c;} init=true; }
    uint32_t c=0xFFFFFFFFu;
    for(size_t i=0;i<n;i++) c=T[(c^d[i])&0xFF]^(c>>8);
    return c^0xFFFFFFFFu;
}

// ── 64-pt FFT (FFTW) ──────────────────────────────────────────────────────
struct FFT64 {
    fftwf_complex *i,*o; fftwf_plan p; cf ltf_t[64];
    FFT64(){ i=fftwf_alloc_complex(64); o=fftwf_alloc_complex(64);
        p=fftwf_plan_dft_1d(64,i,o,FFTW_FORWARD,FFTW_ESTIMATE);
        // LTF time 기준 (matched-filter 용): IFFT of LONG/LTF_FREQ
        fftwf_complex *fi=fftwf_alloc_complex(64),*fo=fftwf_alloc_complex(64);
        fftwf_plan ip=fftwf_plan_dft_1d(64,fi,fo,FFTW_BACKWARD,FFTW_ESTIMATE);
        for(int k=0;k<64;k++){fi[k][0]=0;fi[k][1]=0;}
        for(int sc=-26;sc<=26;sc++) fi[ofdm_bin(sc)][0]=LTF_FREQ[sc+26];
        fftwf_execute(ip); for(int k=0;k<64;k++) ltf_t[k]=cf(fo[k][0],fo[k][1])/64.0f;
        fftwf_destroy_plan(ip); fftwf_free(fi); fftwf_free(fo);
    }
    ~FFT64(){ fftwf_destroy_plan(p); fftwf_free(i); fftwf_free(o); }
    void run(const cf* t, cf* f){ for(int k=0;k<64;k++){i[k][0]=t[k].real();i[k][1]=t[k].imag();}
        fftwf_execute(p); for(int k=0;k<64;k++) f[k]=cf(o[k][0],o[k][1]); }
};

// 분수 리샘플 (windowed-sinc 32탭) → 정확히 out_sr
inline std::vector<cf> ofdm_resample(const std::vector<cf>& in, double in_sr, double out_sr){
    if(std::fabs(in_sr-out_sr)<1.0) return in;
    const double PI=3.14159265358979323846; double step=in_sr/out_sr;
    if((long)(in.size()/step)<=64) return {};
    size_t nout=(size_t)(in.size()/step)-32; std::vector<cf> out; out.reserve(nout); const int T=16;
    for(size_t m=0;m<nout;m++){ double tin=m*step; long c=(long)std::floor(tin); double fr=tin-c;
        cf acc(0,0); double ws=0;
        for(int k=-T+1;k<=T;k++){ long id=c+k; if(id<0||id>=(long)in.size())continue;
            double xx=PI*(k-fr); double s=(std::fabs(xx)<1e-6)?1.0:std::sin(xx)/xx;
            double aa=(double)(k-fr)/T; double w=0.42+0.5*std::cos(PI*aa)+0.08*std::cos(2*PI*aa);
            double h=s*w; acc+=in[id]*(float)h; ws+=h; }
        out.push_back(ws>1e-9?acc/(float)ws:acc); }
    return out;
}

// ── 버퍼 디코드: 20.0 MSPS baseband → FCS 유효 비콘 → on_rec(WifiRecord) ─────
// in_sr != 20e6 면 내부 리샘플. on_rec 는 SSID/채널/보안/PHY 채운 레코드 전달.
inline void decode_buffer(const cf* xin, size_t nin, double in_sr,
                          const std::function<void(const WifiRecord&)>& on_rec, int max_pkt=2000){
    const double PI=3.14159265358979323846, FS=20.0e6;
    std::vector<cf> rs; const cf* xp; size_t N;
    if(std::fabs(in_sr-FS)<1.0){ xp=xin; N=nin; }
    else { std::vector<cf> tmp(xin,xin+nin); rs=ofdm_resample(tmp,in_sr,FS); xp=rs.data(); N=rs.size(); }
    if(N<512) return;
    auto& x=xp;
    static thread_local FFT64 fft;  Viterbi vit;
    double eref=0; for(int k=0;k<64;k++) eref+=std::norm(fft.ltf_t[k]);
    // STF lag-16 검출
    int pkts=0;
    bool inb=false; size_t pstart=0;
    auto autoc16=[&](size_t nn)->float{ cf C=0; double P=0;
        for(int k=0;k<16;k++){ if(nn+k+16>=N)break; C+=x[nn+k]*std::conj(x[nn+k+16]); P+=std::norm(x[nn+k+16]); }
        return P>1e-9?std::abs(C)/(float)P:0.f; };
    for(size_t nn=0; nn+160<N && pkts<max_pkt; nn+=4){
        float m=autoc16(nn);
        if(m>0.6f){ if(!inb){inb=true;pstart=nn;} continue; }
        if(!inb) continue;
        inb=false; if(nn-pstart<80) continue;
        long base=(long)pstart;
        // coarse CFO (STF)
        cf cstf=0; for(int k=0;k<128&&base+k+16<(long)N;k++) cstf+=x[base+k+16]*std::conj(x[base+k]);
        double cfo_c=std::arg(cstf)/(2*PI*16/FS);
        // LTF lag-64 위치
        long best=-1; float bc=0;
        for(long off=base+150; off<base+270&&off+128<(long)N; off++){ cf c=0; double e1=0,e2=0;
            for(int k=0;k<64;k++){ c+=x[off+k]*std::conj(x[off+k+64]); e1+=std::norm(x[off+k]); e2+=std::norm(x[off+k+64]); }
            float cc=(e1>1e-9&&e2>1e-9)?std::abs(c)/(float)std::sqrt(e1*e2):0; if(cc>bc){bc=cc;best=off;} }
        if(best<0||bc<0.7f||best+128>=(long)N) continue;
        pkts++;
        cf ac=0; for(int k=0;k<64;k++) ac+=x[best+64+k]*std::conj(x[best+k]);
        double cfo_f=std::arg(ac)/(2*PI*64/FS), nw=std::round((cfo_c-cfo_f)/312500.0), cfo=cfo_f+nw*312500.0;
        auto dr=[&](long n2){ return std::polar(1.0f,(float)(-2*PI*cfo*(n2-best)/FS)); };
        long l1=best; float mc=0;
        for(long off=best-40; off<=best+40&&off+64<(long)N; off++){ if(off<0)continue; cf c=0; double e=0;
            for(int k=0;k<64;k++){ cf s=x[off+k]*dr(off+k); c+=s*std::conj(fft.ltf_t[k]); e+=std::norm(s); }
            float cc=(e>1e-9&&eref>1e-9)?std::abs(c)/(float)std::sqrt(e*eref):0; if(cc>mc){mc=cc;l1=off;} }
        long l2=l1+64; if(l2+64>=(long)N) continue;
        cf y1[64],y2[64],t1[64],t2[64];
        for(int k=0;k<64;k++){ t1[k]=x[l1+k]*dr(l1+k); t2[k]=x[l2+k]*dr(l2+k); }
        fft.run(t1,y1); fft.run(t2,y2);
        cf H[64]; for(int sc=-26;sc<=26;sc++){ int b=ofdm_bin(sc); cf av=(y1[b]+y2[b])*0.5f; float r=LTF_FREQ[sc+26]; H[b]=r!=0?av/r:cf(0,0); }
        long sig=l2+64+16; if(sig+64>=(long)N) continue;
        // 심볼 등화 → 48 soft (MRC + 선형 pilot 위상)
        auto demap=[&](long ss,int polsym,float* outc)->bool{
            if(ss+64>=(long)N) return false;
            cf st[64],sf[64]; for(int k=0;k<64;k++) st[k]=x[ss+k]*dr(ss+k); fft.run(st,sf);
            double sp=0,sc2=0; int pol=PILOT_POL[polsym%127];
            for(int pi=0;pi<4;pi++){ int b=ofdm_bin(PSC4[pi]); cf mm=sf[b]*std::conj(H[b])*(float)(PVAL4[pi]*pol); sp+=std::arg(mm); sc2+=std::arg(mm)*PSC4[pi]; }
            double aa=sp/4.0, bb=sc2/980.0; float sub[48];
            for(int k=0;k<48;k++){ int b=ofdm_bin(DSC48[k]); cf cor=std::polar(1.0f,(float)(-(aa+bb*DSC48[k]))); cf mm=sf[b]*std::conj(H[b])*cor; sub[k]=-mm.real(); }
            for(int k=0;k<48;k++){ int i=3*(k%16)+k/16; outc[k]=sub[i]; }
            return true;
        };
        float lc[48]; if(!demap(sig,0,lc)) continue;
        std::vector<uint8_t> sb; vit.decode(lc,24,sb);
        int rate=0; for(int i=0;i<4;i++) rate|=sb[i]<<i;
        int len=0;  for(int i=0;i<12;i++) len|=sb[5+i]<<i;
        // reencode 거리로 L-SIG 검증
        uint8_t re[48]; { int reg=0; auto par=[&](int g){int v=reg&g,c=0;while(v){c^=v&1;v>>=1;}return c;};
            for(int i=0;i<24;i++){ reg=((reg<<1)|sb[i])&0x7f; re[2*i]=par(0133); re[2*i+1]=par(0171);} }
        int dist=0; for(int k=0;k<48;k++){ int hd=(lc[k]<0)?1:0; if(hd!=re[k])dist++; }
        if(dist>4 || rate!=0xB || len<24 || len>2400) continue;   // 6Mbps OFDM 비콘만
        int nsym=(16+8*len+6+23)/24;
        if(sig+80+(long)nsym*80>=(long)N) continue;
        std::vector<float> cs; cs.reserve(nsym*48);
        for(int s=0;s<nsym;s++){ float sub[48]; if(!demap(sig+80*(s+1),s+1,sub)){ cs.clear(); break; } for(int k=0;k<48;k++) cs.push_back(sub[k]); }
        if((int)cs.size()!=nsym*48) continue;
        int nbits=16+8*len+6; std::vector<uint8_t> db; vit.decode(cs.data(),nbits,db);
        int scr=0; for(int i=0;i<7;i++) scr|=db[i]<<(6-i);
        std::vector<uint8_t> dsb(nbits);
        for(int i=0;i<nbits;i++){ int fb=((scr>>6)^(scr>>3))&1; dsb[i]=db[i]^fb; scr=((scr<<1)|fb)&0x7f; }
        std::vector<uint8_t> mp(len);
        for(int B=0;B<len;B++){ int v=0; for(int b=0;b<8;b++) v|=dsb[16+B*8+b]<<b; mp[B]=(uint8_t)v; }
        uint32_t fcs=(mp[len-4])|(mp[len-3]<<8)|(mp[len-2]<<16)|((uint32_t)mp[len-1]<<24);
        if(fcs!=crc32_802(mp.data(),len-4)) continue;
        if(mp[0]!=0x80) continue;                                  // beacon 만
        WifiRecord r{};
        snprintf(r.bssid,sizeof(r.bssid),"%02x:%02x:%02x:%02x:%02x:%02x",mp[16],mp[17],mp[18],mp[19],mp[20],mp[21]);
        strcpy(r.sec,"Open"); strcpy(r.phy,"g");
        int p=36;
        while(p+2<=len-4){ int id=mp[p],l=mp[p+1]; if(p+2+l>len-4)break;
            if(id==0){ int nn2=l<32?l:32; memcpy(r.ssid,&mp[p+2],nn2); r.ssid[nn2]=0; }
            else if(id==3&&l>=1) r.wch=mp[p+2];
            else if(id==48) strcpy(r.sec,"WPA2");
            else if(id==45) strcpy(r.phy,"n");
            else if(id==191) strcpy(r.phy,"ac");
            else if(id==221&&l>=4&&mp[p+2]==0&&mp[p+3]==0x50&&mp[p+4]==0xf2&&mp[p+5]==1&&!strcmp(r.sec,"Open")) strcpy(r.sec,"WPA");
            else if(id==255&&l>=1&&mp[p+2]==35) strcpy(r.phy,"ax");
            p+=2+l; }
        on_rec(r);
    }
}

} // namespace wifi_ofdm
