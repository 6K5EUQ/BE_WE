// 오프라인 802.11b DSSS 1Mbps 비콘 디코더 — 캡처 .sigmf-data(ci16) 개발/검증.
//   build: g++ -O2 -I src tools/wifi_dsss.cpp -lm -o /tmp/wifi_dsss
//   run:   /tmp/wifi_dsss <file> <in_sr_hz>
// 체인: 리샘플→22MSPS(2sps/chip) → Barker MF → 심볼타이밍 → DBPSK → self-sync descramble
//       → SFD(0xF3A0) → PLCP(rate/len/CRC16) → MPDU → FCS(CRC32) → beacon IE → SSID
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <complex>
#include <cmath>
#include <cstdint>
using cf=std::complex<float>;
static constexpr double PI=3.14159265358979323846;
static const int BARKER[11]={1,-1,1,1,-1,1,1,1,-1,-1,-1};

static std::vector<cf> load(const char* fn,size_t maxc){
    FILE* f=fopen(fn,"rb"); if(!f){perror("open");exit(1);}
    std::vector<cf> v; v.reserve(maxc); int16_t b[4096]; size_t n;
    while(v.size()<maxc && (n=fread(b,2,4096,f))>1)
        for(size_t i=0;i+1<n&&v.size()<maxc;i+=2) v.push_back(cf(b[i]/2048.f,b[i+1]/2048.f));
    fclose(f); return v;
}
static std::vector<cf> resamp(const std::vector<cf>& in,double isr,double osr){
    if(fabs(isr-osr)<1) return in; double step=isr/osr; size_t no=(size_t)(in.size()/step)-32;
    std::vector<cf> o; o.reserve(no); const int T=12;
    for(size_t m=0;m<no;m++){ double tin=m*step; long c=(long)floor(tin); double fr=tin-c; cf a(0,0); double ws=0;
        for(int k=-T+1;k<=T;k++){ long id=c+k; if(id<0||id>=(long)in.size())continue; double x=PI*(k-fr);
            double s=(fabs(x)<1e-6)?1.0:sin(x)/x; double aa=(double)(k-fr)/T; double w=0.42+0.5*cos(PI*aa)+0.08*cos(2*PI*aa);
            double h=s*w; a+=in[id]*(float)h; ws+=h; } o.push_back(ws>1e-9?a/(float)ws:a); }
    return o;
}
// CRC-16 CCITT (PLCP header): poly 0x1021... 802.11 uses x^16+x^12+x^5+1, init 0xFFFF, complemented
static uint16_t crc16_plcp(const uint8_t* bits,int nbits){
    uint16_t r=0xFFFF;
    for(int i=0;i<nbits;i++){ int in=bits[i]&1; int hi=(r>>15)&1; r<<=1; if(hi^in) r^=0x1021; }
    return ~r;
}
static uint32_t crc32_802(const uint8_t* d,size_t n){
    static uint32_t T[256]; static bool init=false;
    if(!init){for(uint32_t i=0;i<256;i++){uint32_t c=i;for(int k=0;k<8;k++)c=(c&1)?(0xEDB88320u^(c>>1)):(c>>1);T[i]=c;}init=true;}
    uint32_t c=0xFFFFFFFFu; for(size_t i=0;i<n;i++)c=T[(c^d[i])&0xFF]^(c>>8); return c^0xFFFFFFFFu;
}

int main(int argc,char**argv){
    if(argc<2){fprintf(stderr,"usage: %s file [sr]\n",argv[0]);return 1;}
    double isr=argc>2?atof(argv[2]):20e6;
    auto raw=load(argv[1],60'000'000);
    printf("loaded %zu @ %.3f MSPS\n",raw.size(),isr/1e6);
    auto x=resamp(raw,isr,22e6);              // 2 sps/chip
    size_t N=x.size(); printf("resampled %zu @ 22 MSPS (2 sps/chip)\n",N);
    const int SPS=2, SPSYM=11*SPS;            // 22 samples/symbol

    // Barker MF taps (각 chip = 2 sample)
    cf taps[22]; for(int i=0;i<22;i++) taps[i]=cf((float)BARKER[i/2],0);
    // MF 출력 |.| 으로 DSSS 구간 + 심볼타이밍 탐색
    auto mf=[&](size_t n)->cf{ cf s=0; for(int i=0;i<22;i++) s+=x[n+i]*taps[i]; return s; };

    // DSSS 프레임 검출: |MF| 가 지속적으로 높은 구간 (preamble 144us = strong Barker corr)
    int found=0;
    for(size_t base=0; base+300*SPSYM<N && found<200; ){
        // 이 위치에서 평균 |MF| (1 symbol 분)
        double e=0; for(int j=0;j<22;j++) e+=std::norm(mf(base+j));
        double pw=0; for(int j=0;j<SPSYM;j++) pw+=std::norm(x[base+j]);
        double ratio = pw>1e-9 ? e/22.0/(pw/SPSYM) : 0;   // MF gain (Barker ~11 if DSSS)
        if(ratio < 4.0){ base+=SPSYM; continue; }          // DSSS 아님
        // 심볼타이밍: 0..21 중 평균|MF| 최대 offset
        int boff=0; double bmax=0;
        for(int off=0;off<SPSYM;off++){ double a=0; for(int s=0;s<32;s++){ size_t p=base+off+s*SPSYM; if(p+22<N) a+=std::abs(mf(p)); }
            if(a>bmax){bmax=a;boff=off;} }
        // DBPSK 복조 → 비트 (충분히 길게, 최대 4000 심볼)
        std::vector<uint8_t> sb; std::vector<cf> sym;
        cf prev=0;
        for(int s=0;s<4000;s++){ size_t p=base+boff+s*SPSYM; if(p+22>=N) break; cf c=mf(p); sym.push_back(c);
            if(s>0){ float d=(c*std::conj(prev)).real(); sb.push_back(d<0?1:0); } prev=c; }
        if(sb.size()<200){ base+=SPSYM; continue; }
        // self-sync descramble: d[n]=s[n]^s[n-4]^s[n-7]
        std::vector<uint8_t> d(sb.size());
        for(size_t n=0;n<sb.size();n++){ int a=sb[n], b4=(n>=4?sb[n-4]:0), b7=(n>=7?sb[n-7]:0); d[n]=a^b4^b7; }
        // SFD 0xF3A0 탐색 (descrambled). SYNC=all-1 뒤. LSB-first 와 MSB-first 둘 다 시도.
        auto find_sfd=[&](int msbfirst)->int{
            for(size_t n=0;n+16<=d.size();n++){ int v=0;
                for(int k=0;k<16;k++){ int bit=d[n+k]; v |= bit << (msbfirst? (15-k):k); }
                if(v==0xF3A0) return (int)n; }
            return -1;
        };
        int order=0, sfd=find_sfd(1); if(sfd<0){ sfd=find_sfd(0); order=1; }
        if(sfd<0){ base+=300*SPSYM; continue; }
        // PLCP header: SFD 뒤 48비트. SIGNAL(8) SERVICE(8) LENGTH(16) CRC16(16). 비트→바이트 LSB-first.
        int hp=sfd+16; if(hp+48>(int)d.size()){ base+=300*SPSYM; continue; }
        auto getbyte=[&](int bitpos)->int{ int v=0; for(int b=0;b<8;b++) v|=d[bitpos+b]<<b; return v; };
        int sig=getbyte(hp), svc=getbyte(hp+8);
        int len_us = getbyte(hp+16) | (getbyte(hp+24)<<8);
        printf("  [DSSS] @sym%zu sfd@%d(order=%d) SIGNAL=0x%02X SERVICE=0x%02X LEN=%dus mf_ratio=%.1f\n",
               base,sfd,order,sig,svc,len_us,ratio);
        // MPDU: PLCP 뒤. 1Mbps(SIGNAL=0x0A) DBPSK 이면 1bit/sym. LENGTH(us)=octets at 1Mbps → octets=len_us.
        if(sig==0x0A){
            int psdu_oct=len_us;   // 1Mbps: 1us=1bit → 8us=1byte? 실제 LENGTH=microseconds=bits at 1Mbps → octets=len/8
            psdu_oct=len_us/8;
            int mp=hp+48;
            if(psdu_oct>10 && psdu_oct<2400 && mp+psdu_oct*8<=(int)d.size()){
                std::vector<uint8_t> mpdu(psdu_oct);
                for(int B=0;B<psdu_oct;B++) mpdu[B]=getbyte(mp+B*8);
                uint32_t fcs=(mpdu[psdu_oct-4])|(mpdu[psdu_oct-3]<<8)|(mpdu[psdu_oct-2]<<16)|((uint32_t)mpdu[psdu_oct-1]<<24);
                bool ok=(fcs==crc32_802(mpdu.data(),psdu_oct-4));
                printf("    MPDU oct=%d fc=0x%02X FCS=%s\n",psdu_oct,mpdu[0],ok?"OK":"bad");
                if(ok && mpdu[0]==0x80){
                    char ssid[33]="(hidden)"; int wch=0; const char* sec="Open"; char phy[8]="b";
                    char bssid[20]; snprintf(bssid,sizeof(bssid),"%02x:%02x:%02x:%02x:%02x:%02x",mpdu[16],mpdu[17],mpdu[18],mpdu[19],mpdu[20],mpdu[21]);
                    int p=36;
                    while(p+2<=psdu_oct-4){int id=mpdu[p],l=mpdu[p+1]; if(p+2+l>psdu_oct-4)break;
                        if(id==0){int nn=l<32?l:32; if(nn){memcpy(ssid,&mpdu[p+2],nn); ssid[nn]=0;}}
                        else if(id==3&&l>=1) wch=mpdu[p+2];
                        else if(id==48) sec="WPA2";
                        else if(id==45&&!strcmp(phy,"b")) strcpy(phy,"n");
                        else if(id==191) strcpy(phy,"ac");
                        else if(id==221&&l>=4&&mpdu[p+2]==0&&mpdu[p+3]==0x50&&mpdu[p+4]==0xf2&&mpdu[p+5]==1&&!strcmp(sec,"Open")) sec="WPA";
                        else if(id==255&&l>=1&&mpdu[p+2]==35) strcpy(phy,"ax");
                        p+=2+l;}
                    printf("    *** DSSS BEACON: SSID=\"%s\" ch=%d sec=%s phy=11%s bssid=%s ***\n",ssid,wch,sec,phy,bssid); found++;
                }
            }
        }
        base+=300*SPSYM;
    }
    printf("=== DSSS beacons: %d ===\n",found);
    return 0;
}
