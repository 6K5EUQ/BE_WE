#pragma once
// ── ADS-B / Mode S 1090ES 스트리밍 디코더 (자작, 외부 라이브러리/코드 미참조) ──
// 공개 표준만 사용: ICAO Annex 10 Vol IV, RTCA DO-260B, mode-s.org 공식.
// 입력 = 채널 baseband 의 진폭(magnitude) 샘플열 |I+jQ|, 샘플레이트 fs (≈2.4 MHz).
//        RF→baseband 다운컨버전/데시메이션/magnitude 계산은 워커(adsb_decode.cpp).
//
// 물리계층 (DO-260):
//   • 반송파 1090 MHz, PPM(=OOK, 진폭만). 위상 불필요 → magnitude 로 해독.
//   • 1 Mbit/s. 1 bit = 1 µs = 두 0.5 µs half-chip. 앞 half 펄스=1, 뒤 half 펄스=0.
//   • 프리앰블 8 µs: 0.0 / 1.0 / 3.5 / 4.5 µs 에 0.5 µs 펄스 4개, 5.5~8.0 µs 정적.
//   • 프레임: short 56 bit (DF0/4/5/11) / long 112 bit (DF16~21). 데이터는 8 µs 후 시작.
//   • 오류검출: CRC-24, 생성다항식 0x1FFF409 (24-bit 잔차 0xFFF409). DF17/18 은
//     전 프레임 잔차==0 이면 무오류. 단일비트 오류는 잔차→비트위치 표로 정정.
//
// 해독 흐름: magnitude 버퍼 → 프리앰블 상관 → half-chip 슬라이싱 → 비트팩 →
//            CRC-24(필요시 1-bit 정정) → DF/ME 필드 추출(콜사인/고도/CPR위치/속도).
#include <cstdint>
#include <cstring>
#include <cmath>
#include <functional>
#include <vector>
#include <unordered_map>
#include "adsb_meta.hpp"

class AdsbDecoder {
public:
    std::function<void(const AdsbRecord&)> on_record;

    void reset(double sample_rate, int ch_idx){
        sr  = sample_rate>1.0 ? sample_rate : 2400000.0;
        spb = sr/1e6;                 // samples per µs (= per bit)
        ch  = ch_idx;
        buf.clear(); samp_base=0; cpr.clear(); roster.clear();
        build_syndrome();
    }

    // 워커가 magnitude 샘플 묶음을 투입. 내부에서 프리앰블 스캔 + 해독.
    void process(const float* mag, size_t n){
        buf.insert(buf.end(), mag, mag+n);
        size_t need = (size_t)((PRE_US + 112.0)*spb) + 8;     // 8µs 프리앰블 + 최장 프레임
        if(buf.size() < need) return;
        size_t limit = buf.size() - need;
        size_t j = 0;
        while(j <= limit){
            if(preamble_ok(j)){
                int adv = decode_at(j);
                if(adv>0){ j += (size_t)adv; continue; }
            }
            j++;
        }
        if(j>0){
            samp_base += (double)j;
            buf.erase(buf.begin(), buf.begin()+j);
        }
    }

private:
    static constexpr double PRE_US = 8.0;

    double sr=2400000.0, spb=2.4, samp_base=0;
    int    ch=0;
    std::vector<float> buf;

    // ── CPR(Compact Position Reporting) 짝(even/odd) 보관 + 마지막 위치 ──
    struct CprState {
        double even_lat=0, even_lon=0, odd_lat=0, odd_lon=0;
        double even_t=-1e9, odd_t=-1e9;        // 초 단위 (sample clock)
        double ref_lat=0, ref_lon=0; bool has_ref=false;
    };
    std::unordered_map<uint32_t, CprState> cpr;
    std::unordered_map<uint32_t, char>     roster;   // DF17/11 로 확인된 ICAO 집합

    // ── 단일비트 오류 정정용 잔차→비트위치 표 (메시지 길이별) ──
    std::unordered_map<uint32_t,int> synd112, synd56;

    // ── magnitude 윈도우 평균 (µs 오프셋 [u0, u0+0.5) ) ─────────────────────
    inline float win(size_t j, double u0) const {
        long a=(long)(j + (size_t)llround(u0*spb));
        long b=(long)(j + (size_t)llround((u0+0.5)*spb));
        if(b<=a) b=a+1;
        float s=0; int c=0;
        long N=(long)buf.size();
        for(long k=a;k<b;k++){ if(k>=0&&k<N){ s+=buf[k]; c++; } }
        return c? s/c : 0.f;
    }

    // ── 프리앰블 상관: 펄스 4개가 골(gap)/정적부보다 충분히 높은가 (상대진폭) ──
    bool preamble_ok(size_t j) const {
        float p0=win(j,0.0), p1=win(j,1.0), p2=win(j,3.5), p3=win(j,4.5);
        float pmin = std::fmin(std::fmin(p0,p1), std::fmin(p2,p3));
        float pmax = std::fmax(std::fmax(p0,p1), std::fmax(p2,p3));
        if(pmin <= 1e-6f) return false;
        if(pmax > 4.0f*pmin) return false;                    // 4 펄스 진폭 비슷해야
        // 펄스 사이/뒤 골은 펄스보다 낮아야 (펄스 ≥ 2× 골)
        float gmax = 0.f;
        const double g[] = {0.5,1.5,2.0,2.5,3.0,4.0};
        for(double u : g) gmax = std::fmax(gmax, win(j,u));
        if(pmin < 2.0f*gmax) return false;
        // 5.5~8.0 µs 정적 구간은 펄스의 절반 미만
        float qmax = 0.f;
        const double q[] = {5.5,6.0,6.5,7.0,7.5};
        for(double u : q) qmax = std::fmax(qmax, win(j,u));
        if(qmax > 0.5f*pmin) return false;
        return true;
    }

    // 프리앰블 j 에서 프레임 해독 시도. 성공 시 소비 샘플수(>0) 반환, 실패 0.
    int decode_at(size_t j){
        // 먼저 112 bit 까지 슬라이스 → DF 로 실제 길이 판정
        uint8_t msg[14]={0};
        float conf[112];
        for(int b=0;b<112;b++){
            double t = PRE_US + b*1.0;          // 비트셀 시작 µs
            float e0=win(j,t), e1=win(j,t+0.5);
            int bit = (e0>e1)?1:0;
            conf[b] = std::fabs(e0-e1)/(e0+e1+1e-9f);
            if(bit) msg[b>>3] |= (uint8_t)(0x80>>(b&7));
        }
        int df = msg[0]>>3;
        int nbits = (df & 0x10) ? 112 : 56;     // DF16~ = long
        int nbytes = nbits/8;

        // ── CRC-24 ──
        uint32_t rem = crc(msg, nbytes);
        bool ok=false; uint32_t icao=0; bool corrected=false;

        if(df==17 || df==18 || df==19){
            if(rem==0) ok=true;
            else{                                // 단일비트 정정 시도
                auto& tab = synd112;
                auto it = tab.find(rem);
                if(it!=tab.end()){ int p=it->second; msg[p>>3]^=(uint8_t)(0x80>>(p&7));
                                   if(crc(msg,nbytes)==0){ ok=true; corrected=true; } }
            }
            if(ok) icao = getbits(msg,8,24);
        } else if(df==11){
            // All-call reply: PI = parity ⊕ II. II=0 이면 잔차 0. ICAO 는 AA 필드.
            if(rem==0){ ok=true; icao=getbits(msg,8,24); }
            else{
                auto it = synd56.find(rem);
                if(it!=synd56.end()){ int p=it->second; msg[p>>3]^=(uint8_t)(0x80>>(p&7));
                    if(crc(msg,nbytes)==0){ ok=true; corrected=true; icao=getbits(msg,8,24); } }
            }
        } else if(df==0||df==4||df==5||df==16||df==20||df==21){
            // 감시응답: AP = address ⊕ parity → 잔차 == 송신 항공기 주소.
            // 이미 ADS-B/all-call 로 본 ICAO 와 일치할 때만 채택(오탐 방지).
            if(roster.count(rem)){ ok=true; icao=rem; }
        }
        if(!ok) return 0;
        (void)corrected;

        if(df==17||df==18||df==11) roster[icao]=1;

        AdsbRecord m{};
        m.crc_ok = true;
        m.df = df;
        m.icao = icao;
        double tnow = (samp_base + (double)j)/sr;

        if(df==17 || df==18){
            int tc = getbits(msg,32,5);
            m.tc = tc;
            if(tc>=1 && tc<=4)        parse_ident(msg, m);
            else if(tc>=9 && tc<=18)  parse_airpos(msg, m, tnow, /*baro*/true);
            else if(tc>=20 && tc<=22) parse_airpos(msg, m, tnow, /*baro*/false);
            else if(tc==19)           parse_velocity(msg, m);
            // 그 외 TC(표면위치/상태 등)는 ICAO 존재만 기록
        } else if(df==4 || df==20){
            parse_surv_alt(msg, m);          // 13-bit AC 고도
        } else if(df==5 || df==21){
            // identity(squawk) — 위치/고도 없음, ICAO 존재만
        }

        if(on_record) on_record(m);
        return (int)((PRE_US + nbits)*spb);
    }

    // ── DF17 식별(콜사인) TC 1..4 ──────────────────────────────────────────
    void parse_ident(const uint8_t* msg, AdsbRecord& m){
        m.category = getbits(msg,37,3);
        int o=0;
        for(int k=0;k<8;k++){
            char c = adsb_charset(getbits(msg,40+6*k,6));
            m.callsign[o++] = c;
        }
        m.callsign[o]=0;
        for(int i=o-1;i>=0;i--){ if(m.callsign[i]==' '||m.callsign[i]=='#') m.callsign[i]=0; else break; }
    }

    // ── 기압고도 12-bit AC (DF17 airborne position) ────────────────────────
    void parse_airpos(const uint8_t* msg, AdsbRecord& m, double tnow, bool baro){
        if(baro){
            uint32_t ac = getbits(msg,40,12);
            int q = (ac>>4)&1;
            if(q){ int nn = (int)(((ac & 0xFE0)>>1) | (ac & 0xF));
                   m.altitude = nn*25 - 1000; m.has_alt=true; }
            // q==0 (100ft Gillham gray code) 는 드묾 → 생략
        }
        int f       = getbits(msg,53,1);             // 0=even, 1=odd
        uint32_t la = getbits(msg,54,17);
        uint32_t lo = getbits(msg,71,17);
        cpr_position(m, tnow, f, la, lo);
    }

    // ── 속도 TC 19 ─────────────────────────────────────────────────────────
    void parse_velocity(const uint8_t* msg, AdsbRecord& m){
        int st = getbits(msg,37,3);
        // 수직속도 (공통): src(67) sign(68) val(69..77, 9-bit)
        int vr_sign = getbits(msg,68,1);
        int vr_raw  = getbits(msg,69,9);
        if(vr_raw){ m.vert_rate = (vr_raw-1)*64*(vr_sign?-1:1); m.has_vr=true; }

        if(st==1 || st==2){                          // 대지속도(ground speed)
            int    ew_dir = getbits(msg,45,1);
            int    ew_raw = getbits(msg,46,10);
            int    ns_dir = getbits(msg,56,1);
            int    ns_raw = getbits(msg,57,10);
            if(ew_raw && ns_raw){
                double mul = (st==2)?4.0:1.0;        // supersonic
                double vew = (ew_raw-1)*mul*(ew_dir?-1:1);
                double vns = (ns_raw-1)*mul*(ns_dir?-1:1);
                m.speed = (float)std::sqrt(vew*vew + vns*vns);
                double trk = std::atan2(vew, vns)*180.0/M_PI;
                if(trk<0) trk += 360.0;
                m.track = (float)trk;
                m.has_vel=true;
            }
        } else if(st==3 || st==4){                   // 대기속도(airspeed)+heading
            int hdg_st = getbits(msg,45,1);
            int hdg    = getbits(msg,46,10);
            int as_raw = getbits(msg,57,10);
            if(hdg_st){ m.track = (float)(hdg*360.0/1024.0); }
            if(as_raw){ double mul=(st==4)?4.0:1.0; m.speed=(float)((as_raw-1)*mul); }
            if(hdg_st || as_raw) m.has_vel=true;
        }
    }

    // ── DF4/20 감시응답 13-bit 고도 (AC field, bit 20..32) ───────────────────
    void parse_surv_alt(const uint8_t* msg, AdsbRecord& m){
        uint32_t ac = getbits(msg,19,13);            // 13-bit AC
        if(ac==0) return;
        int m_bit = (ac>>6)&1;                        // M bit (미터 단위) — 미지원 시 생략
        int q_bit = (ac>>4)&1;
        if(m_bit) return;
        if(q_bit){
            // 13-bit 에서 M(bit6),Q(bit4) 제거 → 11-bit, ×25 -1000 ft
            int nn = (int)( ((ac & 0x1F80)>>2) | ((ac & 0x20)>>1) | (ac & 0xF) );
            m.altitude = nn*25 - 1000; m.has_alt=true;
        }
    }

    // ── CPR: even/odd 짝으로 글로벌 디코드, 없으면 직전 기준점으로 로컬 디코드 ──
    void cpr_position(AdsbRecord& m, double tnow, int f, uint32_t lacpr, uint32_t locpr){
        CprState& s = cpr[m.icao];
        double cl = lacpr/131072.0, co = locpr/131072.0;   // /2^17
        if(f==0){ s.even_lat=cl; s.even_lon=co; s.even_t=tnow; }
        else    { s.odd_lat =cl; s.odd_lon =co; s.odd_t =tnow; }

        // 글로벌: even/odd 둘 다 10초 이내
        if(s.even_t>-1e8 && s.odd_t>-1e8 && std::fabs(s.even_t-s.odd_t)<10.0){
            double lat, lon;
            if(cpr_global(s, (f==0), lat, lon)){
                m.lat=lat; m.lon=lon; m.has_pos=true;
                s.ref_lat=lat; s.ref_lon=lon; s.has_ref=true;
                return;
            }
        }
        // 로컬: 기준점 있으면 단일 프레임으로
        if(s.has_ref){
            double lat, lon;
            cpr_local(s.ref_lat, s.ref_lon, f, cl, co, lat, lon);
            m.lat=lat; m.lon=lon; m.has_pos=true;
            s.ref_lat=lat; s.ref_lon=lon;
        }
    }

    static double cpr_mod(double a, double b){ double r=std::fmod(a,b); return r<0?r+b:r; }
    static int    NZ(){ return 15; }
    static double NL(double lat){
        if(std::fabs(lat) < 1e-9) return 59;
        if(std::fabs(lat) >= 87.0) return (std::fabs(lat)>87.0)?1:2;
        double a = 1.0 - std::cos(M_PI/(2.0*NZ()));
        double b = std::cos(M_PI/180.0*std::fabs(lat));
        double x = 1.0 - a/(b*b);
        if(x<-1.0) x=-1.0; if(x>1.0) x=1.0;
        return std::floor(2.0*M_PI/std::acos(x));
    }

    // 글로벌 디코드 (recent_even=true 면 마지막 프레임이 even). 성공 false=모호.
    bool cpr_global(const CprState& s, bool recent_even, double& outlat, double& outlon){
        double dlat0 = 360.0/60.0, dlat1 = 360.0/59.0;
        double j = std::floor(59.0*s.even_lat - 60.0*s.odd_lat + 0.5);
        double rle = dlat0*(cpr_mod(j,60.0) + s.even_lat);
        double rlo = dlat1*(cpr_mod(j,59.0) + s.odd_lat);
        if(rle>=270.0) rle-=360.0;
        if(rlo>=270.0) rlo-=360.0;
        if((int)NL(rle) != (int)NL(rlo)) return false;       // 위도존 불일치 → 모호

        if(recent_even){
            double nl = NL(rle); double ni = std::fmax(nl,1.0);
            double dlon = 360.0/ni;
            double mm = std::floor(s.even_lon*(nl-1.0) - s.odd_lon*nl + 0.5);
            double lon = dlon*(cpr_mod(mm,ni) + s.even_lon);
            if(lon>=180.0) lon-=360.0;
            outlat=rle; outlon=lon;
        } else {
            double nl = NL(rlo); double ni = std::fmax(nl-1.0,1.0);
            double dlon = 360.0/ni;
            double mm = std::floor(s.even_lon*(nl-1.0) - s.odd_lon*nl + 0.5);
            double lon = dlon*(cpr_mod(mm,ni) + s.odd_lon);
            if(lon>=180.0) lon-=360.0;
            outlat=rlo; outlon=lon;
        }
        return true;
    }

    // 로컬 디코드 (기준 위치 ref 근방, 단일 프레임)
    void cpr_local(double ref_lat, double ref_lon, int f, double cl, double co,
                   double& outlat, double& outlon){
        double dlat = 360.0/(f? 59.0 : 60.0);
        double j = std::floor(ref_lat/dlat) +
                   std::floor(0.5 + cpr_mod(ref_lat,dlat)/dlat - cl);
        double lat = dlat*(j + cl);
        double nl = NL(lat); double ni = f? std::fmax(nl-1.0,1.0) : std::fmax(nl,1.0);
        double dlon = 360.0/ni;
        double mm = std::floor(ref_lon/dlon) +
                    std::floor(0.5 + cpr_mod(ref_lon,dlon)/dlon - co);
        double lon = dlon*(mm + co);
        outlat=lat; outlon=lon;
    }

    // ── 비트 추출 (msg[0] MSB 부터 0-based) ─────────────────────────────────
    static uint32_t getbits(const uint8_t* msg, int startbit, int len){
        uint32_t v=0;
        for(int i=0;i<len;i++){
            int bit=startbit+i; v=(v<<1)|((msg[bit>>3]>>(7-(bit&7)))&1);
        }
        return v;
    }

    // ── CRC-24 (Mode S, 생성다항식 0xFFF409). 무오류 프레임 잔차 0 ──────────
    static uint32_t crc(const uint8_t* msg, int nbytes){
        uint32_t rem=0;
        for(int i=0;i<nbytes;i++){
            rem ^= (uint32_t)msg[i]<<16;
            for(int b=0;b<8;b++)
                rem = (rem & 0x800000) ? ((rem<<1)^0xFFF409)&0xFFFFFF : (rem<<1)&0xFFFFFF;
        }
        return rem & 0xFFFFFF;
    }

    // 단일비트 오류 잔차표: bit p 만 1인 메시지의 CRC → p. (CRC 선형성)
    void build_syndrome(){
        if(!synd112.empty()) return;
        for(int nbits : {56,112}){
            auto& tab = (nbits==112)? synd112 : synd56;
            int nbytes=nbits/8;
            for(int p=0;p<nbits;p++){
                uint8_t e[14]={0};
                e[p>>3] = (uint8_t)(0x80>>(p&7));
                tab[crc(e,nbytes)] = p;
            }
        }
    }
};
