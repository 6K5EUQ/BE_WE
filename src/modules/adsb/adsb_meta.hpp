#pragma once
// ── ADS-B / Mode S 레코드 + wire 포맷 + 라벨 (ICAO Annex 10 Vol IV / RTCA DO-260) ──
// 1090ES 다운링크. ais_meta.hpp 구조 미러 (항공기도 위치형 → 지도 표시).
// station 은 Record 에만 (MpData 봉투가 운반), Wire 엔 없음.
//
// 한 메시지 = 한 종류 정보만 운반 (식별 / 위치 / 속도 …). 뷰 레이어가 ICAO 별로
// 최신값을 병합해 지도/표에 한 항공기로 모은다 (AIS 가 MMSI 별 병합하는 것과 동일).
#include <cstdint>
#include <cstring>
#include <cmath>

struct AdsbRecord {
    int64_t  t_ms = 0;          // 호스트 스탬프
    float    freq = 0.f;        // 채널 중심 MHz (호스트 스탬프)
    int      ch   = 0;          // 채널필터 인덱스
    bool     crc_ok = false;    // CRC-24 통과 (정정 포함)

    int      df   = 0;          // Downlink Format (17/18=ADS-B, 11=all-call, 4/5/20/21=surv)
    uint32_t icao = 0;          // 24-bit 항공기 주소
    int      tc   = -1;         // DF17/18 Type Code (-1 = n/a)

    char     callsign[9] = {};  // 항공기 식별자 (8자 + null)
    int      category = 0;      // emitter category (0 = n/a)

    bool     has_alt = false;  int altitude = 0;          // ft (기압고도)
    bool     has_pos = false;  double lat = 0.0, lon = 0.0;
    bool     has_vel = false;  float  speed = -1.f, track = -1.f;  // kt / deg (-1 = n/a)
    bool     has_vr  = false;  int    vert_rate = 0;       // ft/min (+상승)

    char     station[16] = {};  // 복조 기지 표시명 (DGS-2 / LOCAL)
};

// wire 포맷 (framework BEWE_MK_DATA payload; station 은 MpData 봉투가 운반)
struct __attribute__((packed)) AdsbWireMsg {
    int64_t  t_ms; float freq; int32_t ch;
    uint8_t  crc_ok; int32_t df; uint32_t icao; int32_t tc;
    char     callsign[9]; int32_t category;
    uint8_t  has_alt; int32_t altitude;
    uint8_t  has_pos; double lat; double lon;
    uint8_t  has_vel; float speed; float track;
    uint8_t  has_vr;  int32_t vert_rate;
};

inline void adsb_msg_to_wire(const AdsbRecord& m, AdsbWireMsg& w){
    memset(&w, 0, sizeof(w));
    w.t_ms=m.t_ms; w.freq=m.freq; w.ch=m.ch;
    w.crc_ok=m.crc_ok?1:0; w.df=m.df; w.icao=m.icao; w.tc=m.tc;
    memcpy(w.callsign, m.callsign, sizeof(w.callsign)); w.category=m.category;
    w.has_alt=m.has_alt?1:0; w.altitude=m.altitude;
    w.has_pos=m.has_pos?1:0; w.lat=m.lat; w.lon=m.lon;
    w.has_vel=m.has_vel?1:0; w.speed=m.speed; w.track=m.track;
    w.has_vr =m.has_vr?1:0;  w.vert_rate=m.vert_rate;
}
inline void adsb_wire_to_msg(const AdsbWireMsg& w, AdsbRecord& m){
    m = AdsbRecord{};
    m.t_ms=w.t_ms; m.freq=w.freq; m.ch=w.ch;
    m.crc_ok=w.crc_ok!=0; m.df=w.df; m.icao=w.icao; m.tc=w.tc;
    memcpy(m.callsign, w.callsign, sizeof(m.callsign)); m.callsign[sizeof(m.callsign)-1]=0;
    m.category=w.category;
    m.has_alt=w.has_alt!=0; m.altitude=w.altitude;
    m.has_pos=w.has_pos!=0; m.lat=w.lat; m.lon=w.lon;
    m.has_vel=w.has_vel!=0; m.speed=w.speed; m.track=w.track;
    m.has_vr =w.has_vr!=0;  m.vert_rate=w.vert_rate;
}

// ── DF (Downlink Format) 라벨 ───────────────────────────────────────────────
inline const char* adsb_df_name(int df){
    switch(df){
        case 0:  return "Short ACAS";
        case 4:  return "Surv Alt";
        case 5:  return "Surv ID";
        case 11: return "All-call";
        case 16: return "Long ACAS";
        case 17: return "ADS-B";
        case 18: return "ADS-B(TIS)";
        case 19: return "Military";
        case 20: return "Comm-B Alt";
        case 21: return "Comm-B ID";
        case 24: return "Comm-D";
        default: return "";
    }
}

// ── DF17/18 Type Code → 메시지 종류 라벨 ────────────────────────────────────
inline const char* adsb_tc_name(int tc){
    if(tc>=1 && tc<=4)   return "Ident";
    if(tc>=5 && tc<=8)   return "Surface Pos";
    if(tc>=9 && tc<=18)  return "Airborne Pos";
    if(tc==19)           return "Velocity";
    if(tc>=20 && tc<=22) return "Airborne Pos(GNSS)";
    if(tc==28)           return "Status";
    if(tc==29)           return "Target State";
    if(tc==31)           return "Op Status";
    return "";
}

// ── 6-bit ADS-B 콜사인 문자셋 (DO-260) ──────────────────────────────────────
// 0=invalid, 1..26=A..Z, 32=space, 48..57=0..9, 그 외 invalid.
inline char adsb_charset(unsigned c){
    static const char* T =
        "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####"   // 0..31
        " ###############0123456789######";  // 32..63
    return (c<64) ? T[c] : '#';
}
