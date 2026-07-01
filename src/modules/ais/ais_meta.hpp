#pragma once
// ── AIS 메시지 레코드 + wire 포맷 + 보조 디코드 헬퍼 ────────────────────────
// ITU-R M.1371 (Universal Shipborne AIS, TDMA VHF). 위치/정적 메시지 필드 추출.
#include <cstdint>
#include <cstring>
#include <cmath>

struct AisRecord {
    int64_t  t_ms = 0;        // wall time (호스트 스탬프)
    float    freq = 0.f;      // 채널 중심 MHz (호스트 스탬프)
    int      ch   = 0;        // 채널필터 인덱스
    bool     crc_ok = false;
    int      msg_type = 0;    // 1..27
    uint32_t mmsi = 0;
    // ── 위치 (1/2/3/4/18/19) ──
    bool     has_pos = false;
    double   lat = 0.0, lon = 0.0;   // 도(degree). has_pos=false 면 무효
    float    sog = -1.f;      // knots (-1 = n/a)
    float    cog = -1.f;      // deg   (-1 = n/a)
    int      heading = 511;   // deg   (511 = n/a)
    int      nav_status = -1; // 0..15 (-1 = n/a / 비위치 메시지)
    // ── 정적 (5/19/24) ──
    char     name[21] = {};       // 선박명
    char     callsign[8] = {};
    int      ship_type = 0;       // 0..99
    // ── 메타 ──
    char     station[16] = {};    // 복조 기지 표시명 (DGS-2 / LOCAL)
    uint32_t rx_cnt = 0;          // >0 = Central 요약이 실어보낸 그 MMSI 실제 누계 수신수 (표 Cnt 권위값). 0=미상(레코드수로 집계)
    // ── Type 5 정적/항해 확장 (정적 msg 에서만 옴) ──
    uint32_t imo = 0;             // IMO 선박식별번호 (0 = n/a)
    char     dest[21] = {};       // 목적지 (UN/LOCODE 등 자유문자)
    float    draught = -1.f;      // 최대 현재 흘수 m (-1 = n/a)
    uint8_t  eta_mon = 0, eta_day = 0, eta_hour = 24, eta_min = 60;  // ETA (mon 0 = n/a)
};

// wire 포맷 (framework BEWE_MK_DATA payload; station 은 MpData 봉투가 운반)
// rx_cnt 는 끝에 추가 — 기존 필드 오프셋 불변(구버전 .dat 레코드도 앞 필드 파싱 안전).
struct __attribute__((packed)) AisWireMsg {
    int64_t  t_ms; float freq; int32_t ch;
    uint8_t  crc_ok; int32_t msg_type; uint32_t mmsi;
    uint8_t  has_pos; double lat; double lon;
    float    sog; float cog; int32_t heading; int32_t nav_status;
    char     name[21]; char callsign[8]; int32_t ship_type;
    uint32_t rx_cnt;
    uint32_t imo; char dest[21]; float draught;
    uint8_t  eta_mon, eta_day, eta_hour, eta_min;
};

inline void ais_msg_to_wire(const AisRecord& m, AisWireMsg& w){
    memset(&w, 0, sizeof(w));
    w.t_ms=m.t_ms; w.freq=m.freq; w.ch=m.ch;
    w.crc_ok=m.crc_ok?1:0; w.msg_type=m.msg_type; w.mmsi=m.mmsi;
    w.has_pos=m.has_pos?1:0; w.lat=m.lat; w.lon=m.lon;
    w.sog=m.sog; w.cog=m.cog; w.heading=m.heading; w.nav_status=m.nav_status;
    memcpy(w.name,m.name,sizeof(w.name)); memcpy(w.callsign,m.callsign,sizeof(w.callsign));
    w.ship_type=m.ship_type; w.rx_cnt=m.rx_cnt;
    w.imo=m.imo; memcpy(w.dest,m.dest,sizeof(w.dest)); w.draught=m.draught;
    w.eta_mon=m.eta_mon; w.eta_day=m.eta_day; w.eta_hour=m.eta_hour; w.eta_min=m.eta_min;
}
inline void ais_wire_to_msg(const AisWireMsg& w, AisRecord& m){
    m = AisRecord{};
    m.t_ms=w.t_ms; m.freq=w.freq; m.ch=w.ch;
    m.crc_ok=w.crc_ok!=0; m.msg_type=w.msg_type; m.mmsi=w.mmsi;
    m.has_pos=w.has_pos!=0; m.lat=w.lat; m.lon=w.lon;
    m.sog=w.sog; m.cog=w.cog; m.heading=w.heading; m.nav_status=w.nav_status;
    memcpy(m.name,w.name,sizeof(m.name)); m.name[sizeof(m.name)-1]=0;
    memcpy(m.callsign,w.callsign,sizeof(m.callsign)); m.callsign[sizeof(m.callsign)-1]=0;
    m.ship_type=w.ship_type; m.rx_cnt=w.rx_cnt;
    m.imo=w.imo; memcpy(m.dest,w.dest,sizeof(m.dest)); m.dest[sizeof(m.dest)-1]=0; m.draught=w.draught;
    m.eta_mon=w.eta_mon; m.eta_day=w.eta_day; m.eta_hour=w.eta_hour; m.eta_min=w.eta_min;
}

// ── AIS 6-bit ASCII → 8-bit ASCII (ITU-R M.1371 Table 47) ──────────────────
inline char ais_sixbit_ascii(unsigned s){
    if(s>=1 && s<=31) return (char)(s+64);   // @A..Z[\]^_  (1='A')
    if(s<=63)         return (char)s;          // space..? (32='space', 48='0' ...)
    return ' ';
}

// ── 항법상태 (nav_status) 라벨 ─────────────────────────────────────────────
inline const char* ais_navstatus(int s){
    static const char* T[16]={
        "Underway(eng)","Anchored","Not under cmd","Restricted manoeuv",
        "Constrained draught","Moored","Aground","Fishing","Sailing",
        "HSC","WIG","Reserved11","Reserved12","Reserved13","AIS-SART","Undefined"};
    return (s>=0 && s<16) ? T[s] : "";
}

// ── 선종 (ship type, ITU-R M.1371 Table 53) 대략 그룹 라벨 ──────────────────
inline const char* ais_shiptype(int t){
    if(t<=0||t>99) return "";
    if(t>=20&&t<=29) return "WIG";
    if(t==30) return "Fishing";
    if(t==31||t==32) return "Towing";
    if(t==33) return "Dredging";
    if(t==34) return "Diving";
    if(t==35) return "Military";
    if(t==36) return "Sailing";
    if(t==37) return "Pleasure";
    if(t>=40&&t<=49) return "HSC";
    if(t==50) return "Pilot";
    if(t==51) return "SAR";
    if(t==52) return "Tug";
    if(t==53) return "Port tender";
    if(t==55) return "Law enforce";
    if(t>=60&&t<=69) return "Passenger";
    if(t>=70&&t<=79) return "Cargo";
    if(t>=80&&t<=89) return "Tanker";
    if(t>=90&&t<=99) return "Other";
    return "Other";
}

// ── MMSI MID(앞 3자리) → 국가 (대표 식별번호만, 자급식) ─────────────────────
inline const char* ais_mid_country(uint32_t mmsi){
    int mid;
    // 9자리 표준 MMSI 외(연안국 콜사인 등)는 앞 3자리 사용. 일부 타입은 8/0 시작.
    if(mmsi>=200000000 && mmsi<=799999999) mid=mmsi/1000000;
    else if(mmsi>=800000000 && mmsi<=899999999) mid=(mmsi/100000)%1000; // base/aux
    else return "";
    switch(mid){
        case 440: case 441: return "Korea";
        case 431: case 432: return "Japan";
        case 412: case 413: case 414: return "China";
        case 416: return "Taiwan";
        case 477: case 457: return "HongKong";
        case 525: return "Indonesia";
        case 533: return "Malaysia";
        case 563: case 564: case 565: case 566: return "Singapore";
        case 567: return "Thailand";
        case 574: return "Vietnam";
        case 548: return "Philippines";
        case 419: return "India";
        case 366: case 367: case 368: case 369: case 338: case 358: return "USA";
        case 316: return "Canada";
        case 503: return "Australia";
        case 512: return "NewZealand";
        case 232: case 233: case 234: case 235: return "UK";
        case 211: case 218: return "Germany";
        case 226: case 227: case 228: return "France";
        case 247: return "Italy";
        case 224: return "Spain";
        case 244: case 245: case 246: return "Netherlands";
        case 257: case 258: case 259: return "Norway";
        case 265: case 266: return "Sweden";
        case 273: return "Russia";
        case 271: return "Turkey";
        case 636: case 637: return "Liberia";
        case 538: return "MarshallIs";
        case 351: case 352: case 353: case 354: case 370: case 371: case 372: case 373: return "Panama";
        default: return "";
    }
}
