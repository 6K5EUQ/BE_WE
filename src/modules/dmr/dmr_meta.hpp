#pragma once
// ── DMR 레코드 + wire 포맷 + 라벨 (ETSI TS 102 361) ─────────────────────────
// 메타데이터/시그널링만 (AMBE 음성 제외). Tier II 컨벤셔널 + 범용 LC/CSBK.
// ais_meta.hpp 구조 미러. station 은 Record 에만(MpData 봉투 운반), Wire 엔 없음.
#include <cstdint>
#include <cstring>

struct DmrRecord {
    int64_t  t_ms = 0;          // 호스트 스탬프
    float    freq = 0.f;        // 채널 중심 MHz (호스트 스탬프)
    int      ch   = 0;          // 채널필터 인덱스
    bool     crc_ok = false;    // FEC/CRC 검증 통과
    int      slot = 0;          // 0=미상, 1/2 = TS1/TS2
    int      color_code = -1;   // 0..15 (-1=미상)
    int      data_type = -1;    // Slot Type DataType (0..11, -1=voice/미상)
    int      flco = -1;         // Full LC opcode (-1=n/a)
    int      csbko = -1;        // CSBK opcode (-1=n/a)
    uint32_t src_id = 0;        // 소스 주소 (radio id)
    uint32_t dst_id = 0;        // 목적 주소 (talkgroup / radio id)
    int      call_type = -1;    // 0=group, 1=individual, -1=n/a
    bool     is_voice = false;  // 음성 활동
    char     station[16] = {};  // 복조 기지 표시명 (DGS-2 / LOCAL)
};

// wire 포맷 (BEWE_MK_DATA payload; station 은 MpData 봉투가 운반)
struct __attribute__((packed)) DmrWireMsg {
    int64_t  t_ms; float freq; int32_t ch;
    uint8_t  crc_ok; int8_t slot; int8_t color_code; int8_t data_type;
    int8_t   call_type; uint8_t is_voice; int16_t flco; int16_t csbko;
    uint32_t src_id; uint32_t dst_id;
};

inline void dmr_msg_to_wire(const DmrRecord& m, DmrWireMsg& w){
    memset(&w,0,sizeof(w));
    w.t_ms=m.t_ms; w.freq=m.freq; w.ch=m.ch;
    w.crc_ok=m.crc_ok?1:0; w.slot=(int8_t)m.slot; w.color_code=(int8_t)m.color_code;
    w.data_type=(int8_t)m.data_type; w.call_type=(int8_t)m.call_type;
    w.is_voice=m.is_voice?1:0; w.flco=(int16_t)m.flco; w.csbko=(int16_t)m.csbko;
    w.src_id=m.src_id; w.dst_id=m.dst_id;
}
inline void dmr_wire_to_msg(const DmrWireMsg& w, DmrRecord& m){
    m = DmrRecord{};
    m.t_ms=w.t_ms; m.freq=w.freq; m.ch=w.ch;
    m.crc_ok=w.crc_ok!=0; m.slot=w.slot; m.color_code=w.color_code;
    m.data_type=w.data_type; m.call_type=w.call_type;
    m.is_voice=w.is_voice!=0; m.flco=w.flco; m.csbko=w.csbko;
    m.src_id=w.src_id; m.dst_id=w.dst_id;
}

// ── DataType (Slot Type 4비트, ETSI 9.3.6) ──────────────────────────────────
inline const char* dmr_datatype_name(int dt){
    static const char* T[16]={"PI Hdr","Voice LC Hdr","Term LC","CSBK","MBC Hdr",
        "MBC Cont","Data Hdr","Rate1/2","Rate3/4","Idle","Rate1","USBD","?","?","?","?"};
    return (dt>=0&&dt<16)?T[dt]:"";
}
// ── FLCO (Full LC opcode 6비트) ──────────────────────────────────────────────
inline const char* dmr_flco_name(int f){
    switch(f){
        case 0x00: return "Grp Voice";       // Group Voice Channel User
        case 0x03: return "UU Voice";        // Unit-Unit Voice Channel User
        case 0x04: return "Talker Alias Hdr";
        case 0x05: case 0x06: case 0x07: return "Talker Alias Blk";
        case 0x08: return "GPS Info";
        case 0x30: return "Term Data LC";
        default: return "";
    }
}
inline const char* dmr_calltype_name(int c){
    return c==0?"Group":c==1?"Individual":"";
}

// flco → call_type (group/individual)
inline int dmr_flco_calltype(int flco){
    if(flco==0x00) return 0;   // group voice
    if(flco==0x03) return 1;   // unit-to-unit
    return -1;
}
