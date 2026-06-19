#pragma once
// ── BLE(Bluetooth Low Energy) 레코드 + wire 포맷 + 라벨 (Bluetooth Core Vol 6) ──
// 광고 채널(37/38/39) PDU 만 디코드. 1M PHY(GFSK 1 Mbps). 1 패킷 = 1 레코드 자기완결.
// CONNECT_IND 는 연결 파라미터(AccessAddress/Hop/Interval/ChannelMap)까지 운반.
// 데이터 채널 홉 추적은 단일 LO + ~8 MHz(USB2) 대역폭 한계로 미지원(adsb_meta.hpp 미러).
#include <cstdint>
#include <cstring>
#include <cstdio>

struct BtleRecord {
    int64_t  t_ms = 0;          // 호스트 스탬프
    float    freq = 0.f;        // 채널 중심 MHz
    int      ch   = 0;          // 채널필터 인덱스
    bool     crc_ok = false;    // CRC-24 통과

    int      adv_chan  = 37;    // 광고 채널 (37/38/39)
    int      pdu_type  = 0;     // 0=ADV_IND..6=ADV_SCAN_IND (Vol6 PartB 2.3)
    int      addr_type = 0;     // 0=public, 1=random (TxAdd)
    uint8_t  mac[6]    = {};    // AdvA (표시순: mac[0]=MSB)

    // ── 광고 데이터 (ADV_IND / ADV_NONCONN_IND / SCAN_RSP / ADV_SCAN_IND) ──
    char     name[24]  = {};    // Complete/Shortened Local Name (0x08/0x09)
    int      flags     = -1;    // AD type 0x01 (-1 = 없음)
    uint16_t company   = 0xFFFF;// 제조사데이터(0xFF) 앞 2바이트 회사ID (0xFFFF=없음)
    int      n_ad      = 0;     // AD 구조 개수
    char     info[48]  = {};    // 요약 (iBeacon / 회사명 등)

    // ── CONNECT_IND (pdu_type==5) 연결 파라미터 ──
    bool     is_connect = false;
    uint8_t  init_mac[6]= {};   // InitA (표시순)
    uint32_t access_addr= 0;     // 데이터 채널 AccessAddress
    uint32_t crc_init   = 0;     // 데이터 채널 CRCInit (24-bit)
    int      interval   = 0;     // connInterval (단위 1.25 ms)
    int      timeout    = 0;     // supervisionTimeout (단위 10 ms)
    int      latency    = 0;     // slaveLatency
    int      hop        = 0;     // hopIncrement (5..16)
    int      sca        = 0;     // sleep clock accuracy (0..7)
    uint64_t chan_map   = 0;     // 37-bit 데이터 채널 맵

    char     station[16]= {};    // 복조 기지 표시명 (DGS-2 / LOCAL)
};

// wire 포맷 (framework BEWE_MK_DATA payload; station 은 MpData 봉투가 운반)
struct __attribute__((packed)) BtleWireMsg {
    int64_t  t_ms; float freq; int32_t ch; uint8_t crc_ok;
    int32_t  adv_chan; int32_t pdu_type; int32_t addr_type; uint8_t mac[6];
    char     name[24]; int32_t flags; uint16_t company; int32_t n_ad; char info[48];
    uint8_t  is_connect; uint8_t init_mac[6];
    uint32_t access_addr; uint32_t crc_init;
    int32_t  interval; int32_t timeout; int32_t latency; int32_t hop; int32_t sca;
    uint64_t chan_map;
};

inline void btle_msg_to_wire(const BtleRecord& m, BtleWireMsg& w){
    memset(&w, 0, sizeof(w));
    w.t_ms=m.t_ms; w.freq=m.freq; w.ch=m.ch; w.crc_ok=m.crc_ok?1:0;
    w.adv_chan=m.adv_chan; w.pdu_type=m.pdu_type; w.addr_type=m.addr_type;
    memcpy(w.mac,m.mac,6);
    memcpy(w.name,m.name,sizeof(w.name)); w.flags=m.flags; w.company=m.company;
    w.n_ad=m.n_ad; memcpy(w.info,m.info,sizeof(w.info));
    w.is_connect=m.is_connect?1:0; memcpy(w.init_mac,m.init_mac,6);
    w.access_addr=m.access_addr; w.crc_init=m.crc_init;
    w.interval=m.interval; w.timeout=m.timeout; w.latency=m.latency;
    w.hop=m.hop; w.sca=m.sca; w.chan_map=m.chan_map;
}
inline void btle_wire_to_msg(const BtleWireMsg& w, BtleRecord& m){
    m = BtleRecord{};
    m.t_ms=w.t_ms; m.freq=w.freq; m.ch=w.ch; m.crc_ok=w.crc_ok!=0;
    m.adv_chan=w.adv_chan; m.pdu_type=w.pdu_type; m.addr_type=w.addr_type;
    memcpy(m.mac,w.mac,6);
    memcpy(m.name,w.name,sizeof(m.name)); m.name[sizeof(m.name)-1]=0;
    m.flags=w.flags; m.company=w.company; m.n_ad=w.n_ad;
    memcpy(m.info,w.info,sizeof(m.info)); m.info[sizeof(m.info)-1]=0;
    m.is_connect=w.is_connect!=0; memcpy(m.init_mac,w.init_mac,6);
    m.access_addr=w.access_addr; m.crc_init=w.crc_init;
    m.interval=w.interval; m.timeout=w.timeout; m.latency=w.latency;
    m.hop=w.hop; m.sca=w.sca; m.chan_map=w.chan_map;
}

// ── PDU 타입 라벨 (Vol6 PartB Table 2.3, 광고 물리 채널) ─────────────────────
inline const char* btle_pdu_name(int t){
    switch(t){
        case 0x0: return "ADV_IND";
        case 0x1: return "ADV_DIRECT_IND";
        case 0x2: return "ADV_NONCONN_IND";
        case 0x3: return "SCAN_REQ";
        case 0x4: return "SCAN_RSP";
        case 0x5: return "CONNECT_IND";
        case 0x6: return "ADV_SCAN_IND";
        case 0x7: return "ADV_EXT_IND";
        default:  return "";
    }
}

// ── 제조사 회사ID → 이름 (Bluetooth SIG, 대표값 일부만 자급식) ────────────────
inline const char* btle_company_name(uint16_t c){
    switch(c){
        case 0x004C: return "Apple";
        case 0x0006: return "Microsoft";
        case 0x0075: return "Samsung";
        case 0x00E0: return "Google";
        case 0x0059: return "Nordic";
        case 0x02E5: return "Espressif";
        case 0x0157: return "Huawei";
        case 0x0001: return "Ericsson";
        case 0x000F: return "Broadcom";
        case 0x0078: return "Nike";
        case 0x0171: return "Amazon";
        case 0x0499: return "Ruuvi";
        default:     return "";
    }
}
