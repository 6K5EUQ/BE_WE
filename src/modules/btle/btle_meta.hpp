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
    float    rssi    = 0.f;     // 패킷 평균전력 dBFS (상대 신호세기, 0=미측정)
    float    cfo_hz  = 0.f;     // 반송파 주파수오프셋 Hz (송신기 RF 지문; RX LO 공통분 포함)

    int      adv_chan  = 37;    // 광고 채널 (37/38/39)
    int      pdu_type  = 0;     // 0=ADV_IND..6=ADV_SCAN_IND (Vol6 PartB 2.3)
    int      addr_type = 0;     // 0=public, 1=random (TxAdd)
    uint8_t  mac[6]    = {};    // AdvA (표시순: mac[0]=MSB)

    // ── 광고 데이터 (ADV_IND / ADV_NONCONN_IND / SCAN_RSP / ADV_SCAN_IND) ──
    char     name[24]  = {};    // Complete/Shortened Local Name (0x08/0x09)
    int      flags     = -1;    // AD type 0x01 (-1 = 없음)
    uint16_t company   = 0xFFFF;// 제조사데이터(0xFF) 앞 2바이트 회사ID (0xFFFF=없음)
    int      appearance= -1;    // AD type 0x19 GAP Appearance (-1 = 없음)
    int      n_ad      = 0;     // AD 구조 개수
    char     info[48]  = {};    // 요약 (iBeacon / 회사명 / 기기모델 등)

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
    float    rssi; float cfo_hz;
    int32_t  adv_chan; int32_t pdu_type; int32_t addr_type; uint8_t mac[6];
    char     name[24]; int32_t flags; uint16_t company; int32_t appearance; int32_t n_ad; char info[48];
    uint8_t  is_connect; uint8_t init_mac[6];
    uint32_t access_addr; uint32_t crc_init;
    int32_t  interval; int32_t timeout; int32_t latency; int32_t hop; int32_t sca;
    uint64_t chan_map;
};

inline void btle_msg_to_wire(const BtleRecord& m, BtleWireMsg& w){
    memset(&w, 0, sizeof(w));
    w.t_ms=m.t_ms; w.freq=m.freq; w.ch=m.ch; w.crc_ok=m.crc_ok?1:0;
    w.rssi=m.rssi; w.cfo_hz=m.cfo_hz;
    w.adv_chan=m.adv_chan; w.pdu_type=m.pdu_type; w.addr_type=m.addr_type;
    memcpy(w.mac,m.mac,6);
    memcpy(w.name,m.name,sizeof(w.name)); w.flags=m.flags; w.company=m.company;
    w.appearance=m.appearance; w.n_ad=m.n_ad; memcpy(w.info,m.info,sizeof(w.info));
    w.is_connect=m.is_connect?1:0; memcpy(w.init_mac,m.init_mac,6);
    w.access_addr=m.access_addr; w.crc_init=m.crc_init;
    w.interval=m.interval; w.timeout=m.timeout; w.latency=m.latency;
    w.hop=m.hop; w.sca=m.sca; w.chan_map=m.chan_map;
}
inline void btle_wire_to_msg(const BtleWireMsg& w, BtleRecord& m){
    m = BtleRecord{};
    m.t_ms=w.t_ms; m.freq=w.freq; m.ch=w.ch; m.crc_ok=w.crc_ok!=0;
    m.rssi=w.rssi; m.cfo_hz=w.cfo_hz;
    m.adv_chan=w.adv_chan; m.pdu_type=w.pdu_type; m.addr_type=w.addr_type;
    memcpy(m.mac,w.mac,6);
    memcpy(m.name,w.name,sizeof(m.name)); m.name[sizeof(m.name)-1]=0;
    m.flags=w.flags; m.company=w.company; m.appearance=w.appearance; m.n_ad=w.n_ad;
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

// ── 제조사 회사ID → 이름 (Bluetooth SIG Assigned Numbers, 소비자기기 위주 발췌) ──
// 값은 공식 SIG company_identifiers.yaml 대조 검증. (Qualcomm/Polar 등 일부는 합병/이전
// 으로 복수 ID 존재 → 그대로 둠.) 미수록 ID 는 호출측에서 raw 0xXXXX 로 표기.
inline const char* btle_company_name(uint16_t c){
    switch(c){
        case 0x0000: return "Ericsson";
        case 0x0001: return "Nokia";
        case 0x0002: return "Intel";
        case 0x0004: return "Toshiba";
        case 0x0006: return "Microsoft";
        case 0x0008: return "Motorola";
        case 0x0009: return "Infineon";
        case 0x000A: return "Qualcomm";
        case 0x000D: return "TI";
        case 0x000F: return "Broadcom";
        case 0x001D: return "Qualcomm";
        case 0x0025: return "NXP";
        case 0x0030: return "STMicro";
        case 0x0036: return "Renesas";
        case 0x003A: return "Panasonic";
        case 0x003F: return "Bluetooth SIG";
        case 0x004C: return "Apple";
        case 0x0055: return "Plantronics";
        case 0x0056: return "Sony Ericsson";
        case 0x0057: return "Harman";
        case 0x0059: return "Nordic";
        case 0x005D: return "Realtek";
        case 0x0067: return "GN";
        case 0x006B: return "Polar";
        case 0x0075: return "Samsung";
        case 0x0078: return "Nike";
        case 0x0087: return "Garmin";
        case 0x0089: return "GN Hearing";
        case 0x009E: return "Bose";
        case 0x009F: return "Suunto";
        case 0x00C4: return "LG";
        case 0x00CC: return "Beats";
        case 0x00CE: return "Eve";
        case 0x00D1: return "Polar";
        case 0x00D7: return "Qualcomm";
        case 0x00DF: return "Misfit";
        case 0x00E0: return "Google";
        case 0x0103: return "Bang & Olufsen";
        case 0x011B: return "HP";
        case 0x012D: return "Sony";
        case 0x012E: return "ASSA ABLOY";
        case 0x013B: return "Allegion";
        case 0x014F: return "Bowers & Wilkins";
        case 0x0154: return "Pebble";
        case 0x0157: return "Huami";
        case 0x0171: return "Amazon";
        case 0x0178: return "Casio";
        case 0x01AB: return "Meta";
        case 0x01B5: return "Nest";
        case 0x01D1: return "August";
        case 0x01DA: return "Logitech";
        case 0x01DD: return "Philips";
        case 0x01FC: return "Wahoo";
        case 0x0211: return "Telink";
        case 0x0225: return "Nestle";
        case 0x022B: return "Tesla";
        case 0x0241: return "Bragi";
        case 0x027D: return "Huawei";
        case 0x02B2: return "Oura";
        case 0x02C5: return "Lenovo";
        case 0x02D1: return "Empatica";
        case 0x02DE: return "Samsung SDS";
        case 0x02E5: return "Espressif";
        case 0x02ED: return "HTC";
        case 0x02EE: return "Citizen";
        case 0x02F2: return "GoPro";
        case 0x02FF: return "Silicon Labs";
        case 0x0304: return "Oura";
        case 0x037F: return "Nestle";
        case 0x0381: return "Sharp";
        case 0x038F: return "Xiaomi";
        case 0x03FF: return "Withings";
        case 0x041E: return "Dell";
        case 0x0450: return "Teenage Eng";
        case 0x0494: return "Sennheiser";
        case 0x0499: return "Ruuvi";
        case 0x04EC: return "Motorola";
        case 0x0553: return "Nintendo";
        case 0x058E: return "Meta";
        case 0x05A7: return "Sonos";
        case 0x05F0: return "Beken";
        case 0x060F: return "Signify";
        case 0x065A: return "Marshall";
        case 0x067C: return "Tile";
        case 0x068E: return "Razer";
        case 0x072F: return "OnePlus";
        case 0x075A: return "Harman";
        case 0x079A: return "Oppo";
        case 0x07C9: return "Skullcandy";
        case 0x0837: return "vivo";
        case 0x0870: return "Wyze";
        case 0x08AA: return "DJI";
        case 0x08C3: return "Chipolo";
        case 0x09C6: return "Honor";
        case 0x0A12: return "Dyson";
        case 0x0BA9: return "Shelly";
        case 0x0BDE: return "Yale";
        case 0x0C93: return "Movesense";
        case 0x0CC2: return "Anker";
        case 0x0CCB: return "Nothing";
        case 0x0CE7: return "TAG Heuer";
        case 0x0E41: return "Asus";
        case 0x10D1: return "Acer";
        default:     return "";
    }
}

// ── GAP Appearance (AD 0x19) → 기기 카테고리 (Assigned Numbers §2.6, 상위 10비트=카테고리) ──
// 16비트 = (category<<6)|subcategory. 대표 카테고리/세부값만 라벨, 미수록은 빈값.
inline const char* btle_appearance_name(int a){
    if(a<0) return "";
    switch(a){                                   // 자주 보이는 세부값 우선
        case 0x0040: return "Phone";
        case 0x0080: return "Computer";
        case 0x00C0: return "Watch";
        case 0x00C1: return "Sports Watch";
        case 0x0100: return "Clock";
        case 0x0140: return "Display";
        case 0x0180: return "Remote";
        case 0x01C0: return "Eyeglasses";
        case 0x0200: return "Tag";
        case 0x0240: return "Keyring";
        case 0x0280: return "Media Player";
        case 0x02C0: return "Barcode Scanner";
        case 0x0300: return "Thermometer";
        case 0x0340: return "Heart Rate";
        case 0x0380: return "Blood Pressure";
        case 0x03C0: return "HID";
        case 0x03C1: return "Keyboard";
        case 0x03C2: return "Mouse";
        case 0x03C3: return "Joystick";
        case 0x03C4: return "Gamepad";
        case 0x03C5: return "Tablet";
        case 0x03C8: return "Pen";              // Digitizer/Stylus
        case 0x0400: return "Glucose";
        case 0x0440: return "Running Sensor";
        case 0x0480: return "Cycling";
        case 0x0444: return "Cadence Sensor";
        case 0x0448: return "Power Sensor";
        case 0x0540: return "Pulse Oximeter";
        case 0x0580: return "Weight Scale";
        case 0x0700: return "Earbuds";          // Audio: Earbud
        case 0x0841: return "Headset";
        case 0x0842: return "Headphones";
        case 0x0843: return "Microphone";
        case 0x0844: return "Speaker";
        default: break;
    }
    switch(a>>6){                                // 카테고리(상위 10비트) 폴백
        case 0x001: return "Phone";
        case 0x002: return "Computer";
        case 0x003: return "Watch";
        case 0x004: return "Clock";
        case 0x005: return "Display";
        case 0x006: return "Remote";
        case 0x008: return "Tag";
        case 0x00A: return "Media Player";
        case 0x00C: return "Thermometer";
        case 0x00D: return "Heart Rate";
        case 0x00F: return "HID";
        case 0x021: return "Audio";
        default:    return "";
    }
}

// ── 16비트 서비스 UUID → 기기/서비스 힌트 (Assigned Numbers; 소비자기기 식별용 발췌) ──
// AD 0x02/0x03(16-bit UUID 목록), 0x16(서비스데이터 16-bit) 에서 추출한 UUID 매핑.
inline const char* btle_uuid_hint(uint16_t u){
    switch(u){
        case 0x1812: return "HID";          // Human Interface Device (키보드/마우스/펜)
        case 0x1108: return "Headset";
        case 0x110B: return "Audio";        // A2DP sink
        case 0x180D: return "Heart Rate";
        case 0x180F: return "Battery";
        case 0x1816: return "Cycling CSC";
        case 0x1818: return "Cycling Power";
        case 0x1814: return "Running Speed";
        case 0x1809: return "Thermometer";
        case 0x1808: return "Glucose";
        case 0x1822: return "Pulse Oximeter";
        case 0x181D: return "Weight Scale";
        case 0x1826: return "Fitness";      // Fitness Machine
        case 0xFD5A: return "Samsung";      // Samsung service data
        case 0xFD6F: return "ExposureNotif";// COVID EN (Apple/Google)
        case 0xFE2C: return "Fast Pair";    // Google Fast Pair
        case 0xFEAA: return "Eddystone";    // Google beacon
        case 0xFE07: return "Sonos";
        case 0xFE03: return "Amazon";
        case 0xFE9F: return "Google";
        case 0xFD44: return "Apple";        // Apple service data
        case 0xFDF0: return "Google";
        case 0xFEED: return "Tile";
        case 0xFE33: return "Chipolo";
        case 0xFE25: return "Apple";
        case 0xFE50: return "Google";
        default:     return "";
    }
}

// ── Apple Continuity ProxPair(0x07) 모델코드 → 기기명 (공개 역공학; AirPods/Beats/Pencil) ──
// 페이로드 byte0=서브타입(0x01 가변), byte1~2 = 모델코드(BE). 안정적 코드만 라벨.
inline const char* btle_apple_model(uint16_t code){
    switch(code){
        case 0x0220: return "AirPods";
        case 0x0F20: return "AirPods Pro";
        case 0x1320: return "AirPods 3";
        case 0x1420: return "AirPods Pro 2";
        case 0x2420: return "AirPods Pro 2 USB-C";
        case 0x0A20: return "AirPods Max";
        case 0x0E20: return "PowerBeats Pro";
        case 0x0320: return "PowerBeats 3";
        case 0x0520: return "BeatsX";
        case 0x0620: return "Beats Solo3";
        case 0x0920: return "Beats Studio3";
        case 0x1120: return "PowerBeats 4";
        case 0x1020: return "Beats Flex";
        case 0x1720: return "Beats Studio Buds";
        case 0x1B20: return "Beats Fit Pro";
        case 0x1E20: return "Beats Studio Buds+";
        default:     return "";
    }
}
