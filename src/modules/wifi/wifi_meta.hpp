#pragma once
// ── WiFi 비콘 레코드 + wire 포맷 ──────────────────────────────────────────
// M1: 진단 레코드(ssid 비어있음 → capture 상태). M2+: beacon IE 채움.
#include <cstdint>
#include <cstring>

struct WifiRecord {
    int64_t t_ms = 0;
    float   freq = 0.f;     // 채널 중심 MHz (host 가 스탬프)
    int     ch   = 0;       // 채널필터 인덱스
    // ── beacon (M2+) ──
    char    bssid[20] = {};   // "aa:bb:cc:dd:ee:ff"
    char    ssid[33]  = {};   // SSID (빈 문자열 = 진단/M1)
    int     wch       = 0;    // DS Param 채널번호
    int     rssi      = 0;    // 상대 전력 (dBFS 근사)
    char    phy[8]    = {};   // "11b/g/n/ac/ax"
    char    sec[12]   = {};   // "Open/WPA2/WPA3"
    int     beacon_ms = 0;    // beacon interval
    // ── M1 진단 (ssid 비었을 때 표시) ──
    float    peak_dbfs   = 0.f;
    int      burst_count = 0;
    uint32_t out_sr      = 0;  // DDC 출력 샘플레이트
};

// wire 포맷 (framework BEWE_MK_DATA payload; station 은 MpData 봉투가 운반)
struct __attribute__((packed)) WifiWireMsg {
    int64_t  t_ms; float freq; int32_t ch;
    char     bssid[20]; char ssid[33]; int32_t wch; int32_t rssi;
    char     phy[8]; char sec[12]; int32_t beacon_ms;
    float    peak_dbfs; int32_t burst_count; uint32_t out_sr;
};

inline void wifi_msg_to_wire(const WifiRecord& m, WifiWireMsg& w){
    memset(&w, 0, sizeof(w));
    w.t_ms=m.t_ms; w.freq=m.freq; w.ch=m.ch;
    memcpy(w.bssid,m.bssid,sizeof(w.bssid)); memcpy(w.ssid,m.ssid,sizeof(w.ssid));
    w.wch=m.wch; w.rssi=m.rssi;
    memcpy(w.phy,m.phy,sizeof(w.phy)); memcpy(w.sec,m.sec,sizeof(w.sec));
    w.beacon_ms=m.beacon_ms; w.peak_dbfs=m.peak_dbfs;
    w.burst_count=m.burst_count; w.out_sr=m.out_sr;
}
inline void wifi_wire_to_msg(const WifiWireMsg& w, WifiRecord& m){
    m = WifiRecord{};
    m.t_ms=w.t_ms; m.freq=w.freq; m.ch=w.ch;
    memcpy(m.bssid,w.bssid,sizeof(m.bssid)); m.bssid[sizeof(m.bssid)-1]=0;
    memcpy(m.ssid,w.ssid,sizeof(m.ssid));    m.ssid[sizeof(m.ssid)-1]=0;
    m.wch=w.wch; m.rssi=w.rssi;
    memcpy(m.phy,w.phy,sizeof(m.phy)); m.phy[sizeof(m.phy)-1]=0;
    memcpy(m.sec,w.sec,sizeof(m.sec)); m.sec[sizeof(m.sec)-1]=0;
    m.beacon_ms=w.beacon_ms; m.peak_dbfs=w.peak_dbfs;
    m.burst_count=w.burst_count; m.out_sr=w.out_sr;
}
