#pragma once
// ── WiFi 802.11 비콘 디코더 (스트리밍) ────────────────────────────────────
// M1: 에너지/버스트 측정만 (census 준비). M2+: OFDM/DSSS 비콘 수신기.
// 입력 = DDC 후 채널 baseband 복소 샘플 (~20 MSPS). 출력 = on_record 콜백.
#include "wifi_meta.hpp"
#include <functional>
#include <cmath>

namespace wifi_mod {

struct WifiDecoder {
    std::function<void(const WifiRecord&)> on_record;

    // 측정 상태
    double pwr_acc = 0.0;     // 평균 전력 누적
    uint64_t pwr_n = 0;
    float    peak  = 0.f;
    float    fast  = 0.f, slow = 1e-9f;
    bool     in_burst = false;
    int      bursts = 0;
    uint32_t out_sr = 0;

    void reset(uint32_t osr){
        pwr_acc=0; pwr_n=0; peak=0; fast=0; slow=1e-9f; in_burst=false; bursts=0; out_sr=osr;
    }
    // 채널 baseband 복소 샘플 1개 (정규화됨, |x|~1 풀스케일)
    inline void feed(float i, float q){
        float p = i*i + q*q;
        pwr_acc += p; pwr_n++;
        if(p > peak) peak = p;
        // 빠른/느린 envelope → 버스트(에너지 상승) 카운트
        fast += 0.05f*(p - fast);
        slow += 0.0005f*(p - slow);
        bool hi = fast > slow*4.0f + 1e-9f;
        if(hi && !in_burst){ in_burst=true; bursts++; }
        else if(!hi && in_burst){ in_burst=false; }
    }
    // 진단 레코드 1개 스냅샷 + 누적 리셋 (호출자가 t_ms/freq/ch 채움)
    WifiRecord snapshot(){
        WifiRecord m{};
        double mean = pwr_n ? pwr_acc/(double)pwr_n : 0.0;
        m.peak_dbfs   = peak>0 ? 10.f*log10f(peak) : -120.f;
        m.burst_count = bursts;
        m.out_sr      = out_sr;
        (void)mean;
        pwr_acc=0; pwr_n=0; peak=0; bursts=0;
        return m;
    }
};

} // namespace wifi_mod
