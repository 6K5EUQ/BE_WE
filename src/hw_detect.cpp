#include "fft_viewer.hpp"
#include <libbladeRF.h>
#include <rtl-sdr.h>
#include <cstdio>

// ── HW 자동 감지 후 초기화 ────────────────────────────────────────────────
// 우선순위: BladeRF > RTL-SDR
// 하나만 연결 시 자동 선택, 둘 다 있으면 BladeRF
bool FFTViewer::initialize(float cf_mhz){
    // BladeRF 감지
    struct bladerf_devinfo* blade_list = nullptr;
    int n_blade = bladerf_get_device_list(&blade_list);
    bool has_blade = (n_blade > 0);
    if(blade_list) bladerf_free_device_list(blade_list);

    // RTL-SDR 감지
    uint32_t n_rtl = rtlsdr_get_device_count();
    bool has_rtl = (n_rtl > 0);

    if(!has_blade && !has_rtl){
        fprintf(stderr,"No SDR device found (BladeRF or RTL-SDR)\n");
        return false;
    }

    if(has_blade){
        printf("HW: BladeRF detected%s\n", has_rtl ? " (RTL-SDR also present, using BladeRF)" : "");
        return initialize_bladerf(cf_mhz, 61.44f);
    }

    printf("HW: RTL-SDR detected (no BladeRF)\n");
    return initialize_rtlsdr(cf_mhz);
}

// ── 게인 설정 (공통) ──────────────────────────────────────────────────────
void FFTViewer::set_gain(float db){
    // 범위 클램프
    if(db < hw.gain_min) db = hw.gain_min;
    if(db > hw.gain_max) db = hw.gain_max;

    if(hw.type == HWType::BLADERF){
        int gain_int = (int)(db + 0.5f);
        bladerf_set_gain(dev_blade, BLADERF_CHANNEL_RX(0), gain_int);
    } else if(hw.type == HWType::RTLSDR){
        int snapped = HWConfig::rtl_snap_gain(db);
        rtlsdr_set_tuner_gain(dev_rtl, snapped);
    }
}