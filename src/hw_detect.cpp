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
