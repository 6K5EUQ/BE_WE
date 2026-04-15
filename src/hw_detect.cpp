#include "fft_viewer.hpp"
#include <libbladeRF.h>
#include <rtl-sdr.h>
#include <iio.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>

// 전역 강제 선택자 (main/login에서 설정). 빈 문자열 = 자동 감지.
// "bladerf" / "rtlsdr" / "pluto" 중 하나.
std::string g_sdr_force;

// UI용: 이 PC에 붙어 있는 SDR 목록 반환 ("bladerf","pluto","rtlsdr")
std::vector<std::string> scan_available_sdrs(){
    std::vector<std::string> out;
    struct bladerf_devinfo* blade_list = nullptr;
    int n_blade = bladerf_get_device_list(&blade_list);
    if(n_blade > 0) out.push_back("bladerf");
    if(blade_list) bladerf_free_device_list(blade_list);

    // Pluto
    {
        struct iio_scan_context* sc = iio_create_scan_context(nullptr, 0);
        bool found = false;
        if(sc){
            struct iio_context_info** info = nullptr;
            ssize_t n = iio_scan_context_get_info_list(sc, &info);
            for(ssize_t i = 0; i < n; i++){
                const char* desc = iio_context_info_get_description(info[i]);
                if(desc && (strstr(desc, "PlutoSDR") || strstr(desc, "ADALM"))){ found = true; break; }
            }
            if(info) iio_context_info_list_free(info);
            iio_scan_context_destroy(sc);
        }
        if(!found){
            struct iio_context* c = iio_create_context_from_uri("usb:");
            if(c){ iio_context_destroy(c); found = true; }
        }
        if(found) out.push_back("pluto");
    }

    if(rtlsdr_get_device_count() > 0) out.push_back("rtlsdr");
    return out;
}

static bool detect_pluto(){
    struct iio_scan_context* sc = iio_create_scan_context(nullptr, 0);
    if(!sc) return false;
    struct iio_context_info** info = nullptr;
    ssize_t n = iio_scan_context_get_info_list(sc, &info);
    bool found = false;
    for(ssize_t i = 0; i < n; i++){
        const char* desc = iio_context_info_get_description(info[i]);
        if(desc && (strstr(desc, "PlutoSDR") || strstr(desc, "ADALM"))){
            found = true; break;
        }
    }
    if(info) iio_context_info_list_free(info);
    iio_scan_context_destroy(sc);
    // 스캔이 못 잡는 경우 대비: USB/네트워크 컨텍스트 시도
    if(!found){
        struct iio_context* c = iio_create_context_from_uri("usb:");
        if(c){ iio_context_destroy(c); found = true; }
    }
    return found;
}

// ── HW 자동 감지 후 초기화 ────────────────────────────────────────────────
// g_sdr_force가 설정되면 그 장치만 시도. 아니면 우선순위: Pluto > BladeRF > RTL-SDR
bool FFTViewer::initialize(float cf_mhz, float sr_msps){
    // BladeRF 감지
    struct bladerf_devinfo* blade_list = nullptr;
    int n_blade = bladerf_get_device_list(&blade_list);
    bool has_blade = (n_blade > 0);
    if(blade_list) bladerf_free_device_list(blade_list);

    // RTL-SDR 감지
    uint32_t n_rtl = rtlsdr_get_device_count();
    bool has_rtl = (n_rtl > 0);

    // Pluto 감지
    bool has_pluto = detect_pluto();

    // 강제 선택자 처리
    if(g_sdr_force == "pluto"){
        if(!has_pluto){ fprintf(stderr,"SDR: Pluto 지정되었으나 감지 안 됨\n"); return false; }
        bewe_log_push(0,"HW: ADALM-Pluto (강제 선택)\n");
        return initialize_pluto(cf_mhz, sr_msps > 0 ? sr_msps : 3.2f);
    }
    if(g_sdr_force == "bladerf"){
        if(!has_blade){ fprintf(stderr,"SDR: BladeRF 지정되었으나 감지 안 됨\n"); return false; }
        bewe_log_push(0,"HW: BladeRF (강제 선택)\n");
        return initialize_bladerf(cf_mhz, sr_msps > 0 ? sr_msps : 61.44f);
    }
    if(g_sdr_force == "rtlsdr"){
        if(!has_rtl){ fprintf(stderr,"SDR: RTL-SDR 지정되었으나 감지 안 됨\n"); return false; }
        bewe_log_push(0,"HW: RTL-SDR (강제 선택)\n");
        return initialize_rtlsdr(cf_mhz);
    }

    if(!has_blade && !has_rtl && !has_pluto){
        fprintf(stderr,"No SDR device found (BladeRF/RTL-SDR/Pluto)\n");
        return false;
    }

    // 우선순위: BladeRF > Pluto > RTL-SDR (기존 BladeRF 우선 유지)
    if(has_blade){
        bewe_log_push(0,"HW: BladeRF detected%s%s\n",
            has_pluto ? " (Pluto also present)" : "",
            has_rtl   ? " (RTL-SDR also present)" : "");
        return initialize_bladerf(cf_mhz, sr_msps > 0 ? sr_msps : 61.44f);
    }
    if(has_pluto){
        bewe_log_push(0,"HW: ADALM-Pluto detected%s\n",
            has_rtl ? " (RTL-SDR also present)" : "");
        return initialize_pluto(cf_mhz, sr_msps > 0 ? sr_msps : 3.2f);
    }
    bewe_log_push(0,"HW: RTL-SDR detected\n");
    return initialize_rtlsdr(cf_mhz);
}

// ── 게인 설정 (공통) ──────────────────────────────────────────────────────
void FFTViewer::set_gain(float db){
    if(db < hw.gain_min) db = hw.gain_min;
    if(db > hw.gain_max) db = hw.gain_max;

    if(hw.type == HWType::BLADERF){
        int gain_int = (int)(db + 0.5f);
        bladerf_set_gain(dev_blade, BLADERF_CHANNEL_RX(0), gain_int);
    } else if(hw.type == HWType::RTLSDR){
        int snapped = HWConfig::rtl_snap_gain(db);
        rtlsdr_set_tuner_gain(dev_rtl, snapped);
    } else if(hw.type == HWType::PLUTO){
        auto* phy = (struct iio_device*)pluto_phy_dev;
        if(!phy) return;
        struct iio_channel* v0 = iio_device_find_channel(phy, "voltage0", false);
        if(!v0) return;
        iio_channel_attr_write(v0, "gain_control_mode", "manual");
        iio_channel_attr_write_longlong(v0, "hardwaregain", (long long)(db + 0.5f));
    }
}
