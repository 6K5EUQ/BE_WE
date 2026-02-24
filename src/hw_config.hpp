#pragma once
#include <cstdint>

// ── 하드웨어 타입 ──────────────────────────────────────────────────────────
enum class HWType { NONE, BLADERF, RTLSDR };

// ── 런타임 HW 파라미터 (초기화 시 채워짐) ────────────────────────────────
struct HWConfig {
    HWType   type            = HWType::NONE;

    // 샘플링
    uint32_t sample_rate     = 0;       // Hz (실제 설정값)
    float    sample_rate_mhz = 0.0f;    // sample_rate / 1e6

    // 주파수 범위
    double   freq_min_hz     = 0.0;
    double   freq_max_hz     = 0.0;

    // IQ 스케일: BladeRF SC16_Q11 = 2048.0, RTL-SDR uint8 offset = 128
    float    iq_scale        = 2048.0f; // int16→float 정규화
    float    iq_offset       = 0.0f;    // uint8 중심값 (RTL=127.5, BladeRF=0)

    // 유효 대역폭 비율 (SDR 롤오프 고려)
    float    eff_bw_ratio    = 0.875f;  // 87.5% 공통

    // 표시용 이름
    const char* name         = "Unknown";

    // ── 파생 계산값 (init 후 채워짐) ──────────────────────────────────────
    // 워터폴 행 속도를 HW에 관계없이 동일하게 유지 (37.5 rows/sec 기준)
    static constexpr float TARGET_ROWS_PER_SEC = 37.5f;

    float eff_bw_mhz() const { return sample_rate_mhz * eff_bw_ratio; }
    float nyq_mhz()    const { return sample_rate_mhz * 0.5f; }

    // fft_size에 맞는 time_average 자동 계산
    int compute_time_average(int fft_sz) const {
        int ta = (int)((float)sample_rate / (float)fft_sz / TARGET_ROWS_PER_SEC);
        return ta < 1 ? 1 : ta;
    }
};

// BladeRF 기본값
inline HWConfig make_bladerf_config(uint32_t actual_sr){
    HWConfig c;
    c.type            = HWType::BLADERF;
    c.sample_rate     = actual_sr;
    c.sample_rate_mhz = actual_sr / 1e6f;
    c.freq_min_hz     = 47e6;
    c.freq_max_hz     = 6000e6;
    c.iq_scale        = 2048.0f;
    c.iq_offset       = 0.0f;
    c.eff_bw_ratio    = 0.875f;
    c.name            = "BladeRF";
    return c;
}

// RTL-SDR 기본값 (2.56 MSPS)
inline HWConfig make_rtlsdr_config(uint32_t actual_sr){
    HWConfig c;
    c.type            = HWType::RTLSDR;
    c.sample_rate     = actual_sr;
    c.sample_rate_mhz = actual_sr / 1e6f;
    c.freq_min_hz     = 500e3;
    c.freq_max_hz     = 1766e6;
    c.iq_scale        = 127.5f;   // uint8 → float: (x-127.5)/127.5
    c.iq_offset       = 127.5f;
    c.eff_bw_ratio    = 0.875f;
    c.name            = "RTL-SDR";
    return c;
}