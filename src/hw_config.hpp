#pragma once
#include <cstdint>
#include <cstdlib>  // abs(int)

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

    // ── 게인 범위 ──────────────────────────────────────────────────────────
    float    gain_min        = 0.0f;    // dB
    float    gain_max        = 49.6f;   // dB
    float    gain_default    = 0.0f;    // dB (초기값)

    // RTL-SDR R828D 이산 게인값 (0.1dB 단위 → /10 = dB)
    // librtlsdr에서 0.1dB 단위 정수 배열로 반환
    static constexpr int RTL_GAIN_STEPS = 29;
    static constexpr int RTL_GAINS_TENTHS[RTL_GAIN_STEPS] = {
        0, 9, 14, 27, 37, 77, 87, 125, 144, 157, 166,
        197, 207, 229, 254, 280, 297, 328, 338, 364,
        372, 386, 402, 421, 434, 439, 445, 480, 496
    };

    // 연속 dB 값 → RTL-SDR 가장 가까운 이산값(0.1dB 단위 정수) 반환
    static int rtl_snap_gain(float db){
        int tenths = (int)(db * 10.0f + 0.5f);
        int best = RTL_GAINS_TENTHS[0], best_diff = abs(tenths - best);
        for(int i=1;i<RTL_GAIN_STEPS;i++){
            int d = abs(tenths - RTL_GAINS_TENTHS[i]);
            if(d < best_diff){ best_diff=d; best=RTL_GAINS_TENTHS[i]; }
        }
        return best;
    }
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
    c.gain_min        = 0.0f;
    c.gain_max        = 60.0f;
    c.gain_default    = (float)BLADERF_RX_GAIN;
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
    c.iq_scale        = 127.5f;
    c.iq_offset       = 127.5f;
    c.eff_bw_ratio    = 0.875f;
    c.name            = "RTL-SDR";
    c.gain_min        = 0.0f;
    c.gain_max        = 49.6f;
    c.gain_default    = (float)RTLSDR_RX_GAIN_TENTHS / 10.0f;
    return c;
}