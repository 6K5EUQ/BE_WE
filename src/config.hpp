#pragma once
#include <cstdint>
#include <array>

// ── BEWE version (창 제목 및 About 표시용) ──────────────────────────────────
// SemVer: vMAJOR.MINOR.PATCH — 자세한 정책은 CLAUDE.md 참조
#define BEWE_VERSION "v10.13.5"

#ifdef BEWE_HEADLESS
  typedef uint32_t ImU32;
  #define IM_COL32(R,G,B,A) \
      (((ImU32)(A)<<24)|((ImU32)(B)<<16)|((ImU32)(G)<<8)|((ImU32)(R)))
#else
  #include <imgui.h>
#endif

// ── Hardware (hw_config.hpp 참조) ─────────────────────────────────────────
// RX_GAIN: BladeRF=10, RTL-SDR=396 (39.6dB, 0.1dB 단위)
#define BLADERF_RX_GAIN        10
#define RTLSDR_RX_GAIN_TENTHS  396   // 39.6 dB

// ── FFT / Display ─────────────────────────────────────────────────────────
#define DEFAULT_FFT_SIZE       8192
#define FFT_PAD_FACTOR         4      // zero-padding 배수 (시각적 해상도 향상)
#define TIME_AVERAGE           200
#define MAX_FFTS_MEMORY        2500   // ~1분
#define HANN_WINDOW_CORRECTION 2.67f
#define NUTTALL_WINDOW_CORRECTION 3.91f  // 1/(a0²+(a1²+a2²+a3²)/2) for Nuttall
#define COLORMAP_LUT_SIZE      65536  // 워터폴 컬러맵 해상도 (was 4096)
#define AXIS_LABEL_WIDTH       50
#define BOTTOM_LABEL_HEIGHT    30
#define TOPBAR_H               32.0f

// ── IQ Ring ───────────────────────────────────────────────────────────────
#define IQ_RING_CAPACITY       (1 << 22)
#define IQ_RING_MASK           (IQ_RING_CAPACITY - 1)

// ── Audio ─────────────────────────────────────────────────────────────────
#define AUDIO_SR               48000u
#define AUDIO_DEVICE           "default"

// ── Channel ───────────────────────────────────────────────────────────────
// Active(가시대역 안) + Holding(밖) 합산 채널 필터 슬롯 상한.
// active/holding 분리 쿼터가 아니라 단일 풀 — 분류는 가시대역 안/밖으로 동적 결정.
#define MAX_CHANNELS           50

// ── Per-channel colors ────────────────────────────────────────────────────
// ch 0~9 는 기존 고정 팔레트 유지, 10~MAX_CHANNELS-1 은 golden-angle HSV 자동 생성.
static const ImU32 CH_BORD_BASE[10] = {
    IM_COL32(255,220, 50,220), IM_COL32( 50,200,255,220),
    IM_COL32(255, 90, 50,220), IM_COL32(180, 60,255,220),
    IM_COL32( 50,255,110,220), IM_COL32(255,120,200,220),
    IM_COL32(255,200,  0,220), IM_COL32(  0,230,200,220),
    IM_COL32(200,100, 50,220), IM_COL32(100,180,255,220)
};
static const ImU32 CH_FILL_BASE[10] = {
    IM_COL32(255,220, 50, 30), IM_COL32( 50,200,255, 30),
    IM_COL32(255, 90, 50, 30), IM_COL32(180, 60,255, 30),
    IM_COL32( 50,255,110, 30), IM_COL32(255,120,200, 30),
    IM_COL32(255,200,  0, 30), IM_COL32(  0,230,200, 30),
    IM_COL32(200,100, 50, 30), IM_COL32(100,180,255, 30)
};
static const ImU32 CH_SFIL_BASE[10] = {
    IM_COL32(255,220, 50, 75), IM_COL32( 50,200,255, 75),
    IM_COL32(255, 90, 50, 75), IM_COL32(180, 60,255, 75),
    IM_COL32( 50,255,110, 75), IM_COL32(255,120,200, 75),
    IM_COL32(255,200,  0, 75), IM_COL32(  0,230,200, 75),
    IM_COL32(200,100, 50, 75), IM_COL32(100,180,255, 75)
};
inline ImU32 ch_hsv(int i, uint8_t a){
    float h = i * 0.61803398875f; h -= (float)(long)h;   // golden-angle frac, [0,1)
    float v = 1.0f, s = (a >= 200 ? 0.85f : 0.80f);      // 테두리는 살짝 더 채도
    float hf = h * 6.0f; int seg = (int)hf; float f = hf - seg;
    float p = v*(1-s), q = v*(1-s*f), t = v*(1-s*(1-f)), r=v,g=v,b=v;
    switch(seg % 6){
        case 0: r=v; g=t; b=p; break; case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break; case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break; default: r=v; g=p; b=q; break;
    }
    return IM_COL32((int)(r*255), (int)(g*255), (int)(b*255), a);
}
inline std::array<ImU32,MAX_CHANNELS> make_ch_palette(const ImU32* base, uint8_t a){
    std::array<ImU32,MAX_CHANNELS> arr{};
    for(int i=0;i<MAX_CHANNELS;i++) arr[i] = (i<10) ? base[i] : ch_hsv(i,a);
    return arr;
}
static const std::array<ImU32,MAX_CHANNELS> CH_BORD = make_ch_palette(CH_BORD_BASE,220);
static const std::array<ImU32,MAX_CHANNELS> CH_FILL = make_ch_palette(CH_FILL_BASE, 30);
static const std::array<ImU32,MAX_CHANNELS> CH_SFIL = make_ch_palette(CH_SFIL_BASE, 75);

// ── FFT File Header ───────────────────────────────────────────────────────
struct FFTHeader {
    char     magic[4];
    uint32_t version, fft_size, sample_rate;
    uint64_t center_frequency;
    uint32_t num_ffts, time_average;
    float    power_min, power_max, reserved[8];
};