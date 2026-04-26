#pragma once
#include <cstdint>

// ── BEWE version (창 제목 및 About 표시용) ──────────────────────────────────
// SemVer: vMAJOR.MINOR.PATCH — 자세한 정책은 CLAUDE.md 참조
#define BEWE_VERSION "v1.5.4"

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
#define DEFAULT_FFT_SIZE       4096
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
#define MAX_CHANNELS           10

// ── Per-channel colors ────────────────────────────────────────────────────
static const ImU32 CH_BORD[MAX_CHANNELS] = {
    IM_COL32(255,220, 50,220), IM_COL32( 50,200,255,220),
    IM_COL32(255, 90, 50,220), IM_COL32(180, 60,255,220),
    IM_COL32( 50,255,110,220), IM_COL32(255,120,200,220),
    IM_COL32(255,200,  0,220), IM_COL32(  0,230,200,220),
    IM_COL32(200,100, 50,220), IM_COL32(100,180,255,220)
};
static const ImU32 CH_FILL[MAX_CHANNELS] = {
    IM_COL32(255,220, 50, 30), IM_COL32( 50,200,255, 30),
    IM_COL32(255, 90, 50, 30), IM_COL32(180, 60,255, 30),
    IM_COL32( 50,255,110, 30), IM_COL32(255,120,200, 30),
    IM_COL32(255,200,  0, 30), IM_COL32(  0,230,200, 30),
    IM_COL32(200,100, 50, 30), IM_COL32(100,180,255, 30)
};
static const ImU32 CH_SFIL[MAX_CHANNELS] = {
    IM_COL32(255,220, 50, 75), IM_COL32( 50,200,255, 75),
    IM_COL32(255, 90, 50, 75), IM_COL32(180, 60,255, 75),
    IM_COL32( 50,255,110, 75), IM_COL32(255,120,200, 75),
    IM_COL32(255,200,  0, 75), IM_COL32(  0,230,200, 75),
    IM_COL32(200,100, 50, 75), IM_COL32(100,180,255, 75)
};

// ── FFT File Header ───────────────────────────────────────────────────────
struct FFTHeader {
    char     magic[4];
    uint32_t version, fft_size, sample_rate;
    uint64_t center_frequency;
    uint32_t num_ffts, time_average;
    float    power_min, power_max, reserved[8];
};