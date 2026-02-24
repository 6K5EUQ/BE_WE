#pragma once
#include <cstdint>
#include <imgui.h>

// ── Hardware ───────────────────────────────────────────────────────────────
#define RX_GAIN                10
#define CHANNEL                BLADERF_CHANNEL_RX(0)

// ── FFT / Display ─────────────────────────────────────────────────────────
#define DEFAULT_FFT_SIZE       8192
#define TIME_AVERAGE           200
#define MAX_FFTS_MEMORY        2500   // ~1분 (37.5행/초 × 60초 + 여유)
#define HANN_WINDOW_CORRECTION 2.67f
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
#define MAX_CHANNELS           5

// ── Per-channel colors ────────────────────────────────────────────────────
static const ImU32 CH_BORD[MAX_CHANNELS] = {
    IM_COL32(255,220, 50,220), IM_COL32( 50,200,255,220),
    IM_COL32(255, 90, 50,220), IM_COL32(180, 60,255,220),
    IM_COL32( 50,255,110,220)
};
static const ImU32 CH_FILL[MAX_CHANNELS] = {
    IM_COL32(255,220, 50, 30), IM_COL32( 50,200,255, 30),
    IM_COL32(255, 90, 50, 30), IM_COL32(180, 60,255, 30),
    IM_COL32( 50,255,110, 30)
};
static const ImU32 CH_SFIL[MAX_CHANNELS] = {
    IM_COL32(255,220, 50, 75), IM_COL32( 50,200,255, 75),
    IM_COL32(255, 90, 50, 75), IM_COL32(180, 60,255, 75),
    IM_COL32( 50,255,110, 75)
};

// ── FFT File Header ───────────────────────────────────────────────────────
struct FFTHeader {
    char     magic[4];
    uint32_t version, fft_size, sample_rate;
    uint64_t center_frequency;
    uint32_t num_ffts, time_average;
    float    power_min, power_max, reserved[8];
};