#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <atomic>
#include <cmath>
#include <complex>
#include <algorithm>

// ── Modulation types ──────────────────────────────────────────────────────
enum class ModType : uint8_t {
    UNKNOWN = 0,
    AM, FM, DSB, SSB, CW,
    OOK, ASK2, ASK4,
    BPSK, QPSK, PSK8, QAM16,
    FSK2, FSK4, GFSK, MSK,
    MOD_TYPE_COUNT
};

inline const char* mod_type_name(ModType m) {
    static const char* names[] = {
        "???", "AM", "FM", "DSB", "SSB", "CW",
        "OOK", "2ASK", "4ASK",
        "BPSK", "QPSK", "8PSK", "16QAM",
        "2FSK", "4FSK", "GFSK", "MSK"
    };
    int i = (int)m;
    return (i >= 0 && i < (int)ModType::MOD_TYPE_COUNT) ? names[i] : "???";
}

inline bool mod_is_digital(ModType m) {
    return (int)m >= (int)ModType::OOK;
}

// ── AMC analysis result ──────────────────────────────────────────────────
struct AmcResult {
    ModType mod         = ModType::UNKNOWN;
    float   confidence  = 0.0f;
    float   baud_rate   = 0.0f;
    float   snr_est     = 0.0f;
    bool    is_digital  = false;
};

// ── Cumulant result ──────────────────────────────────────────────────────
struct CumulantResult {
    std::complex<float> C20, C40;
    float abs_C40 = 0, abs_C42 = 0;
    float C63_real = 0;
    float M42 = 0;
};

CumulantResult compute_cumulants(const float* I, const float* Q, int N);

// ── AMC feature set ──────────────────────────────────────────────────────
struct AmcFeatures {
    float env_mean, env_std, env_cv;
    float env_gamma;           // peak/mean envelope ratio
    float freq_std;            // inst. freq std dev
    int   freq_n_levels;       // number of discrete freq levels
    float spec_asymmetry;      // upper/lower power ratio
    CumulantResult cum;
};

AmcFeatures extract_features(const float* I, const float* Q, int N, float work_sr);
AmcResult   classify_modulation(const float* I, const float* Q, int N, float work_sr);
float       estimate_baud_rate(const float* I, const float* Q, int N,
                               float work_sr, ModType mod_hint);

// ── Signal database entry ────────────────────────────────────────────────
struct SigProfile {
    const char*  name;
    const char*  description;
    float  freq_mhz_min, freq_mhz_max;
    ModType mod_types[4]; int mod_count;
    float  baud_rates[4]; int baud_count; float baud_tolerance;
    uint64_t sync_word; int sync_bits;
    float  bw_khz;
    int    decoder_id;   // -1=generic hex, 0=AIS(skip), 1=POCSAG, 2=ACARS
};

struct SigMatch {
    int   db_idx;
    float score;
};

extern const SigProfile SIG_DB[];
extern const int SIG_DB_COUNT;

std::vector<SigMatch> match_signal(float center_freq_mhz, ModType detected_mod,
                                   float detected_baud, float channel_bw_khz);

// ── Auto-ID worker state ────────────────────────────────────────────────
enum class AutoIdState : int {
    COLLECTING  = 0,
    ANALYZING   = 1,
    MATCHING    = 2,
    DECODING    = 3,
    IDLE        = 4,
};

// ── Per-channel auto-ID result (UI thread readable) ─────────────────────
struct AutoIdResult {
    std::atomic<int>   state{0};
    std::atomic<int>   mod_type{0};
    std::atomic<float> baud_rate{0.0f};
    std::atomic<float> confidence{0.0f};
    char protocol_name[32] = {};
    std::atomic<float> snr_est{0.0f};
};

// ── Standard baud rate snapping ─────────────────────────────────────────
inline float snap_baud(float estimated, float tolerance = 0.08f) {
    static const float STD_BAUDS[] = {
        50, 75, 100, 150, 200, 300, 400, 512, 600, 1200, 1800, 2400,
        4800, 6400, 8000, 9600, 16000, 18000, 19200, 38400, 57600
    };
    float best = estimated;
    float best_err = 1.0f;
    for (float sb : STD_BAUDS) {
        float err = fabsf(estimated - sb) / sb;
        if (err < tolerance && err < best_err) {
            best = sb;
            best_err = err;
        }
    }
    return best;
}
