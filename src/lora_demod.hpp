#pragma once
#include <cstdint>
#include <vector>
#include <string>

struct LoraDemodResult {
    int      sf       = 0;      // detected spreading factor (7..12)
    uint32_t bw       = 0;      // detected bandwidth (Hz) — 1단계는 125000 고정
    uint8_t  sync     = 0;      // detected sync word (upper 4 bits from sync0, lower from sync1)
    int64_t  align    = 0;      // preamble start offset in input samples
    int      n_symbols = 0;     // number of demodulated data symbols
    float    score    = 0.f;    // preamble peak strength / noise floor
    std::string detail;         // human-readable summary
};

// Auto-detect SF (7..12) and decode raw LoRa CSS symbols from a complex IQ buffer.
// Input: separate I / Q float arrays, n_samples, sr (sample rate in Hz).
// Output: out_bits populated with SF bits per symbol (MSB first). info filled.
// Returns true on successful preamble detection and at least 1 data symbol.
bool lora_demod_auto(const float* I, const float* Q,
                     int64_t n_samples, uint32_t sr,
                     std::vector<uint8_t>& out_bits,
                     LoraDemodResult& info);
