#pragma once
// Long-Waterfall (host-side): post-FFT magnitude → 1 byte/bin → disk file,
// rotated on SR/CF/fft_size/IQ-rolling on-off changes. CLI/GUI 공용.
//
// Worker thread polls FFTViewer::fft_data + total_ffts, max-hold compresses
// capture rows down to ~5 row/sec, quantizes float dB to uint8 (db_min..db_max
// → 0..255), appends to .bewewf file under ~/BE_WE/recordings/long_waterfall/.
//
// File format (62B header rounded to 64, then raw rows):
//   "BWWF"(4) ver(2) fft_size(4) sample_rate(8) center_freq(8)
//   row_rate_hz(4 float) db_min(4 float) db_max(4 float) start_utc(8) reserved(16)
// Each row = fft_size bytes.

#include <cstdint>
#include <string>

#include "net_protocol.hpp"

class FFTViewer;

namespace LongWaterfall {

// Disk file header (matches plan §"파일 포맷"; total 64 B padded).
#pragma pack(push, 1)
struct FileHeader {
    char     magic[4];          // "BWWF"
    uint16_t version;           // 0x0001
    uint32_t fft_size;
    uint64_t sample_rate_hz;
    uint64_t center_freq_hz;
    float    row_rate_hz;       // rows per second (target, e.g. 5.0)
    float    db_min;            // dB → byte 0
    float    db_max;            // dB → byte 255
    uint64_t start_utc_unix;    // file creation, UTC seconds
    float    station_lon;       // legacy v1; unreliable due to login coord bug — readers ignore.
    uint32_t fft_input_size;    // user-set FFT size (= fft_size / FFT_PAD_FACTOR); 0 if unknown
    int32_t  utc_offset_hours;  // v2+: host system TZ at file open (tm_gmtoff/3600). v1 files = 0 → fallback to viewer TZ
    uint8_t  reserved[6];       // pad to 64
};
#pragma pack(pop)
static_assert(sizeof(FileHeader) == 64, "LongWaterfall::FileHeader must be 64");

constexpr float DEFAULT_ROW_RATE_HZ = 5.0f;
constexpr float DEFAULT_DB_MIN = -120.0f;
constexpr float DEFAULT_DB_MAX = 0.0f;

// Start the host-side worker. Idempotent: subsequent calls are no-op.
// Worker terminates only on stop_worker(). Records only when v.tm_iq_on=true.
void start_worker(FFTViewer* v);
void stop_worker();

// Trigger a file rotation on the next worker iteration.
// Called from capture/IO or UI on SR/CF/fft_size/IQ on-off events.
void request_rotate();

// Currently-open file path (empty if worker idle / not recording). Thread-safe snapshot.
std::string current_file_path();

// Scan ~/BE_WE/recordings/long_waterfall/ → fill PktLwfList from on-disk headers.
// Skips files that are not valid .bewewf (header missing / wrong magic).
void scan_dir_into_list(::PktLwfList& out);

// Public format constants (so view code can quantize/dequantize identically).
inline uint8_t db_to_byte(float db, float dmin, float dmax){
    if(db <= dmin) return 0;
    if(db >= dmax) return 255;
    float t = (db - dmin) / (dmax - dmin);
    int v = (int)(t * 255.0f + 0.5f);
    if(v < 0) v = 0; else if(v > 255) v = 255;
    return (uint8_t)v;
}
inline float byte_to_db(uint8_t b, float dmin, float dmax){
    return dmin + (dmax - dmin) * (b / 255.0f);
}

} // namespace LongWaterfall

// ── GUI viewer (defined in long_waterfall_view.cpp; not built in headless) ──
class NetClient;
namespace LongWaterfallView {
    void draw_modal(FFTViewer& v, NetClient* cli);  // call once per frame when v.lwf_modal_open
    void close_modal();                             // GL cleanup on shutdown
}
