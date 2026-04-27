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
#include <cstdio>
#include <ctime>
#include <string>
#include <functional>

#include "net_protocol.hpp"

class FFTViewer;

namespace LongWaterfall {

// Disk file header — v3 layout (128 B). v2(64B) and earlier rejected by reader.
#pragma pack(push, 1)
struct FileHeader {
    char     magic[4];          // "BWWF"
    uint16_t version;           // 0x0003
    uint32_t fft_size;
    uint64_t sample_rate_hz;
    uint64_t center_freq_hz;
    float    row_rate_hz;       // rows per second (target, e.g. 5.0)
    float    db_min;            // dB → byte 0
    float    db_max;            // dB → byte 255
    uint64_t start_utc_unix;    // file creation, UTC seconds
    float    station_lon;       // signed degrees (v3: filled from FFTViewer.station_lon)
    uint32_t fft_input_size;    // user-set FFT size (= fft_size / FFT_PAD_FACTOR); 0 if unknown
    int32_t  utc_offset_hours;  // host system TZ at file open (tm_gmtoff/3600)
    uint8_t  reserved_v2[6];    // pad to 64 (v2 layout end)
    // ── v3 extension ────
    char     station_name[32];  // null-terminated; "" if unknown
    float    station_lat;       // signed degrees
    uint8_t  reserved_v3[28];   // pad to 128
};
#pragma pack(pop)
static_assert(sizeof(FileHeader) == 128, "LongWaterfall::FileHeader v3 must be 128");

constexpr uint16_t FILE_VERSION = 0x0003;

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

// ── Live broadcast hooks ───────────────────────────────────────────────
// Set by host wiring (cli_host / ui). Worker calls these inside open/flush/close
// so NetServer can fan out LIVE_START / LIVE_ROW / LIVE_STOP to all JOINs.
// Callbacks must be cheap (queue-only); worker thread invokes them directly.
struct LiveCallbacks {
    std::function<void(const ::PktLwfLiveStart&)> on_start;
    std::function<void(const ::PktLwfLiveRowHdr& hdr,
                       const uint8_t* row, uint32_t row_bytes)> on_row;
    std::function<void(const ::PktLwfLiveStop&)>  on_stop;
};
void set_live_callbacks(const LiveCallbacks& cbs);

// Build PktLwfLiveStart from currently open LIVE file header (for new JOIN).
// Returns false if no file is currently open.
bool snapshot_live_start(::PktLwfLiveStart& out);

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

// ── Mission-code filename helpers (host + JOIN 공용) ──────────────────────
// 양식: <MissCode><DD>_<Mon><DD>.<YYYY>_<F.F>MHz_<HHMM>-LIVE.bewehist
// 종료 시 -LIVE → -<HHMM>Z 로 rename.
// MissCode: A=Jan, B=Feb, ..., L=Dec (alphabet, 'I' 포함).
inline char mission_letter(int mon0_11){ return (char)('A' + mon0_11); }
inline const char* month_abbr3(int mon0_11){
    static const char* m[] = {"Jan","Feb","Mar","Apr","May","Jun",
                               "Jul","Aug","Sep","Oct","Nov","Dec"};
    return m[mon0_11];
}
inline std::string build_hist_filename_live(uint64_t start_utc, uint64_t cf_hz){
    time_t st = (time_t)start_utc;
    struct tm tm_utc; gmtime_r(&st, &tm_utc);
    double cf_mhz = (double)cf_hz / 1e6;
    char buf[96];
    snprintf(buf, sizeof(buf),
        "%c%02d_%s%02d.%04d_%.1fMHz_%02d%02d-LIVE.bewehist",
        mission_letter(tm_utc.tm_mon), tm_utc.tm_mday,
        month_abbr3(tm_utc.tm_mon), tm_utc.tm_mday, 1900 + tm_utc.tm_year,
        cf_mhz, tm_utc.tm_hour, tm_utc.tm_min);
    return buf;
}
// Returns finalized basename when given a "...-LIVE.bewehist" basename + end_utc.
// If input doesn't match, returns input unchanged.
inline std::string build_hist_filename_finalize(const std::string& live_name,
                                                 uint64_t end_utc){
    auto pos = live_name.rfind("-LIVE.bewehist");
    if(pos == std::string::npos) return live_name;
    time_t et = (time_t)end_utc;
    struct tm tm_utc; gmtime_r(&et, &tm_utc);
    char tail[24];
    snprintf(tail, sizeof(tail), "-%02d%02dZ.bewehist",
             tm_utc.tm_hour, tm_utc.tm_min);
    return live_name.substr(0, pos) + tail;
}

} // namespace LongWaterfall

// ── GUI viewer (defined in long_waterfall_view.cpp; not built in headless) ──
class NetClient;
namespace LongWaterfallView {
    void draw_modal(FFTViewer& v, NetClient* cli);  // call once per frame when v.lwf_modal_open
    void close_modal();                             // GL cleanup on shutdown
}
