#include "long_waterfall.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "net_protocol.hpp"

#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

namespace LongWaterfall {

namespace {

// ── State (file-scope, single host process) ──────────────────────────────
std::atomic<bool>   g_running{false};
std::atomic<int>    g_rotate_req_seq{0};   // ++ when external code calls request_rotate()
int                 g_rotate_seen_seq = 0; // worker tracks last seen
std::thread         g_thr;
FFTViewer*          g_v = nullptr;

// Live broadcast callbacks (set by host wiring).
LiveCallbacks       g_live_cb;
std::mutex          g_live_cb_mtx;
// Snapshot of LIVE_START header for join-in-progress (cleared on close).
std::mutex          g_live_state_mtx;
PktLwfLiveStart     g_live_state{};
bool                g_live_state_valid = false;
uint32_t            g_live_row_idx = 0;

// Open file state — accessed only from worker thread except path read.
std::mutex          g_path_mtx;
std::string         g_cur_path;            // empty when no file open
FILE*               g_fp = nullptr;
FileHeader          g_hdr_cur{};

// Max-hold accumulator across capture rows since last flush.
std::vector<float>  g_acc_db;              // size = current fft_size
int                 g_acc_count = 0;

// Last seen capture index (absolute, not modulo).
int                 g_last_total_ffts = 0;

// ── Helpers ──────────────────────────────────────────────────────────────

void close_file_locked(){
    // Live broadcast STOP first (so JOIN closes its mirror file).
    PktLwfLiveStop stop_pkt{};
    bool had_state = false;
    {
        std::lock_guard<std::mutex> lk(g_live_state_mtx);
        if(g_live_state_valid){
            memcpy(stop_pkt.filename, g_live_state.filename, sizeof(stop_pkt.filename));
            had_state = true;
        }
        g_live_state_valid = false;
        g_live_row_idx = 0;
    }
    if(had_state){
        LiveCallbacks cb_copy;
        { std::lock_guard<std::mutex> lk(g_live_cb_mtx); cb_copy = g_live_cb; }
        if(cb_copy.on_stop) cb_copy.on_stop(stop_pkt);
    }

    if(g_fp){ fflush(g_fp); fclose(g_fp); g_fp = nullptr; }
    // Finalize: -LIVE → -<HHMM>Z based on close time.
    {
        std::lock_guard<std::mutex> lk(g_path_mtx);
        if(!g_cur_path.empty()){
            auto slash = g_cur_path.find_last_of('/');
            std::string dir  = (slash == std::string::npos) ? "" : g_cur_path.substr(0, slash+1);
            std::string base = (slash == std::string::npos) ? g_cur_path : g_cur_path.substr(slash+1);
            std::string fin  = build_hist_filename_finalize(base, (uint64_t)time(nullptr));
            if(fin != base){
                std::string final_full = dir + fin;
                if(rename(g_cur_path.c_str(), final_full.c_str()) == 0){
                    printf("[LongWaterfall] rotate finalize: %s → %s\n", base.c_str(), fin.c_str());
                } else {
                    fprintf(stderr, "[LongWaterfall] rename failed: %s → %s errno=%d\n",
                            base.c_str(), fin.c_str(), errno);
                }
            }
        }
        g_cur_path.clear();
    }
    g_acc_db.clear();
    g_acc_count = 0;
}

// Returns true if filename ends with .bewehist (new) or .bewewf (legacy).
static bool is_lwf_filename(const char* n){
    const char* dot = strrchr(n, '.');
    if(!dot) return false;
    return strcmp(dot, ".bewehist") == 0 || strcmp(dot, ".bewewf") == 0;
}

bool open_new_file(uint64_t cf_hz, uint64_t sr_hz, uint32_t fft_size,
                   uint32_t fft_input_size,
                   float dmin, float dmax,
                   float station_lon, float station_lat,
                   const char* station_name){
    std::string dir = BEWEPaths::hist_host_dir();
    mkdir(BEWEPaths::recordings_dir().c_str(), 0755);
    mkdir(BEWEPaths::hist_dir().c_str(), 0755);
    mkdir(dir.c_str(), 0755);

    uint64_t now_utc = (uint64_t)time(nullptr);
    std::string fname = build_hist_filename_live(now_utc, cf_hz);
    // 같은 분에 두 번 시작될 가능성 — 충돌 회피 suffix
    std::string full = dir + "/" + fname;
    for(int n=2; access(full.c_str(), F_OK)==0 && n<100; ++n){
        auto pos = fname.rfind("-LIVE.bewehist");
        if(pos == std::string::npos) break;
        fname = fname.substr(0,pos) + "_" + std::to_string(n) + "-LIVE.bewehist";
        full  = dir + "/" + fname;
    }

    FILE* fp = fopen(full.c_str(), "wb");
    if(!fp){
        fprintf(stderr, "[LongWaterfall] open failed: %s errno=%d\n", full.c_str(), errno);
        return false;
    }
    if(!(dmax > dmin)){ dmin = DEFAULT_DB_MIN; dmax = DEFAULT_DB_MAX; }
    FileHeader h{};
    memcpy(h.magic, "BWWF", 4);
    h.version        = FILE_VERSION;     // 0x0003
    h.fft_size       = fft_size;
    h.sample_rate_hz = sr_hz;
    h.center_freq_hz = cf_hz;
    h.row_rate_hz    = DEFAULT_ROW_RATE_HZ;
    h.db_min         = dmin;
    h.db_max         = dmax;
    h.start_utc_unix = now_utc;
    h.station_lon    = station_lon;
    h.fft_input_size = fft_input_size;
    {
        time_t now = (time_t)now_utc;
        struct tm lt; localtime_r(&now, &lt);
        h.utc_offset_hours = (int32_t)(lt.tm_gmtoff / 3600);
    }
    // v3 fields
    h.station_lat = station_lat;
    if(station_name) strncpy(h.station_name, station_name, sizeof(h.station_name)-1);
    if(fwrite(&h, 1, sizeof(h), fp) != sizeof(h)){
        fclose(fp); return false;
    }
    fflush(fp);

    g_fp = fp;
    g_hdr_cur = h;
    g_acc_db.assign(fft_size, -200.0f);  // very-low init for max-hold
    g_acc_count = 0;
    {
        std::lock_guard<std::mutex> lk(g_path_mtx);
        g_cur_path = full;
    }
    printf("[LongWaterfall] new file: %s (fft=%u, %.3fMHz, %uMSPS, dB=[%.1f..%.1f], station='%s')\n",
           full.c_str(), fft_size, cf_hz/1e6, (unsigned)(sr_hz/1000000),
           h.db_min, h.db_max, h.station_name);

    // Build live-start packet + broadcast.
    PktLwfLiveStart ls{};
    strncpy(ls.filename, fname.c_str(), sizeof(ls.filename)-1);
    ls.fft_size        = fft_size;
    ls.fft_input_size  = fft_input_size;
    ls.sample_rate_hz  = sr_hz;
    ls.center_freq_hz  = cf_hz;
    ls.row_rate_hz     = h.row_rate_hz;
    ls.db_min          = h.db_min;
    ls.db_max          = h.db_max;
    ls.start_utc_unix  = h.start_utc_unix;
    ls.station_lon     = h.station_lon;
    ls.utc_offset_hours= h.utc_offset_hours;
    ls.station_lat     = h.station_lat;
    memcpy(ls.station_name, h.station_name, sizeof(ls.station_name));
    {
        std::lock_guard<std::mutex> lk(g_live_state_mtx);
        g_live_state = ls;
        g_live_state_valid = true;
        g_live_row_idx = 0;
    }
    LiveCallbacks cb_copy;
    { std::lock_guard<std::mutex> lk(g_live_cb_mtx); cb_copy = g_live_cb; }
    if(cb_copy.on_start) cb_copy.on_start(ls);

    return true;
}

// Flush accumulated max-hold row to disk as 1 byte/bin. Resets accumulator.
void flush_row_locked(){
    if(!g_fp || g_acc_count == 0 || g_acc_db.empty()) return;
    std::vector<uint8_t> row(g_acc_db.size());
    float dmin = g_hdr_cur.db_min;
    float dmax = g_hdr_cur.db_max;
    for(size_t i=0; i<g_acc_db.size(); i++){
        row[i] = db_to_byte(g_acc_db[i], dmin, dmax);
    }
    fwrite(row.data(), 1, row.size(), g_fp);
    fflush(g_fp);

    // Live broadcast — JOIN's hist/live/<filename> appends this row.
    PktLwfLiveRowHdr rhdr{};
    {
        std::lock_guard<std::mutex> lk(g_live_state_mtx);
        if(g_live_state_valid){
            memcpy(rhdr.filename, g_live_state.filename, sizeof(rhdr.filename));
            rhdr.row_index = g_live_row_idx++;
        }
    }
    if(rhdr.filename[0]){
        LiveCallbacks cb_copy;
        { std::lock_guard<std::mutex> lk(g_live_cb_mtx); cb_copy = g_live_cb; }
        if(cb_copy.on_row) cb_copy.on_row(rhdr, row.data(), (uint32_t)row.size());
    }

    std::fill(g_acc_db.begin(), g_acc_db.end(), -200.0f);
    g_acc_count = 0;
}

// Pull all new capture rows since g_last_total_ffts, max-hold into g_acc_db.
// Returns true if any rows ingested.
bool ingest_new_rows(FFTViewer* v){
    if(!v) return false;

    std::lock_guard<std::mutex> lk(v->data_mtx);
    int now = v->total_ffts;
    if(now <= g_last_total_ffts){ return false; }

    int fft_size = v->fft_size;
    if(fft_size <= 0) return false;
    if((int)g_acc_db.size() != fft_size){
        // size mismatch — safest: flush whatever we had and resize.
        // (Worker should have rotated already on fft_size change; handle defensively.)
        g_acc_db.assign(fft_size, -200.0f);
        g_acc_count = 0;
    }

    int new_rows = now - g_last_total_ffts;
    // Clamp: if we fell behind by > MAX_FFTS_MEMORY, only the last ring window is valid.
    if(new_rows > MAX_FFTS_MEMORY) new_rows = MAX_FFTS_MEMORY;

    // Capture writes rowp at fi=total_ffts, then increments total_ffts (under data_mtx).
    // So after we observe total_ffts==now, valid rows are at fi for abs_idx in [g_last, now-1].
    int start_abs = now - new_rows;
    for(int abs_idx = start_abs; abs_idx < now; abs_idx++){
        int fi = abs_idx % MAX_FFTS_MEMORY;
        const float* rowp = v->fft_data.data() + (size_t)fi * fft_size;
        for(int i=0; i<fft_size; i++){
            float d = rowp[i];
            if(d > g_acc_db[i]) g_acc_db[i] = d;
        }
        g_acc_count++;
    }
    g_last_total_ffts = now;
    return true;
}

// ── Worker loop ──────────────────────────────────────────────────────────
void worker_loop(){
    using clk = std::chrono::steady_clock;
    auto next_flush = clk::now() + std::chrono::milliseconds(200); // 5 Hz default

    while(g_running.load(std::memory_order_relaxed)){
        if(!g_v){ std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }

        // Rotate requested?
        int req = g_rotate_req_seq.load(std::memory_order_relaxed);
        if(req != g_rotate_seen_seq){
            g_rotate_seen_seq = req;
            // Flush any pending row, close current file.
            flush_row_locked();
            close_file_locked();
            // Reset row counter so we only catch fresh rows after rotate.
            // (avoid one-shot stale-data dump into new file)
            { std::lock_guard<std::mutex> lk(g_v->data_mtx);
              g_last_total_ffts = g_v->total_ffts; }
        }

        // Only record while TM IQ is rolling.
        bool tm_on = g_v->tm_iq_on.load(std::memory_order_relaxed);
        if(!tm_on){
            if(g_fp){ flush_row_locked(); close_file_locked(); }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // Open file lazily once TM is on.
        if(!g_fp){
            uint64_t cf  = g_v->live_cf_hz.load(std::memory_order_relaxed);
            uint64_t sr  = (uint64_t)g_v->header.sample_rate;
            uint32_t fsz, fis;
            float    dmin, dmax;
            { std::lock_guard<std::mutex> lk(g_v->data_mtx);
              fsz  = (uint32_t)g_v->fft_size;
              fis  = (uint32_t)g_v->fft_input_size;
              dmin = g_v->display_power_min;
              dmax = g_v->display_power_max; }
            float    lon = g_v->station_lon;
            float    lat = g_v->station_lat;
            std::string sn = g_v->station_name;     // const std::string copy
            if(cf == 0 || sr == 0 || fsz == 0){
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            if(!open_new_file(cf, sr, fsz, fis, dmin, dmax, lon, lat, sn.c_str())){
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            // Initialize last_total_ffts so we don't dump pre-existing buffer.
            { std::lock_guard<std::mutex> lk(g_v->data_mtx);
              g_last_total_ffts = g_v->total_ffts; }
            next_flush = clk::now() + std::chrono::milliseconds(200);
        }

        // Ingest fresh rows (read-only on fft_data; data_mtx briefly).
        ingest_new_rows(g_v);

        // Flush at row_rate cadence.
        auto now = clk::now();
        if(now >= next_flush){
            flush_row_locked();
            // Schedule next: 1000ms / row_rate_hz, default 200ms.
            int period_ms = 200;
            if(g_hdr_cur.row_rate_hz > 0.5f){
                period_ms = (int)(1000.0f / g_hdr_cur.row_rate_hz);
                if(period_ms < 50) period_ms = 50;
            }
            next_flush = now + std::chrono::milliseconds(period_ms);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Shutdown: close cleanly.
    if(g_fp){ flush_row_locked(); close_file_locked(); }
}

} // anon

// ── Public API ───────────────────────────────────────────────────────────

void start_worker(FFTViewer* v){
    if(g_running.exchange(true)){
        // Already running → just re-bind viewer pointer (safe if same v).
        g_v = v;
        return;
    }
    g_v = v;
    g_rotate_seen_seq = g_rotate_req_seq.load();
    g_last_total_ffts = 0;
    g_thr = std::thread(worker_loop);
}

void stop_worker(){
    if(!g_running.exchange(false)) return;
    if(g_thr.joinable()) g_thr.join();
    g_v = nullptr;
}

void request_rotate(){
    g_rotate_req_seq.fetch_add(1, std::memory_order_relaxed);
}

std::string current_file_path(){
    std::lock_guard<std::mutex> lk(g_path_mtx);
    return g_cur_path;
}

void set_live_callbacks(const LiveCallbacks& cbs){
    std::lock_guard<std::mutex> lk(g_live_cb_mtx);
    g_live_cb = cbs;
}

bool snapshot_live_start(::PktLwfLiveStart& out){
    std::lock_guard<std::mutex> lk(g_live_state_mtx);
    if(!g_live_state_valid) return false;
    out = g_live_state;
    return true;
}

void scan_dir_into_list(::PktLwfList& out){
    memset(&out, 0, sizeof(out));
    std::string dir = BEWEPaths::hist_host_dir();
    // Exclude the file currently being recorded — JOINs see it via LIVE_START
    // (LIVE tab) instead, never as a finished entry in the HOST tab.
    std::string cur_full = current_file_path();
    std::string cur_base;
    if(!cur_full.empty()){
        size_t s = cur_full.find_last_of('/');
        cur_base = (s == std::string::npos) ? cur_full : cur_full.substr(s+1);
    }
    DIR* d = opendir(dir.c_str());
    if(!d) return;

    struct Entry {
        std::string name;
        uint64_t size, start, cf, sr;
        uint32_t fft;
        char     station_name[32];
        float    station_lat, station_lon;
    };
    std::vector<Entry> all;
    struct dirent* de;
    while((de = readdir(d)) != nullptr){
        const char* n = de->d_name;
        if(!n || n[0]=='.') continue;
        if(!is_lwf_filename(n)) continue;
        if(!cur_base.empty() && cur_base == n) continue; // skip active LIVE
        std::string full = dir + "/" + n;
        struct stat st{};
        if(stat(full.c_str(), &st) != 0) continue;
        if((uint64_t)st.st_size < sizeof(FileHeader)) continue;
        FILE* fp = fopen(full.c_str(), "rb");
        if(!fp) continue;
        FileHeader h{};
        if(fread(&h, 1, sizeof(h), fp) != sizeof(h) || memcmp(h.magic, "BWWF", 4) != 0
           || h.version != FILE_VERSION){
            fclose(fp); continue;
        }
        fclose(fp);
        Entry e{};
        e.name  = n;
        e.size  = (uint64_t)st.st_size;
        e.start = h.start_utc_unix;
        e.cf    = h.center_freq_hz;
        e.sr    = h.sample_rate_hz;
        e.fft   = h.fft_size;
        memcpy(e.station_name, h.station_name, sizeof(e.station_name));
        e.station_lat = h.station_lat;
        e.station_lon = h.station_lon;
        all.push_back(e);
    }
    closedir(d);

    // Newest first
    std::sort(all.begin(), all.end(),
              [](const Entry& a, const Entry& b){ return a.start > b.start; });
    if(all.size() > MAX_LWF_FILES) all.resize(MAX_LWF_FILES);

    out.count = (uint16_t)all.size();
    for(size_t i=0; i<all.size(); i++){
        LwfFileEntry& e = out.entries[i];
        memset(&e, 0, sizeof(e));
        strncpy(e.filename, all[i].name.c_str(), sizeof(e.filename)-1);
        e.size_bytes     = all[i].size;
        e.start_utc      = all[i].start;
        e.center_freq_hz = all[i].cf;
        e.sample_rate_hz = all[i].sr;
        e.fft_size       = all[i].fft;
        e.num_rows       = (uint32_t)((all[i].size - sizeof(FileHeader)) / std::max<uint64_t>(1, all[i].fft));
        memcpy(e.station_name, all[i].station_name, sizeof(e.station_name));
        e.station_lat    = all[i].station_lat;
        e.station_lon    = all[i].station_lon;
    }
}

} // namespace LongWaterfall
