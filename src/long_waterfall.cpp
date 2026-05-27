#include "long_waterfall.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "net_protocol.hpp"
#include "mission_push.hpp"

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
// 현재 열린 파일의 "Central 연결 안정성" 표시. LIVE row stream 이 끊긴 적이 있으면
// finalize 시 file push 로 보완. 연결 내내 안정이면 push 생략 (LIVE tap 만으로 충분).
// open_new_file 마다 false 로 reset. cli_host 의 reconnect 흐름이 mark_dirty() 호출.
std::atomic<bool>   g_file_dirty{false};
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
    uint32_t rows_snapshot = 0;
    {
        std::lock_guard<std::mutex> lk(g_live_state_mtx);
        if(g_live_state_valid){
            memcpy(stop_pkt.filename, g_live_state.filename, sizeof(stop_pkt.filename));
            had_state = true;
        }
        rows_snapshot = g_live_row_idx;
        g_live_state_valid = false;
        g_live_row_idx = 0;
    }
    if(had_state){
        LiveCallbacks cb_copy;
        { std::lock_guard<std::mutex> lk(g_live_cb_mtx); cb_copy = g_live_cb; }
        if(cb_copy.on_stop) cb_copy.on_stop(stop_pkt);
    }

    if(g_fp){ fflush(g_fp); fclose(g_fp); g_fp = nullptr; }

    // Empty file (header-only, no rows flushed) — discard instead of finalize.
    // Common at midnight rollover: mission_end + mission_start + utc0_worker call
    // request_rotate() three times within seconds; one cycle opens 0000-LIVE and
    // closes it immediately, leaving a 128B 0000-0000.bewehist.
    if(rows_snapshot == 0){
        std::string discard_path;
        {
            std::lock_guard<std::mutex> lk(g_path_mtx);
            discard_path = g_cur_path;
            g_cur_path.clear();
        }
        if(!discard_path.empty()){
            printf("[LongWaterfall] discard empty file (no rows): %s\n",
                   discard_path.c_str());
            unlink(discard_path.c_str());
            unlink((discard_path + ".info").c_str());
        }
        g_acc_db.clear();
        g_acc_count = 0;
        g_file_dirty.store(false);
        return;
    }

    // Finalize: -LIVE → -<HHMM>Z based on close time.
    std::string finalized_path;
    {
        std::lock_guard<std::mutex> lk(g_path_mtx);
        if(!g_cur_path.empty()){
            auto slash = g_cur_path.find_last_of('/');
            std::string dir  = (slash == std::string::npos) ? "" : g_cur_path.substr(0, slash+1);
            std::string base = (slash == std::string::npos) ? g_cur_path : g_cur_path.substr(slash+1);
            std::string fin  = build_hist_filename_finalize(base, (uint64_t)time(nullptr),
                                                             (int)g_hdr_cur.utc_offset_hours);
            if(fin != base){
                std::string final_full = dir + fin;
                if(rename(g_cur_path.c_str(), final_full.c_str()) == 0){
                    printf("[LongWaterfall] rotate finalize: %s → %s\n", base.c_str(), fin.c_str());
                    finalized_path = final_full;
                } else {
                    fprintf(stderr, "[LongWaterfall] rename failed: %s → %s errno=%d\n",
                            base.c_str(), fin.c_str(), errno);
                    finalized_path = g_cur_path;  // best-effort: original path
                }
            } else {
                finalized_path = g_cur_path;
            }
        }
        g_cur_path.clear();
    }
    g_acc_db.clear();
    g_acc_count = 0;
    // Mission File Push: LIVE row stream 이 안정적으로 도달했으면 (g_file_dirty==false)
    //   → push 생략하고 로컬 파일도 즉시 unlink (Central mirror 만 source-of-truth).
    // 연결이 끊긴 적 있으면 (dirty==true) → 통파일 push (MissionPush 가 ACK 후 unlink 함).
    // 어느 경로든 HOST 로컬엔 .bewehist 가 남지 않음 → 디스크 누적 방지 (v4.4.0).
    if(!finalized_path.empty()){
        bool was_dirty = g_file_dirty.exchange(false);
        if(was_dirty){
            printf("[LongWaterfall] file finalize DIRTY (Central was down) — enqueue push: %s\n",
                   finalized_path.c_str());
            MissionPush::enqueue(finalized_path, MFS_HIST);
        } else {
            // LIVE tap 만으로 Central 에 동일 파일이 이미 finalize 됨 → 로컬 사본 불필요.
            printf("[LongWaterfall] file finalize CLEAN — unlink local (Central has mirror): %s\n",
                   finalized_path.c_str());
            unlink(finalized_path.c_str());
            unlink((finalized_path + ".info").c_str());
        }
    }
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
    // 미션 활성 시 그 미션의 hist 디렉토리 사용. IDLE이면 빈 문자열 → silently
    // 파일 생성 건너뜀 (worker_loop가 500ms마다 재시도, 다음 mission_start
    // request_rotate 호출 시 다시 들어옴).
    std::string dir;
    if(g_v) dir = g_v->active_hist_dir();
    if(dir.empty()){
        // 미션 IDLE → 조용히 skip. worker 가 500ms 후 재시도.
        return false;
    }
    mkdir(BEWEPaths::recordings_dir().c_str(), 0755);

    uint64_t now_utc = (uint64_t)time(nullptr);
    int32_t off_h_now = KST::OFFSET_HOURS;  // KST 강제 (UTC+9)
    std::string fname = build_hist_filename_live(now_utc, cf_hz, (int)off_h_now, station_name);
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
    h.utc_offset_hours = off_h_now;
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
    g_file_dirty.store(false);  // 새 파일 = LIVE tap 안정 가정으로 시작
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

    // v4.5.3 — 소스는 4× padded (fft_size), HIST 저장은 1× (fft_input_size).
    // 4 bin 묶음 max-hold 로 폴딩 — 좁은 피크 보존.
    int src_fft = v->fft_size;
    int dst_fft = v->fft_input_size;
    if(src_fft <= 0 || dst_fft <= 0) return false;
    int pad = src_fft / dst_fft; if(pad < 1) pad = 1;
    if((int)g_acc_db.size() != dst_fft){
        // size mismatch — safest: flush whatever we had and resize.
        // (Worker should have rotated already on fft_size change; handle defensively.)
        g_acc_db.assign(dst_fft, -200.0f);
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
        const float* rowp = v->fft_data.data() + (size_t)fi * src_fft;
        if(pad == 1){
            for(int i=0; i<dst_fft; i++){
                float d = rowp[i];
                if(d > g_acc_db[i]) g_acc_db[i] = d;
            }
        } else {
            for(int o=0; o<dst_fft; o++){
                const float* gp = rowp + (size_t)o * pad;
                float mx = gp[0];
                for(int k=1; k<pad; k++){ if(gp[k] > mx) mx = gp[k]; }
                if(mx > g_acc_db[o]) g_acc_db[o] = mx;
            }
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
              // v4.5.3 — HIST 파일은 1× FFT (fft_input_size) 로 저장.
              // 4× zero-pad 는 display 전용 — HIST 디스크 소비 4× 절감.
              fis  = (uint32_t)g_v->fft_input_size;
              fsz  = fis;
              dmin = g_v->display_power_min;
              dmax = g_v->display_power_max; }
            float    lon = g_v->station_lon;
            float    lat = g_v->station_lat;
            std::string sn = g_v->station_name;     // const std::string copy
            if(cf == 0 || sr == 0 || fsz == 0){
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            static int s_open_fail_cnt = 0;
            if(!open_new_file(cf, sr, fsz, fis, dmin, dmax, lon, lat, sn.c_str())){
                s_open_fail_cnt++;
                if(s_open_fail_cnt == 1 || s_open_fail_cnt == 10 ||
                   (s_open_fail_cnt % 120) == 0){
                    fprintf(stderr, "[LWF] open_new_file FAILED x%d in a row\n",
                            s_open_fail_cnt);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            s_open_fail_cnt = 0;
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
    // 기존 실행에서 crash로 남은 -LIVE.bewehist 파일을 mtime 기준으로 finalize.
    finalize_stale_live_all();
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

// Central 연결이 끊겼을 때 cli_host 가 호출. 현재 열려있는 HIST 파일에 dirty flag set.
// finalize 시점에 dirty 면 MissionPush 로 통파일 push (LIVE tap 동안 누락 row 보완).
void mark_dirty(){
    g_file_dirty.store(true);
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

// Finalize any "...-LIVE.bewehist" files in `dir` that aren't the active recording.
// Uses file mtime as end time → renames to "...-HHMM.bewehist" (KST).
void finalize_stale_live_in_dir(const std::string& dir,
                                 const std::string& active_basename){
    DIR* d = opendir(dir.c_str());
    if(!d) return;
    struct dirent* de;
    std::vector<std::string> targets;
    while((de = readdir(d)) != nullptr){
        const char* n = de->d_name;
        if(!n || n[0]=='.') continue;
        std::string name = n;
        if(name.rfind("-LIVE.bewehist") == std::string::npos) continue;
        if(!active_basename.empty() && name == active_basename) continue;
        targets.push_back(name);
    }
    closedir(d);
    for(auto& base : targets){
        std::string full = dir + "/" + base;
        struct stat st{};
        if(stat(full.c_str(), &st) != 0) continue;
        uint64_t end_utc = (uint64_t)st.st_mtime;
        std::string fin = build_hist_filename_finalize(base, end_utc, 0);
        if(fin == base) continue;
        std::string new_full = dir + "/" + fin;
        // 충돌 회피: 같은 이름 이미 있으면 _2, _3 ... suffix
        std::string try_full = new_full;
        std::string try_fin  = fin;
        for(int n=2; access(try_full.c_str(), F_OK)==0 && n<100; ++n){
            auto dot = fin.rfind(".bewehist");
            if(dot == std::string::npos) break;
            try_fin  = fin.substr(0, dot) + "_" + std::to_string(n) + ".bewehist";
            try_full = dir + "/" + try_fin;
        }
        if(rename(full.c_str(), try_full.c_str()) == 0){
            printf("[LongWaterfall] stale-LIVE finalize: %s → %s\n",
                   base.c_str(), try_fin.c_str());
        } else {
            fprintf(stderr, "[LongWaterfall] stale-LIVE rename failed: %s → %s errno=%d\n",
                    base.c_str(), try_fin.c_str(), errno);
        }
    }
}

void finalize_stale_live_all(){
    // 현재 recording 중인 파일 basename (skip 대상)
    std::string cur = current_file_path();
    std::string cur_base;
    if(!cur.empty()){
        auto s = cur.find_last_of('/');
        cur_base = (s == std::string::npos) ? cur : cur.substr(s+1);
    }
    finalize_stale_live_in_dir(BEWEPaths::hist_host_dir(), cur_base);
    finalize_stale_live_in_dir(BEWEPaths::hist_join_dir(), cur_base);
    finalize_stale_live_in_dir(BEWEPaths::hist_live_dir(), cur_base);
    // 활성 미션 hist 디렉토리
    if(g_v){
        std::string md = g_v->active_hist_dir();
        if(!md.empty()) finalize_stale_live_in_dir(md, cur_base);
    }
    // 모든 미션 디렉토리 (지난 미션의 stale LIVE도 정리)
    // v3.20.0 layout: missions/<station>/<year>/<code>/hist/ — 3 단계 루프.
    DIR* dr = opendir(BEWEPaths::missions_root().c_str());
    if(!dr) return;
    struct dirent* de;
    while((de = readdir(dr)) != nullptr){
        const char* sname = de->d_name;
        if(!sname || sname[0]=='.') continue;
        // 4자리 숫자(YYYY) 폴더는 legacy — 마이그레이션 안 됐으면 무시.
        if(strlen(sname) == 4){
            bool all_digit = true;
            for(int i=0;i<4;i++) if(sname[i]<'0'||sname[i]>'9'){ all_digit=false; break; }
            if(all_digit) continue;
        }
        std::string sdir = BEWEPaths::missions_root() + "/" + sname;
        DIR* ds = opendir(sdir.c_str());
        if(!ds) continue;
        struct dirent* de_y;
        while((de_y = readdir(ds)) != nullptr){
            const char* yn = de_y->d_name;
            if(!yn || yn[0]=='.') continue;
            if(strlen(yn) != 4) continue;
            bool num = true;
            for(int i=0;i<4;i++) if(yn[i]<'0'||yn[i]>'9'){ num=false; break; }
            if(!num) continue;
            std::string ydir = sdir + "/" + yn;
            DIR* dy = opendir(ydir.c_str());
            if(!dy) continue;
            struct dirent* de2;
            while((de2 = readdir(dy)) != nullptr){
                const char* m = de2->d_name;
                if(!m || m[0]=='.') continue;
                std::string hd = ydir + "/" + m + "/hist";
                struct stat st;
                if(stat(hd.c_str(), &st) == 0 && S_ISDIR(st.st_mode)){
                    finalize_stale_live_in_dir(hd, cur_base);
                }
            }
            closedir(dy);
        }
        closedir(ds);
    }
    closedir(dr);
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
