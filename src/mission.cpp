// SIGINT Mission System — lifecycle 구현 (자동 캡처 모델).
// 미션 시작 시 운영자 입력 없이 station/host/lat/lon/SDR/안테나를 FFTViewer 컨텍스트에서
// 자동 캡처. KST(UTC+9) 정시마다 HIST rotate (1시간 1파일). KST 0시는 미션 자체 rollover.

#include "mission.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "long_waterfall.hpp"
#include "kst_time.hpp"
#include "hw_config.hpp"
#include "login.hpp"
#include "mission_push.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <thread>
#include <vector>

// ── Toast storage (mission_view.cpp가 draw_toast로 읽음) ──────────────────
namespace MissionView {
    std::string g_toast_msg;
    double      g_toast_expire = 0.0;
    void show_toast(const char* msg){
        if(!msg) return;
        g_toast_msg = msg;
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        g_toast_expire = std::chrono::duration<double>(now).count() + 3.0;
    }
}

namespace Mission {

static std::thread       g_utc0_thr;
static std::atomic<bool> g_utc0_stop{false};
static int               g_last_rotate_hour_id = -1;

static void utc0_worker(FFTViewer* v){
    // 1초씩 쪼개 sleep — stop_utc0_worker() 호출 시 최대 1초 안에 종료.
    // 20초 단위 정시 검사는 last_check_sec으로 dedup.
    // 정시 기준은 KST(UTC+9). KST 0시 = mission rollover.
    int last_check_sec = -1;
    while(!g_utc0_stop.load()){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if(g_utc0_stop.load()) break;
        if(!v) continue;
        time_t now = time(nullptr);
        if((int)(now / 20) == last_check_sec) continue;
        last_check_sec = (int)(now / 20);
        struct tm tm_kst; KST::to_tm(now, tm_kst);
        if(tm_kst.tm_min != 0) continue;
        int hour_id = tm_kst.tm_yday * 24 + tm_kst.tm_hour;
        if(hour_id == g_last_rotate_hour_id) continue;
        g_last_rotate_hour_id = hour_id;
        if(tm_kst.tm_hour == 0){
            v->mission_rollover_utc0();
        } else {
            LongWaterfall::request_rotate();
        }
    }
}

void start_utc0_worker(FFTViewer* v){
    if(g_utc0_thr.joinable()) return;
    g_utc0_stop.store(false);
    g_utc0_thr = std::thread(utc0_worker, v);
}

void stop_utc0_worker(){
    g_utc0_stop.store(true);
    if(g_utc0_thr.joinable()) g_utc0_thr.join();
}

} // namespace Mission

// ── 자동 캡처 helpers ─────────────────────────────────────────────────────
static const char* mission_sdr_kind_str(const FFTViewer& v){
    if(v.dev_blade)             return "BladeRF";
    if(v.pluto_ctx)             return "Pluto";
    if(v.dev_rtl)               return "RTL-SDR";
    if(v.hw.type == HWType::BLADERF) return "BladeRF";
    if(v.hw.type == HWType::PLUTO)   return "Pluto";
    if(v.hw.type == HWType::RTLSDR)  return "RTL-SDR";
    return "Unknown";
}

static void fill_host_name(char* dst, size_t cap){
    // login_get_id가 비어있을 수도 → 시스템 hostname 폴백
    const char* lid = login_get_id();
    if(lid && lid[0]){
        size_t n = strlen(lid); if(n >= cap) n = cap - 1;
        memcpy(dst, lid, n); dst[n] = 0;
        return;
    }
    char h[64] = {};
    if(gethostname(h, sizeof(h)-1) == 0){
        size_t n = strlen(h); if(n >= cap) n = cap - 1;
        memcpy(dst, h, n); dst[n] = 0;
    } else {
        dst[0] = 0;
    }
}

// "By: " 필드용 폴백 — login_id → USER env → hostname → "unknown"
static void fill_started_by(char* dst, size_t cap, const char* requested){
    auto cpy = [&](const char* src){
        if(!src || !src[0]) return false;
        size_t n = strlen(src); if(n >= cap) n = cap - 1;
        memcpy(dst, src, n); dst[n] = 0;
        return true;
    };
    if(cpy(requested)) return;
    if(cpy(login_get_id())) return;
    if(cpy(getenv("USER"))) return;
    if(cpy(getenv("LOGNAME"))) return;
    char h[64] = {};
    if(gethostname(h, sizeof(h)-1) == 0 && cpy(h)) return;
    cpy("unknown");
}

// ── FFTViewer mission_* 구현 ─────────────────────────────────────────────

bool FFTViewer::mission_start(const char* started_by, uint8_t op_index, bool rollover){
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        if(mission_state != Mission::State::IDLE) return false;

        time_t now = time(nullptr);
        struct tm tm_kst; KST::to_tm(now, tm_kst);
        mission_year = 1900 + tm_kst.tm_year;
        auto code = Mission::make_code(mission_year, tm_kst.tm_mon, tm_kst.tm_mday);

        auto cpy = [](char* dst, size_t cap, const char* src){
            if(!src){ dst[0] = 0; return; }
            size_t n = strlen(src);
            if(n >= cap) n = cap - 1;
            memcpy(dst, src, n); dst[n] = 0;
        };
        cpy(mission_code,       sizeof(mission_code),       code.c_str());
        // started_by 폴백 (인자 → login_id → USER env → hostname → "unknown")
        fill_started_by(mission_started_by, sizeof(mission_started_by), started_by);
        mission_op_index   = op_index;
        mission_start_utc  = now;
        mission_end_utc    = 0;
        mission_state      = Mission::State::ACTIVE;

        // 자동 캡처 메타데이터
        cpy(mission_station_name, sizeof(mission_station_name), station_name.c_str());
        fill_host_name(mission_host_name, sizeof(mission_host_name));
        mission_lat = station_lat;
        mission_lon = station_lon;
        cpy(mission_sdr_kind, sizeof(mission_sdr_kind), mission_sdr_kind_str(*this));
        cpy(mission_antenna,  sizeof(mission_antenna),  host_antenna);

        // 디렉토리 생성
        mkdir(BEWEPaths::missions_root().c_str(), 0755);
        mkdir(BEWEPaths::missions_year_dir(mission_year).c_str(), 0755);
        mkdir(BEWEPaths::mission_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(BEWEPaths::mission_iq_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(BEWEPaths::mission_audio_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(BEWEPaths::mission_hist_dir(mission_year, mission_code).c_str(), 0755);

        bewe_log_push(0, "[MISSION] start: %04d/%s by %s (rollover=%d, sdr=%s)\n",
                      mission_year, mission_code, mission_started_by,
                      (int)rollover, mission_sdr_kind);
    }

    LongWaterfall::request_rotate();
    // 미션 시작 시 활성 hist dir에 남아있을 수 있는 stale -LIVE 파일 정리.
    LongWaterfall::finalize_stale_live_all();
    mission_save_meta_to_disk();
    mission_broadcast_sync();
    return true;
}

bool FFTViewer::mission_end(){
    int ended_year = 0;
    char ended_code[8] = {};
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        if(mission_state != Mission::State::ACTIVE) return false;
        mission_state   = Mission::State::CLOSING;
        mission_end_utc = time(nullptr);
        ended_year = mission_year;
        memcpy(ended_code, mission_code, sizeof(ended_code));
        bewe_log_push(0, "[MISSION] end: %04d/%s\n", mission_year, mission_code);
    }

    LongWaterfall::request_rotate();
    if(rec_on.load()) stop_rec();
    for(int i = 0; i < MAX_CHANNELS; ++i){
        if(channels[i].iq_rec_on.load()) stop_iq_rec(i);
        if(channels[i].audio_rec_on.load()){
            if(remote_mode) stop_join_audio_rec(i);
            else            stop_audio_rec(i);
        }
    }

    // Mission File Push: 미션 종료 시점에 잔여 파일 모두 enqueue (race-safe)
    MissionPush::scan_mission_dir_enqueue(ended_year, ended_code);

    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        MissionEntry done{};
        done.year      = mission_year;
        memcpy(done.code,         mission_code,         sizeof(done.code));
        memcpy(done.started_by,   mission_started_by,   sizeof(done.started_by));
        memcpy(done.station_name, mission_station_name, sizeof(done.station_name));
        memcpy(done.host_name,    mission_host_name,    sizeof(done.host_name));
        memcpy(done.sdr_kind,     mission_sdr_kind,     sizeof(done.sdr_kind));
        memcpy(done.antenna,      mission_antenna,      sizeof(done.antenna));
        done.lat       = mission_lat;
        done.lon       = mission_lon;
        done.op_index  = mission_op_index;
        done.start_utc = mission_start_utc;
        done.end_utc   = mission_end_utc;
        mission_history.push_back(done);

        // IDLE 전이
        mission_state         = Mission::State::IDLE;
        mission_code[0]       = 0;
        mission_year          = 0;
        mission_started_by[0] = 0;
        mission_station_name[0] = 0;
        mission_host_name[0]    = 0;
        mission_sdr_kind[0]     = 0;
        mission_antenna[0]      = 0;
        mission_lat = mission_lon = 0.f;
        mission_op_index      = 0;
        mission_start_utc     = 0;
        mission_end_utc       = 0;
    }
    mission_save_meta_to_disk();
    mission_broadcast_sync();
    return true;
}

void FFTViewer::mission_rollover_utc0(){
    char prev_started_by[32];
    uint8_t prev_op;
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        if(mission_state != Mission::State::ACTIVE) return;
        memcpy(prev_started_by, mission_started_by, sizeof(prev_started_by));
        prev_op = mission_op_index;
    }
    mission_end();
    mission_start(prev_started_by, prev_op, /*rollover=*/true);
}

// ── Persistence helpers ──────────────────────────────────────────────────
static void mj_escape(std::string& out, const char* s){
    for(; *s; s++){
        char c = *s;
        if(c == '"' || c == '\\'){ out += '\\'; out += c; }
        else if(c == '\n') out += "\\n";
        else if(c == '\r') out += "\\r";
        else if((unsigned char)c < 0x20){}
        else out += c;
    }
}

static void mj_entry_write(std::string& out, int year, const char* code,
                           time_t start_utc, time_t end_utc,
                           const char* started_by,
                           const char* station_name, const char* host_name,
                           float lat, float lon,
                           const char* sdr_kind, const char* antenna,
                           uint8_t op_index, uint8_t rollover, uint8_t state){
    char buf[160];
    snprintf(buf, sizeof(buf),
        "{\"state\":%u,\"op_index\":%u,\"rollover\":%u,\"year\":%d,\"code\":\"",
        (unsigned)state, (unsigned)op_index, (unsigned)rollover, year);
    out += buf;
    mj_escape(out, code);
    snprintf(buf, sizeof(buf),
        "\",\"start_utc\":%lld,\"end_utc\":%lld,\"lat\":%.6f,\"lon\":%.6f,",
        (long long)start_utc, (long long)end_utc, (double)lat, (double)lon);
    out += buf;
    out += "\"started_by\":\""; mj_escape(out, started_by);
    out += "\",\"station_name\":\""; mj_escape(out, station_name);
    out += "\",\"host_name\":\"";    mj_escape(out, host_name);
    out += "\",\"sdr_kind\":\"";     mj_escape(out, sdr_kind);
    out += "\",\"antenna\":\"";      mj_escape(out, antenna);
    out += "\"}";
}

void FFTViewer::mission_save_meta_to_disk(){
    std::lock_guard<std::mutex> lk(mission_mtx);

    // missions.json
    std::string out = "{\n  \"version\": 1,\n  \"missions\": [\n";
    bool first = true;
    if(mission_state == Mission::State::ACTIVE && mission_code[0]){
        if(!first) out += ",\n";
        out += "    ";
        mj_entry_write(out, mission_year, mission_code,
                       mission_start_utc, 0,
                       mission_started_by, mission_station_name, mission_host_name,
                       mission_lat, mission_lon, mission_sdr_kind, mission_antenna,
                       mission_op_index, 0, (uint8_t)Mission::State::ACTIVE);
        first = false;
    }
    for(auto& e : mission_history){
        if(!first) out += ",\n";
        out += "    ";
        mj_entry_write(out, e.year, e.code, e.start_utc, e.end_utc,
                       e.started_by, e.station_name, e.host_name,
                       e.lat, e.lon, e.sdr_kind, e.antenna,
                       e.op_index, e.rollover, (uint8_t)Mission::State::IDLE);
        first = false;
    }
    out += "\n  ]\n}\n";
    mkdir(BEWEPaths::missions_root().c_str(), 0755);
    FILE* fp = fopen(BEWEPaths::missions_json_path().c_str(), "w");
    if(fp){ fwrite(out.data(), 1, out.size(), fp); fclose(fp); }
    else   bewe_log_push(1, "[MISSION] save missions.json failed errno=%d\n", errno);

    // mission.info (활성 미션만)
    if(mission_state == Mission::State::ACTIVE && mission_code[0]){
        std::string ipath = BEWEPaths::mission_info_path(mission_year, mission_code);
        FILE* ifp = fopen(ipath.c_str(), "w");
        if(ifp){
            time_t st = mission_start_utc;
            struct tm tu; KST::to_tm(st, tu);
            fprintf(ifp,
                "Mission Code: %04d/%s\n"
                "Started By: %s\n"
                "Start: %04d-%02d-%02d %02d:%02d:%02d\n"
                "Station: %s\n"
                "Host: %s\n"
                "Lat: %.6f\n"
                "Lon: %.6f\n"
                "SDR: %s\n"
                "Antenna: %s\n"
                "Op Index: %u\n",
                mission_year, mission_code,
                mission_started_by,
                1900+tu.tm_year, 1+tu.tm_mon, tu.tm_mday,
                tu.tm_hour, tu.tm_min, tu.tm_sec,
                mission_station_name, mission_host_name,
                (double)mission_lat, (double)mission_lon,
                mission_sdr_kind, mission_antenna,
                (unsigned)mission_op_index);
            fclose(ifp);
        }
    }
}

static bool mj_parse_str(const char*& p, char* dst, size_t cap){
    while(*p && *p != '"') p++;
    if(!*p) return false;
    p++;
    size_t i = 0;
    while(*p && *p != '"'){
        if(*p == '\\' && p[1]){ p++; }
        if(i < cap-1) dst[i++] = *p;
        p++;
    }
    if(!*p) return false;
    p++; dst[i] = 0;
    return true;
}

void FFTViewer::mission_load_history(){
    FILE* fp = fopen(BEWEPaths::missions_json_path().c_str(), "r");
    if(!fp){
        bewe_log_push(0, "[MISSION] missions.json absent — starting IDLE\n");
        return;
    }
    fseek(fp, 0, SEEK_END); long sz = ftell(fp); fseek(fp, 0, SEEK_SET);
    std::vector<char> buf((size_t)sz + 1, 0);
    if((long)fread(buf.data(), 1, sz, fp) != sz){ fclose(fp); return; }
    fclose(fp);

    std::lock_guard<std::mutex> lk(mission_mtx);
    mission_history.clear();
    const char* p = strstr(buf.data(), "\"missions\"");
    if(!p) return;
    while(*p && *p != '[') p++;
    if(!*p) return;
    p++;
    bool active_restored = false;
    while(*p){
        while(*p && (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t')) p++;
        if(*p == ']') break;
        if(*p != '{') break;
        p++;
        MissionEntry e{};
        char tmp[256];
        while(*p && *p != '}'){
            while(*p && (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t')) p++;
            if(*p == '}') break;
            char key[32] = {};
            if(!mj_parse_str(p, key, sizeof(key))) break;
            while(*p && *p != ':') p++;
            if(!*p) break;
            p++;
            while(*p == ' ' || *p == '\t') p++;
            if(*p == '"'){
                tmp[0] = 0; mj_parse_str(p, tmp, sizeof(tmp));
                if      (!strcmp(key,"code"))         { strncpy(e.code,         tmp, sizeof(e.code)-1); }
                else if (!strcmp(key,"started_by"))   { strncpy(e.started_by,   tmp, sizeof(e.started_by)-1); }
                else if (!strcmp(key,"station_name")) { strncpy(e.station_name, tmp, sizeof(e.station_name)-1); }
                else if (!strcmp(key,"host_name"))    { strncpy(e.host_name,    tmp, sizeof(e.host_name)-1); }
                else if (!strcmp(key,"sdr_kind"))     { strncpy(e.sdr_kind,     tmp, sizeof(e.sdr_kind)-1); }
                else if (!strcmp(key,"antenna"))      { strncpy(e.antenna,      tmp, sizeof(e.antenna)-1); }
            } else if((*p>='0' && *p<='9') || *p=='-' || *p=='+'){
                // 숫자(정수/실수). lat/lon만 float, 나머지는 정수.
                char* endp = nullptr;
                if(!strcmp(key,"lat"))      { e.lat = (float)strtod(p, &endp); p = endp; }
                else if(!strcmp(key,"lon")) { e.lon = (float)strtod(p, &endp); p = endp; }
                else {
                    long long v = strtoll(p, (char**)&p, 10);
                    if      (!strcmp(key,"op_index"))  e.op_index  = (uint8_t)v;
                    else if (!strcmp(key,"rollover"))  e.rollover  = (uint8_t)v;
                    else if (!strcmp(key,"year"))      e.year      = (int)v;
                    else if (!strcmp(key,"start_utc")) e.start_utc = (time_t)v;
                    else if (!strcmp(key,"end_utc"))   e.end_utc   = (time_t)v;
                    // state는 ACTIVE 복원에 사용 안 함 (end_utc=0 검사로 충분)
                }
            }
            while(*p && (*p == ' ' || *p == '\t' || *p == '\n')) p++;
        }
        if(*p == '}') p++;
        if(e.end_utc == 0 && !active_restored){
            mission_state    = Mission::State::ACTIVE;
            mission_year     = e.year;
            memcpy(mission_code,         e.code,         sizeof(mission_code));
            memcpy(mission_started_by,   e.started_by,   sizeof(mission_started_by));
            memcpy(mission_station_name, e.station_name, sizeof(mission_station_name));
            memcpy(mission_host_name,    e.host_name,    sizeof(mission_host_name));
            memcpy(mission_sdr_kind,     e.sdr_kind,     sizeof(mission_sdr_kind));
            memcpy(mission_antenna,      e.antenna,      sizeof(mission_antenna));
            mission_lat        = e.lat;
            mission_lon        = e.lon;
            mission_op_index   = e.op_index;
            mission_start_utc  = e.start_utc;
            mission_end_utc    = 0;
            active_restored = true;
            bewe_log_push(0, "[MISSION] restored ACTIVE %04d/%s station=%s sdr=%s\n",
                          e.year, e.code, e.station_name, e.sdr_kind);
        } else {
            mission_history.push_back(e);
        }
    }
    bewe_log_push(0, "[MISSION] loaded %zu history entries (active=%d)\n",
                  mission_history.size(), (int)active_restored);
}

// ── Broadcast ────────────────────────────────────────────────────────────
static void fill_sync_entry(MissionSyncEntry& dst,
                            const FFTViewer::MissionEntry& src,
                            uint8_t state_val){
    dst.valid    = 1;
    dst.state    = state_val;
    dst.op_index = src.op_index;
    dst.rollover = src.rollover;
    dst.year     = (uint16_t)src.year;
    memcpy(dst.code,         src.code,         sizeof(dst.code));
    dst.start_utc = (int64_t)src.start_utc;
    dst.end_utc   = (int64_t)src.end_utc;
    memcpy(dst.started_by,   src.started_by,   sizeof(dst.started_by));
    memcpy(dst.station_name, src.station_name, sizeof(dst.station_name));
    memcpy(dst.host_name,    src.host_name,    sizeof(dst.host_name));
    dst.lat = src.lat;
    dst.lon = src.lon;
    memcpy(dst.sdr_kind, src.sdr_kind, sizeof(dst.sdr_kind));
    memcpy(dst.antenna,  src.antenna,  sizeof(dst.antenna));
}

void FFTViewer::mission_broadcast_sync(){
    if(!net_srv) return;
    PktMissionSync pkt{};
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        if(mission_state == Mission::State::ACTIVE && mission_code[0]){
            pkt.active_valid = 1;
            MissionEntry act{};
            act.year      = mission_year;
            memcpy(act.code,         mission_code,         sizeof(act.code));
            memcpy(act.started_by,   mission_started_by,   sizeof(act.started_by));
            memcpy(act.station_name, mission_station_name, sizeof(act.station_name));
            memcpy(act.host_name,    mission_host_name,    sizeof(act.host_name));
            memcpy(act.sdr_kind,     mission_sdr_kind,     sizeof(act.sdr_kind));
            memcpy(act.antenna,      mission_antenna,      sizeof(act.antenna));
            act.lat       = mission_lat;
            act.lon       = mission_lon;
            act.op_index  = mission_op_index;
            act.start_utc = mission_start_utc;
            act.end_utc   = 0;
            fill_sync_entry(pkt.active, act, (uint8_t)Mission::State::ACTIVE);
        }
        size_t n = mission_history.size();
        size_t taken = 0;
        for(size_t i = 0; i < n && taken < MAX_MISSION_HISTORY_PER_PKT; ++i){
            const auto& e = mission_history[n - 1 - i];
            fill_sync_entry(pkt.entries[taken], e, (uint8_t)Mission::State::IDLE);
            taken++;
        }
        pkt.history_count = (uint16_t)taken;
    }
    net_srv->broadcast_mission_sync(pkt);
}

// ── Delete ──────────────────────────────────────────────────────────────
// 미션 디렉토리(파일 모두) + history 엔트리 영구 삭제.
// 활성 미션이면 mission_end() 먼저 호출 → IDLE 전이 후 삭제.
static void rm_rf_dir(const std::string& path){
    DIR* d = opendir(path.c_str());
    if(!d) return;
    struct dirent* ent;
    while((ent = readdir(d)) != nullptr){
        const char* n = ent->d_name;
        if(!n || (n[0]=='.' && (n[1]==0 || (n[1]=='.' && n[2]==0)))) continue;
        std::string full = path + "/" + n;
        struct stat st;
        if(stat(full.c_str(), &st) != 0) continue;
        if(S_ISDIR(st.st_mode)) rm_rf_dir(full);
        else                    unlink(full.c_str());
    }
    closedir(d);
    rmdir(path.c_str());
}

bool FFTViewer::mission_delete(int year, const char* code){
    if(!code || !code[0] || year <= 0) return false;
    // 안전: code에 슬래시 금지
    if(strchr(code, '/')) return false;

    // 활성 미션 삭제 시 먼저 mission_end()
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        if(mission_state == Mission::State::ACTIVE &&
           mission_year == year && strcmp(mission_code, code) == 0){
            // 락 풀고 mission_end 호출 (mission_end가 자체 lock 잡음)
        } else {
            // history에서만 제거 (활성 미션 아님)
            for(auto it = mission_history.begin(); it != mission_history.end(); ){
                if(it->year == year && strcmp(it->code, code) == 0){
                    it = mission_history.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }
    // 활성 미션이었으면 종료 (락 풀린 후)
    if(mission_state == Mission::State::ACTIVE &&
       mission_year == year && strncmp(mission_code, code, 8) == 0){
        mission_end();
        // history에 들어갔으니 다시 제거
        std::lock_guard<std::mutex> lk(mission_mtx);
        for(auto it = mission_history.begin(); it != mission_history.end(); ){
            if(it->year == year && strcmp(it->code, code) == 0){
                it = mission_history.erase(it);
            } else {
                ++it;
            }
        }
    }

    // 디스크 디렉토리 통째 삭제
    std::string dir = BEWEPaths::mission_dir(year, code);
    rm_rf_dir(dir);

    bewe_log_push(0, "[MISSION] delete %04d/%s (dir removed)\n", year, code);

    mission_save_meta_to_disk();
    mission_broadcast_sync();
    return true;
}

// ── active_*_dir helpers ─────────────────────────────────────────────────
std::string FFTViewer::active_iq_dir() const {
    std::lock_guard<std::mutex> lk(mission_mtx);
    if(mission_state == Mission::State::ACTIVE && mission_code[0]){
        std::string d = BEWEPaths::mission_iq_dir(mission_year, mission_code);
        mkdir(BEWEPaths::missions_root().c_str(), 0755);
        mkdir(BEWEPaths::missions_year_dir(mission_year).c_str(), 0755);
        mkdir(BEWEPaths::mission_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(d.c_str(), 0755);
        return d;
    }
    return std::string();
}

std::string FFTViewer::active_audio_dir() const {
    std::lock_guard<std::mutex> lk(mission_mtx);
    if(mission_state == Mission::State::ACTIVE && mission_code[0]){
        std::string d = BEWEPaths::mission_audio_dir(mission_year, mission_code);
        mkdir(BEWEPaths::missions_root().c_str(), 0755);
        mkdir(BEWEPaths::missions_year_dir(mission_year).c_str(), 0755);
        mkdir(BEWEPaths::mission_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(d.c_str(), 0755);
        return d;
    }
    return std::string();
}

std::string FFTViewer::active_hist_dir() const {
    std::lock_guard<std::mutex> lk(mission_mtx);
    if(mission_state == Mission::State::ACTIVE && mission_code[0]){
        std::string d = BEWEPaths::mission_hist_dir(mission_year, mission_code);
        mkdir(BEWEPaths::missions_root().c_str(), 0755);
        mkdir(BEWEPaths::missions_year_dir(mission_year).c_str(), 0755);
        mkdir(BEWEPaths::mission_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(d.c_str(), 0755);
        return d;
    }
    return std::string();
}

std::string FFTViewer::mission_active_station_name() const {
    std::lock_guard<std::mutex> lk(mission_mtx);
    if(mission_state != Mission::State::ACTIVE) return std::string();
    return std::string(mission_station_name);
}

int FFTViewer::mission_active_year() const {
    std::lock_guard<std::mutex> lk(mission_mtx);
    if(mission_state != Mission::State::ACTIVE) return 0;
    return mission_year;
}

std::string FFTViewer::mission_active_code() const {
    std::lock_guard<std::mutex> lk(mission_mtx);
    if(mission_state != Mission::State::ACTIVE) return std::string();
    return std::string(mission_code);
}
