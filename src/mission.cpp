// SIGINT Mission System — lifecycle 구현.
// Phase A: stubs (start/end 메모리 상태만 변경, 디렉토리 생성, 로그).
// Phase B에서 stop_rec/stop_audio_rec 등 실 stop 호출과 broadcast_sync/save 채움.

#include "mission.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "long_waterfall.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <thread>

// Toast storage (UI / headless 양쪽 빌드. 표시는 GUI 빌드의 mission_view.cpp).
namespace MissionView {
    // 정의는 여기 (mission.cpp), 그림은 mission_view.cpp의 draw_toast가 읽음.
    std::string g_toast_msg;
    double      g_toast_expire = 0.0;   // steady_clock seconds since epoch
    void show_toast(const char* msg){
        if(!msg) return;
        g_toast_msg = msg;
        // 단조 시계 기반 (GUI도 같은 시계로 비교 가능하게 std::chrono 사용)
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        double s = std::chrono::duration<double>(now).count();
        g_toast_expire = s + 3.0;
    }
}

namespace Mission {

static std::thread        g_utc0_thr;
static std::atomic<bool>  g_utc0_stop{false};
// UTC 시간 단위 dedup: yday*24 + hour. 매 정시(분=0)당 1회만 트리거.
static int                g_last_rotate_hour_id = -1;

static void utc0_worker(FFTViewer* v){
    // 20초마다 깨어나 UTC 정시 검사. 분=0 60초 윈도우 안에 1회만 발사.
    // - UTC 00:00 → mission_rollover_utc0() (미션 자체 rollover + HIST rotate 포함)
    // - 그 외 정시 (01:00, 02:00, ..., 23:00) → HIST rotate만 (활성 미션 안에서 1시간 단위 파일)
    while(!g_utc0_stop.load()){
        std::this_thread::sleep_for(std::chrono::seconds(20));
        if(g_utc0_stop.load()) break;
        if(!v) continue;
        time_t now = time(nullptr);
        struct tm tm_utc; gmtime_r(&now, &tm_utc);
        if(tm_utc.tm_min != 0) continue;
        int hour_id = tm_utc.tm_yday * 24 + tm_utc.tm_hour;
        if(hour_id == g_last_rotate_hour_id) continue;
        g_last_rotate_hour_id = hour_id;
        if(tm_utc.tm_hour == 0){
            v->mission_rollover_utc0();
        } else {
            // HIST rotate만 — IDLE이면 long_waterfall.cpp::open_new_file이 알아서 skip
            LongWaterfall::request_rotate();
        }
    }
}

void start_utc0_worker(FFTViewer* v){
    if(g_utc0_thr.joinable()) return;  // 중복 시작 방지
    g_utc0_stop.store(false);
    g_utc0_thr = std::thread(utc0_worker, v);
}

void stop_utc0_worker(){
    g_utc0_stop.store(true);
    if(g_utc0_thr.joinable()) g_utc0_thr.join();
}

} // namespace Mission

// ── FFTViewer mission_* 구현 ───────────────────────────────────────────────

bool FFTViewer::mission_start(const char* name, const char* purpose,
                              const char* target, const char* started_by,
                              uint8_t op_index, bool rollover)
{
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        if(mission_state != Mission::State::IDLE) return false;

        time_t now = time(nullptr);
        struct tm tm_utc; gmtime_r(&now, &tm_utc);
        mission_year = 1900 + tm_utc.tm_year;
        auto code = Mission::make_code(mission_year, tm_utc.tm_mon, tm_utc.tm_mday);

        auto cpy = [](char* dst, size_t cap, const char* src){
            if(!src){ dst[0] = 0; return; }
            size_t n = strlen(src);
            if(n >= cap) n = cap - 1;
            memcpy(dst, src, n); dst[n] = 0;
        };
        cpy(mission_code,       sizeof(mission_code),       code.c_str());
        cpy(mission_name,       sizeof(mission_name),       name);
        cpy(mission_purpose,    sizeof(mission_purpose),    purpose);
        cpy(mission_target,     sizeof(mission_target),     target);
        cpy(mission_started_by, sizeof(mission_started_by), started_by);
        mission_op_index   = op_index;
        mission_start_utc  = now;
        mission_end_utc    = 0;
        mission_notes[0]   = 0;
        mission_state      = Mission::State::ACTIVE;

        // 디렉토리 생성
        mkdir(BEWEPaths::missions_root().c_str(), 0755);
        mkdir(BEWEPaths::missions_year_dir(mission_year).c_str(), 0755);
        mkdir(BEWEPaths::mission_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(BEWEPaths::mission_iq_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(BEWEPaths::mission_audio_dir(mission_year, mission_code).c_str(), 0755);
        mkdir(BEWEPaths::mission_hist_dir(mission_year, mission_code).c_str(), 0755);

        bewe_log_push(0, "[MISSION] start: %04d/%s '%s' by %s (rollover=%d)\n",
                      mission_year, mission_code, mission_name,
                      mission_started_by, (int)rollover);
    }

    // 락 풀린 상태에서: HIST rotate + disk/network IO (broadcast_sync 안에서 lock 재진입)
    LongWaterfall::request_rotate();
    mission_save_meta_to_disk();
    mission_broadcast_sync();
    return true;
}

bool FFTViewer::mission_end(){
    // 진행 중 녹음 stop은 별도 락 없는 함수들이라 mission_mtx 잡고 부르면
    // 데드락 위험 → 진행 중 녹음 정리는 락 풀고 수행, 그 후 다시 락 잡고 상태 전이.
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        if(mission_state != Mission::State::ACTIVE) return false;
        mission_state   = Mission::State::CLOSING;
        mission_end_utc = time(nullptr);
        bewe_log_push(0, "[MISSION] end: %04d/%s\n", mission_year, mission_code);
    }

    // HIST: 즉시 rotate → 현재 파일 finalize, 새 파일은 IDLE이라 안 만들어짐.
    LongWaterfall::request_rotate();
    // 진행 중 IQ/Audio 녹음 자동 stop (확인 모달은 UI 레이어에서 처리, 여기는 unconditional stop).
    if(rec_on.load()) stop_rec();
    for(int i = 0; i < MAX_CHANNELS; ++i){
        if(channels[i].iq_rec_on.load()) stop_iq_rec(i);
        if(channels[i].audio_rec_on.load()){
            if(remote_mode) stop_join_audio_rec(i);
            else            stop_audio_rec(i);
        }
    }

    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        // history에 snapshot 추가
        MissionEntry done{};
        done.year       = mission_year;
        memcpy(done.code,        mission_code,        sizeof(done.code));
        memcpy(done.name,        mission_name,        sizeof(done.name));
        memcpy(done.purpose,     mission_purpose,     sizeof(done.purpose));
        memcpy(done.target,      mission_target,      sizeof(done.target));
        memcpy(done.started_by,  mission_started_by,  sizeof(done.started_by));
        memcpy(done.notes,       mission_notes,       sizeof(done.notes));
        done.op_index   = mission_op_index;
        done.start_utc  = mission_start_utc;
        done.end_utc    = mission_end_utc;
        mission_history.push_back(done);

        // IDLE 전이
        mission_state         = Mission::State::IDLE;
        mission_code[0]       = 0;
        mission_year          = 0;
        mission_name[0]       = 0;
        mission_purpose[0]    = 0;
        mission_target[0]     = 0;
        mission_started_by[0] = 0;
        mission_op_index      = 0;
        mission_notes[0]      = 0;
        mission_start_utc     = 0;
        mission_end_utc       = 0;
    }
    // 락 풀린 상태에서 disk/network IO (broadcast_sync는 mission_mtx를 자체 잡음)
    mission_save_meta_to_disk();
    mission_broadcast_sync();
    return true;
}

void FFTViewer::mission_rollover_utc0(){
    // 활성 미션이 있으면 이름/목적 승계, 없으면 no-op.
    char prev_name[64], prev_purpose[128], prev_target[64], prev_started_by[32];
    uint8_t prev_op;
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        if(mission_state != Mission::State::ACTIVE) return;
        memcpy(prev_name,       mission_name,       sizeof(prev_name));
        memcpy(prev_purpose,    mission_purpose,    sizeof(prev_purpose));
        memcpy(prev_target,     mission_target,     sizeof(prev_target));
        memcpy(prev_started_by, mission_started_by, sizeof(prev_started_by));
        prev_op = mission_op_index;
    }
    mission_end();
    mission_start(prev_name, prev_purpose, prev_target, prev_started_by,
                  prev_op, /*rollover=*/true);
}

// JSON escape (간단 — 따옴표/백슬래시/제어문자만)
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

// MissionEntry → JSON object
static void mj_entry_write(std::string& out, int year, const char* code,
                           time_t start_utc, time_t end_utc,
                           const char* name, const char* purpose,
                           const char* target, const char* started_by,
                           uint8_t op_index, uint8_t rollover,
                           const char* notes, uint8_t state){
    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"state\":%u,\"op_index\":%u,\"rollover\":%u,\"year\":%d,\"code\":\"",
        (unsigned)state, (unsigned)op_index, (unsigned)rollover, year);
    out += buf;
    mj_escape(out, code);
    snprintf(buf, sizeof(buf), "\",\"start_utc\":%lld,\"end_utc\":%lld,\"name\":\"",
             (long long)start_utc, (long long)end_utc);
    out += buf;
    mj_escape(out, name);
    out += "\",\"purpose\":\""; mj_escape(out, purpose);
    out += "\",\"target\":\"";  mj_escape(out, target);
    out += "\",\"started_by\":\""; mj_escape(out, started_by);
    out += "\",\"notes\":\""; mj_escape(out, notes);
    out += "\"}";
}

void FFTViewer::mission_save_meta_to_disk(){
    // HOST local missions.json — 활성 미션 + history 모두 저장 (end_utc=0이 활성).
    // mission.info 파일 — 활성 미션의 키-밸류 메타 (iq_record.cpp의 .info 패턴 유사).
    std::lock_guard<std::mutex> lk(mission_mtx);

    // 1) missions.json
    std::string out = "{\n  \"version\": 1,\n  \"missions\": [\n";
    bool first = true;
    // 활성 미션 먼저 (있으면)
    if(mission_state == Mission::State::ACTIVE && mission_code[0]){
        if(!first) out += ",\n";
        out += "    ";
        mj_entry_write(out, mission_year, mission_code,
                       mission_start_utc, 0,
                       mission_name, mission_purpose, mission_target,
                       mission_started_by, mission_op_index, 0,
                       mission_notes, (uint8_t)Mission::State::ACTIVE);
        first = false;
    }
    // history
    for(auto& e : mission_history){
        if(!first) out += ",\n";
        out += "    ";
        mj_entry_write(out, e.year, e.code, e.start_utc, e.end_utc,
                       e.name, e.purpose, e.target, e.started_by,
                       e.op_index, e.rollover, e.notes,
                       (uint8_t)Mission::State::IDLE);
        first = false;
    }
    out += "\n  ]\n}\n";
    mkdir(BEWEPaths::missions_root().c_str(), 0755);
    FILE* fp = fopen(BEWEPaths::missions_json_path().c_str(), "w");
    if(fp){ fwrite(out.data(), 1, out.size(), fp); fclose(fp); }
    else bewe_log_push(1, "[MISSION] save missions.json failed errno=%d\n", errno);

    // 2) mission.info (활성 미션만)
    if(mission_state == Mission::State::ACTIVE && mission_code[0]){
        std::string ipath = BEWEPaths::mission_info_path(mission_year, mission_code);
        FILE* ifp = fopen(ipath.c_str(), "w");
        if(ifp){
            time_t st = mission_start_utc;
            struct tm tu; gmtime_r(&st, &tu);
            fprintf(ifp,
                "Mission Code: %04d/%s\n"
                "Name: %s\n"
                "Purpose: %s\n"
                "Target: %s\n"
                "Started By: %s\n"
                "Start UTC: %04d-%02d-%02d %02d:%02d:%02d\n"
                "End UTC: %s\n"
                "Op Index: %u\n"
                "Rollover: %u\n"
                "Notes: %s\n",
                mission_year, mission_code,
                mission_name, mission_purpose, mission_target,
                mission_started_by,
                1900+tu.tm_year, 1+tu.tm_mon, tu.tm_mday,
                tu.tm_hour, tu.tm_min, tu.tm_sec,
                mission_end_utc ? "" : "-",
                (unsigned)mission_op_index,
                (unsigned)0,
                mission_notes);
            fclose(ifp);
        }
    }
}

// 간단 JSON parser helpers
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
    // missions.json 파싱. end_utc=0인 entry 발견 시 ACTIVE 복원.
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
        uint8_t state_val = 0;
        char tmp[512];
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
                if      (!strcmp(key,"code"))       { strncpy(e.code,       tmp, sizeof(e.code)-1); }
                else if (!strcmp(key,"name"))       { strncpy(e.name,       tmp, sizeof(e.name)-1); }
                else if (!strcmp(key,"purpose"))    { strncpy(e.purpose,    tmp, sizeof(e.purpose)-1); }
                else if (!strcmp(key,"target"))     { strncpy(e.target,     tmp, sizeof(e.target)-1); }
                else if (!strcmp(key,"started_by")) { strncpy(e.started_by, tmp, sizeof(e.started_by)-1); }
                else if (!strcmp(key,"notes"))      { strncpy(e.notes,      tmp, sizeof(e.notes)-1); }
            } else {
                long long v = strtoll(p, (char**)&p, 10);
                if      (!strcmp(key,"state"))     state_val   = (uint8_t)v;
                else if (!strcmp(key,"op_index"))  e.op_index  = (uint8_t)v;
                else if (!strcmp(key,"rollover"))  e.rollover  = (uint8_t)v;
                else if (!strcmp(key,"year"))      e.year      = (int)v;
                else if (!strcmp(key,"start_utc")) e.start_utc = (time_t)v;
                else if (!strcmp(key,"end_utc"))   e.end_utc   = (time_t)v;
            }
            while(*p && (*p == ' ' || *p == '\t' || *p == '\n')) p++;
        }
        if(*p == '}') p++;
        // end_utc=0이고 아직 활성 미션 안 복원했다면 → ACTIVE 복원
        if(e.end_utc == 0 && !active_restored){
            mission_state    = Mission::State::ACTIVE;
            mission_year     = e.year;
            memcpy(mission_code,       e.code,       sizeof(mission_code));
            memcpy(mission_name,       e.name,       sizeof(mission_name));
            memcpy(mission_purpose,    e.purpose,    sizeof(mission_purpose));
            memcpy(mission_target,     e.target,     sizeof(mission_target));
            memcpy(mission_started_by, e.started_by, sizeof(mission_started_by));
            memcpy(mission_notes,      e.notes,      sizeof(mission_notes));
            mission_op_index   = e.op_index;
            mission_start_utc  = e.start_utc;
            mission_end_utc    = 0;
            active_restored = true;
            bewe_log_push(0, "[MISSION] restored ACTIVE %04d/%s '%s'\n",
                          e.year, e.code, e.name);
        } else {
            mission_history.push_back(e);
        }
        (void)state_val;
    }
    bewe_log_push(0, "[MISSION] loaded %zu history entries (active=%d)\n",
                  mission_history.size(), (int)active_restored);
}

// 한 MissionEntry → 패킷용 MissionSyncEntry
static void fill_sync_entry(MissionSyncEntry& dst,
                            const FFTViewer::MissionEntry& src,
                            uint8_t state_val){
    dst.valid    = 1;
    dst.state    = state_val;
    dst.op_index = src.op_index;
    dst.rollover = src.rollover;
    dst.year     = (uint16_t)src.year;
    memcpy(dst.code,       src.code,       sizeof(dst.code));
    dst.start_utc = (int64_t)src.start_utc;
    dst.end_utc   = (int64_t)src.end_utc;
    memcpy(dst.name,       src.name,       sizeof(dst.name));
    memcpy(dst.purpose,    src.purpose,    sizeof(dst.purpose));
    memcpy(dst.target,     src.target,     sizeof(dst.target));
    memcpy(dst.started_by, src.started_by, sizeof(dst.started_by));
    memcpy(dst.notes,      src.notes,      sizeof(dst.notes));
}

void FFTViewer::mission_broadcast_sync(){
    // HOST만 broadcast 송신. JOIN은 본 콜백 무시.
    if(!net_srv) return;

    PktMissionSync pkt{};
    {
        std::lock_guard<std::mutex> lk(mission_mtx);
        // 활성 미션
        if(mission_state == Mission::State::ACTIVE && mission_code[0]){
            pkt.active_valid = 1;
            MissionEntry act{};
            act.year       = mission_year;
            memcpy(act.code,       mission_code,       sizeof(act.code));
            memcpy(act.name,       mission_name,       sizeof(act.name));
            memcpy(act.purpose,    mission_purpose,    sizeof(act.purpose));
            memcpy(act.target,     mission_target,     sizeof(act.target));
            memcpy(act.started_by, mission_started_by, sizeof(act.started_by));
            memcpy(act.notes,      mission_notes,      sizeof(act.notes));
            act.op_index  = mission_op_index;
            act.start_utc = mission_start_utc;
            act.end_utc   = 0;
            fill_sync_entry(pkt.active, act, (uint8_t)Mission::State::ACTIVE);
        }
        // history (최신 우선, 최대 MAX_MISSION_HISTORY_PER_PKT)
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
