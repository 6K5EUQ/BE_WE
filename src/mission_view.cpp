// SIGINT Mission System — GUI viewer (자동 캡처 모델, 운영자 입력 없음).
// - draw_modal: 미션 전용 모달 (좌 트리, 우 자동 캡처 메타 + Current Session + 파일 탭)
// - draw_toast: IDLE 차단 등 일시 알림 (메인페이지 하단 중앙)
// - Start: 단순 확인 모달 → v.mission_start(login_id, 0, false)
// - End:   확인 모달 → v.mission_end()

#include "mission.hpp"
#include "fft_viewer.hpp"
#include "net_client.hpp"
#include "bewe_paths.hpp"
#include "login.hpp"
#include "kst_time.hpp"

#include <imgui.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

// long_waterfall_view.cpp의 file-scope 함수 (별도 헤더에 선언 안 됨).
bool lwf_open_file(const std::string& path);

namespace MissionView {

extern std::string g_toast_msg;
extern double      g_toast_expire;

void draw_toast(){
    auto now_d = std::chrono::steady_clock::now().time_since_epoch();
    double now = std::chrono::duration<double>(now_d).count();
    if(g_toast_msg.empty() || now >= g_toast_expire) return;
    double remain = g_toast_expire - now;
    float  alpha  = (float)std::min(1.0, remain / 0.6);
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 sz = ImGui::CalcTextSize(g_toast_msg.c_str());
    float  pad = 12.f;
    float  x0 = (io.DisplaySize.x - sz.x) * 0.5f - pad;
    float  y0 = io.DisplaySize.y * 0.72f;
    float  x1 = x0 + sz.x + pad*2;
    float  y1 = y0 + sz.y + pad;
    ImDrawList* fdl = ImGui::GetForegroundDrawList();
    fdl->AddRectFilled(ImVec2(x0,y0), ImVec2(x1,y1),
        IM_COL32(30, 30, 40, (int)(220*alpha)), 6.f);
    fdl->AddRect(ImVec2(x0,y0), ImVec2(x1,y1),
        IM_COL32(180, 140, 60, (int)(220*alpha)), 6.f, 0, 1.5f);
    fdl->AddText(ImVec2(x0+pad, y0+pad*0.5f),
        IM_COL32(255, 230, 180, (int)(255*alpha)), g_toast_msg.c_str());
}

// ── Helpers ──────────────────────────────────────────────────────────────
// 모든 시간은 KST(UTC+9) 기준.
static std::string fmt_utc(time_t t){
    if(t <= 0) return "-";
    struct tm tu; KST::to_tm(t, tu);
    char b[48];
    snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d",
        1900+tu.tm_year, 1+tu.tm_mon, tu.tm_mday,
        tu.tm_hour, tu.tm_min, tu.tm_sec);
    return b;
}
static std::string fmt_hms(int sec){
    if(sec < 0) sec = 0;
    char b[16];
    snprintf(b, sizeof(b), "%02d:%02d:%02d",
        sec/3600, (sec/60)%60, sec%60);
    return b;
}

// disk 스캔: missions/<year>/<code>/
struct DiskMission { int year; std::string code; };
static std::vector<DiskMission> scan_disk_missions(){
    std::vector<DiskMission> out;
    DIR* dr = opendir(BEWEPaths::missions_root().c_str());
    if(!dr) return out;
    struct dirent* de;
    while((de = readdir(dr)) != nullptr){
        const char* n = de->d_name;
        if(!n || n[0]=='.') continue;
        if(strlen(n) != 4) continue;
        bool num = true;
        for(int i=0;i<4;i++) if(n[i]<'0'||n[i]>'9'){ num=false; break; }
        if(!num) continue;
        int y = atoi(n);
        std::string ydir = BEWEPaths::missions_year_dir(y);
        DIR* dy = opendir(ydir.c_str());
        if(!dy) continue;
        struct dirent* de2;
        while((de2 = readdir(dy)) != nullptr){
            const char* m = de2->d_name;
            if(!m || m[0]=='.') continue;
            int mon0, mday;
            if(!Mission::parse_code(m, mon0, mday)) continue;
            out.push_back({y, m});
        }
        closedir(dy);
    }
    closedir(dr);
    std::sort(out.begin(), out.end(), [](const DiskMission& a, const DiskMission& b){
        if(a.year != b.year) return a.year > b.year;
        return a.code > b.code;
    });
    return out;
}

struct FileItem { std::string name; uint64_t size; time_t mtime; };
static std::vector<FileItem> list_dir(const std::string& dir, const char* ext){
    std::vector<FileItem> out;
    DIR* dr = opendir(dir.c_str());
    if(!dr) return out;
    struct dirent* de;
    while((de = readdir(dr)) != nullptr){
        const char* n = de->d_name;
        if(!n || n[0]=='.') continue;
        if(ext && ext[0]){
            size_t L = strlen(n), E = strlen(ext);
            if(L < E || strcmp(n + L - E, ext) != 0) continue;
        }
        std::string full = dir + "/" + n;
        struct stat st;
        if(stat(full.c_str(), &st) != 0) continue;
        if(!S_ISREG(st.st_mode)) continue;
        FileItem f; f.name = n; f.size = (uint64_t)st.st_size; f.mtime = st.st_mtime;
        out.push_back(std::move(f));
    }
    closedir(dr);
    std::sort(out.begin(), out.end(),
        [](const FileItem& a, const FileItem& b){ return a.mtime > b.mtime; });
    return out;
}

// 좌측 트리 선택
static int         g_sel_year = 0;
static std::string g_sel_code;
// Delete 확인 모달 대기 (트리/모달 어디서나 트리거 → 모달이 처리)
static int         g_del_year = 0;
static std::string g_del_code;

// ── Central archive 캐시 (NetClient on_mission_file_list 콜백이 채움) ────
struct CentralFileRow {
    char     station[64];
    uint16_t year;
    uint8_t  subdir;
    char     code[8];
    char     filename[128];
    uint64_t size_bytes;
    int64_t  mtime_unix;
};
static std::mutex                     g_cf_mtx;
static std::vector<CentralFileRow>    g_cf_rows;            // 모든 mission 합쳐서 누적
static bool                           g_cf_last_page = true;
static int                            g_cf_req_year  = 0;
static std::string                    g_cf_req_code;
static char                           g_cf_req_station[64] = {};
static double                         g_cf_last_req_time = 0.0;

// ── Download 진행 상태 (단일 동시 다운로드) ──────────────────────────────
static std::mutex   g_dl_mtx;
static FILE*        g_dl_fp = nullptr;
static std::string  g_dl_local_path;
static std::string  g_dl_filename;  // basename 비교용 — Central row 매칭
static uint64_t     g_dl_total = 0;
static uint64_t     g_dl_written = 0;
static bool         g_dl_active = false;
static double       g_dl_started_at = 0.0;
static double       g_dl_last_sample_t = 0.0;
static uint64_t     g_dl_last_sample_bytes = 0;
static double       g_dl_speed_bps = 0.0;   // EWMA bytes/sec

// ── 선택 상태 (Delete 키 처리용; 단일 선택) ──────────────────────────────
enum class SelKind : uint8_t { NONE, LOCAL, CENTRAL };
static SelKind     g_sel_kind = SelKind::NONE;
static std::string g_sel_local_path;          // LOCAL 선택 시 full path
// CENTRAL 선택 시 key
static char     g_sel_c_station[64] = {};
static uint16_t g_sel_c_year   = 0;
static uint8_t  g_sel_c_subdir = 0;
static char     g_sel_c_code[8] = {};
static char     g_sel_c_filename[128] = {};

// ── Rename submodal 상태 ─────────────────────────────────────────────────
static bool         g_rn_open = false;
static char         g_rn_station[64] = {};
static int          g_rn_year = 0;
static char         g_rn_code[8] = {};
static uint8_t      g_rn_subdir = 0;
static char         g_rn_old[128] = {};
static char         g_rn_new[128] = {};

// ── Start / End 서브모달 ────────────────────────────────────────────────
static void draw_start_submodal(FFTViewer& v, NetClient* cli){
    if(!v.mission_start_modal_open) return;
    ImGui::SetNextWindowSize(ImVec2(440, 0));
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x*0.5f - 220.f,
                                   ImGui::GetIO().DisplaySize.y*0.30f),
                            ImGuiCond_Appearing);
    ImGui::OpenPopup("Start Mission##mission_start");
    bool open = true;
    if(ImGui::BeginPopupModal("Start Mission##mission_start", &open,
        ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize)){
        time_t now = time(nullptr);
        struct tm tu; KST::to_tm(now, tu);
        auto code = Mission::make_code(1900+tu.tm_year, tu.tm_mon, tu.tm_mday);
        ImGui::TextDisabled("Mission Code:");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f,0.9f,0.6f,1.f),
            "%s  (%04d-%02d-%02d)",
            code.c_str(), 1900+tu.tm_year, 1+tu.tm_mon, tu.tm_mday);
        ImGui::Separator();
        ImGui::TextDisabled("Auto-captured metadata at start:");
        ImGui::BulletText("Station: %s", v.station_name.empty() ? "(unset)" : v.station_name.c_str());
        ImGui::BulletText("Lat/Lon: %.4f, %.4f", (double)v.station_lat, (double)v.station_lon);
        const char* sdr =
            (v.dev_blade) ? "BladeRF" :
            (v.pluto_ctx) ? "Pluto"   :
            (v.dev_rtl)   ? "RTL-SDR" : "Unknown";
        ImGui::BulletText("SDR: %s", sdr);
        ImGui::BulletText("Antenna: %s", v.host_antenna[0] ? v.host_antenna : "(unset)");
        // login_get_id가 비면 USER env / hostname 으로 폴백 (mission.cpp와 동일 정책)
        const char* sb = login_get_id();
        if(!sb || !sb[0]) sb = getenv("USER");
        if(!sb || !sb[0]) sb = getenv("LOGNAME");
        char hbuf[64] = {};
        if(!sb || !sb[0]){
            if(gethostname(hbuf, sizeof(hbuf)-1) == 0) sb = hbuf;
        }
        if(!sb || !sb[0]) sb = "unknown";
        ImGui::BulletText("Started by: %s", sb);
        ImGui::Spacing();
        if(ImGui::Button("Cancel", ImVec2(120, 0))){
            v.mission_start_modal_open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.55f, 0.25f, 1.f));
        if(ImGui::Button("Start", ImVec2(120, 0))){
            if(cli) cli->send_mission_start();
            else    v.mission_start(login_get_id(), /*op_index=*/0, /*rollover=*/false);
            v.mission_start_modal_open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor();
        ImGui::EndPopup();
    }
    if(!open) v.mission_start_modal_open = false;
}

static void draw_end_confirm_submodal(FFTViewer& v, NetClient* cli){
    if(!v.mission_end_confirm_open) return;
    ImGui::SetNextWindowSize(ImVec2(460, 0));
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x*0.5f - 230.f,
                                   ImGui::GetIO().DisplaySize.y*0.30f),
                            ImGuiCond_Appearing);
    ImGui::OpenPopup("End Mission##mission_end");
    bool open = true;
    if(ImGui::BeginPopupModal("End Mission##mission_end", &open,
        ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize)){
        char codebuf[8] = {}; int yr = 0;
        {
            std::lock_guard<std::mutex> lk(v.mission_mtx);
            memcpy(codebuf, v.mission_code, sizeof(codebuf));
            yr = v.mission_year;
        }
        ImGui::Text("End mission %04d/%s?", yr, codebuf);
        ImGui::Separator();
        ImGui::TextDisabled("Active recordings will be stopped:");
        bool any = false;
        if(v.rec_on.load()){
            ImGui::BulletText("Main IQ REC");
            any = true;
        }
        for(int i = 0; i < MAX_CHANNELS; ++i){
            if(v.channels[i].iq_rec_on.load()){
                ImGui::BulletText("IQ ch%d", i);
                any = true;
            }
            if(v.channels[i].audio_rec_on.load()){
                ImGui::BulletText("Audio ch%d", i);
                any = true;
            }
        }
        ImGui::BulletText("HIST file finalize");
        if(!any) ImGui::TextDisabled("  (no active recordings)");
        ImGui::Spacing();
        if(ImGui::Button("Cancel", ImVec2(120, 0))){
            v.mission_end_confirm_open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.20f, 0.20f, 1.f));
        if(ImGui::Button("End", ImVec2(120, 0))){
            if(cli) cli->send_mission_end();
            else    v.mission_end();
            v.mission_end_confirm_open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor();
        ImGui::EndPopup();
    }
    if(!open) v.mission_end_confirm_open = false;
}

// ── Meta block (자동 캡처 필드만 표시) ──────────────────────────────────
static void draw_meta_block(FFTViewer& v){
    bool   is_active = false;
    int    year = 0;
    char   code[8] = {}, station[64] = {}, host[32] = {},
           started_by[32] = {}, sdr_kind[24] = {}, antenna[64] = {};
    float  lat = 0.f, lon = 0.f;
    time_t start_utc = 0, end_utc = 0;
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        if(v.mission_state == Mission::State::ACTIVE && v.mission_code[0] &&
           (g_sel_code.empty() ||
            (g_sel_year == v.mission_year && g_sel_code == v.mission_code))){
            is_active = true;
            year = v.mission_year;
            memcpy(code,        v.mission_code,         sizeof(code));
            memcpy(station,     v.mission_station_name, sizeof(station));
            memcpy(host,        v.mission_host_name,    sizeof(host));
            memcpy(started_by,  v.mission_started_by,   sizeof(started_by));
            memcpy(sdr_kind,    v.mission_sdr_kind,     sizeof(sdr_kind));
            memcpy(antenna,     v.mission_antenna,      sizeof(antenna));
            lat = v.mission_lat;
            lon = v.mission_lon;
            start_utc = v.mission_start_utc;
        } else {
            for(auto& e : v.mission_history){
                if(e.year == g_sel_year && g_sel_code == e.code){
                    year = e.year;
                    memcpy(code,        e.code,         sizeof(code));
                    memcpy(station,     e.station_name, sizeof(station));
                    memcpy(host,        e.host_name,    sizeof(host));
                    memcpy(started_by,  e.started_by,   sizeof(started_by));
                    memcpy(sdr_kind,    e.sdr_kind,     sizeof(sdr_kind));
                    memcpy(antenna,     e.antenna,      sizeof(antenna));
                    lat = e.lat; lon = e.lon;
                    start_utc = e.start_utc;
                    end_utc   = e.end_utc;
                    break;
                }
            }
        }
    }
    if(year == 0){
        ImGui::TextDisabled("(no mission selected)");
        return;
    }
    ImGui::PushStyleColor(ImGuiCol_Text, is_active
        ? ImVec4(0.55f, 0.90f, 0.65f, 1.f)
        : ImVec4(0.70f, 0.75f, 0.85f, 1.f));
    ImGui::Text("Mission %04d/%s%s", year, code, is_active ? "  (ACTIVE)" : "");
    ImGui::PopStyleColor();
    ImGui::Separator();
    const float L = 110.f;
    auto row = [&](const char* k, const char* val){
        ImGui::TextDisabled("%s", k); ImGui::SameLine(L);
        ImGui::TextWrapped("%s", (val && val[0]) ? val : "-");
    };
    row("Status:",  is_active ? "ACTIVE" : "CLOSED");
    row("Started:", fmt_utc(start_utc).c_str());
    if(!is_active) row("Ended:", fmt_utc(end_utc).c_str());
    row("By:",      started_by);
    row("Station:", station);
    row("Host:",    host);
    // host의 좌표 그대로 N/S/E/W 단위 표기 (long_waterfall.hpp 공용 헬퍼).
    std::string latlon = LongWaterfall::fmt_lat_lon(lat, lon);
    row("Lat/Lon:", latlon.c_str());
    row("SDR:",     sdr_kind);
    row("Antenna:", antenna);
}

static void draw_current_session(FFTViewer& v){
    bool show = false;
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        show = (v.mission_state == Mission::State::ACTIVE &&
                g_sel_year == v.mission_year &&
                (g_sel_code.empty() || g_sel_code == v.mission_code));
    }
    if(!show) return;
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.6f,0.85f,1.f,1.f), "Current Session");
    ImGui::Separator();
    bool any = false;
    if(v.rec_on.load()){
        float el = v.rec_sr > 0 ? (float)v.rec_frames.load()/(float)v.rec_sr : 0.f;
        uint64_t fr = v.rec_frames.load();
        float mb = (float)(fr*4) / 1048576.0f;
        ImGui::TextColored(ImVec4(1.0f,0.4f,0.4f,1.f),
            "[REC] main IQ  %s  %.1f MB", fmt_hms((int)el).c_str(), mb);
        any = true;
    }
    for(int i = 0; i < MAX_CHANNELS; ++i){
        Channel& ch = v.channels[i];
        if(ch.iq_rec_on.load()){
            uint64_t fr = ch.iq_rec_frames;
            uint32_t sr = ch.iq_rec_sr ? ch.iq_rec_sr : 1;
            int sec = (int)(fr / sr);
            float mb = (float)(fr*8) / 1048576.0f;
            ImGui::TextColored(ImVec4(1.0f,0.55f,0.35f,1.f),
                "[IQ ch%d] %s  %.1f MB", i, fmt_hms(sec).c_str(), mb);
            any = true;
        }
        if(ch.audio_rec_on.load()){
            uint64_t fr = ch.audio_rec_frames;
            uint32_t sr = ch.audio_rec_sr ? ch.audio_rec_sr : 1;
            int sec = (int)(fr / sr);
            float mb = (float)(fr*2) / 1048576.0f;
            ImGui::TextColored(ImVec4(1.0f,0.8f,0.4f,1.f),
                "[Audio ch%d] %s  %.1f MB", i, fmt_hms(sec).c_str(), mb);
            any = true;
        }
    }
    if(v.spectrum_pause.load()){
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.0f, 1.f),
            "[PAUSED]  spectrum frozen");
        any = true;
    }
    {
        std::lock_guard<std::mutex> lk(v.sched_mtx);
        if(v.sched_active_idx >= 0){
            auto& se = v.sched_entries[v.sched_active_idx];
            float el = (float)std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - se.rec_started).count();
            float remain = se.duration_sec - el;
            if(remain < 0) remain = 0;
            ImGui::TextColored(ImVec4(1.0f,0.45f,0.45f,1.f),
                "[SCHED REC] %.3f MHz  remain %.0fs", se.freq_mhz, remain);
            any = true;
        }
        int waiting = 0;
        for(auto& se : v.sched_entries)
            if(se.status == FFTViewer::SchedEntry::WAITING) waiting++;
        if(waiting > 0){
            ImGui::TextColored(ImVec4(0.85f,0.85f,0.45f,1.f),
                "[SCHED] %d waiting", waiting);
            any = true;
        }
    }
    if(!any) ImGui::TextDisabled("  (idle)");
}

// ── Helper: send LIST_REQ if mission selection changed or auto-refresh due ──
static void maybe_request_central_list(FFTViewer& v, NetClient* cli){
    if(!cli) return;
    if(g_sel_year == 0 || g_sel_code.empty()) return;
    // station 결정: active mission 의 station_name (or history entry's station)
    char station[64] = {};
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        bool found = false;
        if(v.mission_state == Mission::State::ACTIVE &&
           v.mission_year == g_sel_year &&
           strncmp(v.mission_code, g_sel_code.c_str(), 8) == 0){
            strncpy(station, v.mission_station_name, sizeof(station) - 1);
            found = true;
        }
        if(!found){
            for(auto& e : v.mission_history){
                if(e.year == g_sel_year && strncmp(e.code, g_sel_code.c_str(), 8) == 0){
                    strncpy(station, e.station_name, sizeof(station) - 1);
                    found = true;
                    break;
                }
            }
        }
    }
    bool sel_changed = (g_cf_req_year != g_sel_year) ||
                       (g_cf_req_code != g_sel_code) ||
                       (strncmp(g_cf_req_station, station, 64) != 0);
    double now = ImGui::GetTime();
    bool refresh_due = (now - g_cf_last_req_time) > 5.0;  // 5초마다 refresh
    if(sel_changed || refresh_due){
        {
            std::lock_guard<std::mutex> lk(g_cf_mtx);
            // 같은 mission 만 비워 (다른 mission rows 는 유지해도 무방하나 단순화)
            if(sel_changed) g_cf_rows.clear();
            g_cf_last_page = false;
        }
        g_cf_req_year = g_sel_year;
        g_cf_req_code = g_sel_code;
        strncpy(g_cf_req_station, station, sizeof(g_cf_req_station) - 1);
        g_cf_last_req_time = now;
        cli->send_mission_file_list_req(station, (uint16_t)g_sel_year,
                                        g_sel_code.c_str(), /*subdir=*/0);
    }
}

static const char* subdir_label(uint8_t s){
    switch(s){
        case MFS_IQ:    return "IQ";
        case MFS_AUDIO: return "DEMOD";
        case MFS_HIST:  return "HIST";
        default:        return "?";
    }
}

// LOCAL 다운로드 파일을 IQ/DEMOD/HIST 로 분류.
// .bewehist        → HIST
// "Audio_..."/"SCHED_AUDIO_..."  → DEMOD
// 나머지 (.wav 포함) → IQ
static uint8_t classify_local_file(const std::string& name){
    size_t n = name.size();
    if(n >= 9 && name.compare(n - 9, 9, ".bewehist") == 0) return MFS_HIST;
    if(name.rfind("Audio_", 0) == 0 ||
       name.rfind("SCHED_AUDIO_", 0) == 0 ||
       name.rfind("Demod_", 0) == 0) return MFS_AUDIO;
    return MFS_IQ;
}

// .info sidecar 인지.
static bool is_info_file(const std::string& name){
    size_t n = name.size();
    return n >= 5 && name.compare(n - 5, 5, ".info") == 0;
}

// 선택 초기화.
static void clear_selection(){
    g_sel_kind = SelKind::NONE;
    g_sel_local_path.clear();
    memset(g_sel_c_station,  0, sizeof(g_sel_c_station));
    memset(g_sel_c_code,     0, sizeof(g_sel_c_code));
    memset(g_sel_c_filename, 0, sizeof(g_sel_c_filename));
    g_sel_c_year   = 0;
    g_sel_c_subdir = 0;
}

// Delete 키 → 선택된 파일 삭제. 미션 모달 활성 시 매 프레임 호출.
static void process_delete_key(NetClient* cli){
    if(g_sel_kind == SelKind::NONE) return;
    if(!ImGui::IsKeyPressed(ImGuiKey_Delete, false)) return;
    if(g_sel_kind == SelKind::LOCAL){
        unlink(g_sel_local_path.c_str());
        unlink((g_sel_local_path + ".info").c_str());
        MissionView::show_toast("Local file deleted");
    } else if(g_sel_kind == SelKind::CENTRAL && cli){
        MissionFileKey k{};
        strncpy(k.station,  g_sel_c_station,  sizeof(k.station)  - 1);
        k.year   = g_sel_c_year;
        k.subdir = g_sel_c_subdir;
        strncpy(k.code,     g_sel_c_code,     sizeof(k.code)     - 1);
        strncpy(k.filename, g_sel_c_filename, sizeof(k.filename) - 1);
        cli->send_mission_file_delete(k);
        g_cf_last_req_time = 0;
        MissionView::show_toast("Central file delete requested");
    }
    clear_selection();
}

// 다운로드 진행률 폴링 (UI 스레드 — main).
static void poll_dl_speed(){
    if(!g_dl_active) return;
    double now = ImGui::GetTime();
    double dt = now - g_dl_last_sample_t;
    if(dt < 0.2) return;
    uint64_t bytes_now;
    {
        std::lock_guard<std::mutex> lk(g_dl_mtx);
        bytes_now = g_dl_written;
    }
    double inst = dt > 0 ? (double)(bytes_now - g_dl_last_sample_bytes) / dt : 0.0;
    // EWMA (반응성 + 평활)
    g_dl_speed_bps = (g_dl_speed_bps <= 0.0) ? inst : (g_dl_speed_bps * 0.5 + inst * 0.5);
    g_dl_last_sample_t = now;
    g_dl_last_sample_bytes = bytes_now;
}

static std::string fmt_size(uint64_t bytes){
    char b[32];
    if(bytes >= 1ull << 30) snprintf(b, sizeof(b), "%.2f GB", bytes / (double)(1ull<<30));
    else if(bytes >= 1ull << 20) snprintf(b, sizeof(b), "%.1f MB", bytes / (double)(1ull<<20));
    else if(bytes >= 1ull << 10) snprintf(b, sizeof(b), "%.1f KB", bytes / (double)(1ull<<10));
    else snprintf(b, sizeof(b), "%llu B", (unsigned long long)bytes);
    return b;
}

// Start download: open file at downloads_dir/<filename> and send DL_REQ.
// Returns true if request was sent.
static bool start_download(NetClient* cli, const CentralFileRow& row){
    if(!cli) return false;
    std::lock_guard<std::mutex> lk(g_dl_mtx);
    if(g_dl_active){
        MissionView::show_toast("Download already in progress");
        return false;
    }
    std::string dir = BEWEPaths::downloads_dir();
    mkdir(BEWEPaths::data_dir().c_str(), 0755);
    mkdir(dir.c_str(), 0755);
    std::string path = dir + "/" + row.filename;
    g_dl_fp = fopen(path.c_str(), "wb");
    if(!g_dl_fp){
        char msg[256]; snprintf(msg, sizeof(msg),
            "Download open failed: %s (errno=%d)", path.c_str(), errno);
        MissionView::show_toast(msg);
        return false;
    }
    g_dl_local_path = path;
    g_dl_filename   = row.filename;
    g_dl_total      = 0;
    g_dl_written    = 0;
    g_dl_active     = true;
    g_dl_started_at      = ImGui::GetTime();
    g_dl_last_sample_t   = g_dl_started_at;
    g_dl_last_sample_bytes = 0;
    g_dl_speed_bps       = 0.0;
    MissionFileKey k{};
    strncpy(k.station,  row.station, sizeof(k.station) - 1);
    k.year   = row.year;
    k.subdir = row.subdir;
    strncpy(k.code,     row.code,     sizeof(k.code) - 1);
    strncpy(k.filename, row.filename, sizeof(k.filename) - 1);
    bool ok = cli->send_mission_file_dl_req(k);
    if(!ok){
        if(g_dl_fp){ fclose(g_dl_fp); g_dl_fp = nullptr; }
        unlink(g_dl_local_path.c_str());
        g_dl_active = false;
        MissionView::show_toast("Download request failed");
    }
    return ok;
}

// Right-click context for a Central-side file row.
static void central_context_menu(NetClient* cli, const CentralFileRow& row){
    if(ImGui::BeginPopupContextItem("##cf_ctx")){
        ImGui::TextDisabled("%s/%s", subdir_label(row.subdir), row.filename);
        ImGui::Separator();
        if(ImGui::MenuItem("Download to LOCAL")){
            start_download(cli, row);
        }
        if(ImGui::MenuItem("Rename...")){
            memset(g_rn_station, 0, sizeof(g_rn_station));
            memset(g_rn_code,    0, sizeof(g_rn_code));
            memset(g_rn_old,     0, sizeof(g_rn_old));
            memset(g_rn_new,     0, sizeof(g_rn_new));
            strncpy(g_rn_station, row.station,  sizeof(g_rn_station) - 1);
            g_rn_year = row.year;
            strncpy(g_rn_code,    row.code,     sizeof(g_rn_code) - 1);
            g_rn_subdir = row.subdir;
            strncpy(g_rn_old,     row.filename, sizeof(g_rn_old) - 1);
            strncpy(g_rn_new,     row.filename, sizeof(g_rn_new) - 1);
            g_rn_open = true;
        }
        ImGui::Separator();
        if(ImGui::MenuItem("Delete (Central)")){
            MissionFileKey k{};
            strncpy(k.station,  row.station,  sizeof(k.station)  - 1);
            k.year   = row.year;
            k.subdir = row.subdir;
            strncpy(k.code,     row.code,     sizeof(k.code)     - 1);
            strncpy(k.filename, row.filename, sizeof(k.filename) - 1);
            if(cli) cli->send_mission_file_delete(k);
            g_cf_last_req_time = 0;  // force refresh
            MissionView::show_toast("Delete requested");
        }
        ImGui::EndPopup();
    }
}

// 다운로드한 LOCAL 파일을 viewer로 열기 (HIST → LWF viewer, .wav → SA panel).
// lwf_open_file는 long_waterfall_view.cpp 의 file-scope 자유 함수 — :: 으로 호출.
static void open_local_in_viewer(FFTViewer& v, const std::string& path){
    auto ends_with = [&](const char* ext){
        size_t L = path.size(), E = strlen(ext);
        return L >= E && path.compare(L - E, E, ext) == 0;
    };
    if(ends_with(".bewehist") || ends_with(".bewewf")){
        if(::lwf_open_file(path)){
            v.lwf_modal_open = true;
            v.mission_modal_open = false;  // 미션 모달 닫고 viewer 보이게
        } else {
            MissionView::show_toast("LWF open failed");
        }
        return;
    }
    if(ends_with(".wav")){
        v.sa_temp_path = path;
        v.eid_panel_open = true;
        v.mission_modal_open = false;
        return;
    }
    MissionView::show_toast("No viewer for this file type");
}

// LOCAL row 우클릭 — 메인 페이지의 file_ctx 메뉴 (Signal Analysis / Info /
// Report / Save DB / Delete) 를 동일하게 띄우도록 v.pending_file_ctx 에 신호.
// ui.cpp run_streaming_viewer() 가 다음 프레임에 file_ctx.open=true 로 옮긴다.
static void local_request_main_ctx(FFTViewer& v, const std::string& full_path,
                                   const std::string& name){
    if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
        ImVec2 mp = ImGui::GetMousePos();
        v.pending_file_ctx.filepath = full_path;
        v.pending_file_ctx.filename = name;
        v.pending_file_ctx.x = mp.x;
        v.pending_file_ctx.y = mp.y;
        v.pending_file_ctx.pending.store(true);
    }
}

static void draw_central_list(FFTViewer& v, NetClient* cli, uint8_t subdir){
    maybe_request_central_list(v, cli);
    std::vector<CentralFileRow> rows;
    {
        std::lock_guard<std::mutex> lk(g_cf_mtx);
        for(auto& r : g_cf_rows){
            if(r.subdir != subdir) continue;
            if(r.year != g_sel_year) continue;
            if(strncmp(r.code, g_sel_code.c_str(), 8) != 0) continue;
            rows.push_back(r);
        }
    }
    // 최신순 정렬 (mtime DESC) — 주파수 무관, 최신 파일이 위로.
    std::sort(rows.begin(), rows.end(),
        [](const CentralFileRow& a, const CentralFileRow& b){
            return a.mtime_unix > b.mtime_unix;
        });
    ImGui::TextDisabled("Central: %s/%04d/%s/%s",
        g_cf_req_station, g_sel_year, g_sel_code.c_str(), subdir_label(subdir));
    // Refresh 버튼 우측 정렬 (현재 child 영역의 오른쪽 끝에 배치)
    {
        const char* btn_label = "Refresh";
        ImVec2 btn_size = ImGui::CalcTextSize(btn_label);
        btn_size.x += ImGui::GetStyle().FramePadding.x * 2.f;
        ImGui::SameLine(ImGui::GetContentRegionMax().x - btn_size.x);
        if(ImGui::SmallButton("Refresh##cf")){
            g_cf_last_req_time = 0;
        }
    }
    ImGui::Separator();
    if(!cli){
        ImGui::TextColored(ImVec4(0.85f, 0.7f, 0.4f, 1.f),
            "Not connected to Central (LOCAL mode) — see LOCAL tab.");
        return;
    }
    if(rows.empty()){
        ImGui::TextDisabled("  (no files in Central archive)");
        return;
    }
    poll_dl_speed();
    std::string dl_dir = BEWEPaths::downloads_dir();
    ImGui::BeginChild("##cf_scroll", ImVec2(0, 0), false);
    for(auto& r : rows){
        float pw = ImGui::GetContentRegionAvail().x;
        ImGui::PushID(r.filename);

        std::string dl_path = dl_dir + "/" + r.filename;
        bool already_dl = (access(dl_path.c_str(), F_OK) == 0);
        bool downloading = false;
        double frac = 0.0;
        uint64_t dl_total_now = 0, dl_written_now = 0;
        {
            std::lock_guard<std::mutex> lk(g_dl_mtx);
            if(g_dl_active && g_dl_filename == r.filename){
                downloading = true;
                dl_total_now = g_dl_total;
                dl_written_now = g_dl_written;
                if(g_dl_total > 0) frac = (double)g_dl_written / (double)g_dl_total;
            }
        }

        bool sel = (g_sel_kind == SelKind::CENTRAL &&
                    g_sel_c_year == r.year && g_sel_c_subdir == r.subdir &&
                    strncmp(g_sel_c_code,     r.code,     sizeof(g_sel_c_code))     == 0 &&
                    strncmp(g_sel_c_filename, r.filename, sizeof(g_sel_c_filename)) == 0);

        ImVec4 text_col = already_dl
            ? ImVec4(0.35f, 0.95f, 0.45f, 1.f)   // 다운로드 완료 → 초록
            : ImGui::GetStyleColorVec4(ImGuiCol_Text);
        ImGui::PushStyleColor(ImGuiCol_Text, text_col);
        bool clicked = ImGui::Selectable(r.filename, sel,
            ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick,
            ImVec2(pw, 0));
        ImGui::PopStyleColor();

        if(clicked){
            // 단일 클릭 = 선택; 더블클릭 = open/download
            g_sel_kind = SelKind::CENTRAL;
            strncpy(g_sel_c_station,  r.station,  sizeof(g_sel_c_station)  - 1);
            g_sel_c_year   = r.year;
            g_sel_c_subdir = r.subdir;
            strncpy(g_sel_c_code,     r.code,     sizeof(g_sel_c_code)     - 1);
            strncpy(g_sel_c_filename, r.filename, sizeof(g_sel_c_filename) - 1);
        }
        if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
            if(already_dl) open_local_in_viewer(v, dl_path);
            else           start_download(cli, r);
        }
        central_context_menu(cli, r);

        // 우측 표시: 다운로드 중이면 진행률+속도, 아니면 파일 크기
        char info[80];
        if(downloading){
            double speed_mb = g_dl_speed_bps / 1048576.0;
            if(dl_total_now > 0){
                snprintf(info, sizeof(info), "%.0f%%  %.1fMB/s",
                         frac * 100.0, speed_mb);
            } else {
                snprintf(info, sizeof(info), "%.1fMB  %.1fMB/s",
                         dl_written_now / 1048576.0, speed_mb);
            }
        } else {
            std::string s = fmt_size(r.size_bytes);
            snprintf(info, sizeof(info), "%s", s.c_str());
        }
        float tw = ImGui::CalcTextSize(info).x;
        ImGui::SameLine(pw - tw - 4.f);
        ImVec4 info_col = downloading
            ? ImVec4(0.4f, 0.85f, 1.0f, 1.f)
            : ImVec4(0.6f, 0.6f, 0.6f, 1.f);
        ImGui::TextColored(info_col, "%s", info);
        ImGui::PopID();
    }
    ImGui::EndChild();
}

static void draw_local_list(FFTViewer& v){
    std::string dir = BEWEPaths::downloads_dir();
    mkdir(BEWEPaths::data_dir().c_str(), 0755);
    mkdir(dir.c_str(), 0755);
    auto items = list_dir(dir, nullptr);
    // .info 사이드카 제외 (데이터 파일에 종속)
    items.erase(std::remove_if(items.begin(), items.end(),
        [](const FileItem& f){ return is_info_file(f.name); }),
        items.end());

    ImGui::TextDisabled("Local downloads: %s", dir.c_str());
    ImGui::Separator();
    if(items.empty()){
        ImGui::TextDisabled("  (no downloads yet)");
        return;
    }

    // IQ / DEMOD / HIST 분류 (Central 탭과 동일한 라벨)
    std::vector<const FileItem*> by[3] = {{}, {}, {}};  // 0=IQ 1=DEMOD 2=HIST
    for(auto& it : items){
        uint8_t s = classify_local_file(it.name);
        int slot = (s == MFS_IQ) ? 0 : (s == MFS_AUDIO ? 1 : 2);
        by[slot].push_back(&it);
    }
    const char* labels[3]  = {"IQ", "DEMOD", "HIST"};
    const uint8_t subs[3]  = {MFS_IQ, MFS_AUDIO, MFS_HIST};

    ImGui::BeginChild("##loc_scroll", ImVec2(0, 0), false);
    for(int s = 0; s < 3; s++){
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.85f, 1.0f, 1.f));
        ImGui::TextUnformatted(labels[s]);
        ImGui::PopStyleColor();
        ImGui::Separator();
        if(by[s].empty()){
            ImGui::TextDisabled("  (none)");
            ImGui::Spacing();
            continue;
        }
        (void)subs;  // 분류 ID 는 시각적 그룹핑에만 사용
        for(const FileItem* itp : by[s]){
            const FileItem& it = *itp;
            float pw = ImGui::GetContentRegionAvail().x;
            std::string full = dir + "/" + it.name;
            ImGui::PushID(it.name.c_str());

            bool sel = (g_sel_kind == SelKind::LOCAL && g_sel_local_path == full);
            bool clicked = ImGui::Selectable(it.name.c_str(), sel,
                ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick,
                ImVec2(pw, 0));
            if(clicked){
                g_sel_kind = SelKind::LOCAL;
                g_sel_local_path = full;
            }
            if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                open_local_in_viewer(v, full);
            }
            local_request_main_ctx(v, full, it.name);
            std::string info = fmt_size(it.size);
            float tw = ImGui::CalcTextSize(info.c_str()).x;
            ImGui::SameLine(pw - tw - 4.f);
            ImGui::TextDisabled("%s", info.c_str());
            ImGui::PopID();
        }
        ImGui::Spacing();
    }
    ImGui::EndChild();
}

static void draw_rename_submodal(NetClient* cli){
    if(!g_rn_open) return;
    ImGui::SetNextWindowSize(ImVec2(480, 0));
    ImGui::OpenPopup("Rename##cf_rn");
    bool open = true;
    if(ImGui::BeginPopupModal("Rename##cf_rn", &open,
        ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize)){
        ImGui::TextDisabled("Central archive rename");
        ImGui::Text("%s/%04d/%s/%s", g_rn_station, g_rn_year, g_rn_code,
                    subdir_label(g_rn_subdir));
        ImGui::TextDisabled("Old:");
        ImGui::SameLine(); ImGui::TextWrapped("%s", g_rn_old);
        ImGui::PushItemWidth(-1);
        ImGui::InputText("##rn_new", g_rn_new, sizeof(g_rn_new));
        ImGui::PopItemWidth();
        ImGui::Spacing();
        if(ImGui::Button("Cancel", ImVec2(120, 0))){
            g_rn_open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        bool empty_or_same = (g_rn_new[0] == 0) || (strcmp(g_rn_new, g_rn_old) == 0);
        if(empty_or_same) ImGui::BeginDisabled();
        if(ImGui::Button("Rename", ImVec2(120, 0))){
            if(cli){
                MissionFileKey k{};
                strncpy(k.station, g_rn_station, sizeof(k.station) - 1);
                k.year = (uint16_t)g_rn_year;
                k.subdir = g_rn_subdir;
                strncpy(k.code,     g_rn_code, sizeof(k.code)     - 1);
                strncpy(k.filename, g_rn_old,  sizeof(k.filename) - 1);
                cli->send_mission_file_rename(k, g_rn_new);
                g_cf_last_req_time = 0;
                MissionView::show_toast("Rename requested");
            }
            g_rn_open = false;
            ImGui::CloseCurrentPopup();
        }
        if(empty_or_same) ImGui::EndDisabled();
        ImGui::EndPopup();
    }
    if(!open) g_rn_open = false;
}

static void draw_file_tabs(FFTViewer& v, NetClient* cli){
    if(g_sel_year == 0 || g_sel_code.empty()){
        // mission 선택 없음 — LOCAL 만 표시
        ImGui::Spacing();
        if(ImGui::BeginTabBar("##mission_file_tabs_idle")){
            if(ImGui::BeginTabItem("LOCAL")){
                draw_local_list(v);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        return;
    }
    ImGui::Spacing();
    if(ImGui::BeginTabBar("##mission_file_tabs")){
        if(ImGui::BeginTabItem("IQ")){
            draw_central_list(v, cli, MFS_IQ);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("DEMOD")){
            draw_central_list(v, cli, MFS_AUDIO);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("HIST")){
            draw_central_list(v, cli, MFS_HIST);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("LOCAL")){
            draw_local_list(v);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

static void draw_left_tree(FFTViewer& v){
    if(ImGui::Button("[+] Start Mission", ImVec2(-1, 0))){
        v.mission_start_modal_open = true;
    }
    ImGui::Separator();

    int act_year = 0;
    std::string act_code;
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        if(v.mission_state == Mission::State::ACTIVE && v.mission_code[0]){
            act_year = v.mission_year;
            act_code = v.mission_code;
        }
    }

    auto disk = scan_disk_missions();
    if(act_year > 0){
        bool found = false;
        for(auto& d : disk) if(d.year == act_year && d.code == act_code){ found = true; break; }
        if(!found) disk.insert(disk.begin(), {act_year, act_code});
    }
    std::map<int, std::vector<std::string>, std::greater<int>> by_year;
    for(auto& d : disk) by_year[d.year].push_back(d.code);
    for(auto& kv : by_year)
        std::sort(kv.second.begin(), kv.second.end(), std::greater<std::string>());

    if(by_year.empty()){
        ImGui::TextDisabled("  (no missions)");
        return;
    }
    for(auto& kv : by_year){
        int y = kv.first;
        char hdr[16]; snprintf(hdr, sizeof(hdr), "%04d##yr", y);
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if(!ImGui::TreeNode(hdr)) continue;
        for(auto& code : kv.second){
            bool is_active = (y == act_year && code == act_code);
            bool sel = (g_sel_year == y && g_sel_code == code);
            ImGui::PushID(code.c_str());
            ImGui::PushID(y);
            char label[24];
            if(is_active) snprintf(label, sizeof(label), "[*] %s", code.c_str());
            else          snprintf(label, sizeof(label), "    %s", code.c_str());
            ImVec4 col = is_active
                ? ImVec4(0.55f, 0.90f, 0.65f, 1.f)
                : ImVec4(0.75f, 0.80f, 0.90f, 1.f);
            ImGui::PushStyleColor(ImGuiCol_Text, col);
            if(ImGui::Selectable(label, sel)){
                g_sel_year = y; g_sel_code = code;
            }
            // 우클릭 context menu: Delete Mission
            if(ImGui::BeginPopupContextItem("##mission_ctx")){
                if(is_active) ImGui::TextDisabled("Mission %s (ACTIVE)", code.c_str());
                else          ImGui::TextDisabled("Mission %s", code.c_str());
                ImGui::Separator();
                if(ImGui::MenuItem("Delete Mission...")){
                    g_del_year = y;
                    g_del_code = code;
                }
                ImGui::EndPopup();
            }
            ImGui::PopStyleColor();
            ImGui::PopID();
            ImGui::PopID();
        }
        ImGui::TreePop();
    }
}

// ── Delete 확인 모달 ────────────────────────────────────────────────────
static void draw_delete_confirm_submodal(FFTViewer& v, NetClient* cli){
    if(g_del_year == 0 || g_del_code.empty()) return;
    ImGui::SetNextWindowSize(ImVec2(480, 0));
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x*0.5f - 240.f,
                                   ImGui::GetIO().DisplaySize.y*0.30f),
                            ImGuiCond_Appearing);
    ImGui::OpenPopup("Delete Mission##mission_del");
    bool open = true;
    if(ImGui::BeginPopupModal("Delete Mission##mission_del", &open,
        ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize)){
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.f),
            "PERMANENTLY DELETE mission %04d/%s ?",
            g_del_year, g_del_code.c_str());
        ImGui::Separator();
        std::string mdir = BEWEPaths::mission_dir(g_del_year, g_del_code);
        ImGui::TextDisabled("Directory:");
        ImGui::SameLine();
        ImGui::TextWrapped("%s", mdir.c_str());
        ImGui::Spacing();
        ImGui::TextDisabled("All files will be removed:");
        ImGui::BulletText("IQ recordings (iq/*.wav)");
        ImGui::BulletText("DEMOD recordings (audio/*.wav)");
        ImGui::BulletText("HIST waterfalls (hist/*.bewehist)");
        ImGui::BulletText("mission.info + .info sidecars");
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.f),
            "This action cannot be undone.");
        ImGui::Spacing();
        if(ImGui::Button("Cancel", ImVec2(120, 0))){
            g_del_year = 0; g_del_code.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.2f, 0.2f, 1.f));
        if(ImGui::Button("Delete", ImVec2(120, 0))){
            if(cli) cli->send_mission_delete(g_del_year, g_del_code.c_str());
            else    v.mission_delete(g_del_year, g_del_code.c_str());
            // 선택 미션이 삭제 대상이었으면 좌측 트리 선택 해제
            if(g_sel_year == g_del_year && g_sel_code == g_del_code){
                g_sel_year = 0; g_sel_code.clear();
            }
            g_del_year = 0; g_del_code.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::PopStyleColor();
        ImGui::EndPopup();
    }
    if(!open){ g_del_year = 0; g_del_code.clear(); }
}

void draw_modal(FFTViewer& v, NetClient* cli){
    if(!v.mission_modal_open) return;
    ImGuiIO& io = ImGui::GetIO();
    float W = io.DisplaySize.x * 0.78f;
    float H = io.DisplaySize.y * 0.78f;
    if(W < 700) W = 700;
    if(H < 460) H = 460;
    ImGui::SetNextWindowSize(ImVec2(W, H), ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2((io.DisplaySize.x - W)*0.5f,
                                   (io.DisplaySize.y - H)*0.5f),
                            ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.96f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.08f, 0.14f, 1.f));
    if(ImGui::Begin("MISSION##mission_modal", &v.mission_modal_open,
        ImGuiWindowFlags_NoCollapse)){

        int hdr_year = 0; char hdr_code[8] = {};
        bool hdr_active = false;
        {
            std::lock_guard<std::mutex> lk(v.mission_mtx);
            hdr_active = (v.mission_state == Mission::State::ACTIVE && v.mission_code[0]);
            hdr_year = v.mission_year;
            memcpy(hdr_code, v.mission_code, sizeof(hdr_code));
        }

        if(hdr_active){
            ImGui::TextColored(ImVec4(0.55f,0.9f,0.65f,1.f),
                "Active: %04d/%s", hdr_year, hdr_code);
        } else {
            ImGui::TextColored(ImVec4(0.85f,0.85f,0.45f,1.f), "IDLE");
        }
        if(hdr_active){
            ImGui::SameLine();
            float btnw = 130.f;
            ImGui::SetCursorPosX(ImGui::GetWindowWidth() - btnw - 16.f);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f,0.20f,0.20f,1.f));
            if(ImGui::Button("End Mission", ImVec2(btnw, 0))){
                v.mission_end_confirm_open = true;
            }
            ImGui::PopStyleColor();
        }
        ImGui::Separator();

        float left_w = 200.f;
        ImGui::BeginChild("##mission_left", ImVec2(left_w, 0), true);
        draw_left_tree(v);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("##mission_right", ImVec2(0, 0), true);
        if(g_sel_year == 0 && hdr_active){
            g_sel_year = hdr_year;
            g_sel_code = hdr_code;
        }
        draw_meta_block(v);
        draw_current_session(v);
        draw_file_tabs(v, cli);
        ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();

    draw_start_submodal(v, cli);
    draw_end_confirm_submodal(v, cli);
    draw_delete_confirm_submodal(v, cli);
    draw_rename_submodal(cli);

    // 미션 모달 활성 시 Delete 키 → 선택 파일 삭제
    process_delete_key(cli);

    // 모달 닫혔으면 선택 클리어
    if(!v.mission_modal_open) clear_selection();
}

// ── NetClient callback hooks (ui.cpp 에서 registration) ─────────────────
void on_mission_file_list_recv(const PktMissionFileList& page,
                               const std::vector<MissionFileEntry>& rows){
    std::lock_guard<std::mutex> lk(g_cf_mtx);
    // 같은 mission_year/code 의 기존 entries 제거 (refresh 모드)
    auto same_mission = [&](const CentralFileRow& r){
        return r.year == g_cf_req_year &&
               strncmp(r.code, g_cf_req_code.c_str(), 8) == 0 &&
               (g_cf_req_station[0] == 0 ||
                strncmp(r.station, g_cf_req_station, 64) == 0);
    };
    // 첫 page (g_cf_last_page=false) 시 기존 동일 mission rows 제거
    if(!g_cf_last_page){
        g_cf_rows.erase(std::remove_if(g_cf_rows.begin(), g_cf_rows.end(), same_mission),
                        g_cf_rows.end());
        g_cf_last_page = true;  // 첫 page 처리 완료 마크 (아래에서 추가)
    }
    for(auto& src : rows){
        CentralFileRow r{};
        memcpy(r.station, src.station, sizeof(r.station));
        r.year   = src.year;
        r.subdir = src.subdir;
        memcpy(r.code, src.code, sizeof(r.code));
        memcpy(r.filename, src.filename, sizeof(r.filename));
        r.size_bytes = src.size_bytes;
        r.mtime_unix = src.mtime_unix;
        g_cf_rows.push_back(r);
    }
    if(page.is_last_page) g_cf_last_page = true;
}

void on_mission_file_dl_data_recv(const PktMissionFileDlData& d,
                                  const uint8_t* chunk, uint32_t chunk_len){
    std::lock_guard<std::mutex> lk(g_dl_mtx);
    if(!g_dl_active) return;
    // is_first 일 때 .info sidecar 작성 + total 기록
    if(d.is_first){
        g_dl_total = d.total_bytes;
        // .info sidecar (info_data가 non-empty 일 때만)
        bool has_info = (d.info_data[0] != 0);
        if(has_info){
            FILE* fi = fopen((g_dl_local_path + ".info").c_str(), "w");
            if(fi){
                size_t n = strnlen(d.info_data, sizeof(d.info_data));
                fwrite(d.info_data, 1, n, fi);
                fclose(fi);
            }
        }
    }
    if(g_dl_fp && chunk_len > 0){
        fseeko(g_dl_fp, (off_t)d.offset, SEEK_SET);
        size_t w = fwrite(chunk, 1, chunk_len, g_dl_fp);
        g_dl_written += w;
    }
    if(d.is_last){
        if(g_dl_fp){
            fflush(g_dl_fp);
            fclose(g_dl_fp);
            g_dl_fp = nullptr;
        }
        // chunk_bytes=0 + is_first + is_last 면 "not found" 응답 → 빈 파일 unlink
        if(d.total_bytes == 0 && g_dl_written == 0){
            unlink(g_dl_local_path.c_str());
            unlink((g_dl_local_path + ".info").c_str());
            MissionView::show_toast("Download failed: file not found on Central");
        } else {
            char msg[256];
            snprintf(msg, sizeof(msg), "Downloaded: %s (%.1f MB)",
                     g_dl_filename.c_str(), g_dl_written / 1048576.0);
            MissionView::show_toast(msg);
        }
        g_dl_active = false;
        g_dl_filename.clear();
        g_dl_local_path.clear();
        g_dl_total = 0;
        g_dl_written = 0;
    }
}

} // namespace MissionView
