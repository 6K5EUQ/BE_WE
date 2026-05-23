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
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

// long_waterfall_view.cpp의 file-scope 함수 (별도 헤더에 선언 안 됨).
bool lwf_open_file(const std::string& path);

// fft_viewer.cpp 의 글로벌 (namespace 밖) 정의 — DB 탭에서 사용.
extern std::vector<DbFileEntry> g_db_list;
extern std::mutex               g_db_list_mtx;

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
    float  y0 = io.DisplaySize.y * 0.78f;
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

// disk 스캔: missions/<station>/<year>/<code>/
// (v3.20.0 station-keyed layout — 모든 station 폴더 순회)
struct DiskMission { int year; std::string code; std::string station; };
static std::vector<DiskMission> scan_disk_missions(){
    std::vector<DiskMission> out;
    DIR* dr = opendir(BEWEPaths::missions_root().c_str());
    if(!dr) return out;
    struct dirent* de;
    while((de = readdir(dr)) != nullptr){
        const char* n = de->d_name;
        if(!n || n[0]=='.') continue;
        // 4자리 숫자(YYYY) 폴더는 legacy — migration 안 됐을 수도 있으니 무시.
        if(strlen(n) == 4){
            bool all_digit = true;
            for(int i=0;i<4;i++) if(n[i]<'0'||n[i]>'9'){ all_digit=false; break; }
            if(all_digit) continue;
        }
        std::string sdir = BEWEPaths::missions_root() + "/" + n;
        // station 폴더 안에 year 폴더들이 있음
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
            int y = atoi(yn);
            std::string ydir = sdir + "/" + yn;
            DIR* dy = opendir(ydir.c_str());
            if(!dy) continue;
            struct dirent* de2;
            while((de2 = readdir(dy)) != nullptr){
                const char* m = de2->d_name;
                if(!m || m[0]=='.') continue;
                int mon0, mday;
                if(!Mission::parse_code(m, mon0, mday)) continue;
                out.push_back({y, m, n});
            }
            closedir(dy);
        }
        closedir(ds);
    }
    closedir(dr);
    std::sort(out.begin(), out.end(), [](const DiskMission& a, const DiskMission& b){
        if(a.year != b.year) return a.year > b.year;
        return a.code > b.code;
    });
    return out;
}

// Central archive 기반 미션 목록 캐시 (좌측 트리 source).
// 광역 LIST_REQ(station, year=0, code="") 응답에서 unique (year, code, station)
// 누적. mission_history 와 union 되어 disk/history 비어있어도 좌측 트리에 표시.
struct KnownMission { int year; std::string code; std::string station; };
static std::mutex                     g_cf_km_mtx;
static std::vector<KnownMission>      g_cf_known_missions;
static double                         g_cf_km_last_req_time = 0.0;
static std::string                    g_cf_km_last_req_station;

// 미션 목록 통합: disk 폴더 스캔 + mission_history + Central archive(g_cf_known_missions)
// 3개 source union. Central 가 source-of-truth (영구 보관) — HOST 재부팅으로 메모리
// history 손상돼도 Central 응답이 좌측 트리 채움. 결과 year DESC, code DESC.
static std::vector<DiskMission> collect_visible_missions(FFTViewer& v,
                                                         const std::string& cur_station){
    auto out = scan_disk_missions();
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        for(const auto& e : v.mission_history){
            if(e.year == 0 || e.code[0] == 0) continue;
            std::string ecode(e.code, strnlen(e.code, sizeof(e.code)));
            bool dup = false;
            for(const auto& d : out)
                if(d.year == e.year && d.code == ecode){ dup = true; break; }
            if(dup) continue;
            DiskMission dm;
            dm.year = e.year;
            dm.code = ecode;
            dm.station.assign(e.station_name, strnlen(e.station_name, sizeof(e.station_name)));
            out.push_back(std::move(dm));
        }
    }
    {
        std::lock_guard<std::mutex> lk(g_cf_km_mtx);
        for(const auto& km : g_cf_known_missions){
            if(km.year == 0 || km.code.empty()) continue;
            bool dup = false;
            for(const auto& d : out)
                if(d.year == km.year && d.code == km.code){ dup = true; break; }
            if(dup) continue;
            DiskMission dm;
            dm.year = km.year;
            dm.code = km.code;
            dm.station = km.station;
            out.push_back(std::move(dm));
        }
    }
    if(!cur_station.empty()){
        out.erase(std::remove_if(out.begin(), out.end(),
            [&](const DiskMission& d){
                return !d.station.empty() && d.station != cur_station;
            }), out.end());
    }
    std::sort(out.begin(), out.end(), [](const DiskMission& a, const DiskMission& b){
        if(a.year != b.year) return a.year > b.year;
        return a.code > b.code;
    });
    return out;
}

// Central 광역 LIST_REQ — station 의 모든 미션 (year=0, code="") 요청.
// 30초 캐시 (같은 station 광역 req 중복 방지). station 변경 시 캐시 무효화.
// 응답은 on_mission_file_list_recv 에서 g_cf_known_missions 누적.
static void maybe_request_all_missions(NetClient* cli, const std::string& station){
    if(!cli || station.empty()) return;
    double now = ImGui::GetTime();
    if(station == g_cf_km_last_req_station && (now - g_cf_km_last_req_time) < 30.0) return;
    g_cf_km_last_req_station = station;
    g_cf_km_last_req_time = now;
    cli->send_mission_file_list_req(station.c_str(), /*year=*/0, /*code=*/"", /*subdir=*/0);
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
// LIST_REQ 가 in-flight 면 다음 refresh skip. 큰 다운로드 중 응답이 큐에
// 쌓였다 한꺼번에 도착해 같은 파일이 여러 번 append 되는 문제 방지.
// 안전 타임아웃 12초 — 응답이 영영 안 와도 갱신 복귀.
static bool                           g_cf_req_pending = false;
static double                         g_cf_req_sent_at = 0.0;


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

// ── Upload 진행 상태 (단일 동시 업로드, 워커 스레드 기반) ────────────────
static std::mutex   g_ul_mtx;
static std::string  g_ul_filename;        // LOCAL row 매칭용 basename
static std::string  g_ul_full_path;
static uint64_t     g_ul_total = 0;
static uint64_t     g_ul_written = 0;
static bool         g_ul_active = false;
static double       g_ul_last_sample_t = 0.0;
static uint64_t     g_ul_last_sample_bytes = 0;
static double       g_ul_speed_bps = 0.0;
static std::mutex   g_ul_toast_mtx;
static std::string  g_ul_pending_toast;   // 워커 완료 후 UI 스레드에서 표시

// ── 선택 상태 (Delete 키 + multi-upload; Ctrl+클릭 다중선택 지원) ──────────
enum class SelKind : uint8_t { NONE, LOCAL, CENTRAL };
// 마지막-선택 트랙(highlight 우선순위/색 통일용). 두 집합은 동시 사용 가능 (Mix 가능).
static SelKind     g_sel_kind = SelKind::NONE;

// 다중선택 집합 — LOCAL: full path 들, CENTRAL: 5-tuple 들.
struct CentralSelKey {
    char     station[64];
    uint16_t year;
    uint8_t  subdir;
    char     code[8];
    char     filename[128];
    bool eq(const CentralSelKey& o) const {
        return year == o.year && subdir == o.subdir
            && strncmp(station,  o.station,  sizeof(station))  == 0
            && strncmp(code,     o.code,     sizeof(code))     == 0
            && strncmp(filename, o.filename, sizeof(filename)) == 0;
    }
};
static std::set<std::string>      g_sel_local_paths;     // LOCAL 다중선택
static std::vector<CentralSelKey> g_sel_central_keys;    // CENTRAL 다중선택

// 컨텍스트 메뉴 등 단일행 액션이 selection-aware 인지 확인 헬퍼.
static bool local_in_selection(const std::string& full){
    return g_sel_local_paths.count(full) > 0;
}
static bool central_in_selection(const CentralSelKey& k){
    for(auto& e : g_sel_central_keys) if(e.eq(k)) return true;
    return false;
}

// ── Start / End 서브모달 ────────────────────────────────────────────────
static void draw_start_submodal(FFTViewer& v, NetClient* cli){
    // OpenPopup은 IDLE→OPEN 전환 1회만 호출. 매 프레임 호출하면 popup 내부 상태가
    // 매번 reset돼서 버튼 click 이벤트가 누락될 수 있음.
    static bool s_prev_start_open = false;
    if(v.mission_start_modal_open && !s_prev_start_open){
        ImGui::OpenPopup("Start Mission##mission_start");
    }
    s_prev_start_open = v.mission_start_modal_open;
    if(!v.mission_start_modal_open) return;
    ImGui::SetNextWindowSize(ImVec2(440, 0));
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x*0.5f - 220.f,
                                   ImGui::GetIO().DisplaySize.y*0.30f),
                            ImGuiCond_Appearing);
    bool open = true;
    bool popup_shown = ImGui::BeginPopupModal("Start Mission##mission_start", &open,
        ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize);
    {
        static bool s_logged_state = false;
        if(popup_shown && !s_logged_state){
            bewe_log_push(0, "[MISSION] Start popup shown\n");
            s_logged_state = true;
        }
        if(!popup_shown) s_logged_state = false;
    }
    if(popup_shown){
        time_t now = time(nullptr);
        struct tm tu; KST::to_tm(now, tu);
        auto code = Mission::make_code(1900+tu.tm_year, tu.tm_mon, tu.tm_mday);
        ImGui::TextDisabled("Mission Code:");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f,0.9f,0.6f,1.f),
            "%s  (%04d-%02d-%02d)",
            code.c_str(), 1900+tu.tm_year, 1+tu.tm_mon, tu.tm_mday);
        ImGui::Separator();
        // JOIN 모드: HOST 정보는 net_cli에서 가져옴 (lat/lon은 v.station_lat에 동기화됨)
        char antenna_buf[32] = {};
        if(v.remote_mode && cli){
            std::lock_guard<std::mutex> lk(cli->remote_antenna_mtx);
            memcpy(antenna_buf, cli->remote_antenna, sizeof(antenna_buf));
            antenna_buf[sizeof(antenna_buf)-1] = '\0';
        } else {
            memcpy(antenna_buf, v.host_antenna, sizeof(antenna_buf));
            antenna_buf[sizeof(antenna_buf)-1] = '\0';
        }
        ImGui::BulletText("Station: %s", v.station_name.empty() ? "(unset)" : v.station_name.c_str());
        ImGui::BulletText("Lat/Lon: %s", LongWaterfall::fmt_lat_lon(v.station_lat, v.station_lon).c_str());
        char sdr_buf[16] = {};
        if(v.remote_mode && cli){
            std::lock_guard<std::mutex> lk(cli->remote_antenna_mtx);
            memcpy(sdr_buf, cli->remote_sdr_kind, sizeof(sdr_buf));
            sdr_buf[sizeof(sdr_buf)-1] = '\0';
        }
        const char* sdr =
            v.remote_mode      ? (sdr_buf[0] ? sdr_buf :
                                  v.mission_sdr_kind[0] ? v.mission_sdr_kind : "(remote)") :
            (v.dev_blade)      ? "BladeRF" :
            (v.pluto_ctx)      ? "Pluto"   :
            (v.dev_rtl)        ? "RTL-SDR" : "Unknown";
        ImGui::BulletText("SDR: %s", sdr);
        ImGui::BulletText("Antenna: %s", antenna_buf[0] ? antenna_buf : "(unset)");
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
            bewe_log_push(0, "[MISSION] Start button clicked: remote_mode=%d cli=%p net_cli=%p\n",
                          (int)v.remote_mode, (void*)cli, (void*)v.net_cli);
            bool ok = false;
            // cli 파라미터가 null이어도 remote_mode면 v.net_cli로 폴백
            NetClient* use_cli = cli ? cli : v.net_cli;
            if(v.remote_mode && use_cli){
                ok = use_cli->send_mission_start();
                bewe_log_push(0, "[MISSION] cli->send_mission_start() → ok=%d\n", (int)ok);
                if(!ok) MissionView::show_toast("Mission start failed: not connected");
            } else if(!v.remote_mode){
                ok = v.mission_start(login_get_id(), /*op_index=*/0, /*rollover=*/false);
                bewe_log_push(0, "[MISSION] v.mission_start() → ok=%d state=%d\n",
                              (int)ok, (int)v.mission_state);
                if(!ok) MissionView::show_toast("Mission start failed: already active");
            } else {
                bewe_log_push(1, "[MISSION] start failed: remote_mode=1 but no NetClient available\n");
                MissionView::show_toast("Mission start failed: net client missing");
            }
            if(ok) MissionView::show_toast("Mission start requested");
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

// ── Helper: send LIST_REQ if mission selection changed or auto-refresh due ──
static void maybe_request_central_list(FFTViewer& v, NetClient* cli){
    if(!cli) return;
    if(g_sel_year == 0 || g_sel_code.empty()) return;
    // station 결정: 우선 v.station_name (현재 연결 station) — 같은 코드의 다른 station 미션을
    // 잘못 잡지 않도록. 없으면 active mission, 없으면 history (현재 station 매칭 우선).
    char station[64] = {};
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        if(!v.station_name.empty()){
            strncpy(station, v.station_name.c_str(), sizeof(station) - 1);
        } else if(v.mission_state == Mission::State::ACTIVE &&
                  v.mission_year == g_sel_year &&
                  strncmp(v.mission_code, g_sel_code.c_str(), 8) == 0){
            strncpy(station, v.mission_station_name, sizeof(station) - 1);
        } else {
            for(auto& e : v.mission_history){
                if(e.year == g_sel_year && strncmp(e.code, g_sel_code.c_str(), 8) == 0){
                    strncpy(station, e.station_name, sizeof(station) - 1);
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
    // 이전 LIST_REQ 의 응답을 기다리는 중이면 새 요청 막음 (12초 timeout 안전망).
    // sel_changed 는 사용자 명시 의도라 강제 진행.
    bool pending = g_cf_req_pending && (now - g_cf_req_sent_at) < 12.0;
    if(!sel_changed && pending) return;
    if(sel_changed || refresh_due){
        {
            std::lock_guard<std::mutex> lk(g_cf_mtx);
            // 같은 mission 만 비워 (다른 mission rows 는 유지해도 무방하나 단순화).
            // refresh-due 경로에서는 선제 비우지 않음 — 응답 지연 시 화면이 일시 비어 보이는
            // 부작용 방지. 응답 도착 시 page-level dedup (on_mission_file_list_recv) 이 멱등 갱신.
            if(sel_changed) g_cf_rows.clear();
            g_cf_last_page = false;
        }
        g_cf_req_year = g_sel_year;
        g_cf_req_code = g_sel_code;
        strncpy(g_cf_req_station, station, sizeof(g_cf_req_station) - 1);
        g_cf_last_req_time = now;
        g_cf_req_pending = true;
        g_cf_req_sent_at = now;
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

// .info sidecar 인지.
static bool is_info_file(const std::string& name){
    size_t n = name.size();
    return n >= 5 && name.compare(n - 5, 5, ".info") == 0;
}

// 선택 초기화 (LOCAL + CENTRAL 모두).
static void clear_selection(){
    g_sel_kind = SelKind::NONE;
    g_sel_local_paths.clear();
    g_sel_central_keys.clear();
}

// Delete 키 → 선택된 모든 파일 삭제. 미션 모달 활성 시 매 프레임 호출.
static void process_delete_key(NetClient* cli){
    if(g_sel_local_paths.empty() && g_sel_central_keys.empty()) return;
    if(!ImGui::IsKeyPressed(ImGuiKey_Delete, false)) return;
    int n_local = 0;
    for(const std::string& full : g_sel_local_paths){
        unlink(full.c_str());
        unlink((full + ".info").c_str());
        n_local++;
    }
    int n_central = 0;
    if(cli){
        for(auto& s : g_sel_central_keys){
            MissionFileKey k{};
            strncpy(k.station,  s.station,  sizeof(k.station)  - 1);
            k.year   = s.year;
            k.subdir = s.subdir;
            strncpy(k.code,     s.code,     sizeof(k.code)     - 1);
            strncpy(k.filename, s.filename, sizeof(k.filename) - 1);
            cli->send_mission_file_delete(k);
            n_central++;
        }
        if(n_central > 0) g_cf_last_req_time = 0;
    }
    char tbuf[96];
    if(n_local && n_central)
        snprintf(tbuf, sizeof(tbuf), "Deleted: %d local, %d central", n_local, n_central);
    else if(n_local)
        snprintf(tbuf, sizeof(tbuf), "Deleted %d local file(s)", n_local);
    else
        snprintf(tbuf, sizeof(tbuf), "Deleted %d central file(s)", n_central);
    MissionView::show_toast(tbuf);
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

// 업로드 진행률 폴링 + 워커 완료 토스트 (UI 스레드).
static void poll_ul_speed(){
    // 워커가 남긴 완료 토스트는 UI 스레드에서만 표시 (show_toast는 thread-safe 아님).
    std::string pending;
    {
        std::lock_guard<std::mutex> lk(g_ul_toast_mtx);
        if(!g_ul_pending_toast.empty()){
            pending = g_ul_pending_toast;
            g_ul_pending_toast.clear();
        }
    }
    if(!pending.empty()) MissionView::show_toast(pending.c_str());

    if(!g_ul_active) return;
    double now = ImGui::GetTime();
    double dt = now - g_ul_last_sample_t;
    if(dt < 0.2) return;
    uint64_t bytes_now;
    {
        std::lock_guard<std::mutex> lk(g_ul_mtx);
        bytes_now = g_ul_written;
    }
    double inst = dt > 0 ? (double)(bytes_now - g_ul_last_sample_bytes) / dt : 0.0;
    g_ul_speed_bps = (g_ul_speed_bps <= 0.0) ? inst : (g_ul_speed_bps * 0.5 + inst * 0.5);
    g_ul_last_sample_t = now;
    g_ul_last_sample_bytes = bytes_now;
}

static std::string fmt_size(uint64_t bytes){
    char b[32];
    if(bytes >= 1ull << 30) snprintf(b, sizeof(b), "%.2f GB", bytes / (double)(1ull<<30));
    else if(bytes >= 1ull << 20) snprintf(b, sizeof(b), "%.1f MB", bytes / (double)(1ull<<20));
    else if(bytes >= 1ull << 10) snprintf(b, sizeof(b), "%.1f KB", bytes / (double)(1ull<<10));
    else snprintf(b, sizeof(b), "%llu B", (unsigned long long)bytes);
    return b;
}

// Start download: 기존 LOCAL 파일이 있으면 그 크기를 start_offset 으로 resume 다운로드.
// 없으면 처음부터.
static bool start_download(NetClient* cli, const CentralFileRow& row){
    if(!cli) return false;
    std::lock_guard<std::mutex> lk(g_dl_mtx);
    if(g_dl_active){
        MissionView::show_toast("Download already in progress");
        return false;
    }
    // 미션별 다운로드 폴더 (station/year/code/{iq,audio,hist}) 에 저장 — Central archive 와 동일 레이아웃.
    const char* sub_name = (row.subdir == MFS_IQ)    ? "iq"
                          : (row.subdir == MFS_AUDIO) ? "audio"
                          : (row.subdir == MFS_HIST)  ? "hist"
                          : "misc";
    std::string dir = BEWEPaths::downloads_mission_dir(row.station, row.year,
                                                       row.code, sub_name);
    // mkdir -p chain (data_dir → downloads → station → year → code → sub)
    auto mkparents = [](const std::string& p){
        std::string cur;
        for(size_t i = 0; i < p.size(); i++){
            cur += p[i];
            if(p[i] == '/' && cur.size() > 1) mkdir(cur.c_str(), 0755);
        }
        mkdir(p.c_str(), 0755);
    };
    mkparents(dir);
    std::string path = dir + "/" + row.filename;

    // 기존 파일 존재 → resume 모드 (rb+ append). 없거나 size 0 이면 처음부터 (wb).
    struct stat st{};
    uint64_t start = 0;
    bool resume = (stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode) && st.st_size > 0);
    if(resume){
        g_dl_fp = fopen(path.c_str(), "rb+");
        if(!g_dl_fp){
            g_dl_fp = fopen(path.c_str(), "wb");  // fallback
            start = 0;
        } else {
            start = (uint64_t)st.st_size;
            fseeko(g_dl_fp, 0, SEEK_END);
        }
    } else {
        g_dl_fp = fopen(path.c_str(), "wb");
    }
    if(!g_dl_fp){
        char msg[256]; snprintf(msg, sizeof(msg),
            "Download open failed: %s (errno=%d)", path.c_str(), errno);
        MissionView::show_toast(msg);
        return false;
    }
    g_dl_local_path = path;
    g_dl_filename   = row.filename;
    g_dl_total      = 0;
    g_dl_written    = start;
    g_dl_active     = true;
    g_dl_started_at      = ImGui::GetTime();
    g_dl_last_sample_t   = g_dl_started_at;
    g_dl_last_sample_bytes = start;
    g_dl_speed_bps       = 0.0;
    MissionFileKey k{};
    strncpy(k.station,  row.station, sizeof(k.station) - 1);
    k.year   = row.year;
    k.subdir = row.subdir;
    strncpy(k.code,     row.code,     sizeof(k.code) - 1);
    strncpy(k.filename, row.filename, sizeof(k.filename) - 1);
    bool ok = cli->send_mission_file_dl_req(k, start);
    if(!ok){
        if(g_dl_fp){ fclose(g_dl_fp); g_dl_fp = nullptr; }
        if(!resume) unlink(g_dl_local_path.c_str());
        g_dl_active = false;
        MissionView::show_toast("Download request failed");
    }
    return ok;
}

// Right-click context for a Central-side file row.
static void central_context_menu(NetClient* cli, const CentralFileRow& row){
    if(ImGui::BeginPopupContextItem("##cf_ctx")){
        if(ImGui::MenuItem("Download")){
            start_download(cli, row);
        }
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.35f, 0.35f, 1.f));
        if(ImGui::MenuItem("Delete")){
            MissionFileKey k{};
            strncpy(k.station,  row.station,  sizeof(k.station)  - 1);
            k.year   = row.year;
            k.subdir = row.subdir;
            strncpy(k.code,     row.code,     sizeof(k.code)     - 1);
            strncpy(k.filename, row.filename, sizeof(k.filename) - 1);
            if(cli) cli->send_mission_file_delete(k);
            g_cf_last_req_time = 0;
            MissionView::show_toast("Delete requested");
        }
        ImGui::PopStyleColor();
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
            // 미션 모달은 닫지 않음 — ESC로 LWF만 끄고 미션으로 복귀.
        } else {
            MissionView::show_toast("LWF open failed");
        }
        return;
    }
    if(ends_with(".wav")){
        // 메인 페이지의 우클릭 Signal Analysis 항목과 동일하게 EID + SA 데이터 로드.
        v.sa_temp_path = path;
        v.eid_panel_open = true;
        v.eid_view_mode = 1;       // Amp 기본
        v.audio_play_stop();
        v.eid_audio_cursor_sample = 0;
        v.eid_cleanup();
        v.eid_start(path);
        v.sa_cleanup();
        v.sa_mode = false;
        v.sa_view_x0 = 0.f; v.sa_view_x1 = 1.f;
        v.sa_view_y0 = 0.f; v.sa_view_y1 = 1.f;
        v.sa_start(path);
        return;
    }
    MissionView::show_toast("No viewer for this file type");
}

// LOCAL → Central 미션 archive 로 파일 push.
// 선택된 미션(g_sel_year/g_sel_code) + 분류된 subdir(IQ/DEMOD) 로 PUSH_META + PUSH_DATA 청크 전송.
// HOST push worker 의 transfer_id 풀(1..127) 과 충돌 회피를 위해 JOIN-upload 는 0x80~0xFF 사용.
static uint8_t alloc_upload_xfer_id(){
    static std::atomic<uint8_t> next{0x80};
    uint8_t id = next.fetch_add(1, std::memory_order_relaxed);
    if(id < 0x80) id = 0x80;
    return id;
}

static void start_upload(NetClient* cli, FFTViewer& v,
                         const std::string& full_path,
                         const std::string& filename,
                         uint8_t subdir){
    (void)v;
    if(!cli){
        MissionView::show_toast("Upload: not connected to Central");
        return;
    }
    if(g_sel_year == 0 || g_sel_code.empty()){
        MissionView::show_toast("Upload: select a mission first");
        return;
    }
    if(g_cf_req_station[0] == 0){
        MissionView::show_toast("Upload: station unknown for selected mission");
        return;
    }
    {
        std::lock_guard<std::mutex> lk(g_ul_mtx);
        if(g_ul_active){
            MissionView::show_toast("Upload: another upload in progress");
            return;
        }
    }
    FILE* fp = fopen(full_path.c_str(), "rb");
    if(!fp){
        MissionView::show_toast("Upload: cannot open file");
        return;
    }
    fseeko(fp, 0, SEEK_END);
    uint64_t total = (uint64_t)ftello(fp);
    fseeko(fp, 0, SEEK_SET);

    std::string info_data_str;
    {
        FILE* fi = fopen((full_path + ".info").c_str(), "r");
        if(fi){
            char buf[512] = {};
            size_t r = fread(buf, 1, sizeof(buf) - 1, fi);
            (void)r;
            fclose(fi);
            info_data_str.assign(buf, strnlen(buf, sizeof(buf)));
        }
    }

    MissionFileKey k{};
    strncpy(k.station, g_cf_req_station,    sizeof(k.station)  - 1);
    k.year   = (uint16_t)g_sel_year;
    k.subdir = subdir;
    strncpy(k.code,     g_sel_code.c_str(), sizeof(k.code)     - 1);
    strncpy(k.filename, filename.c_str(),   sizeof(k.filename) - 1);

    // 진행률 상태 초기화 (UI 스레드).
    {
        std::lock_guard<std::mutex> lk(g_ul_mtx);
        g_ul_active = true;
        g_ul_filename = filename;
        g_ul_full_path = full_path;
        g_ul_total = total;
        g_ul_written = 0;
        g_ul_last_sample_t = ImGui::GetTime();
        g_ul_last_sample_bytes = 0;
        g_ul_speed_bps = 0.0;
    }

    // 워커 스레드 — UI freeze 없이 청크 전송 + 진행률 갱신.
    std::thread([cli, fp, k, total, info_data_str](){
        uint8_t xid = alloc_upload_xfer_id();
        cli->send_mission_file_push_meta(k, total, xid, /*mode=*/0, info_data_str.c_str());

        constexpr uint32_t CHUNK = 256 * 1024;
        std::vector<uint8_t> buf(CHUNK);
        uint64_t off = 0;
        bool ok = true;
        if(total == 0){
            cli->send_mission_file_push_data(xid, 0, nullptr, 0, /*is_last=*/true);
        } else {
            while(off < total){
                uint32_t want = (uint32_t)std::min((uint64_t)CHUNK, total - off);
                size_t got = fread(buf.data(), 1, want, fp);
                if(got != want){ ok = false; break; }
                bool last = (off + want >= total);
                cli->send_mission_file_push_data(xid, off, buf.data(), want, last);
                off += want;
                {
                    std::lock_guard<std::mutex> lk(g_ul_mtx);
                    g_ul_written = off;
                }
            }
        }
        fclose(fp);
        g_cf_last_req_time = 0;
        {
            std::lock_guard<std::mutex> lk(g_ul_mtx);
            g_ul_active = false;
            g_ul_filename.clear();
            g_ul_full_path.clear();
            g_ul_total = 0;
            g_ul_written = 0;
        }
        {
            std::lock_guard<std::mutex> lk(g_ul_toast_mtx);
            g_ul_pending_toast = ok ? "Upload complete" : "Upload aborted (read error)";
        }
    }).detach();
}

// LOCAL row 우클릭 — Upload (미션 archive 에 push) / Save DB (DB 공용 저장소) / Delete (로컬 unlink).
static void local_context_menu(FFTViewer& v, NetClient* cli,
                               const std::string& full_path,
                               const std::string& name,
                               uint8_t bucket){
    if(ImGui::BeginPopupContextItem("##loc_ctx")){
        if(ImGui::MenuItem("Upload")){
            start_upload(cli, v, full_path, name, bucket);
        }
        if(ImGui::MenuItem("Save DB")){
            if(!cli){
                MissionView::show_toast("Save DB: not connected to Central");
            } else {
                std::string fpath_copy = full_path;
                std::string op = login_get_id() ? std::string(login_get_id()) : std::string();
                // 워커 스레드 — UI freeze 방지. cmd_db_save 가 내부적으로 on_db_upload_progress fire.
                std::thread([cli, fpath_copy, op](){
                    cli->cmd_db_save(fpath_copy.c_str(), op.c_str());
                }).detach();
                MissionView::show_toast("DB upload started");
            }
        }
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.35f, 0.35f, 1.f));
        if(ImGui::MenuItem("Delete")){
            unlink(full_path.c_str());
            unlink((full_path + ".info").c_str());
            MissionView::show_toast("Local file deleted");
        }
        ImGui::PopStyleColor();
        ImGui::EndPopup();
    }
    (void)v;
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
            g_cf_req_pending = false;  // 강제 새로고침
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
    ImGui::BeginChild("##cf_scroll", ImVec2(0, 0), false);
    for(int cf_i = 0; cf_i < (int)rows.size(); cf_i++){
        auto& r = rows[cf_i];
        float pw = ImGui::GetContentRegionAvail().x;
        ImGui::PushID(cf_i);

        // 미션별 다운로드 폴더에서 already_dl 검사 (start_download 와 동일 경로).
        const char* sub_name = (r.subdir == MFS_IQ)    ? "iq"
                              : (r.subdir == MFS_AUDIO) ? "audio"
                              : (r.subdir == MFS_HIST)  ? "hist"
                              : "misc";
        std::string dl_dir  = BEWEPaths::downloads_mission_dir(r.station, r.year,
                                                               r.code, sub_name);
        std::string dl_path = dl_dir + "/" + r.filename;
        std::string actual_path = dl_path;
        bool already_dl = (access(dl_path.c_str(), F_OK) == 0);
        // 로컬 recordings/missions/ 도 확인 (업로드 후 원본 보유 케이스)
        if(!already_dl){
            std::string rec_dir;
            std::string st(r.station);
            if(r.subdir == MFS_IQ)
                rec_dir = BEWEPaths::mission_iq_dir(st, r.year, r.code);
            else if(r.subdir == MFS_AUDIO)
                rec_dir = BEWEPaths::mission_audio_dir(st, r.year, r.code);
            else if(r.subdir == MFS_HIST)
                rec_dir = BEWEPaths::mission_hist_dir(st, r.year, r.code);
            if(!rec_dir.empty()){
                std::string rec_path = rec_dir + "/" + r.filename;
                if(access(rec_path.c_str(), F_OK) == 0){
                    already_dl = true;
                    actual_path = rec_path;
                }
            }
        }
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

        CentralSelKey row_k{};
        strncpy(row_k.station,  r.station,  sizeof(row_k.station)  - 1);
        row_k.year   = r.year;
        row_k.subdir = r.subdir;
        strncpy(row_k.code,     r.code,     sizeof(row_k.code)     - 1);
        strncpy(row_k.filename, r.filename, sizeof(row_k.filename) - 1);
        bool sel = central_in_selection(row_k);

        ImVec4 text_col = already_dl
            ? ImVec4(0.35f, 0.95f, 0.45f, 1.f)   // 다운로드 완료 → 초록
            : ImGui::GetStyleColorVec4(ImGuiCol_Text);
        ImGui::PushStyleColor(ImGuiCol_Text, text_col);
        bool clicked = ImGui::Selectable(r.filename, sel,
            ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick,
            ImVec2(pw, 0));
        ImGui::PopStyleColor();

        if(clicked){
            // 단일 클릭 = 선택; Ctrl+클릭 = 토글; 더블클릭 = open/download
            bool ctrl = ImGui::GetIO().KeyCtrl;
            g_sel_kind = SelKind::CENTRAL;
            if(ctrl){
                if(sel){
                    for(auto it = g_sel_central_keys.begin(); it != g_sel_central_keys.end(); ++it){
                        if(it->eq(row_k)){ g_sel_central_keys.erase(it); break; }
                    }
                } else {
                    g_sel_central_keys.push_back(row_k);
                }
            } else {
                g_sel_local_paths.clear();
                g_sel_central_keys.clear();
                g_sel_central_keys.push_back(row_k);
            }
        }
        if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
            if(already_dl) open_local_in_viewer(v, actual_path);
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

struct LocalFileEntry { std::string name; uint64_t size; time_t mtime; std::string full; };

static void draw_local_list(FFTViewer& v, NetClient* cli){
    // 미션별 LOCAL: 두 위치를 합쳐 표시.
    //   (a) ~/BE_WE/downloads/<station>/<year>/<code>/{iq,audio,hist}/   ← 다운로드 캐시
    //   (b) ~/BE_WE/recordings/missions/<year>/<code>/{iq,audio,hist}/   ← 머신 로컬 녹음
    //       (JOIN 의 선택영역/Demod 녹음, HOST 의 push 전 임시 파일)
    if(g_sel_year == 0 || g_sel_code.empty() || g_cf_req_station[0] == 0){
        ImGui::TextDisabled("Select a mission to see its local downloads.");
        return;
    }
    std::string station = g_cf_req_station;
    int    year = g_sel_year;
    std::string code = g_sel_code;
    const char* subs_name[3] = {"iq", "audio", "hist"};
    const char* labels[3]    = {"IQ", "DEMOD", "HIST"};

    auto scan_into = [&](const std::string& dir, std::vector<LocalFileEntry>& out){
        auto items = list_dir(dir, nullptr);
        for(auto& it : items){
            if(is_info_file(it.name)) continue;
            LocalFileEntry e;
            e.name = it.name; e.size = it.size; e.mtime = it.mtime;
            e.full = dir + "/" + it.name;
            // 중복 (downloads + missions 둘 다에 같은 이름) 제거 — 더 최신 mtime 우선
            bool found = false;
            for(auto& ex : out) if(ex.name == it.name){
                if(it.mtime > ex.mtime) ex = std::move(e);
                found = true; break;
            }
            if(!found) out.push_back(std::move(e));
        }
    };

    std::vector<LocalFileEntry> by[3];
    for(int s = 0; s < 3; s++){
        scan_into(BEWEPaths::downloads_mission_dir(station, year, code, subs_name[s]), by[s]);
        // 머신 로컬 mission 폴더 (HOST 본인 녹음, JOIN region/demod 녹음).
        // station-keyed: recordings/missions/<station>/<year>/<code>/<sub>/
        std::string mi_dir = (s == 0) ? BEWEPaths::mission_iq_dir(station, year, code)
                            : (s == 1) ? BEWEPaths::mission_audio_dir(station, year, code)
                            :            BEWEPaths::mission_hist_dir(station, year, code);
        scan_into(mi_dir, by[s]);
        // mtime 내림차순 정렬
        std::sort(by[s].begin(), by[s].end(),
            [](const LocalFileEntry& a, const LocalFileEntry& b){ return a.mtime > b.mtime; });
    }

    char hdr[256];
    snprintf(hdr, sizeof(hdr), "Local: %s/%04d/%s",
             station.c_str(), year, code.c_str());
    ImGui::TextDisabled("%s", hdr);
    ImGui::Separator();

    poll_ul_speed();
    ImGui::BeginChild("##loc_scroll", ImVec2(0, 0), false);
    int loc_i = 0;
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
        for(const LocalFileEntry& it : by[s]){
            float pw = ImGui::GetContentRegionAvail().x;
            const std::string& full = it.full;
            ImGui::PushID(loc_i++);

            // 이 행이 현재 업로드 중인지 (basename + full_path 둘 다 일치해야 충돌 방지).
            bool uploading = false;
            double frac = 0.0;
            uint64_t ul_total_now = 0, ul_written_now = 0;
            double xfer_bps = 0.0;
            bool xfer_is_db = false;
            {
                std::lock_guard<std::mutex> lk(g_ul_mtx);
                if(g_ul_active && g_ul_filename == it.name && g_ul_full_path == full){
                    uploading = true;
                    ul_total_now = g_ul_total;
                    ul_written_now = g_ul_written;
                    if(g_ul_total > 0) frac = (double)g_ul_written / (double)g_ul_total;
                }
            }
            // 위 Upload 진행 아니면 DB Save 업로드 진행도 동일하게 인라인 표시.
            if(!uploading){
                std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                for(auto& x : v.file_xfers){
                    if(x.filename == it.name && !x.finished
                       && x.dir == FFTViewer::FileXfer::DIR_UPLOAD){
                        uploading = true;
                        xfer_is_db = true;
                        ul_total_now   = x.total_bytes;
                        ul_written_now = x.done_bytes;
                        if(x.total_bytes > 0)
                            frac = (double)x.done_bytes / (double)x.total_bytes;
                        // EWMA bps 갱신 (DB 탭이 안 열려 있어도 LOCAL 탭에서 직접 계산)
                        int64_t now_us = (int64_t)std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
                        if(x.last_steady_us == 0){
                            x.last_steady_us = now_us; x.last_done_bytes = (int64_t)x.done_bytes;
                        } else if(now_us - x.last_steady_us >= 200000){
                            int64_t db = (int64_t)x.done_bytes - x.last_done_bytes;
                            double  dt = (now_us - x.last_steady_us) / 1e6;
                            if(dt > 0){
                                double inst = (db > 0) ? (double)db / dt : 0.0;
                                x.bps_ewma = (x.bps_ewma == 0.0) ? inst
                                                                 : x.bps_ewma * 0.7 + inst * 0.3;
                            }
                            x.last_steady_us  = now_us;
                            x.last_done_bytes = (int64_t)x.done_bytes;
                        }
                        xfer_bps = x.bps_ewma;
                        break;
                    }
                }
            }

            bool sel = local_in_selection(full);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.95f, 0.45f, 1.f));
            bool clicked = ImGui::Selectable(it.name.c_str(), sel,
                ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick,
                ImVec2(pw, 0));
            ImGui::PopStyleColor();
            if(clicked){
                bool ctrl = ImGui::GetIO().KeyCtrl;
                g_sel_kind = SelKind::LOCAL;
                if(ctrl){
                    // 토글
                    if(sel) g_sel_local_paths.erase(full);
                    else    g_sel_local_paths.insert(full);
                } else {
                    // 단일 선택 — 전체 reset 후 추가
                    g_sel_local_paths.clear();
                    g_sel_central_keys.clear();
                    g_sel_local_paths.insert(full);
                }
            }
            if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                open_local_in_viewer(v, full);
            }
            uint8_t bucket = (s == 0) ? MFS_IQ : (s == 1) ? MFS_AUDIO : MFS_HIST;
            local_context_menu(v, cli, full, it.name, bucket);

            char info[80];
            if(uploading){
                double speed_bps = xfer_is_db ? xfer_bps : g_ul_speed_bps;
                double speed_mb = speed_bps / 1048576.0;
                if(ul_total_now > 0){
                    snprintf(info, sizeof(info), "%.0f%%  %.1fMB/s",
                             frac * 100.0, speed_mb);
                } else {
                    snprintf(info, sizeof(info), "%.1fMB  %.1fMB/s",
                             ul_written_now / 1048576.0, speed_mb);
                }
            } else {
                std::string s2 = fmt_size(it.size);
                snprintf(info, sizeof(info), "%s", s2.c_str());
            }
            float tw = ImGui::CalcTextSize(info).x;
            ImGui::SameLine(pw - tw - 4.f);
            ImVec4 info_col = uploading
                ? ImVec4(0.4f, 0.85f, 1.0f, 1.f)
                : ImVec4(0.6f, 0.6f, 0.6f, 1.f);
            ImGui::TextColored(info_col, "%s", info);
            ImGui::PopID();
        }
        ImGui::Spacing();
    }
    ImGui::EndChild();
}

// ── DB 탭 (mission-agnostic 공유 저장소) ────────────────────────────────
// g_db_list / g_db_list_mtx 는 fft_viewer.cpp 의 글로벌 (namespace 밖) 정의.
// MissionView namespace 안에서 사용 시 :: 명시 필요.

static double g_db_last_req_time = 0.0;  // 5초마다 cmd_request_db_list

// DB 파일명 → "iq" / "audio" / "hist" 분류 (Central side 와 동일 규칙).
static const char* db_classify_sub(const char* fn){
    if(!fn || !fn[0]) return "iq";
    size_t n = strlen(fn);
    if(n >= 9 && strcmp(fn + n - 9, ".bewehist") == 0) return "hist";
    if(strstr(fn, "_DE_")) return "audio";
    return "iq";
}

static void draw_db_list(FFTViewer& v, NetClient* cli){
    // 5초마다 list refresh 요청 (단, 무한 요청 방지 위해 cli 있을 때만).
    if(cli){
        double now = ImGui::GetTime();
        if(now - g_db_last_req_time > 5.0){
            g_db_last_req_time = now;
            cli->cmd_request_db_list();
        }
    }
    // 스냅샷 복사 — 렌더 중 mutex 풀어두기 위해.
    std::vector<DbFileEntry> entries;
    {
        std::lock_guard<std::mutex> lk(::g_db_list_mtx);
        entries = ::g_db_list;
    }

    // 헤더 — 다른 탭과 동일한 경로 표시 스타일.
    ImGui::TextDisabled("Central: DataBase");
    ImGui::SameLine();
    float rgt = ImGui::GetContentRegionMax().x;
    const char* rb = "Refresh";
    ImVec2 rbs = ImGui::CalcTextSize(rb);
    ImGui::SameLine(rgt - rbs.x - 16.f);
    if(ImGui::SmallButton("Refresh##db") && cli){
        g_db_last_req_time = 0.0;
        cli->cmd_request_db_list();
    }
    ImGui::Separator();

    if(!cli){
        ImGui::TextColored(ImVec4(0.85f,0.7f,0.4f,1.f),
            "Not connected to Central — DB unavailable.");
        return;
    }

    // 3 sub-section: IQ / DEMOD / HIST
    static const char* sub_names[3]   = {"iq",   "audio", "hist"};
    static const char* sub_labels[3]  = {"IQ",   "DEMOD", "HIST"};
    std::vector<int> idx_by_sub[3];
    for(int i = 0; i < (int)entries.size(); i++){
        const char* s = db_classify_sub(entries[i].filename);
        int si = (strcmp(s,"audio")==0) ? 1 : (strcmp(s,"hist")==0) ? 2 : 0;
        idx_by_sub[si].push_back(i);
    }

    ImGui::BeginChild("##db_scroll", ImVec2(0, 0), false);
    int row_id = 0;
    for(int si = 0; si < 3; si++){
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.65f, 0.85f, 1.0f, 1.f));
        ImGui::TextUnformatted(sub_labels[si]);
        ImGui::PopStyleColor();
        ImGui::Separator();
        if(idx_by_sub[si].empty()){
            ImGui::TextDisabled("  (none)");
            ImGui::Spacing();
            continue;
        }
        for(int ei : idx_by_sub[si]){
            const DbFileEntry& e = entries[ei];
            ImGui::PushID(row_id++);

            // 이미 로컬에 다운로드돼 있나?
            std::string local_path = BEWEPaths::db_downloads_sub(sub_names[si]) + "/" + e.filename;
            bool already_dl = (access(local_path.c_str(), F_OK) == 0);

            // file_xfers 매칭 (업로드/다운로드 진행률)
            bool xfer_active = false;
            bool xfer_up = false;
            double frac = 0.0;
            double bps = 0.0;
            uint64_t done_now = 0, total_now = 0;
            {
                std::lock_guard<std::mutex> lk(v.file_xfer_mtx);
                for(auto& x : v.file_xfers){
                    if(x.filename == e.filename && !x.finished){
                        xfer_active = true;
                        xfer_up = (x.dir == FFTViewer::FileXfer::DIR_UPLOAD);
                        done_now = x.done_bytes;
                        total_now = x.total_bytes;
                        if(total_now > 0) frac = (double)done_now / (double)total_now;
                        bps = x.bps_ewma;
                        break;
                    }
                }
            }

            float pw = ImGui::GetContentRegionAvail().x;
            ImVec4 tcol = already_dl
                ? ImVec4(0.35f, 0.95f, 0.45f, 1.f)
                : ImGui::GetStyleColorVec4(ImGuiCol_Text);
            ImGui::PushStyleColor(ImGuiCol_Text, tcol);
            ImGui::Selectable(e.filename, false,
                ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick,
                ImVec2(pw, 0));
            ImGui::PopStyleColor();

            // 우클릭: Delete
            if(ImGui::BeginPopupContextItem("##db_ctx")){
                if(already_dl){
                    if(ImGui::MenuItem("Open in viewer")){
                        open_local_in_viewer(v, local_path);
                    }
                    ImGui::Separator();
                }
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f, 0.35f, 0.35f, 1.f));
                if(ImGui::MenuItem("Delete (DB)")){
                    const char* op = login_get_id();
                    cli->cmd_db_delete(e.filename, op ? op : "");
                    MissionView::show_toast("DB delete requested");
                }
                ImGui::PopStyleColor();
                ImGui::EndPopup();
            }

            // 더블클릭: 받은 파일이면 viewer, 아니면 다운로드 시작
            if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                if(already_dl){
                    open_local_in_viewer(v, local_path);
                } else if(!xfer_active){
                    const char* op = login_get_id();
                    cli->cmd_db_download(e.filename, op ? op : "");
                    MissionView::show_toast("DB download requested");
                }
            }

            // 우측 표시: 진행률 / 크기 + uploader
            char info[96];
            if(xfer_active){
                double mb_s = bps / 1048576.0;
                if(total_now > 0)
                    snprintf(info, sizeof(info), "%s  %.0f%%  %.1fMB/s",
                             xfer_up ? "UL" : "DL", frac * 100.0, mb_s);
                else
                    snprintf(info, sizeof(info), "%s  %.1fMB",
                             xfer_up ? "UL" : "DL", done_now / 1048576.0);
            } else {
                std::string s = fmt_size(e.size_bytes);
                if(e.operator_name[0])
                    snprintf(info, sizeof(info), "[%s]  %s", e.operator_name, s.c_str());
                else
                    snprintf(info, sizeof(info), "%s", s.c_str());
            }
            float tw = ImGui::CalcTextSize(info).x;
            ImGui::SameLine(pw - tw - 4.f);
            ImVec4 info_col = xfer_active
                ? ImVec4(0.4f, 0.85f, 1.0f, 1.f)
                : ImVec4(0.6f, 0.6f, 0.6f, 1.f);
            ImGui::TextColored(info_col, "%s", info);

            ImGui::PopID();
        }
        ImGui::Spacing();
    }
    ImGui::EndChild();
}

// 미션창 활성 + sub-modal/viewer 아닐 때 1/2/3/4/5 키로 탭 강제 선택.
// SetSelected 는 그 프레임만 적용 — 다음 프레임부터 사용자 마우스 클릭 평소대로.
static int g_tab_request = -1;   // 0=HIST 1=IQ 2=DEMOD 3=LOCAL 4=DB

static void poll_tab_keys(FFTViewer& v){
    // 미션 modal window 가 키보드 focus 일 때만 처리 — 다른 입력에 영향 X.
    if(!ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) return;
    if(ImGui::IsAnyItemActive()) return;
    if(v.eid_panel_open || v.lwf_modal_open) return;
    ImGuiIO& io = ImGui::GetIO();
    if(io.WantTextInput) return;
    if(ImGui::IsKeyPressed(ImGuiKey_1, false)) g_tab_request = 0;
    if(ImGui::IsKeyPressed(ImGuiKey_2, false)) g_tab_request = 1;
    if(ImGui::IsKeyPressed(ImGuiKey_3, false)) g_tab_request = 2;
    if(ImGui::IsKeyPressed(ImGuiKey_4, false)) g_tab_request = 3;
    if(ImGui::IsKeyPressed(ImGuiKey_5, false)) g_tab_request = 4;
}

static void draw_file_tabs(FFTViewer& v, NetClient* cli){
    poll_tab_keys(v);
    auto tab_flags = [&](int idx) -> ImGuiTabItemFlags {
        return (g_tab_request == idx) ? ImGuiTabItemFlags_SetSelected : 0;
    };
    if(g_sel_year == 0 || g_sel_code.empty()){
        // mission 선택 없음 — LOCAL + DB 만 표시 (DB 는 mission-agnostic)
        ImGui::Spacing();
        if(ImGui::BeginTabBar("##mission_file_tabs_idle")){
            if(ImGui::BeginTabItem("LOCAL", nullptr, tab_flags(3))){
                draw_local_list(v, cli);
                ImGui::EndTabItem();
            }
            if(ImGui::BeginTabItem("DB", nullptr, tab_flags(4))){
                draw_db_list(v, cli);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        g_tab_request = -1;
        return;
    }
    ImGui::Spacing();
    if(ImGui::BeginTabBar("##mission_file_tabs")){
        if(ImGui::BeginTabItem("HIST", nullptr, tab_flags(0))){
            draw_central_list(v, cli, MFS_HIST);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("IQ", nullptr, tab_flags(1))){
            draw_central_list(v, cli, MFS_IQ);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("DEMOD", nullptr, tab_flags(2))){
            draw_central_list(v, cli, MFS_AUDIO);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("LOCAL", nullptr, tab_flags(3))){
            draw_local_list(v, cli);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("DB", nullptr, tab_flags(4))){
            draw_db_list(v, cli);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    g_tab_request = -1;
}

static void draw_left_tree(FFTViewer& v){
    // [+] 제거. "Start Mission" 중앙정렬 (Button 의 ImVec2(-1,0) 은 full width).
    if(ImGui::Button("Start Mission", ImVec2(-1, 0))){
        v.mission_start_modal_open = true;
    }
    ImGui::Separator();

    // 현재 station — JOIN 연결 station 또는 HOST 자기 station. left tree 를 이 station 의
    // 미션으로 필터링. cur_station 비면 LOCAL 모드 fallback (전부 표시).
    std::string cur_station = v.station_name;
    int act_year = 0;
    std::string act_code;
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        if(v.mission_state == Mission::State::ACTIVE && v.mission_code[0]){
            act_year = v.mission_year;
            act_code = v.mission_code;
        }
        if(cur_station.empty() && v.mission_station_name[0])
            cur_station = v.mission_station_name;
    }

    if(!cur_station.empty()){
        ImGui::TextColored(ImVec4(0.55f, 0.85f, 1.0f, 1.f), "Station: %s", cur_station.c_str());
        ImGui::Separator();
    }

    // disk 폴더 + mission_history union (종료 후 push&unlink 된 미션도 표시).
    auto disk = collect_visible_missions(v, cur_station);
    if(act_year > 0){
        bool found = false;
        for(auto& d : disk) if(d.year == act_year && d.code == act_code){ found = true; break; }
        if(!found){
            DiskMission act{};
            act.year    = act_year;
            act.code    = act_code;
            act.station = cur_station;
            disk.insert(disk.begin(), act);
        }
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
        // station 은 history 에서 lookup (못 찾으면 _unknown_)
        std::string del_station;
        {
            std::lock_guard<std::mutex> lk(v.mission_mtx);
            for(auto& h : v.mission_history){
                if(h.year == g_del_year &&
                   strncmp(h.code, g_del_code.c_str(), sizeof(h.code)) == 0){
                    if(h.station_name[0]) del_station = h.station_name;
                    break;
                }
            }
            if(del_station.empty() && v.mission_state == Mission::State::ACTIVE &&
               v.mission_year == g_del_year &&
               strncmp(v.mission_code, g_del_code.c_str(), sizeof(v.mission_code)) == 0){
                if(v.mission_station_name[0]) del_station = v.mission_station_name;
            }
        }
        if(del_station.empty()) del_station = "_unknown_";
        std::string mdir = BEWEPaths::mission_dir(del_station, g_del_year, g_del_code);
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
            // JOIN: Central 에 forward (Central archive wipe + HOST 로 forward → HOST local 정리).
            //       동시에 자기 측 disk/history 도 정리 (broadcast 받기 전에 즉시 UI 반영).
            // HOST: 직접 mission_delete (rm_rf + history erase + broadcast).
            if(cli) cli->send_mission_delete(g_del_year, g_del_code.c_str());
            v.mission_delete(g_del_year, g_del_code.c_str());
            // Central archive 캐시도 즉시 정리 — 다음 LIST_REQ 가 빈 결과 받을 때까지 갭 방지.
            {
                std::lock_guard<std::mutex> lk(g_cf_mtx);
                g_cf_rows.erase(std::remove_if(g_cf_rows.begin(), g_cf_rows.end(),
                    [&](const CentralFileRow& r){
                        return r.year == g_del_year &&
                               strncmp(r.code, g_del_code.c_str(), 8) == 0;
                    }), g_cf_rows.end());
            }
            g_cf_last_req_time = 0;  // 다음 프레임 즉시 refresh
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

        // 이 창이 포커스이고 다른 viewer가 떠있지 않을 때만 ESC로 닫기 —
        // viewer(HIST/SA)가 떠있으면 그쪽 ESC 핸들러가 먼저 처리.
        if(!v.lwf_modal_open && !v.eid_panel_open &&
           ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
           ImGui::IsKeyPressed(ImGuiKey_Escape, false)){
            v.mission_modal_open = false;
        }

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

        // Central 광역 LIST_REQ — station 의 모든 미션 목록을 좌측 트리에 표시하기 위해.
        // 30초 캐시 (maybe_request_all_missions 내부). station 변경 시 즉시 재요청.
        if(cli && !v.station_name.empty())
            maybe_request_all_missions(cli, v.station_name);

        float left_w = 200.f;
        ImGui::BeginChild("##mission_left", ImVec2(left_w, 0), true);
        draw_left_tree(v);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("##mission_right", ImVec2(0, 0), true);
        if(g_sel_year == 0){
            if(hdr_active){
                g_sel_year = hdr_year;
                g_sel_code = hdr_code;
            } else {
                // IDLE 부팅 후: disk+history union 의 가장 최신 미션 자동 선택 →
                // 우측 패널이 즉시 LIST_REQ 트리거하여 파일 표시.
                std::string cur_station_auto = v.station_name;
                if(cur_station_auto.empty()){
                    std::lock_guard<std::mutex> lk(v.mission_mtx);
                    if(v.mission_station_name[0])
                        cur_station_auto = v.mission_station_name;
                }
                auto missions = collect_visible_missions(v, cur_station_auto);
                if(!missions.empty()){
                    g_sel_year = missions[0].year;
                    g_sel_code = missions[0].code;
                }
            }
        }
        draw_meta_block(v);
        // (Current Session 영역 제거 — 사용자 요청)
        draw_file_tabs(v, cli);
        ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();

    draw_start_submodal(v, cli);
    draw_end_confirm_submodal(v, cli);
    draw_delete_confirm_submodal(v, cli);

    // 미션 모달 활성 시 Delete 키 → 선택 파일 삭제
    process_delete_key(cli);

    // 모달 닫혔으면 선택 클리어
    if(!v.mission_modal_open) clear_selection();
}

// ── NetClient callback hooks (ui.cpp 에서 registration) ─────────────────
void on_mission_file_list_recv(const PktMissionFileList& page,
                               const std::vector<MissionFileEntry>& rows){
    std::lock_guard<std::mutex> lk(g_cf_mtx);
    // 페이지 단위 멱등 갱신: 이 페이지에 포함된 (station,year,code,subdir,filename)
    // 키와 일치하는 기존 row를 제거 후 append. 응답이 중첩으로 도착해도 중복 누적 방지.
    auto in_page = [&](const CentralFileRow& r) -> bool {
        for(auto& src : rows){
            if(r.year == src.year &&
               r.subdir == src.subdir &&
               strncmp(r.station, src.station, sizeof(r.station)) == 0 &&
               strncmp(r.code, src.code, sizeof(r.code)) == 0 &&
               strncmp(r.filename, src.filename, sizeof(r.filename)) == 0)
                return true;
        }
        return false;
    };
    g_cf_rows.erase(std::remove_if(g_cf_rows.begin(), g_cf_rows.end(), in_page),
                    g_cf_rows.end());
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
    // Central archive 의 unique (year, code, station) 누적 — 좌측 트리 source.
    // 광역 LIST_REQ 응답이든 selection 응답이든 모두 통과 (멱등).
    {
        std::lock_guard<std::mutex> lkm(g_cf_km_mtx);
        for(auto& src : rows){
            if(src.year == 0 || src.code[0] == 0) continue;
            std::string code_s(src.code, strnlen(src.code, sizeof(src.code)));
            std::string st_s(src.station, strnlen(src.station, sizeof(src.station)));
            bool dup = false;
            for(auto& km : g_cf_known_missions){
                if(km.year == (int)src.year && km.code == code_s && km.station == st_s){
                    dup = true; break;
                }
            }
            if(!dup) g_cf_known_missions.push_back({(int)src.year, code_s, st_s});
        }
    }
    if(page.is_last_page){
        g_cf_last_page = true;
        g_cf_req_pending = false;   // 다음 refresh 허용
    }
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
