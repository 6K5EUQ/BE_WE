// SIGINT Mission System — GUI viewer.
// - draw_modal: 미션 전용 모달 (좌: 트리, 우: 메타 + Current Session + 파일 탭)
// - draw_toast: IDLE 차단 같은 일시 알림 (메인페이지 하단 중앙, 3초 페이드)
// - Start Mission / End Mission 서브모달 포함

#include "mission.hpp"
#include "fft_viewer.hpp"
#include "net_client.hpp"
#include "bewe_paths.hpp"
#include "login.hpp"

#include <imgui.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cstdio>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

// External HIST viewer entry point (defined in long_waterfall_view.cpp)
extern bool lwf_open_file(const std::string& path);

namespace MissionView {

// ── Toast: storage / set 함수는 mission.cpp(양쪽 빌드)에 정의 ─────────────
extern std::string g_toast_msg;
extern double      g_toast_expire;

void draw_toast(){
    auto now_d = std::chrono::steady_clock::now().time_since_epoch();
    double now = std::chrono::duration<double>(now_d).count();
    if(g_toast_msg.empty() || now >= g_toast_expire) return;
    double remain = g_toast_expire - now;
    float  alpha  = (float)std::min(1.0, remain / 0.6);   // 마지막 0.6초 페이드
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
static std::string fmt_utc(time_t t){
    if(t <= 0) return "-";
    struct tm tu; gmtime_r(&t, &tu);
    char b[40];
    snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d UTC",
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

// 디스크 스캔: missions/<year>/<code>/ 디렉토리들 수집.
// year 4자리, code 3글자 매칭.
struct DiskMission {
    int         year;
    std::string code;     // "A03"
};
static std::vector<DiskMission> scan_disk_missions(){
    std::vector<DiskMission> out;
    DIR* dr = opendir(BEWEPaths::missions_root().c_str());
    if(!dr) return out;
    struct dirent* de;
    while((de = readdir(dr)) != nullptr){
        const char* n = de->d_name;
        if(!n || n[0]=='.') continue;
        // year 디렉토리: 4자리 숫자
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
    // 최신 우선: year desc → code desc (L31, L30, ...)
    std::sort(out.begin(), out.end(), [](const DiskMission& a, const DiskMission& b){
        if(a.year != b.year) return a.year > b.year;
        return a.code > b.code;
    });
    return out;
}

// 디렉토리 파일 리스트 (확장자 필터)
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

// ── Selected mission ─────────────────────────────────────────────────────
// 좌측 트리에서 선택된 미션. (0, "") = 활성 미션 (있으면).
static int         g_sel_year = 0;
static std::string g_sel_code;

// Tabs (IQ / Audio / HIST)
static int g_tab = 0;   // 0=IQ 1=Audio 2=HIST

// ── Start / End 서브모달 헬퍼 ───────────────────────────────────────────
static void draw_start_submodal(FFTViewer& v, NetClient* cli){
    if(!v.mission_start_modal_open) return;
    ImGui::SetNextWindowSize(ImVec2(380, 0));
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x*0.5f - 190.f,
                                   ImGui::GetIO().DisplaySize.y*0.30f),
                            ImGuiCond_Appearing);
    ImGui::OpenPopup("Start Mission##mission_start");
    bool open = true;
    if(ImGui::BeginPopupModal("Start Mission##mission_start", &open,
        ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize)){
        // Mission code preview (only field — code = identifier; station/host/coords/SDR
        // captured automatically from current state)
        time_t now = time(nullptr);
        struct tm tu; gmtime_r(&now, &tu);
        auto code = Mission::make_code(1900+tu.tm_year, tu.tm_mon, tu.tm_mday);
        ImGui::TextDisabled("Mission code:");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f,0.9f,0.6f,1.f),
            "%s  (%04d-%02d-%02d UTC)",
            code.c_str(), 1900+tu.tm_year, 1+tu.tm_mon, tu.tm_mday);
        ImGui::Spacing();
        ImGui::TextDisabled("Station / host / coords / SDR / antenna are captured automatically.");
        ImGui::Spacing();
        if(ImGui::Button("Cancel", ImVec2(120, 0))){
            v.mission_start_modal_open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.55f, 0.25f, 1.f));
        if(ImGui::Button("Start", ImVec2(120, 0))){
            // JOIN: net_cli 경유로 HOST에 요청. LOCAL/HOST: 직접 호출.
            if(cli){
                cli->send_mission_start();
            } else {
                v.mission_start(login_get_id(), /*op_index=*/0, /*rollover=*/false);
            }
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

// ── Right-pane: 메타 + Current Session + 파일 탭 ────────────────────────
static void draw_meta_block(FFTViewer& v){
    // 활성 미션 메타 vs history 미션 메타 (모두 자동 캡처된 필드)
    bool   is_active = false;
    int    year = 0;
    char   code[8] = {}, started_by[32] = {};
    char   station[64] = {}, host[32] = {}, sdr[24] = {}, antenna[64] = {};
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
            memcpy(started_by,  v.mission_started_by,   sizeof(started_by));
            memcpy(station,     v.mission_station_name, sizeof(station));
            memcpy(host,        v.mission_host_name,    sizeof(host));
            memcpy(sdr,         v.mission_sdr_kind,     sizeof(sdr));
            memcpy(antenna,     v.mission_antenna,      sizeof(antenna));
            lat = v.mission_lat;
            lon = v.mission_lon;
            start_utc = v.mission_start_utc;
            end_utc   = v.mission_end_utc;
        } else {
            // history에서 매칭
            for(auto& e : v.mission_history){
                if(e.year == g_sel_year && g_sel_code == e.code){
                    year = e.year;
                    memcpy(code,        e.code,         sizeof(code));
                    memcpy(started_by,  e.started_by,   sizeof(started_by));
                    memcpy(station,     e.station_name, sizeof(station));
                    memcpy(host,        e.host_name,    sizeof(host));
                    memcpy(sdr,         e.sdr_kind,     sizeof(sdr));
                    memcpy(antenna,     e.antenna,      sizeof(antenna));
                    lat = e.lat;
                    lon = e.lon;
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
    ImGui::Text("Mission %04d/%s", year, code);
    ImGui::PopStyleColor();
    ImGui::Separator();
    const float L = 110.f;
    auto row = [&](const char* k, const char* val){
        ImGui::TextDisabled("%s", k); ImGui::SameLine(L);
        ImGui::TextWrapped("%s", val[0] ? val : "-");
    };
    auto row_coord = [&](){
        ImGui::TextDisabled("Coords:"); ImGui::SameLine(L);
        if(lat == 0.f && lon == 0.f) ImGui::TextWrapped("-");
        else ImGui::Text("%.4f, %.4f", (double)lat, (double)lon);
    };
    row("Status:",   is_active ? "ACTIVE" : "CLOSED");
    row("Station:",  station);
    row("Host:",     host);
    row_coord();
    row("SDR:",      sdr);
    row("Antenna:",  antenna);
    row("Started:",  fmt_utc(start_utc).c_str());
    if(!is_active) row("Ended:", fmt_utc(end_utc).c_str());
    row("By:",       started_by);
}

static void draw_current_session(FFTViewer& v){
    // 활성 미션인 경우에만 표시
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
    // SCHED
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

// FileKind: drives ctx-menu behavior (open/delete semantics per type).
enum FileKind { FK_IQ, FK_AUDIO, FK_HIST };

// Sched-add modal state (per-frame transient).
static bool   g_sched_add_open = false;
static char   g_sched_add_date[16]  = {};   // YYYY-MM-DD UTC
static char   g_sched_add_time[16]  = {};   // HH:MM UTC
static float  g_sched_add_dur_sec   = 60.f;
static float  g_sched_add_freq_mhz  = 100.0f;
static float  g_sched_add_bw_khz    = 200.f;
static char   g_sched_add_target[32] = {};

static void draw_file_tabs(FFTViewer& v){
    if(g_sel_year == 0 || g_sel_code.empty()) return;
    std::string dir_iq    = BEWEPaths::mission_iq_dir   (g_sel_year, g_sel_code);
    std::string dir_audio = BEWEPaths::mission_audio_dir(g_sel_year, g_sel_code);
    std::string dir_hist  = BEWEPaths::mission_hist_dir (g_sel_year, g_sel_code);

    // Deferred actions to apply after rendering (avoids mutating list mid-iter).
    static std::string pending_delete;
    static std::string pending_hist_open;
    static std::string pending_sa_open;      // IQ → Signal Analyzer offline load
    static std::string pending_xdg_open;     // Audio → system default player
    static int         pending_sched_remove_idx = -1;
    // Rename modal state
    static bool        rename_modal_open = false;
    static std::string rename_old_path;
    static char        rename_new_name[128] = {};

    ImGui::Spacing();
    if(ImGui::BeginTabBar("##mission_file_tabs")){
        auto draw_list = [&](const std::string& dir, const char* ext, FileKind kind){
            auto items = list_dir(dir, ext);
            if(items.empty()){
                ImGui::TextDisabled("  (no files)");
                return;
            }
            ImGui::BeginChild("##mission_files_scroll", ImVec2(0,0), false);
            for(auto& it : items){
                float pw = ImGui::GetContentRegionAvail().x;
                std::string full = dir + "/" + it.name;
                ImGui::PushID(it.name.c_str());
                bool clicked = ImGui::Selectable(it.name.c_str(), false,
                    ImGuiSelectableFlags_SpanAllColumns
                    | ImGuiSelectableFlags_AllowDoubleClick,
                    ImVec2(pw, 0));
                // Double-click: HIST → viewer; IQ → SA; Audio → system default
                if(clicked && ImGui::IsMouseDoubleClicked(0)){
                    if(kind == FK_HIST)       pending_hist_open = full;
                    else if(kind == FK_IQ)    pending_sa_open  = full;
                    else                       pending_xdg_open = full;
                }
                // Right-click context menu
                if(ImGui::BeginPopupContextItem("##file_ctx")){
                    if(kind == FK_HIST){
                        if(ImGui::MenuItem("Open in viewer")) pending_hist_open = full;
                    } else if(kind == FK_IQ){
                        if(ImGui::MenuItem("Open in Signal Analyzer")) pending_sa_open = full;
                    } else if(kind == FK_AUDIO){
                        if(ImGui::MenuItem("Play (system default)")) pending_xdg_open = full;
                    }
                    ImGui::Separator();
                    if(ImGui::MenuItem("Rename...")){
                        rename_modal_open = true;
                        rename_old_path   = full;
                        strncpy(rename_new_name, it.name.c_str(), sizeof(rename_new_name)-1);
                        rename_new_name[sizeof(rename_new_name)-1] = 0;
                    }
                    if(ImGui::MenuItem("Show in folder")){
                        char cmd[1024];
                        snprintf(cmd, sizeof(cmd),
                            "xdg-open '%s' >/dev/null 2>&1 &", dir.c_str());
                        int rc = system(cmd); (void)rc;
                    }
                    ImGui::Separator();
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.f,0.5f,0.5f,1.f));
                    if(ImGui::MenuItem("Delete")) pending_delete = full;
                    ImGui::PopStyleColor();
                    ImGui::EndPopup();
                }
                // 우측 정렬 크기 + mtime
                std::string info = FFTViewer::format_file_info(0.0,
                    (uint64_t)it.size);
                float tw = ImGui::CalcTextSize(info.c_str()).x;
                ImGui::SameLine(pw - tw - 4.f);
                ImGui::TextDisabled("%s", info.c_str());
                ImGui::PopID();
            }
            ImGui::EndChild();
        };
        if(ImGui::BeginTabItem("IQ")){
            g_tab = 0;
            draw_list(dir_iq, ".wav", FK_IQ);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("Audio")){
            g_tab = 1;
            draw_list(dir_audio, ".wav", FK_AUDIO);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("HIST")){
            g_tab = 2;
            draw_list(dir_hist, ".bewehist", FK_HIST);
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("Schedule")){
            g_tab = 3;
            // [+ Add] 버튼: 새 스케줄 예약 모달 열기 (UTC 입력)
            if(ImGui::Button("[+] Add scheduled recording", ImVec2(-1, 0))){
                g_sched_add_open = true;
                time_t now = time(nullptr) + 60;
                struct tm tu; gmtime_r(&now, &tu);
                snprintf(g_sched_add_date, sizeof(g_sched_add_date),
                    "%04d-%02d-%02d", 1900+tu.tm_year, 1+tu.tm_mon, tu.tm_mday);
                snprintf(g_sched_add_time, sizeof(g_sched_add_time),
                    "%02d:%02d", tu.tm_hour, tu.tm_min);
                g_sched_add_dur_sec  = 60.f;
                g_sched_add_freq_mhz = (float)(v.live_cf_hz.load() / 1e6);
                g_sched_add_bw_khz   = 200.f;
                g_sched_add_target[0]= 0;
            }
            ImGui::Separator();
            std::lock_guard<std::mutex> lk(v.sched_mtx);
            // Filter by selected mission (mission_year+mission_code match).
            std::vector<int> order;
            order.reserve(v.sched_entries.size());
            for(int i=0; i<(int)v.sched_entries.size(); i++){
                const auto& s = v.sched_entries[i];
                if(s.mission_year == g_sel_year
                   && strncmp(s.mission_code, g_sel_code.c_str(), sizeof(s.mission_code)) == 0){
                    order.push_back(i);
                }
            }
            if(order.empty()){
                ImGui::TextDisabled("  (no scheduled recordings for this mission)");
            } else {
                ImGui::BeginChild("##mission_sched_scroll", ImVec2(0,0), false);
                std::sort(order.begin(), order.end(), [&](int a, int b){
                    return v.sched_entries[a].start_time < v.sched_entries[b].start_time;
                });
                time_t now_t = time(nullptr);
                for(int oi=0; oi<(int)order.size(); oi++){
                    int i = order[oi];
                    auto& s = v.sched_entries[i];
                    ImGui::PushID(i);
                    char tbuf[40] = {};
                    struct tm tu; gmtime_r(&s.start_time, &tu);
                    snprintf(tbuf, sizeof(tbuf),
                        "%04d-%02d-%02d %02d:%02d UTC",
                        1900+tu.tm_year, 1+tu.tm_mon, tu.tm_mday,
                        tu.tm_hour, tu.tm_min);
                    const char* status_str = "?";
                    ImVec4 col(1,1,1,1);
                    switch(s.status){
                        case FFTViewer::SchedEntry::WAITING:
                            status_str="WAIT"; col=ImVec4(0.7f,0.7f,0.8f,1); break;
                        case FFTViewer::SchedEntry::ARMED:
                            status_str="ARM";  col=ImVec4(1.f,0.85f,0.2f,1); break;
                        case FFTViewer::SchedEntry::RECORDING:
                            status_str="REC";  col=ImVec4(1.f,0.3f,0.3f,1); break;
                        case FFTViewer::SchedEntry::DONE:
                            status_str="DONE"; col=ImVec4(0.3f,0.9f,0.3f,1); break;
                        case FFTViewer::SchedEntry::FAILED:
                            status_str="FAIL"; col=ImVec4(0.9f,0.2f,0.2f,1); break;
                    }
                    ImGui::TextColored(col, "[%s]", status_str);
                    ImGui::SameLine();
                    char tail[40] = "";
                    if(s.status == FFTViewer::SchedEntry::WAITING){
                        int d = (int)(s.start_time - now_t);
                        if(d > 0) snprintf(tail, sizeof(tail), "  in %dm%02ds", d/60, d%60);
                    }
                    ImGui::Text("%s  %.3f MHz  BW=%.0fkHz  %.0fs  by %s%s%s%s",
                        tbuf, (double)s.freq_mhz, (double)s.bw_khz, (double)s.duration_sec,
                        s.operator_name[0] ? s.operator_name : "?",
                        s.target[0] ? "  '" : "", s.target,
                        s.target[0] ? "'" : "");
                    ImGui::SameLine();
                    bool can_remove = (s.status != FFTViewer::SchedEntry::RECORDING
                                    && s.status != FFTViewer::SchedEntry::ARMED);
                    if(!can_remove) ImGui::BeginDisabled();
                    if(ImGui::SmallButton("X")) pending_sched_remove_idx = i;
                    if(!can_remove) ImGui::EndDisabled();
                    ImGui::PopID();
                }
                ImGui::EndChild();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    // Apply pending actions
    if(!pending_delete.empty()){
        if(remove(pending_delete.c_str()) == 0){
            // info sidecar 동반 삭제 (Archive 패턴 — .info, .wav.info 등 가능성 모두 시도)
            std::string info1 = pending_delete + ".info";
            remove(info1.c_str());
        }
        pending_delete.clear();
    }
    if(!pending_hist_open.empty()){
        if(lwf_open_file(pending_hist_open)){
            v.lwf_modal_open = true;
        }
        pending_hist_open.clear();
    }
    if(!pending_sa_open.empty()){
        v.sa_temp_path   = pending_sa_open;
        v.eid_panel_open = true;
        pending_sa_open.clear();
    }
    if(!pending_xdg_open.empty()){
        char cmd[1100];
        snprintf(cmd, sizeof(cmd), "xdg-open '%s' >/dev/null 2>&1 &",
                 pending_xdg_open.c_str());
        int rc = system(cmd); (void)rc;
        pending_xdg_open.clear();
    }
    if(pending_sched_remove_idx >= 0){
        std::lock_guard<std::mutex> lk(v.sched_mtx);
        if(pending_sched_remove_idx < (int)v.sched_entries.size()){
            auto& s = v.sched_entries[pending_sched_remove_idx];
            if(v.remote_mode && v.net_cli){
                v.net_cli->cmd_remove_sched((int64_t)s.start_time, s.freq_mhz);
            } else {
                v.sched_entries.erase(v.sched_entries.begin() + pending_sched_remove_idx);
                v.broadcast_sched_list_locked();
            }
        }
        pending_sched_remove_idx = -1;
    }

    // ── Rename modal ──────────────────────────────────────────────────────
    if(rename_modal_open){
        ImGui::SetNextWindowSize(ImVec2(420, 0));
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x*0.5f - 210.f,
                                       ImGui::GetIO().DisplaySize.y*0.30f),
                                ImGuiCond_Appearing);
        ImGui::OpenPopup("Rename file##mv_rename");
        bool open = true;
        if(ImGui::BeginPopupModal("Rename file##mv_rename", &open,
            ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize)){
            ImGui::TextDisabled("%s", rename_old_path.c_str());
            ImGui::InputText("New name", rename_new_name, sizeof(rename_new_name));
            ImGui::Spacing();
            if(ImGui::Button("Cancel", ImVec2(120, 0))){
                rename_modal_open = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if(ImGui::Button("Rename", ImVec2(120, 0))){
                if(rename_new_name[0]){
                    auto slash = rename_old_path.find_last_of('/');
                    std::string dir = (slash == std::string::npos) ? "."
                                                                    : rename_old_path.substr(0, slash);
                    std::string new_path = dir + "/" + rename_new_name;
                    if(rename(rename_old_path.c_str(), new_path.c_str()) == 0){
                        // info sidecar 동반 rename
                        std::string old_info = rename_old_path + ".info";
                        std::string new_info = new_path + ".info";
                        ::rename(old_info.c_str(), new_info.c_str());
                    }
                }
                rename_modal_open = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
        if(!open) rename_modal_open = false;
    }

    // ── Schedule add modal ────────────────────────────────────────────────
    if(g_sched_add_open){
        ImGui::SetNextWindowSize(ImVec2(420, 0));
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x*0.5f - 210.f,
                                       ImGui::GetIO().DisplaySize.y*0.30f),
                                ImGuiCond_Appearing);
        ImGui::OpenPopup("Add Scheduled Recording##sched_add");
        bool open = true;
        if(ImGui::BeginPopupModal("Add Scheduled Recording##sched_add", &open,
            ImGuiWindowFlags_NoResize|ImGuiWindowFlags_AlwaysAutoResize)){
            ImGui::TextDisabled("Time is interpreted as UTC.");
            ImGui::InputText("Date (YYYY-MM-DD)", g_sched_add_date, sizeof(g_sched_add_date));
            ImGui::InputText("Time (HH:MM)",       g_sched_add_time, sizeof(g_sched_add_time));
            ImGui::InputFloat("Duration (s)",      &g_sched_add_dur_sec,  1.f, 10.f, "%.0f");
            ImGui::InputFloat("Freq (MHz)",        &g_sched_add_freq_mhz, 0.001f, 1.f, "%.3f");
            ImGui::InputFloat("BW (kHz)",          &g_sched_add_bw_khz,   1.f, 10.f, "%.0f");
            ImGui::InputText("Target (optional)",  g_sched_add_target, sizeof(g_sched_add_target));
            ImGui::Spacing();
            if(ImGui::Button("Cancel", ImVec2(120, 0))){
                g_sched_add_open = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.55f, 0.25f, 1.f));
            if(ImGui::Button("Add", ImVec2(120, 0))){
                // Parse "YYYY-MM-DD" + "HH:MM" as UTC.
                int y=0, mo=0, da=0, h=0, mi=0;
                bool ok = (sscanf(g_sched_add_date, "%d-%d-%d", &y, &mo, &da) == 3)
                       && (sscanf(g_sched_add_time, "%d:%d", &h, &mi) == 2);
                if(ok && g_sched_add_dur_sec > 0.f){
                    struct tm tu{};
                    tu.tm_year = y - 1900;
                    tu.tm_mon  = mo - 1;
                    tu.tm_mday = da;
                    tu.tm_hour = h;
                    tu.tm_min  = mi;
                    tu.tm_sec  = 0;
                    time_t start_utc = timegm(&tu);
                    if(v.remote_mode && v.net_cli){
                        // JOIN: HOST가 자신의 active mission으로 stamp함
                        v.net_cli->cmd_add_sched((int64_t)start_utc,
                            g_sched_add_dur_sec, g_sched_add_freq_mhz,
                            g_sched_add_bw_khz, g_sched_add_target);
                    } else {
                        std::lock_guard<std::mutex> lk(v.sched_mtx);
                        if(!v.sched_has_overlap(start_utc, g_sched_add_dur_sec)
                           && (int)v.sched_entries.size() < MAX_SCHED_ENTRIES){
                            FFTViewer::SchedEntry e{};
                            e.start_time   = start_utc;
                            e.duration_sec = g_sched_add_dur_sec;
                            e.freq_mhz     = g_sched_add_freq_mhz;
                            e.bw_khz       = g_sched_add_bw_khz;
                            e.status       = FFTViewer::SchedEntry::WAITING;
                            strncpy(e.operator_name, login_get_id(), sizeof(e.operator_name)-1);
                            strncpy(e.target, g_sched_add_target, sizeof(e.target)-1);
                            // Stamp with currently-viewed mission (g_sel_year/code).
                            e.mission_year = g_sel_year;
                            strncpy(e.mission_code, g_sel_code.c_str(), sizeof(e.mission_code)-1);
                            v.sched_entries.push_back(e);
                            v.broadcast_sched_list_locked();
                        }
                    }
                }
                g_sched_add_open = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor();
            ImGui::EndPopup();
        }
        if(!open) g_sched_add_open = false;
    }
}

// ── Left-pane: 미션 트리 (디스크 + history + 활성) ──────────────────────
static void draw_left_tree(FFTViewer& v){
    // [+ Start Mission]
    if(ImGui::Button("[+] Start Mission", ImVec2(-1, 0))){
        v.mission_start_modal_open = true;
    }
    ImGui::Separator();

    // 활성 미션 데이터 snapshot
    int    act_year = 0;
    std::string act_code;
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        if(v.mission_state == Mission::State::ACTIVE && v.mission_code[0]){
            act_year = v.mission_year;
            act_code = v.mission_code;
        }
    }

    // 디스크 스캔 + 활성 미션 합치기 (디스크에 이미 있으면 dedup)
    auto disk = scan_disk_missions();
    if(act_year > 0){
        bool found = false;
        for(auto& d : disk) if(d.year == act_year && d.code == act_code){ found = true; break; }
        if(!found) disk.insert(disk.begin(), {act_year, act_code});
    }
    // year 별로 그룹
    std::map<int, std::vector<std::string>, std::greater<int>> by_year;
    for(auto& d : disk) by_year[d.year].push_back(d.code);
    for(auto& kv : by_year)
        std::sort(kv.second.begin(), kv.second.end(), std::greater<std::string>());

    // 빈 리스트면 안내
    if(by_year.empty()){
        ImGui::TextDisabled("  (no missions)");
        return;
    }

    for(auto& kv : by_year){
        int y = kv.first;
        char hdr[16]; snprintf(hdr, sizeof(hdr), "%04d##yr", y);
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if(!ImGui::TreeNode(hdr)){
            // year 그룹 닫혀있어도 표시는 됨
            continue;
        }
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
            ImGui::PopStyleColor();
            ImGui::PopID();
            ImGui::PopID();
        }
        ImGui::TreePop();
    }
}

// ── Main draw ────────────────────────────────────────────────────────────
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

        // 활성 미션 정보 (헤더용)
        int hdr_year = 0; char hdr_code[8] = {};
        bool hdr_active = false;
        {
            std::lock_guard<std::mutex> lk(v.mission_mtx);
            hdr_active = (v.mission_state == Mission::State::ACTIVE && v.mission_code[0]);
            hdr_year = v.mission_year;
            memcpy(hdr_code, v.mission_code, sizeof(hdr_code));
        }

        // 헤더 + End 버튼
        if(hdr_active){
            ImGui::TextColored(ImVec4(0.55f,0.9f,0.65f,1.f),
                "Active: %04d/%s", hdr_year, hdr_code);
        } else {
            ImGui::TextColored(ImVec4(0.85f,0.85f,0.45f,1.f), "IDLE");
        }
        ImGui::SameLine();
        float btnw = 130.f;
        ImGui::SetCursorPosX(ImGui::GetContentRegionAvail().x - btnw + ImGui::GetCursorPosX());
        if(hdr_active){
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f,0.20f,0.20f,1.f));
            if(ImGui::Button("End Mission", ImVec2(btnw, 0))){
                v.mission_end_confirm_open = true;
            }
            ImGui::PopStyleColor();
        }
        ImGui::Separator();

        // 좌 / 우 split
        float left_w = 200.f;
        ImGui::BeginChild("##mission_left", ImVec2(left_w, 0), true);
        draw_left_tree(v);
        ImGui::EndChild();

        ImGui::SameLine();

        ImGui::BeginChild("##mission_right", ImVec2(0, 0), true);
        if(g_sel_year == 0 && hdr_active){
            // 기본 선택: 활성 미션
            g_sel_year = hdr_year;
            g_sel_code = hdr_code;
        }
        draw_meta_block(v);
        draw_current_session(v);
        draw_file_tabs(v);
        ImGui::EndChild();
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();

    draw_start_submodal(v, cli);
    draw_end_confirm_submodal(v, cli);
}

} // namespace MissionView
