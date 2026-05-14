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

#include <imgui.h>
#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

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
        struct tm tu; gmtime_r(&now, &tu);
        auto code = Mission::make_code(1900+tu.tm_year, tu.tm_mon, tu.tm_mday);
        ImGui::TextDisabled("Mission Code:");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.5f,0.9f,0.6f,1.f),
            "%s  (%04d-%02d-%02d UTC)",
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
        ImGui::BulletText("Started by: %s", login_get_id());
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
    char latlon[64];
    snprintf(latlon, sizeof(latlon), "%.4f, %.4f", (double)lat, (double)lon);
    row("Lat/Lon:", latlon);
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

static void draw_file_tabs(FFTViewer& /*v*/){
    if(g_sel_year == 0 || g_sel_code.empty()) return;
    std::string dir_iq    = BEWEPaths::mission_iq_dir   (g_sel_year, g_sel_code);
    std::string dir_audio = BEWEPaths::mission_audio_dir(g_sel_year, g_sel_code);
    std::string dir_hist  = BEWEPaths::mission_hist_dir (g_sel_year, g_sel_code);

    ImGui::Spacing();
    if(ImGui::BeginTabBar("##mission_file_tabs")){
        auto draw_list = [&](const std::string& dir, const char* ext){
            auto items = list_dir(dir, ext);
            if(items.empty()){
                ImGui::TextDisabled("  (no files)");
                return;
            }
            ImGui::BeginChild("##mission_files_scroll", ImVec2(0,0), false);
            for(auto& it : items){
                float pw = ImGui::GetContentRegionAvail().x;
                ImGui::PushID(it.name.c_str());
                ImGui::Selectable(it.name.c_str(), false,
                    ImGuiSelectableFlags_SpanAllColumns, ImVec2(pw, 0));
                std::string info = FFTViewer::format_file_info(0.0, (uint64_t)it.size);
                float tw = ImGui::CalcTextSize(info.c_str()).x;
                ImGui::SameLine(pw - tw - 4.f);
                ImGui::TextDisabled("%s", info.c_str());
                ImGui::PopID();
            }
            ImGui::EndChild();
        };
        if(ImGui::BeginTabItem("IQ")){
            draw_list(dir_iq, ".wav");
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("Audio")){
            draw_list(dir_audio, ".wav");
            ImGui::EndTabItem();
        }
        if(ImGui::BeginTabItem("HIST")){
            draw_list(dir_hist, ".bewehist");
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
            ImGui::PopStyleColor();
            ImGui::PopID();
            ImGui::PopID();
        }
        ImGui::TreePop();
    }
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
