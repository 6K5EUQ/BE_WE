// ── DEMOD 패널 (코어, 모듈 무관) ────────────────────────────────────────────
// 하단바 DEMOD 버튼 / D 키 → 이 패널.
// Modules 탭(런처): 설치 모듈 목록 + 선택 모듈의 전 스테이션 채널 타깃 테이블.
//   - 타깃 목록은 Central 집계 (CH_LIST, 2초 폴링). LOCAL 은 로컬 채널.
//   - Start/Stop 으로 어느 기지 어느 채널이든 디코드 on/off (전 유저 동기화).
//   - 동작 중 타깃은 [RUN] 초록 표시.
// 모듈별 데이터 뷰는 탭으로 (다중 오픈, 전환).
#include "fft_viewer.hpp"
#include "module_api.hpp"
#include <imgui.h>
#include <cstring>
#include <vector>

static const char* mode_name(uint8_t m){
    static const char* n[] = {"NONE","AM","FM","CONST","OFDM"};
    return (m < 5) ? n[m] : "?";
}
// station_id "DGS-2_DGS-2" → 표시명 "DGS-2"
static void station_disp(const char* sid, char* out, size_t cap){
    size_t o=0;
    for(const char* p=sid; *p && *p!='_' && o+1<cap; ++p) out[o++]=*p;
    if(o==0 && cap>5){ strncpy(out,"LOCAL",cap-1); out[cap-1]=0; return; }
    out[o]=0;
}

// ── 런처: 선택 모듈의 타깃 테이블 ──
static void draw_launcher(FFTViewer& v, const BeweModule& m, std::vector<bool>& open, size_t mi){
    ImGui::TextColored(ImVec4(0.5f,0.8f,1.f,1.f), "%s", m.label);
    ImGui::SameLine(0,20);
    if(m.draw_content && ImGui::Button("Open View")) open[mi] = true;
    ImGui::SameLine(0,12);
    ImGui::TextDisabled("decode targets — any user can start/stop, synced to all");
    ImGui::Dummy(ImVec2(0,6));

    // 원격이면 주기적으로 타깃 목록 갱신
    if(v.remote_mode){
        static double s_last_req = 0;
        double now = ImGui::GetTime();
        if(now - s_last_req > 2.0){ bewe_mod_req_ch_list(m.id); s_last_req = now; }
    }

    auto targets = bewe_mod_targets(v, m.id);
    ImGuiTableFlags tf = ImGuiTableFlags_RowBg|ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_ScrollY;
    float th = ImGui::GetContentRegionAvail().y - 8;
    if(ImGui::BeginTable("##mod_targets", 6, tf, ImVec2(720, th>120?th:120))){
        ImGui::TableSetupColumn("Station", ImGuiTableColumnFlags_WidthFixed, 90);
        ImGui::TableSetupColumn("CH",      ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Freq (MHz)", ImGuiTableColumnFlags_WidthFixed, 170);
        ImGui::TableSetupColumn("Mode",    ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("State",   ImGuiTableColumnFlags_WidthFixed, 80);
        ImGui::TableSetupColumn("",        ImGuiTableColumnFlags_WidthFixed, 80);
        ImGui::TableHeadersRow();

        if(targets.empty()){
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("(no active channel filters)");
        }
        for(size_t i=0;i<targets.size();i++){
            MpChEntry& e = targets[i];
            bool eligible = m.target_modes & (1u << e.mode);
            bool running  = e.decode_on != 0;
            ImGui::TableNextRow();
            ImGui::PushID((int)i);
            char sd[16]; station_disp(e.station, sd, sizeof(sd));
            if(!eligible) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.45f,0.45f,0.45f,1.f));
            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(sd);
            ImGui::TableSetColumnIndex(1); ImGui::Text("%d", e.ch);
            ImGui::TableSetColumnIndex(2); ImGui::Text("%.4f - %.4f", e.lo, e.hi);
            ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(mode_name(e.mode));
            if(!eligible) ImGui::PopStyleColor();
            ImGui::TableSetColumnIndex(4);
            if(running) ImGui::TextColored(ImVec4(0.3f,0.9f,0.3f,1.f), "[RUN]");
            else        ImGui::TextDisabled("-");
            ImGui::TableSetColumnIndex(5);
            if(running){
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f,0.15f,0.15f,1.f));
                if(ImGui::SmallButton("Stop"))
                    bewe_mod_set_target(v, m.id, e.station, e.ch, false);
                ImGui::PopStyleColor();
            } else if(eligible){
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f,0.42f,0.2f,1.f));
                if(ImGui::SmallButton("Start"))
                    bewe_mod_set_target(v, m.id, e.station, e.ch, true);
                ImGui::PopStyleColor();
            } else ImGui::TextDisabled("mode");
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
}

void demod_draw_panel(FFTViewer& v, bool just_opened){
    ImGuiIO& io = ImGui::GetIO();
    float W = io.DisplaySize.x, H = io.DisplaySize.y - TOPBAR_H;
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(W, H));
    if(just_opened) ImGui::SetNextWindowFocus();
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
    ImGui::Begin("##demod_panel", nullptr,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
        ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoScrollWithMouse);

    // ESC 로 닫기 (필터 입력 중엔 InputText 가 ESC 소비)
    if(ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
       ImGui::IsKeyPressed(ImGuiKey_Escape, false))
        v.demod_panel_open = false;

    auto& mods = bewe_modules();
    static std::vector<bool> open(mods.size(), false);    // 모듈별 데이터 뷰 탭 열림
    static std::vector<bool> was_active(mods.size(), false);
    static int sel_mod = 0;                               // 런처에서 선택된 모듈
    if(open.size() != mods.size()){ open.assign(mods.size(), false); was_active.assign(mods.size(), false); }

    ImGui::SetCursorPos(ImVec2(8, 4));
    if(ImGui::BeginTabBar("##demod_tabs", ImGuiTabBarFlags_Reorderable)){

        // ── Modules 탭 (런처): 좌측 모듈 목록 + 우측 타깃 테이블 ──
        if(ImGui::BeginTabItem("Modules")){
            for(size_t i=0;i<mods.size();i++) was_active[i]=false;
            ImGui::Dummy(ImVec2(0,6));
            ImGui::BeginChild("##mod_list", ImVec2(180, 0), true);
            for(size_t i=0;i<mods.size();i++){
                if(!mods[i].target_modes && !mods[i].draw_content) continue;
                ImGui::PushID((int)i);
                bool running_any = false;
                {   // 어느 스테이션에서든 동작 중이면 초록 점등
                    auto tg = bewe_mod_targets(v, mods[i].id);
                    for(auto& t : tg) if(t.decode_on){ running_any = true; break; }
                }
                if(ImGui::Selectable(mods[i].label, sel_mod==(int)i)) sel_mod=(int)i;
                if(running_any){
                    ImGui::SameLine();
                    float rx = ImGui::GetContentRegionAvail().x;
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX()+rx-44);
                    ImGui::TextColored(ImVec4(0.3f,0.9f,0.3f,1.f), "[RUN]");
                }
                ImGui::PopID();
            }
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("##mod_detail", ImVec2(0, 0), false);
            if(sel_mod >= 0 && sel_mod < (int)mods.size())
                draw_launcher(v, mods[sel_mod], open, (size_t)sel_mod);
            ImGui::EndChild();
            ImGui::EndTabItem();
        }

        // ── 열린 모듈 데이터 뷰 탭들 ──
        for(size_t i=0;i<mods.size();i++){
            if(!open[i] || !mods[i].draw_content) continue;
            bool keep = true;
            if(ImGui::BeginTabItem(mods[i].label, &keep)){
                bool ja = !was_active[i];
                was_active[i] = true;
                mods[i].draw_content(v, ja);
                ImGui::EndTabItem();
            } else was_active[i] = false;
            if(!keep){ open[i]=false; was_active[i]=false; }
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
    ImGui::PopStyleVar();
}
