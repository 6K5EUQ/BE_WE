// ── DEMOD 패널 (코어, 모듈 무관) ────────────────────────────────────────────
// 하단바 DEMOD 버튼 → 이 패널. 설치된 모듈 목록(레지스트리)을 보여주고,
// 선택한 모듈을 탭으로 띄움 (여러 모듈 동시 오픈, 탭 전환).
// 모듈이 하나도 없으면 하단바 버튼 자체가 안 보임 (ui.cpp 에서 가드).
#include "fft_viewer.hpp"
#include "module_api.hpp"
#include <imgui.h>
#include <vector>

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
    static std::vector<bool> open(mods.size(), false);    // 모듈별 탭 열림
    static std::vector<bool> was_active(mods.size(), false);
    static int select_next = -1;                          // 목록에서 방금 연 탭 강제 선택
    if(open.size() != mods.size()){ open.assign(mods.size(), false); was_active.assign(mods.size(), false); }

    ImGui::SetCursorPos(ImVec2(8, 4));
    if(ImGui::BeginTabBar("##demod_tabs",
        ImGuiTabBarFlags_Reorderable|ImGuiTabBarFlags_AutoSelectNewTabs)){

        // ── 모듈 목록 탭 (런처) ──
        if(ImGui::BeginTabItem("Modules")){
            for(size_t i=0;i<mods.size();i++) was_active[i]=false;
            ImGui::Dummy(ImVec2(0,12));
            ImGui::Indent(20);
            ImGui::TextDisabled("Installed demod modules");
            ImGui::Dummy(ImVec2(0,8));
            for(size_t i=0;i<mods.size();i++){
                if(!mods[i].draw_content) continue;
                ImGui::PushID((int)i);
                if(ImGui::Button(mods[i].label, ImVec2(180, 40))){
                    open[i] = true;
                    select_next = (int)i;
                }
                ImGui::PopID();
                ImGui::Dummy(ImVec2(0,4));
            }
            ImGui::Unindent(20);
            ImGui::EndTabItem();
        }

        // ── 열린 모듈 탭들 ──
        for(size_t i=0;i<mods.size();i++){
            if(!open[i] || !mods[i].draw_content) continue;
            bool keep = true;
            ImGuiTabItemFlags fl = (select_next==(int)i) ? ImGuiTabItemFlags_SetSelected : 0;
            if(ImGui::BeginTabItem(mods[i].label, &keep, fl)){
                bool ja = !was_active[i];          // 이 탭이 막 활성화됨
                was_active[i] = true;
                mods[i].draw_content(v, ja);
                ImGui::EndTabItem();
            } else was_active[i] = false;
            if(!keep){ open[i]=false; was_active[i]=false; }
        }
        if(select_next>=0) select_next=-1;
        ImGui::EndTabBar();
    }

    ImGui::End();
    ImGui::PopStyleVar();
}
