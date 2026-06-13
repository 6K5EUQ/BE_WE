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
#include <imgui_internal.h>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>

static const char* mode_name(uint8_t m){
    static const char* n[] = {"NONE","AM","FM"};
    return (m < 3) ? n[m] : "?";
}
// station_id "DGS-2_DGS-2" → 표시명 "DGS-2"
static void station_disp(const char* sid, char* out, size_t cap){
    size_t o=0;
    for(const char* p=sid; *p && *p!='_' && o+1<cap; ++p) out[o++]=*p;
    if(o==0 && cap>5){ strncpy(out,"LOCAL",cap-1); out[cap-1]=0; return; }
    out[o]=0;
}

// 셀 텍스트 중앙정렬
static void cell_ctr(const char* s){
    float av=ImGui::GetContentRegionAvail().x, tw=ImGui::CalcTextSize(s).x;
    if(tw<av) ImGui::SetCursorPosX(ImGui::GetCursorPosX()+(av-tw)*0.5f);
    ImGui::TextUnformatted(s);
}
static void cell_ctr_col(const char* s, ImVec4 c){ ImGui::PushStyleColor(ImGuiCol_Text,c); cell_ctr(s); ImGui::PopStyleColor(); }
// 셀 안에서 다음 버튼을 가로 중앙에 배치
static void cell_btn_center(const char* label){
    float bw = ImGui::CalcTextSize(label).x + ImGui::GetStyle().FramePadding.x*2.f;
    float av = ImGui::GetContentRegionAvail().x;
    if(bw<av) ImGui::SetCursorPosX(ImGui::GetCursorPosX()+(av-bw)*0.5f);
}

// ── 런처: 선택 모듈의 타깃 테이블 (예정 모듈은 안내만) ──
static void draw_launcher(FFTViewer& v, const BeweModule& m){
    if(m.planned){   // 복조기 미설치 — UI 프리뷰
        ImGui::Dummy(ImVec2(0,10)); ImGui::Indent(6);
        ImGui::TextDisabled("%s - UI preview. Decoder module not installed yet.", m.label);
        ImGui::Dummy(ImVec2(0,4));
        ImGui::TextDisabled("Double-click the module in the list to open its view.");
        ImGui::Unindent(6);
        return;
    }
    // 원격이면 주기적으로 타깃 목록 갱신
    if(v.remote_mode){
        static double s_last_req = 0;
        double now = ImGui::GetTime();
        if(now - s_last_req > 2.0){ bewe_mod_req_ch_list(m.id); s_last_req = now; }
    }

    auto targets = bewe_mod_targets(v, m.id);
    ImGuiTableFlags tf = ImGuiTableFlags_RowBg|ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_ScrollY;
    float th = ImGui::GetContentRegionAvail().y - 8;
    float tw = ImGui::GetContentRegionAvail().x - 8;
    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(8,5));
    if(ImGui::BeginTable("##mod_targets", 6, tf, ImVec2(tw>200?tw:200, th>120?th:120))){
        ImGui::TableSetupColumn("Station",    ImGuiTableColumnFlags_WidthFixed, 100);
        ImGui::TableSetupColumn("CH",         ImGuiTableColumnFlags_WidthFixed, 44);
        ImGui::TableSetupColumn("Freq (MHz)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Mode",       ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("State",      ImGuiTableColumnFlags_WidthFixed, 72);
        ImGui::TableSetupColumn("",           ImGuiTableColumnFlags_WidthFixed, 84);
        // 헤더: Station 좌, 나머지 중앙 (셀과 정렬 일치)
        ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
        const char* hn[6]={"Station","CH","Freq (MHz)","Mode","State",""};
        const bool  hc[6]={false,true,true,true,true,false};
        for(int c=0;c<6;c++){ ImGui::TableSetColumnIndex(c); if(hc[c]) cell_ctr(hn[c]); else ImGui::TextUnformatted(hn[c]); }

        if(targets.empty()){
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("(no channels)");
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
            ImGui::TableSetColumnIndex(1); { char b[8];  snprintf(b,sizeof(b),"%d",e.ch); cell_ctr(b); }
            ImGui::TableSetColumnIndex(2); { char b[40]; snprintf(b,sizeof(b),"%.4f - %.4f",e.lo,e.hi); cell_ctr(b); }
            ImGui::TableSetColumnIndex(3); cell_ctr(mode_name(e.mode));
            if(!eligible) ImGui::PopStyleColor();
            ImGui::TableSetColumnIndex(4);
            if(running) cell_ctr_col("[RUN]", ImVec4(0.3f,0.9f,0.3f,1.f));
            else        cell_ctr_col("-",     ImVec4(0.5f,0.5f,0.5f,1.f));
            ImGui::TableSetColumnIndex(5);
            if(running){
                cell_btn_center("Stop");
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f,0.15f,0.15f,1.f));
                if(ImGui::SmallButton("Stop")) bewe_mod_set_target(v, m.id, e.station, e.ch, false);
                ImGui::PopStyleColor();
            } else if(eligible){
                cell_btn_center("Start");
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f,0.42f,0.2f,1.f));
                if(ImGui::SmallButton("Start")) bewe_mod_set_target(v, m.id, e.station, e.ch, true);
                ImGui::PopStyleColor();
            } else cell_ctr_col("mode", ImVec4(0.5f,0.5f,0.5f,1.f));
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::PopStyleVar();
}

// ── 예정 모듈 데이터 뷰 (복조기 미설치 — 모듈-무관 제네릭 프리뷰) ──
static void draw_preview_content(FFTViewer& v, const char* label){
    (void)v;
    float W = ImGui::GetContentRegionAvail().x;
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f,0.12f,0.16f,1.f));
    ImGui::BeginChild("##pv_hdr", ImVec2(W, 30), false);
    float th=ImGui::GetTextLineHeight();
    ImGui::SetCursorPos(ImVec2(12, 15.f-th*0.5f));
    ImGui::TextColored(ImVec4(0.5f,0.8f,1.f,1.f), "%s", label);
    ImGui::SameLine(0,16); ImGui::SetCursorPosY(15.f-th*0.5f); ImGui::TextDisabled("0 msg");
    ImGui::EndChild();
    ImGui::PopStyleColor();

    ImVec2 rem = ImGui::GetContentRegionAvail();
    const char* l1 = "Decoder module not installed.";
    const char* l2 = "UI preview - no live data.";
    float lh = ImGui::GetTextLineHeightWithSpacing();
    if(rem.y > lh*3) ImGui::Dummy(ImVec2(0, rem.y*0.5f - lh));
    float w1=ImGui::CalcTextSize(l1).x; ImGui::SetCursorPosX((W-w1)*0.5f); ImGui::TextDisabled("%s", l1);
    float w2=ImGui::CalcTextSize(l2).x; ImGui::SetCursorPosX((W-w2)*0.5f); ImGui::TextDisabled("%s", l2);
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
    static std::vector<int>  open_seq;                    // 켠 순서 = 탭 제출 순서
    static std::vector<std::string> tab_disp;             // 직전 프레임 탭 표시 순서 (drag reorder 반영)
    static int sel_mod = 0;                               // 런처에서 선택된 모듈
    if(open.size() != mods.size()){ open.assign(mods.size(), false); was_active.assign(mods.size(), false); open_seq.clear(); }
    auto open_tab = [&](int i){ if(i>=0 && i<(int)mods.size() && !open[i]){ open[i]=true; open_seq.push_back(i); } };

    // 숫자 키 1~9 → 현재 탭(표시) 순서대로 이동 (텍스트 입력 중엔 무시).
    // tab_disp = 직전 프레임 표시 순서 → drag 로 순서 바꿔도 그 순서로 매핑.
    int jump_to = 0, tab_ord = 0;
    if(ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !io.WantTextInput){
        for(int k=0;k<9;k++)
            if(ImGui::IsKeyPressed((ImGuiKey)(ImGuiKey_1+k), false)){ jump_to = k+1; break; }
    }
    std::string jump_name = (jump_to>=1 && jump_to<=(int)tab_disp.size()) ? tab_disp[jump_to-1] : std::string();
    auto sel_flag = [&](const char* nm)->ImGuiTabItemFlags {
        tab_ord++;
        bool hit = jump_name.empty() ? (jump_to==tab_ord) : (jump_name==nm);
        return hit ? ImGuiTabItemFlags_SetSelected : 0;
    };

    ImGui::SetCursorPos(ImVec2(8, 4));
    extern bool GImCenterTabLabels;   // ImGui 패치: 탭 라벨 중앙정렬 (DEMOD 탭바 한정)
    GImCenterTabLabels = true;
    if(ImGui::BeginTabBar("##demod_tabs", ImGuiTabBarFlags_Reorderable)){

        // ── Modules 탭 (런처): 좌측 모듈 목록 + 우측 타깃 테이블 ──
        if(ImGui::BeginTabItem("Modules", nullptr, sel_flag("Modules"))){
            for(size_t i=0;i<mods.size();i++) was_active[i]=false;
            auto shown=[&](int i){ return i>=0 && i<(int)mods.size() &&
                (mods[i].planned || mods[i].target_modes || mods[i].draw_content); };
            ImGui::Dummy(ImVec2(0,6));
            ImGui::BeginChild("##mod_list", ImVec2(180, 0), true);
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(ImGui::GetStyle().ItemSpacing.x, 7));
            ImGui::Indent(8.0f);   // 라벨 앞 여백
            ImGui::Dummy(ImVec2(0,2));
            // 동작 모듈: 클릭=선택, 더블클릭=데이터 뷰 탭 열기
            for(size_t i=0;i<mods.size();i++){
                if(mods[i].planned) continue;
                if(!mods[i].target_modes && !mods[i].draw_content) continue;
                ImGui::PushID((int)i);
                bool running_any = false;
                {   // 어느 스테이션에서든 동작 중이면 초록 점등
                    auto tg = bewe_mod_targets(v, mods[i].id);
                    for(auto& t : tg) if(t.decode_on){ running_any = true; break; }
                }
                if(ImGui::Selectable(mods[i].label, sel_mod==(int)i, ImGuiSelectableFlags_AllowDoubleClick)){
                    sel_mod=(int)i;
                    if(mods[i].draw_content && ImGui::IsMouseDoubleClicked(0)) open_tab((int)i);
                }
                if(running_any){
                    ImGui::SameLine();
                    float rx = ImGui::GetContentRegionAvail().x;
                    ImGui::SetCursorPosX(ImGui::GetCursorPosX()+rx-44);
                    ImGui::TextColored(ImVec4(0.3f,0.9f,0.3f,1.f), "[RUN]");
                }
                ImGui::PopID();
            }
            // 예정 모듈: 동일하게 선택/더블클릭 가능 (복조기 미설치 → 데이터 없음)
            bool any_planned=false;
            for(size_t i=0;i<mods.size();i++) if(mods[i].planned){ any_planned=true; break; }
            if(any_planned){
                ImGui::Dummy(ImVec2(0,3)); ImGui::Separator(); ImGui::Dummy(ImVec2(0,3));
                for(size_t i=0;i<mods.size();i++){
                    if(!mods[i].planned) continue;
                    ImGui::PushID((int)i);
                    if(ImGui::Selectable(mods[i].label, sel_mod==(int)i, ImGuiSelectableFlags_AllowDoubleClick)){
                        sel_mod=(int)i;
                        if(ImGui::IsMouseDoubleClicked(0)) open_tab((int)i);
                    }
                    ImGui::PopID();
                }
            }
            ImGui::Dummy(ImVec2(0,2));
            ImGui::Unindent(8.0f);
            ImGui::PopStyleVar();
            ImGui::EndChild();
            ImGui::SameLine();
            // sel_mod 가 목록에 보이는 모듈을 가리키도록 보정
            if(!shown(sel_mod)){ sel_mod=-1; for(size_t i=0;i<mods.size();i++) if(shown((int)i)){ sel_mod=(int)i; break; } }
            ImGui::BeginChild("##mod_detail", ImVec2(0, 0), false);
            if(sel_mod >= 0 && sel_mod < (int)mods.size())
                draw_launcher(v, mods[sel_mod]);
            ImGui::EndChild();
            ImGui::EndTabItem();
        }

        // ── 열린 모듈 데이터 뷰 탭들: 켠 순서(open_seq)대로 제출. 예정 모듈은 제네릭 프리뷰 ──
        for(size_t s=0; s<open_seq.size(); s++){
            int i = open_seq[s];
            if(i<0 || i>=(int)mods.size() || !open[i] || !(mods[i].draw_content || mods[i].planned)) continue;
            bool keep = true;
            ImGuiTabItemFlags jf = sel_flag(mods[i].label);
            if(ImGui::BeginTabItem(mods[i].label, &keep, jf)){
                bool ja = !was_active[i];
                was_active[i] = true;
                if(mods[i].draw_content) mods[i].draw_content(v, ja);
                else                     draw_preview_content(v, mods[i].label);
                ImGui::EndTabItem();
            } else was_active[i] = false;
            if(!keep){ open[i]=false; was_active[i]=false; }
        }
        // 닫힌 탭은 open_seq 에서 제거 (순서 유지)
        open_seq.erase(std::remove_if(open_seq.begin(), open_seq.end(),
            [&](int i){ return i<0 || i>=(int)mods.size() || !open[i]; }), open_seq.end());
        // 현재 표시 순서 캐시 (drag reorder 반영) — 다음 프레임 숫자키 매핑에 사용
        if(ImGuiTabBar* tb = ImGui::GetCurrentTabBar()){
            tab_disp.clear();
            for(int k=0;k<tb->Tabs.Size;k++) tab_disp.push_back(ImGui::TabBarGetTabName(tb, &tb->Tabs[k]));
        }
        ImGui::EndTabBar();
    }
    GImCenterTabLabels = false;

    ImGui::End();
    ImGui::PopStyleVar();
}
