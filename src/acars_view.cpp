#include "fft_viewer.hpp"
#include <imgui.h>
#include <cstring>
#include <cctype>
#include <vector>
#include <algorithm>

namespace {
bool ci_find(const char* hay, const char* nee){
    if(!nee||!nee[0]) return true;
    char h[300]; int i=0; for(; hay[i] && i<299; i++) h[i]=(char)tolower((unsigned char)hay[i]); h[i]=0;
    char n[64];  int j=0; for(; nee[j] && j<63;  j++) n[j]=(char)tolower((unsigned char)nee[j]);  n[j]=0;
    return strstr(h,n)!=nullptr;
}
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
bool match(const AcarsMsg& m, const char* f){
    if(!f||!f[0]) return true;
    char ts[12]; hms(m.t_ms, ts);
    char fq[16]; snprintf(fq,sizeof(fq),"%.3f", m.freq);
    return ci_find(ts,f) || ci_find(fq,f) || ci_find(m.reg,f) || ci_find(m.flight,f)
        || ci_find(acars_country(m.reg),f) || ci_find(acars_airline_of(m),f)
        || ci_find(acars_db_type(m.reg),f) || ci_find(m.downlink?"Down":"Up", f)
        || ci_find(m.label,f) || ci_find(m.crc_ok?"OK":"FAIL", f) || ci_find(m.text,f);
}
// 셀 내 중앙정렬 텍스트
void ctr(const char* s){
    float avail=ImGui::GetContentRegionAvail().x, tw=ImGui::CalcTextSize(s).x;
    if(tw<avail) ImGui::SetCursorPosX(ImGui::GetCursorPosX()+(avail-tw)*0.5f);
    ImGui::TextUnformatted(s);
}
void ctrc(const char* s, ImVec4 col){ ImGui::PushStyleColor(ImGuiCol_Text,col); ctr(s); ImGui::PopStyleColor(); }
// 컬럼별 정렬 비교 (Text 외 헤더 인덱스와 동일 순서)
int col_cmp(int c, const AcarsMsg& a, const AcarsMsg& b){
    switch(c){
        case 0:  return a.t_ms<b.t_ms?-1:(a.t_ms>b.t_ms?1:0);
        case 1:  return a.freq<b.freq?-1:(a.freq>b.freq?1:0);
        case 2:  return strcmp(a.reg,b.reg);
        case 3:  { char x[5],y[5]; snprintf(x,5,"%s",acars_db_type(a.reg)); snprintf(y,5,"%s",acars_db_type(b.reg)); return strcmp(x,y); } // Type (thread_local 복사)
        case 4:  return strcmp(a.flight,b.flight);
        case 5:  return strcmp(acars_country(a.reg),acars_country(b.reg));
        case 6:  return strcmp(acars_airline_of(a),acars_airline_of(b));
        case 7:  return (int)a.downlink-(int)b.downlink;
        case 8:  return strcmp(a.label,b.label);
        case 9:  return (int)a.crc_ok-(int)b.crc_ok;
        default: return strcmp(a.text,b.text);
    }
}
}

// ── ACARS 오버레이 (A키) — 메시지 테이블 + 필터 + 상세 ──────────────────────
void acars_draw_overlay(FFTViewer& v, bool just_opened){
    ImGuiIO& io = ImGui::GetIO();
    // SA 오버레이와 동일: 상단부터 꽉 채우고 하단바만 남김 (상단 주파수바 덮음)
    float W = io.DisplaySize.x, H = io.DisplaySize.y - TOPBAR_H;
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(W, H));
    if(just_opened) ImGui::SetNextWindowFocus();   // 열릴 때만 앞으로 (필터 입력 focus 보존)
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
    ImGui::Begin("##acars_overlay", nullptr,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
        ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoScrollWithMouse);

    // ESC로 닫기 (Close 버튼과 동일) — 포커스된 경우만. 필터 입력 중엔 InputText가 ESC 소비.
    if(ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
       ImGui::IsKeyPressed(ImGuiKey_Escape, false))
        v.acars_panel_open = false;

    // Ctrl+F / Tab → 필터 입력창 포커스. draw_overlay 는 ACARS 열렸을 때만 호출되므로 항상 적용.
    // (창 focus 여부 무관 — SetKeyboardFocusHere 가 필터+ACARS창 focus 를 가져와 메인 freq Tab 으로 새는 것 차단)
    bool focus_filter =
        (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F, false)) || ImGui::IsKeyPressed(ImGuiKey_Tab, false);

    static AcarsMsg sel; static bool has_sel=false;
    static int  sort_col=-1;     // -1=정렬안함(삽입순), 0~9=해당 컬럼
    static bool sort_asc=true;

    // ── 헤더 바 ──
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f,0.12f,0.16f,1.f));
    ImGui::BeginChild("##acars_hdr", ImVec2(W, 30), false);
    float fh=ImGui::GetFrameHeight(), th=ImGui::GetTextLineHeight();
    float ty=15.f-th*0.5f, fy=15.f-fh*0.5f;   // 30px 헤더 세로 중앙 (text/frame 각각)
    ImGui::SetCursorPos(ImVec2(12, ty));
    ImGui::TextColored(ImVec4(0.5f,0.8f,1.f,1.f), "ACARS");
    ImGui::SameLine(0,16); ImGui::SetCursorPosY(ty);
    int total; { std::lock_guard<std::mutex> lk(v.acars_mtx); total=(int)v.acars_log.size(); }
    ImGui::Text("%d msg", total);
    ImGui::SameLine(0,20); ImGui::SetCursorPosY(fy);
    if(focus_filter) ImGui::SetKeyboardFocusHere();   // Ctrl+F / Tab 시 입력 활성화
    ImGui::SetNextItemWidth(220);
    ImGui::InputText("##flt", v.acars_filter, sizeof(v.acars_filter));   // 힌트 없음(빈칸)
    float cwb=ImGui::CalcTextSize("Clear").x + ImGui::GetStyle().FramePadding.x*2;
    ImGui::SameLine(); ImGui::SetCursorPos(ImVec2(W - cwb - 12, fy));    // Clear 우측정렬
    if(ImGui::Button("Clear")){ std::lock_guard<std::mutex> lk(v.acars_mtx); v.acars_log.clear(); has_sel=false; }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    float detail_h = has_sel ? 150.f : 0.f;
    float table_h  = H - 30 - detail_h - 16;   // 헤더30 + 블록간 ItemSpacing 보정 → 창 오버플로/휠스크롤 방지

    // ── 메시지 테이블 (가로+세로 스크롤, Text 외 중앙정렬, 좌측 3열 고정) ──
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
                         ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##acars_tbl", 11, tf, ImVec2(W, table_h))){
        ImGui::TableSetupScrollFreeze(3,1);   // Time/Freq/Reg + 헤더 고정
        ImGui::TableSetupColumn("Time",   ImGuiTableColumnFlags_WidthFixed, 68);
        ImGui::TableSetupColumn("Freq",   ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("Reg",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Type",   ImGuiTableColumnFlags_WidthFixed, 52);
        ImGui::TableSetupColumn("Flight", ImGuiTableColumnFlags_WidthFixed, 58);
        ImGui::TableSetupColumn("Country",ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("Airline",ImGuiTableColumnFlags_WidthFixed, 130);
        ImGui::TableSetupColumn("Link",   ImGuiTableColumnFlags_WidthFixed, 52);
        ImGui::TableSetupColumn("Lbl",    ImGuiTableColumnFlags_WidthFixed, 36);
        ImGui::TableSetupColumn("CRC",    ImGuiTableColumnFlags_WidthFixed, 44);
        ImGui::TableSetupColumn("Text",   ImGuiTableColumnFlags_WidthFixed, 2400);
        // 중앙정렬 헤더 + 클릭 정렬 (Text 만 좌측)
        ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
        for(int c=0;c<11;c++){
            ImGui::TableSetColumnIndex(c);
            const char* nm=ImGui::TableGetColumnName(c);
            char hdr[24];
            if(c==sort_col) snprintf(hdr,sizeof(hdr),"%s%s", nm, sort_asc?" ^":" v");
            else            snprintf(hdr,sizeof(hdr),"%s", nm);
            float cw=ImGui::GetContentRegionAvail().x;
            ImVec2 hp=ImGui::GetCursorPos();
            ImGui::PushID(c);
            if(ImGui::InvisibleButton("##h", ImVec2(cw>1?cw:1, ImGui::GetFrameHeight()))){
                if(sort_col==c) sort_asc=!sort_asc; else { sort_col=c; sort_asc=true; }
            }
            ImGui::PopID();
            float tw=ImGui::CalcTextSize(hdr).x;
            float ox=(c==10)?0.f:(tw<cw?(cw-tw)*0.5f:0.f);
            ImGui::SetCursorPos(ImVec2(hp.x+ox, hp.y+2));
            ImGui::TextUnformatted(hdr);
        }

        std::lock_guard<std::mutex> lk(v.acars_mtx);
        static std::vector<int> vis; vis.clear();
        for(int i=0;i<(int)v.acars_log.size();i++) if(match(v.acars_log[i], v.acars_filter)) vis.push_back(i);
        if(sort_col>=0 && vis.size()>1)
            std::stable_sort(vis.begin(), vis.end(), [&](int a, int b){
                int cmp=col_cmp(sort_col, v.acars_log[a], v.acars_log[b]);
                return sort_asc ? cmp<0 : cmp>0;
            });

        ImGuiListClipper clip; clip.Begin((int)vis.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const AcarsMsg& m = v.acars_log[vis[r]];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            float cw0 = ImGui::GetContentRegionAvail().x;   // col0 폭 (selectable 전에 캡처)
            ImVec2 rp = ImGui::GetCursorPos();
            bool is_sel = has_sel && m.t_ms==sel.t_ms && !strcmp(m.reg,sel.reg) && !strcmp(m.text,sel.text);
            ImGui::PushID(vis[r]);
            if(ImGui::Selectable("##s", is_sel,
                 ImGuiSelectableFlags_SpanAllColumns|ImGuiSelectableFlags_AllowOverlap)){ sel=m; has_sel=true; }
            ImGui::PopID();
            char ts[12]; hms(m.t_ms, ts);
            float tw0=ImGui::CalcTextSize(ts).x;
            ImGui::SetCursorPos(ImVec2(rp.x + (tw0<cw0?(cw0-tw0)*0.5f:0.f), rp.y));   // Time 중앙, 같은 줄 (selectable 위 overlay)
            ImGui::TextUnformatted(ts);
            ImGui::TableSetColumnIndex(1);
            char fq[16]; snprintf(fq,sizeof(fq),"%.3f",m.freq); ctr(fq);              // Freq
            ImGui::TableSetColumnIndex(2); ctr(m.reg);                                // Reg
            ImGui::TableSetColumnIndex(3); ctrc(acars_db_type(m.reg), ImVec4(0.72f,0.85f,1.f,1.f)); // Type
            ImGui::TableSetColumnIndex(4); ctr(m.flight);                             // Flight
            ImGui::TableSetColumnIndex(5); ctrc(acars_country(m.reg), ImVec4(0.6f,0.6f,0.6f,1.f)); // Country
            ImGui::TableSetColumnIndex(6); ctrc(acars_airline_of(m), ImVec4(0.62f,0.7f,0.62f,1.f)); // Airline
            ImGui::TableSetColumnIndex(7);
            ctrc(m.downlink?"Down":"Up", m.downlink?ImVec4(0.5f,0.8f,1.f,1.f):ImVec4(0.7f,0.7f,0.75f,1.f)); // Link
            ImGui::TableSetColumnIndex(8); ctr(m.label);                              // Lbl
            ImGui::TableSetColumnIndex(9);
            ctrc(m.crc_ok?"OK":"FAIL", m.crc_ok?ImVec4(0.4f,0.85f,0.4f,1.f):ImVec4(0.85f,0.5f,0.35f,1.f)); // CRC
            ImGui::TableSetColumnIndex(10);                                           // Text (좌측정렬)
            if(m.crc_ok) ImGui::TextUnformatted(m.text);
            else { ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f,0.6f,0.6f,1.f)); ImGui::TextUnformatted(m.text); ImGui::PopStyleColor(); }
        }
        // tail 스크롤: 맨 아래면 새 메시지 자동 따라감, 위로 올려두면 유지(수동), 맨 아래 복귀 시 재개
        static bool s_at_bottom=true;
        bool forced=false;
        if(v.acars_scroll){
            if(s_at_bottom && sort_col<0){ ImGui::SetScrollHereY(1.0f); forced=true; }
            v.acars_scroll=false;
        }
        if(forced) s_at_bottom=true;
        else s_at_bottom = (ImGui::GetScrollMaxY()<=0.0f) || (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()-4.0f);
        ImGui::EndTable();
    }

    // ── 상세 패널 ──
    if(has_sel){
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f,0.09f,0.12f,1.f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12,10));   // 내부 여백 (좌/상 딱붙음 방지)
        ImGui::BeginChild("##acars_detail", ImVec2(W, detail_h), true);
        const char* cc=acars_country(sel.reg); const char* al=acars_airline_of(sel);
        const char* ty=acars_db_type(sel.reg);
        char tsd[12]; hms(sel.t_ms, tsd);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);   // 일반 값 색
        // 표와 동일 순서/구성: Time Freq Reg Type Flight Country Airline Link Lbl CRC
        ImGui::Text("Time:");    ImGui::SameLine(); ImGui::TextColored(V,"%s", tsd);
        ImGui::SameLine(0,16); ImGui::Text("Freq:");   ImGui::SameLine(); ImGui::TextColored(V,"%.3f", sel.freq);
        ImGui::SameLine(0,16); ImGui::Text("Reg:");    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.5f,0.8f,1.f,1.f),"%s", sel.reg[0]?sel.reg:"-");
        ImGui::SameLine(0,16); ImGui::Text("Type:");   ImGui::SameLine(); ImGui::TextColored(ImVec4(0.72f,0.85f,1.f,1.f),"%s", ty[0]?ty:"-");
        ImGui::SameLine(0,16); ImGui::Text("Flight:"); ImGui::SameLine(); ImGui::TextColored(V,"%s", sel.flight[0]?sel.flight:"-");
        ImGui::SameLine(0,16); ImGui::Text("Country:");ImGui::SameLine(); ImGui::TextDisabled("%s", cc[0]?cc:"-");
        ImGui::SameLine(0,16); ImGui::Text("Airline:");ImGui::SameLine(); ImGui::TextColored(ImVec4(0.62f,0.7f,0.62f,1.f),"%s", al[0]?al:"-");
        ImGui::SameLine(0,16); ImGui::Text("Link:");   ImGui::SameLine(); ImGui::TextColored(sel.downlink?ImVec4(0.5f,0.8f,1.f,1.f):ImVec4(0.7f,0.7f,0.75f,1.f),"%s", sel.downlink?"Down":"Up");
        ImGui::SameLine(0,16); ImGui::Text("Lbl:");    ImGui::SameLine(); ImGui::TextUnformatted(sel.label[0]?sel.label:"-");
        ImGui::SameLine(0,16); ImGui::Text("CRC:");    ImGui::SameLine(); ImGui::TextColored(sel.crc_ok?ImVec4(0.4f,0.85f,0.4f,1.f):ImVec4(0.85f,0.5f,0.35f,1.f),"%s", sel.crc_ok?"OK":"FAIL");
        ImGui::SameLine(); ImGui::SetCursorPosX(W-72);
        if(ImGui::SmallButton("Close")) has_sel=false;
        ImGui::Separator();
        ImGui::TextWrapped("%s", sel.text);
        ImGui::EndChild();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
    }

    ImGui::End();
    ImGui::PopStyleVar();
}
