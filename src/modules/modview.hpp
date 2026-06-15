#pragma once
// ── 해독 모듈 공용 데이터뷰 위젯 (GUI 전용, *_view.cpp 에서만 include) ───────
// 모든 해독 모듈(acars/ais/wifi/…)의 데이터 뷰가 동일한 헤더바·셀정렬·행 다중선택·
// 세부창·복사 동작을 "한 구현"으로 공유하기 위한 재사용 컴포넌트.
// 표준 동작 기준 = ACARS 뷰 (이 헬퍼들은 거기서 추출). 새 해독 모듈은 이 헬퍼만
// 호출하면 자동으로 통일성을 갖춘다 (CLAUDE.md/BE_WE.md "해독 모듈 UI 통일" 참조).
#include "fft_viewer.hpp"
#include "module_api.hpp"
#include <imgui.h>
#include <string>
#include <vector>
#include <set>
#include <functional>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cctype>

namespace modview {

// ── 셀 텍스트 정렬 ──────────────────────────────────────────────────────────
inline void cell(const char* s){
    float av=ImGui::GetContentRegionAvail().x, tw=ImGui::CalcTextSize(s).x;
    if(tw<av) ImGui::SetCursorPosX(ImGui::GetCursorPosX()+(av-tw)*0.5f);
    ImGui::TextUnformatted(s);
}
inline void cell(const char* s, const ImVec4& col){
    ImGui::PushStyleColor(ImGuiCol_Text, col); cell(s); ImGui::PopStyleColor();
}
inline void cell_left(const char* s, const ImVec4* col=nullptr){
    if(col){ ImGui::PushStyleColor(ImGuiCol_Text,*col); ImGui::TextUnformatted(s); ImGui::PopStyleColor(); }
    else ImGui::TextUnformatted(s);
}

// ── 대소문자 무시 부분일치 (필터 공용) ──────────────────────────────────────
inline bool ci_find(const char* hay, const char* nee){
    if(!nee||!nee[0]) return true;
    if(!hay) return false;
    char h[320]; int i=0; for(; hay[i]&&i<319; i++) h[i]=(char)tolower((unsigned char)hay[i]); h[i]=0;
    char n[80];  int j=0; for(; nee[j]&&j<79;  j++) n[j]=(char)tolower((unsigned char)nee[j]); n[j]=0;
    return strstr(h,n)!=nullptr;
}

// ── 행 다중선택 알고리즘 (ACARS 동작 표준, 단일 구현) ───────────────────────
// 단일클릭=단독선택(이미 단독이면 해제) / Ctrl=토글 / Shift=anchor~행 범위(가산).
inline void apply_click(std::set<std::string>& keys, std::string& anchor,
                        const std::string& k, int vis_pos,
                        const std::function<std::string(int)>& key_at, int vis_n,
                        bool ctrl, bool shift){
    if(shift && !anchor.empty()){
        int a=-1; for(int i=0;i<vis_n;i++) if(key_at(i)==anchor){ a=i; break; }
        if(a>=0){ int lo=a<vis_pos?a:vis_pos, hi=a<vis_pos?vis_pos:a;
            for(int i=lo;i<=hi;i++) keys.insert(key_at(i)); }
        else { keys.clear(); keys.insert(k); anchor=k; }
    } else if(ctrl){
        if(keys.count(k)) keys.erase(k); else keys.insert(k);
        anchor=k;
    } else {
        if(keys.size()==1 && keys.count(k)){ keys.clear(); anchor.clear(); } // 재클릭 → 해제
        else { keys.clear(); keys.insert(k); anchor=k; }
    }
}

// 선택 상태 한 덩어리 (새 모듈용 — focus=세부창 표시 기준 행).
struct Selection {
    std::set<std::string> keys;
    std::string anchor, focus;
    bool        has_focus=false;
    bool selected(const std::string& k) const { return keys.count(k)!=0; }
    void clear(){ keys.clear(); anchor.clear(); focus.clear(); has_focus=false; }
    void click(const std::string& k, int vis_pos,
               const std::function<std::string(int)>& key_at, int vis_n, bool ctrl, bool shift){
        apply_click(keys, anchor, k, vis_pos, key_at, vis_n, ctrl, shift);
        focus=k; has_focus=!keys.empty();
    }
};

// SpanAllColumns 선택 가능 행 (overlay 허용). 클릭되면 true.
inline bool row_select(int uid, bool selected){
    ImGui::PushID(uid);
    bool clicked = ImGui::Selectable("##s", selected,
        ImGuiSelectableFlags_SpanAllColumns|ImGuiSelectableFlags_AllowOverlap);
    ImGui::PopID();
    return clicked;
}
// col0: selectable + 첫 셀 텍스트 중앙정렬 overlay. 반환=클릭됨.
inline bool row_col0(int uid, bool selected, const char* col0_text){
    ImGui::TableSetColumnIndex(0);
    float cw=ImGui::GetContentRegionAvail().x; ImVec2 rp=ImGui::GetCursorPos();
    bool clicked=row_select(uid, selected);
    float tw=ImGui::CalcTextSize(col0_text).x;
    ImGui::SetCursorPos(ImVec2(rp.x+(tw<cw?(cw-tw)*0.5f:0.f), rp.y));
    ImGui::TextUnformatted(col0_text);
    return clicked;
}

// ── 헤더바: 좌[filter | N msg | loading]  우[Recv | Clear] ──────────────────
// on_clear: Clear 누름 OR Recv 켤 때 호출 (로그+선택 비우기). focus_filter: Ctrl+F/Tab.
inline void header_bar(FFTViewer& v, const char* id, char* filter, size_t cap,
                       int count, bool remote, bool focus_filter,
                       const std::function<void()>& on_clear){
    float W = ImGui::GetContentRegionAvail().x;
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f,0.12f,0.16f,1.f));
    char cid[40]; snprintf(cid,sizeof(cid),"##%s_hdr",id);
    ImGui::BeginChild(cid, ImVec2(W,30), false);
    float fh=ImGui::GetFrameHeight(), th=ImGui::GetTextLineHeight();
    float tcy=15.f-th*0.5f, fy=15.f-fh*0.5f;
    ImGui::SetCursorPos(ImVec2(12, fy));
    if(focus_filter) ImGui::SetKeyboardFocusHere();
    ImGui::SetNextItemWidth(220);
    char fid[40]; snprintf(fid,sizeof(fid),"##%s_flt",id);
    ImGui::InputText(fid, filter, cap);                       // 힌트 없음
    ImGui::SameLine(0,16); ImGui::SetCursorPosY(tcy); ImGui::Text("%d msg", count);
    if(bewe_mod_hist_loading(id)){ ImGui::SameLine(0,12); ImGui::SetCursorPosY(tcy); ImGui::TextDisabled("loading..."); }
    float cwb=ImGui::CalcTextSize("Clear").x   +ImGui::GetStyle().FramePadding.x*2;
    float rwb=ImGui::CalcTextSize("Recv OFF").x+ImGui::GetStyle().FramePadding.x*2;  // 넓은쪽 기준 고정폭
    if(remote){
        bool rcv=bewe_mod_recv(id);
        ImGui::SameLine(); ImGui::SetCursorPos(ImVec2(W-cwb-rwb-20, fy));
        // 기본 Recv OFF(빨강) → 켜면 Recv ON(초록)
        ImGui::PushStyleColor(ImGuiCol_Button, rcv?ImVec4(0.15f,0.55f,0.2f,1.f):ImVec4(0.6f,0.15f,0.15f,1.f));
        if(ImGui::Button(rcv?"Recv ON":"Recv OFF")){ if(!rcv) on_clear(); bewe_mod_set_recv(v,id,!rcv); }
        ImGui::PopStyleColor();
        ImGui::SameLine(); ImGui::SetCursorPos(ImVec2(W-cwb-12, fy));
    } else { ImGui::SameLine(); ImGui::SetCursorPos(ImVec2(W-cwb-12, fy)); }
    if(ImGui::Button("Clear")) on_clear();
    ImGui::EndChild();
    ImGui::PopStyleColor();
}

// 스페이스바 → Recv 토글 (이 탭 활성 + 텍스트입력 아님). 버튼과 동일 동작.
inline void space_toggle_recv(FFTViewer& v, const char* id, bool remote, bool win_focus,
                              const std::function<void()>& on_clear){
    if(!remote || !win_focus) return;
    ImGuiIO& io=ImGui::GetIO();
    if(io.WantTextInput) return;
    if(ImGui::IsKeyPressed(ImGuiKey_Space,false)){
        bool rcv=bewe_mod_recv(id);
        if(!rcv) on_clear();
        bewe_mod_set_recv(v,id,!rcv);
    }
}

// ── 3-state 정렬 헤더 (오름차 → 내림차 → 원래순). text_col 은 좌측정렬 ─────────
inline void sortable_headers(int ncol, int& sort_col, bool& sort_asc, int text_col=-1){
    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
    for(int c=0;c<ncol;c++){
        ImGui::TableSetColumnIndex(c);
        const char* nm=ImGui::TableGetColumnName(c);
        char hdr[48];
        if(c==sort_col) snprintf(hdr,sizeof(hdr),"%s%s",nm,sort_asc?" ^":" v");
        else            snprintf(hdr,sizeof(hdr),"%s",nm);
        float cw=ImGui::GetContentRegionAvail().x; ImVec2 hp=ImGui::GetCursorPos();
        ImGui::PushID(c);
        if(ImGui::InvisibleButton("##h", ImVec2(cw>1?cw:1, ImGui::GetFrameHeight()))){
            if(sort_col!=c){ sort_col=c; sort_asc=true; }   // 1) 오름차순
            else if(sort_asc) sort_asc=false;               // 2) 내림차순
            else sort_col=-1;                               // 3) 원래순
        }
        ImGui::PopID();
        float tw=ImGui::CalcTextSize(hdr).x;
        float ox=(c==text_col)?0.f:(tw<cw?(cw-tw)*0.5f:0.f);
        ImGui::SetCursorPos(ImVec2(hp.x+ox, hp.y+2));
        ImGui::TextUnformatted(hdr);
    }
}

// vis 인덱스를 col_cmp 로 안정정렬. cmp(col,a,b) = -/0/+ (a,b = 원본 인덱스).
template<class Cmp>
inline void sort_vis(std::vector<int>& vis, int sort_col, bool asc, Cmp cmp){
    if(sort_col>=0 && vis.size()>1)
        std::stable_sort(vis.begin(),vis.end(),
            [&](int a,int b){ int c=cmp(sort_col,a,b); return asc? c<0 : c>0; });
}

// 원래상태(append) 뷰의 tail 자동 따라가기. table 루프 직후 EndTable 전에 호출.
inline void tail_follow(bool& at_bottom, bool grew){
    if(grew && at_bottom) ImGui::SetScrollHereY(1.0f);
    at_bottom = (ImGui::GetScrollMaxY()<=0.0f) || (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()-4.0f);
}

// ── 세부 패널 chrome ────────────────────────────────────────────────────────
inline void detail_begin(const char* id, float x0, float W, float h){
    ImGui::SetCursorPosX(x0);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f,0.09f,0.12f,1.f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12,10));
    char cid[40]; snprintf(cid,sizeof(cid),"##%s_detail",id);
    ImGui::BeginChild(cid, ImVec2(W,h), true);
}
inline void detail_end(){ ImGui::EndChild(); ImGui::PopStyleVar(); ImGui::PopStyleColor(); }
// 읽기전용 복사가능 본문 텍스트 (남은 영역 채움)
inline void detail_body(const char* uid, const char* text){
    static char buf[2048];
    snprintf(buf,sizeof(buf),"%s",text?text:"");
    ImVec2 ta=ImGui::GetContentRegionAvail();
    ImGui::InputTextMultiline(uid, buf, sizeof(buf),
        ImVec2(ta.x, ta.y>0?ta.y:ImGui::GetTextLineHeight()*2), ImGuiInputTextFlags_ReadOnly);
}
// 세부창 "키:값" 한 항목 (같은 줄 누적). 첫 항목은 same_line=false.
inline void kv(const char* key, const char* val, const ImVec4& vcol, bool same_line=true){
    if(same_line) ImGui::SameLine(0,16);
    ImGui::Text("%s", key); ImGui::SameLine(); ImGui::TextColored(vcol, "%s", (val&&val[0])?val:"-");
}

} // namespace modview
