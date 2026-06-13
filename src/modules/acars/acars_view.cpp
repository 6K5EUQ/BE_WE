// ── ACARS 모듈 GUI: DEMOD 패널 데이터 뷰 탭 ─────────────────────────────────
#include "fft_viewer.hpp"
#include "acars_module.hpp"
#include "acars_db.hpp"
#include "module_api.hpp"
#include <imgui.h>
#include <cstring>
#include <cctype>
#include <vector>
#include <algorithm>
#include <set>
#include <string>

namespace acars_mod {

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
        || ci_find(m.station,f)
        || ci_find(m.label,f) || ci_find(m.crc_ok?"OK":"FAIL", f) || ci_find(m.text,f);
}
// 셀 내 중앙정렬 텍스트
void ctr(const char* s){
    float avail=ImGui::GetContentRegionAvail().x, tw=ImGui::CalcTextSize(s).x;
    if(tw<avail) ImGui::SetCursorPosX(ImGui::GetCursorPosX()+(avail-tw)*0.5f);
    ImGui::TextUnformatted(s);
}
void ctrc(const char* s, ImVec4 col){ ImGui::PushStyleColor(ImGuiCol_Text,col); ctr(s); ImGui::PopStyleColor(); }
// 컬럼별 정렬 비교 (헤더 인덱스 순서)
// 0 Time / 1 Freq / 2 Reg / 3 Type / 4 Flight / 5 Country / 6 Airline / 7 Link / 8 Station / 9 Lbl / 10 CRC / 11 Text
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
        case 8:  return strcmp(a.station,b.station);
        case 9:  return strcmp(a.label,b.label);
        case 10: return (int)a.crc_ok-(int)b.crc_ok;
        default: return strcmp(a.text,b.text);
    }
}
} // anonymous namespace

void draw_content(FFTViewer& v, bool just_opened){
    ImGuiIO& io = ImGui::GetIO();
    bool remote = v.remote_mode;
    if(just_opened && !remote) local_load_today(v);   // LOCAL: 오늘 아카이브 로드

    ImVec2 avail = ImGui::GetContentRegionAvail();
    float W = avail.x, H = avail.y;
    float x0 = ImGui::GetCursorPosX();

    // Ctrl+F / Tab → 필터 입력창 포커스 (이 탭이 활성일 때만 호출됨)
    bool focus_filter =
        (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F, false)) || ImGui::IsKeyPressed(ImGuiKey_Tab, false);

    // 다중 선택: 행 식별 키 = t_ms|reg|station|text (재정렬/front-erase 안전).
    // focus = 마지막 클릭 행 (상세창 표시 + Shift range anchor). 미션창 파일 선택과 동일.
    static std::set<std::string> sel_keys;
    static AcarsMsg focus{}; static bool has_focus=false;
    static std::string anchor_key;          // Shift+클릭 range 기준
    static int  sort_col=-1;     // -1=원래상태(수신순), 0~11=해당 컬럼
    static bool sort_asc=true;
    auto msg_key=[](const AcarsMsg& m){
        char b[400]; snprintf(b,sizeof(b),"%lld|%s|%s|%s",(long long)m.t_ms,m.reg,m.station,m.text);
        return std::string(b);
    };
    bool win_focus = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    // 스페이스바 → Recv 토글 (이 탭 활성 + 텍스트 입력 중 아님). 버튼과 동일 동작, 모듈별 독립.
    if(remote && win_focus && !io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Space, false)){
        bool rcv = bewe_mod_recv("acars");
        if(!rcv){ std::lock_guard<std::mutex> lk(mtx); msglog.clear(); sel_keys.clear(); has_focus=false; }
        bewe_mod_set_recv(v, "acars", !rcv);
    }

    // Ctrl+C → 선택된 행 전체(모든 컬럼, 탭 구분)를 클립보드 복사. 한 행당 한 줄.
    // (상세창 텍스트에 포커스면 io.WantTextInput=true → InputText 가 Ctrl+C 처리하므로 여기선 무시)
    if(win_focus && !io.WantTextInput && io.KeyCtrl && !io.KeyShift
       && ImGui::IsKeyPressed(ImGuiKey_C, false) && !sel_keys.empty()){
        std::string out;
        std::lock_guard<std::mutex> lk(mtx);
        for(const AcarsMsg& m : msglog){
            if(!sel_keys.count(msg_key(m))) continue;
            char ts[12]; hms(m.t_ms, ts);
            char ty[8],co[40],al[80];   // thread_local 공유 가능성 → 각각 복사
            snprintf(ty,sizeof(ty),"%s",acars_db_type(m.reg));
            snprintf(co,sizeof(co),"%s",acars_country(m.reg));
            snprintf(al,sizeof(al),"%s",acars_airline_of(m));
            char line[640];
            snprintf(line,sizeof(line),"%s\t%.3f\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n",
                ts, m.freq, m.reg, ty, m.flight, co, al,
                m.crc_ok?(m.downlink?"Down":"Up"):"-", m.station, m.label, m.crc_ok?"OK":"FAIL", m.text);
            out += line;
        }
        if(!out.empty()) ImGui::SetClipboardText(out.c_str());
    }

    // ── 헤더 바 ──
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f,0.12f,0.16f,1.f));
    ImGui::BeginChild("##acars_hdr", ImVec2(W, 30), false);
    float fh=ImGui::GetFrameHeight(), th=ImGui::GetTextLineHeight();
    float ty=15.f-th*0.5f, fy=15.f-fh*0.5f;   // 30px 헤더 세로 중앙 (text/frame 각각)
    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)msglog.size(); }
    ImGui::SetCursorPos(ImVec2(12, fy));
    if(focus_filter) ImGui::SetKeyboardFocusHere();   // Ctrl+F / Tab 시 입력 활성화
    ImGui::SetNextItemWidth(220);
    ImGui::InputText("##flt", filter, sizeof(filter));   // 힌트 없음(빈칸)
    ImGui::SameLine(0,16); ImGui::SetCursorPosY(ty);
    ImGui::Text("%d msg", total);
    if(bewe_mod_hist_loading("acars")){ ImGui::SameLine(0,12); ImGui::SetCursorPosY(ty); ImGui::TextDisabled("loading..."); }
    // 우측: [Recv][Clear]
    float cwb = ImGui::CalcTextSize("Clear").x + ImGui::GetStyle().FramePadding.x*2;
    float rwb = ImGui::CalcTextSize("Recv").x  + ImGui::GetStyle().FramePadding.x*2;
    if(remote){
        bool rcv = bewe_mod_recv("acars");
        ImGui::SameLine(); ImGui::SetCursorPos(ImVec2(W - cwb - rwb - 20, fy));
        // Recv: 기본 빨강(미수신) / 초록(Central 수신 중)
        ImGui::PushStyleColor(ImGuiCol_Button, rcv?ImVec4(0.15f,0.55f,0.2f,1.f):ImVec4(0.6f,0.15f,0.15f,1.f));
        if(ImGui::Button("Recv")){
            if(!rcv){ std::lock_guard<std::mutex> lk(mtx); msglog.clear(); sel_keys.clear(); has_focus=false; }
            bewe_mod_set_recv(v, "acars", !rcv);   // on → 오늘 히스토리 + 라이브
        }
        ImGui::PopStyleColor();
        ImGui::SameLine(); ImGui::SetCursorPos(ImVec2(W - cwb - 12, fy));
    } else {
        ImGui::SameLine(); ImGui::SetCursorPos(ImVec2(W - cwb - 12, fy));
    }
    if(ImGui::Button("Clear")){ std::lock_guard<std::mutex> lk(mtx); msglog.clear(); sel_keys.clear(); has_focus=false; }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    float detail_h = has_focus ? 150.f : 0.f;
    float table_h  = H - 30 - detail_h - 16;   // 헤더30 + 블록간 ItemSpacing 보정 → 오버플로 방지
    if(table_h < 60) table_h = 60;

    // ── 메시지 테이블 (가로+세로 스크롤, Text 외 중앙정렬, 좌측 3열 고정) ──
    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
                         ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##acars_tbl", 12, tf, ImVec2(W, table_h))){
        ImGui::TableSetupScrollFreeze(3,1);   // Time/Freq/Reg + 헤더 고정
        ImGui::TableSetupColumn("Time",   ImGuiTableColumnFlags_WidthFixed, 68);
        ImGui::TableSetupColumn("Freq",   ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("Reg",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Type",   ImGuiTableColumnFlags_WidthFixed, 52);
        ImGui::TableSetupColumn("Flight", ImGuiTableColumnFlags_WidthFixed, 58);
        ImGui::TableSetupColumn("Country",ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("Airline",ImGuiTableColumnFlags_WidthFixed, 130);
        ImGui::TableSetupColumn("Link",   ImGuiTableColumnFlags_WidthFixed, 52);
        ImGui::TableSetupColumn("Station",ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("Lbl",    ImGuiTableColumnFlags_WidthFixed, 36);
        ImGui::TableSetupColumn("CRC",    ImGuiTableColumnFlags_WidthFixed, 44);
        ImGui::TableSetupColumn("Text",   ImGuiTableColumnFlags_WidthFixed, 2400);
        // 중앙정렬 헤더 + 클릭 정렬: 오름차순 → 내림차순 → 원래상태 (3단계 순환)
        ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
        for(int c=0;c<12;c++){
            ImGui::TableSetColumnIndex(c);
            const char* nm=ImGui::TableGetColumnName(c);
            char hdr[24];
            if(c==sort_col) snprintf(hdr,sizeof(hdr),"%s%s", nm, sort_asc?" ^":" v");
            else            snprintf(hdr,sizeof(hdr),"%s", nm);
            float cw=ImGui::GetContentRegionAvail().x;
            ImVec2 hp=ImGui::GetCursorPos();
            ImGui::PushID(c);
            if(ImGui::InvisibleButton("##h", ImVec2(cw>1?cw:1, ImGui::GetFrameHeight()))){
                if(sort_col!=c){ sort_col=c; sort_asc=true; }       // 1) 오름차순
                else if(sort_asc) sort_asc=false;                   // 2) 내림차순
                else sort_col=-1;                                   // 3) 원래상태 (tail 자동스크롤 복귀)
            }
            ImGui::PopID();
            float tw=ImGui::CalcTextSize(hdr).x;
            float ox=(c==11)?0.f:(tw<cw?(cw-tw)*0.5f:0.f);
            ImGui::SetCursorPos(ImVec2(hp.x+ox, hp.y+2));
            ImGui::TextUnformatted(hdr);
        }

        std::lock_guard<std::mutex> lk(mtx);
        static std::vector<int> vis; vis.clear();
        for(int i=0;i<(int)msglog.size();i++) if(match(msglog[i], filter)) vis.push_back(i);
        if(sort_col>=0 && vis.size()>1)
            std::stable_sort(vis.begin(), vis.end(), [&](int a, int b){
                int cmp=col_cmp(sort_col, msglog[a], msglog[b]);
                return sort_asc ? cmp<0 : cmp>0;
            });

        ImGuiListClipper clip; clip.Begin((int)vis.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const AcarsMsg& m = msglog[vis[r]];
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            float cw0 = ImGui::GetContentRegionAvail().x;   // col0 폭 (selectable 전에 캡처)
            ImVec2 rp = ImGui::GetCursorPos();
            bool row_sel = sel_keys.count(msg_key(m))>0;
            ImGui::PushID(vis[r]);
            if(ImGui::Selectable("##s", row_sel,
                 ImGuiSelectableFlags_SpanAllColumns|ImGuiSelectableFlags_AllowOverlap)){
                // 단일=선택, Ctrl=토글, Shift=anchor~클릭 range (미션창 파일 선택과 동일)
                std::string k = msg_key(m);
                if(io.KeyShift && !anchor_key.empty()){
                    int a=-1; for(int i=0;i<(int)vis.size();i++) if(msg_key(msglog[vis[i]])==anchor_key){ a=i; break; }
                    if(a>=0){ int lo=std::min(a,r), hi=std::max(a,r);
                        for(int i=lo;i<=hi;i++) sel_keys.insert(msg_key(msglog[vis[i]]));
                    } else { sel_keys.clear(); sel_keys.insert(k); anchor_key=k; }
                } else if(io.KeyCtrl){
                    if(row_sel) sel_keys.erase(k); else sel_keys.insert(k);
                    anchor_key=k;
                } else {
                    sel_keys.clear(); sel_keys.insert(k); anchor_key=k;
                }
                focus=m; has_focus=!sel_keys.empty();
            }
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
            if(m.crc_ok) ctrc(m.downlink?"Down":"Up", m.downlink?ImVec4(0.5f,0.8f,1.f,1.f):ImVec4(0.7f,0.7f,0.75f,1.f)); // Link (CRC OK 일 때만)
            ImGui::TableSetColumnIndex(8); ctrc(m.station, ImVec4(0.85f,0.75f,0.5f,1.f)); // Station
            ImGui::TableSetColumnIndex(9); ctr(m.label);                              // Lbl
            ImGui::TableSetColumnIndex(10);
            ctrc(m.crc_ok?"OK":"FAIL", m.crc_ok?ImVec4(0.4f,0.85f,0.4f,1.f):ImVec4(0.85f,0.5f,0.35f,1.f)); // CRC
            ImGui::TableSetColumnIndex(11);                                           // Text (좌측정렬)
            if(m.crc_ok) ImGui::TextUnformatted(m.text);
            else { ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f,0.6f,0.6f,1.f)); ImGui::TextUnformatted(m.text); ImGui::PopStyleColor(); }
        }
        // tail 스크롤: 원래상태(정렬 해제)에서 맨 아래면 새 메시지 자동 따라감
        static bool s_at_bottom=true;
        bool forced=false;
        if(scroll){
            if(s_at_bottom && sort_col<0){ ImGui::SetScrollHereY(1.0f); forced=true; }
            scroll=false;
        }
        if(forced) s_at_bottom=true;
        else s_at_bottom = (ImGui::GetScrollMaxY()<=0.0f) || (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()-4.0f);
        ImGui::EndTable();
    }

    // ── 상세 패널 (Close 버튼 없음. 텍스트 본문은 읽기전용 — 선택/복사 가능) ──
    if(has_focus){
        ImGui::SetCursorPosX(x0);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f,0.09f,0.12f,1.f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12,10));   // 내부 여백 (좌/상 딱붙음 방지)
        ImGui::BeginChild("##acars_detail", ImVec2(W, detail_h), true);
        const char* cc=acars_country(focus.reg); const char* al=acars_airline_of(focus);
        const char* ty2=acars_db_type(focus.reg);
        char tsd[12]; hms(focus.t_ms, tsd);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);   // 일반 값 색
        // 표와 동일 순서/구성: Time Freq Reg Type Flight Country Airline Link Station Lbl CRC
        ImGui::Text("Time:");    ImGui::SameLine(); ImGui::TextColored(V,"%s", tsd);
        ImGui::SameLine(0,16); ImGui::Text("Freq:");   ImGui::SameLine(); ImGui::TextColored(V,"%.3f", focus.freq);
        ImGui::SameLine(0,16); ImGui::Text("Reg:");    ImGui::SameLine(); ImGui::TextColored(ImVec4(0.5f,0.8f,1.f,1.f),"%s", focus.reg[0]?focus.reg:"-");
        ImGui::SameLine(0,16); ImGui::Text("Type:");   ImGui::SameLine(); ImGui::TextColored(ImVec4(0.72f,0.85f,1.f,1.f),"%s", ty2[0]?ty2:"-");
        ImGui::SameLine(0,16); ImGui::Text("Flight:"); ImGui::SameLine(); ImGui::TextColored(V,"%s", focus.flight[0]?focus.flight:"-");
        ImGui::SameLine(0,16); ImGui::Text("Country:");ImGui::SameLine(); ImGui::TextDisabled("%s", cc[0]?cc:"-");
        ImGui::SameLine(0,16); ImGui::Text("Airline:");ImGui::SameLine(); ImGui::TextColored(ImVec4(0.62f,0.7f,0.62f,1.f),"%s", al[0]?al:"-");
        ImGui::SameLine(0,16); ImGui::Text("Link:");   ImGui::SameLine(); ImGui::TextColored(focus.downlink?ImVec4(0.5f,0.8f,1.f,1.f):ImVec4(0.7f,0.7f,0.75f,1.f),"%s", focus.crc_ok?(focus.downlink?"Down":"Up"):"-");
        ImGui::SameLine(0,16); ImGui::Text("Station:");ImGui::SameLine(); ImGui::TextColored(ImVec4(0.85f,0.75f,0.5f,1.f),"%s", focus.station[0]?focus.station:"-");
        ImGui::SameLine(0,16); ImGui::Text("Lbl:");    ImGui::SameLine(); ImGui::TextUnformatted(focus.label[0]?focus.label:"-");
        ImGui::SameLine(0,16); ImGui::Text("CRC:");    ImGui::SameLine(); ImGui::TextColored(focus.crc_ok?ImVec4(0.4f,0.85f,0.4f,1.f):ImVec4(0.85f,0.5f,0.35f,1.f),"%s", focus.crc_ok?"OK":"FAIL");
        ImGui::Separator();
        // 텍스트 본문: 읽기전용 — 마우스 선택 + Ctrl+C / Ctrl+A 로 복사 (수정 불가, 외부 추출용)
        static char txtbuf[300];
        snprintf(txtbuf, sizeof(txtbuf), "%s", focus.text);
        ImVec2 ta = ImGui::GetContentRegionAvail();
        ImGui::InputTextMultiline("##acars_txt", txtbuf, sizeof(txtbuf),
            ImVec2(ta.x, ta.y>0?ta.y:ImGui::GetTextLineHeight()*2), ImGuiInputTextFlags_ReadOnly);
        ImGui::EndChild();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
    }
}

} // namespace acars_mod
