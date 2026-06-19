// ── DEMOD 패널 (코어, 모듈 무관) ────────────────────────────────────────────
// 하단바 DEMOD 버튼 / D 키 → 이 패널.
// Modules 탭: 좌측 모듈 목록(데이터 뷰 열기용) + 우측 통합 채널 타깃 테이블.
//   - 우측은 어느 모듈을 클릭했는지와 무관한 단일 통합 목록: 전 스테이션 채널필터 한 줄씩.
//   - 채널 행마다 Decode 콤보로 그 채널에 돌릴 복조 모듈을 고른다 (None / 모드 적합 모듈).
//     한 채널 = 한 모듈만 (콤보가 기존 모듈 stop → 새 모듈 start). 전 유저 동기화.
//   - 타깃 목록은 Central 집계 (CH_LIST, 2초 폴링). LOCAL 은 로컬 채널.
// 모듈별 데이터 뷰는 탭으로 (다중 오픈, 전환).
#include "fft_viewer.hpp"
#include "module_api.hpp"
#include <imgui.h>
#include <imgui_internal.h>
#include <cstring>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <chrono>

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
// 천단위 콤마 (1234567 → "1,234,567")
static void fmt_commas(long n, char* out, size_t cap){
    char t[24]; int len=snprintf(t,sizeof(t),"%ld", n<0?0:n);
    size_t o=0;
    for(int i=0;i<len && o+1<cap;i++){
        if(i>0 && (len-i)%3==0 && o+1<cap) out[o++]=',';
        out[o++]=t[i];
    }
    out[o]=0;
}
// 초 → HH:MM:SS
static void fmt_hms(int64_t sec, char* out, size_t cap){
    if(sec<0) sec=0;
    snprintf(out,cap,"%02d:%02d:%02d",(int)(sec/3600),(int)((sec%3600)/60),(int)(sec%60));
}

// ── 통합 타깃 테이블 (모듈 선택 무관): 전 스테이션 채널필터 1목록 + 행별 Decode 콤보 ──
// 한 채널 = 한 복조 모듈. 콤보에서 전 복조 모듈 중 아무거나 고르면(모드 무관 — 워커가 IQ
// 직접 탭) 기존 모듈 stop 후 새 모듈 start (None = 정지). 어느 기지든 동일 제어, 전 유저 동기화.
// Run Time 로컬 보간: 호스트 dec_runtime_s 는 폴링(~2s)마다 갱신 → 갱신 사이엔
//   로컬 시계로 부드럽게 틱(anchor=수신시 now-runtime). 새 폴링값 오면 재동기(드리프트 보정).
struct RunAnchor{ uint32_t last=0xFFFFFFFFu; int64_t anchor=0; };

static void draw_targets(FFTViewer& v){
    static std::map<std::string,RunAnchor> s_run;   // station#ch → Run Time anchor
    auto& mods = bewe_modules();
    // 복조 가능한(설치+채널타깃형) 모듈 인덱스
    std::vector<int> dm;
    for(size_t i=0;i<mods.size();i++) if(!mods[i].planned && mods[i].target_modes) dm.push_back((int)i);

    // 원격: 모든 복조 모듈의 CH_LIST 주기 폴링 (통합 목록이라 전부 필요)
    if(v.remote_mode){
        static double s_last=0; double now=ImGui::GetTime();
        if(now-s_last>2.0){ for(int mi:dm) bewe_mod_req_ch_list(mods[mi].id); s_last=now; }
    }

    // 채널 행 통합 (key = station|ch). freq/mode 는 채널 고유, running = decode_on 인 모듈.
    struct Row{ std::string station; int ch; uint8_t mode; float lo,hi; int running; int hold; float cf_mhz, sr_msps; int dnum; uint32_t dcount, druns; };
    std::vector<Row> rows;
    for(int mi:dm){
        auto ts = bewe_mod_targets(v, mods[mi].id);
        for(auto& e : ts){
            Row* r=nullptr;
            for(auto& x:rows) if(x.ch==(int)e.ch && x.station==e.station){ r=&x; break; }
            if(!r){ rows.push_back({e.station,(int)e.ch,e.mode,e.lo,e.hi,-1,(int)e.hold,e.cf_mhz,e.sr_msps,(int)e.dnum,e.dec_count,e.dec_runtime_s}); r=&rows.back(); }
            if(e.decode_on) r->running = mi;
            if(e.dec_count) r->dcount = e.dec_count;          // 디코드 통계 = 채널 단위(어느 모듈 행이든 동일)
            if(e.dec_runtime_s) r->druns = e.dec_runtime_s;
        }
    }
    // 기지별 그룹 + 주파수정렬 표시번호(dnum) 순 → State창/스펙트럼 라벨과 동일 순서/번호
    std::sort(rows.begin(),rows.end(),[](const Row&a,const Row&b){
        if(a.station!=b.station) return a.station<b.station;
        if(a.dnum!=b.dnum) return a.dnum<b.dnum;
        return a.ch<b.ch; });

    // 데이터 컬럼은 고정폭, 남는 폭은 끝의 빈 stretch 컬럼이 흡수 → Center 안 늘어나고 Decode 안 짤림
    ImGuiTableFlags tf = ImGuiTableFlags_RowBg|ImGuiTableFlags_BordersInner|
                         ImGuiTableFlags_BordersOuter|ImGuiTableFlags_ScrollY;
    float th=ImGui::GetContentRegionAvail().y-8;
    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(6,5));
    int64_t now_ms=(int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if(ImGui::BeginTable("##uni_targets", 10, tf, ImVec2(0, th>120?th:120))){
        ImGui::TableSetupColumn("CH",           ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Center (MHz)", ImGuiTableColumnFlags_WidthFixed, 104);
        ImGui::TableSetupColumn("BW (kHz)",     ImGuiTableColumnFlags_WidthFixed, 96);
        ImGui::TableSetupColumn("Mode",         ImGuiTableColumnFlags_WidthFixed, 66);
        ImGui::TableSetupColumn("Channel",      ImGuiTableColumnFlags_WidthFixed, 72);
        ImGui::TableSetupColumn("State",        ImGuiTableColumnFlags_WidthFixed, 56);
        ImGui::TableSetupColumn("Data",         ImGuiTableColumnFlags_WidthFixed, 96);
        ImGui::TableSetupColumn("Run Time",     ImGuiTableColumnFlags_WidthFixed, 78);
        ImGui::TableSetupColumn("Decode",       ImGuiTableColumnFlags_WidthFixed, 152);
        ImGui::TableSetupColumn("##sp",         ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
        const char* hn[10]={"CH","Center (MHz)","BW (kHz)","Mode","Channel","State","Data","Run Time","Decode",""};
        const bool  hc[10]={true,true,true,true,true,true,true,true,false,false};
        for(int c=0;c<10;c++){ ImGui::TableSetColumnIndex(c); if(hc[c]) cell_ctr(hn[c]); else ImGui::TextUnformatted(hn[c]); }

        if(rows.empty()){
            ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("(no channels)");
        }
        for(size_t i=0;i<rows.size();i++){
            Row& r=rows[i];
            // ── 기지 그룹 헤더 (기지 바뀔 때마다) : 기지명 + CF/SR 편집 (어느 기지든 원격 동기화) ──
            if(i==0 || rows[i-1].station != r.station){
                // ── 기지 그룹 간 간격 (시각 분리) ──
                if(i>0){ ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::Dummy(ImVec2(0,4)); }
                ImGui::TableNextRow();
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(22,44,68,255)); // 기지헤더 밴드
                ImGui::PushID(("grp#"+r.station).c_str());
                ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(0.17f,0.20f,0.25f,1.0f));
                ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.22f,0.26f,0.32f,1.0f));
                char gsd[16]; station_disp(r.station.c_str(), gsd, sizeof(gsd));
                // ── 기지 헤더: [기지명 CF SR] 전체를 한 줄로 묶어 전체폭 중앙정렬 ──
                //    (컬럼 그리드에 비종속 — 테이블 WorkRect 전체폭에 클립 확장 후 그룹 배치)
                ImGui::TableSetColumnIndex(0);
                float rowY = ImGui::GetCursorScreenPos().y, frH = ImGui::GetFrameHeight();
                ImGui::Dummy(ImVec2(0, frH));                            // 행 높이 확보
                ImGuiTable* tbl = ImGui::GetCurrentTable();
                float x0=tbl->WorkRect.Min.x, x1=tbl->WorkRect.Max.x;
                const float inpCF=92.f, inpSR=72.f, sp=12.f, lp=4.f;
                ImVec2 nts=ImGui::CalcTextSize(gsd), cfl=ImGui::CalcTextSize("CF"), srl=ImGui::CalcTextSize("SR");
                float gw = nts.x + sp + cfl.x+lp+inpCF + sp + srl.x+lp+inpSR;
                float sx = x0 + ((x1-x0)-gw)*0.5f; if(sx<x0+4) sx=x0+4;
                ImGui::PushClipRect(ImVec2(x0,rowY-2), ImVec2(x1,rowY+frH+2), false);
                ImGui::SetCursorScreenPos(ImVec2(sx, rowY));
                ImGui::AlignTextToFramePadding();
                ImGui::TextColored(ImVec4(0.50f,0.86f,1.0f,1.f), "%s", gsd);   // 기지명
                ImGui::SameLine(0,sp);
                ImGui::AlignTextToFramePadding(); ImGui::TextUnformatted("CF"); ImGui::SameLine(0,lp);
                { float cfv=r.cf_mhz; ImGui::SetNextItemWidth(inpCF);
                  ImGui::InputFloat("##scf",&cfv,0,0,"%.4f");
                  if(ImGui::IsItemDeactivatedAfterEdit() && cfv>0.f) bewe_mod_tune(v, r.station.c_str(), cfv, 0.f); }
                ImGui::SameLine(0,sp);
                ImGui::AlignTextToFramePadding(); ImGui::TextUnformatted("SR"); ImGui::SameLine(0,lp);
                { float srv=r.sr_msps; ImGui::SetNextItemWidth(inpSR);
                  ImGui::InputFloat("##ssr",&srv,0,0,"%.3f");
                  if(ImGui::IsItemDeactivatedAfterEdit() && srv>0.f) bewe_mod_tune(v, r.station.c_str(), 0.f, srv); }
                ImGui::PopClipRect();
                ImGui::PopStyleColor(2);
                ImGui::PopID();
            }
            ImGui::TableNextRow();
            // ── Holding/Active 행 미세 구분 (Active=살짝 청록 / Holding=살짝 호박·흐림) ──
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                r.hold ? IM_COL32(48,38,22,55) : IM_COL32(20,42,40,60));
            char rid[64]; snprintf(rid,sizeof(rid),"%s#%d",r.station.c_str(),r.ch);
            ImGui::PushID(rid);                                  // 정렬 순서 무관 안정 ID (편집 중 끊김 방지)
            // 편집 위젯 공용 대비 스타일 (input/combo 프레임 + 팝업)
            ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(0.17f,0.20f,0.25f,1.0f));
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(0.22f,0.26f,0.32f,1.0f));
            ImGui::PushStyleColor(ImGuiCol_PopupBg,        ImVec4(0.11f,0.13f,0.17f,1.0f));
            ImGui::PushStyleColor(ImGuiCol_Header,         ImVec4(0.20f,0.42f,0.30f,1.0f));
            ImGui::PushStyleColor(ImGuiCol_HeaderHovered,  ImVec4(0.18f,0.40f,0.55f,1.0f));
            ImGui::TableSetColumnIndex(0); { char b[8]; snprintf(b,sizeof(b),"%d", r.dnum>0?r.dnum:r.ch); cell_ctr(b); }  // CH = State창 표시번호
            float cf=(r.lo+r.hi)*0.5f, bw=(r.hi>r.lo? r.hi-r.lo : r.lo-r.hi);
            // ── Center(MHz) 편집 → lo/hi 갱신(BW 유지), 전 유저 동기화 ──
            ImGui::TableSetColumnIndex(1);
            { float cfv=cf; ImGui::SetNextItemWidth(-1);
              ImGui::InputFloat("##cf",&cfv,0,0,"%.4f");
              if(ImGui::IsItemDeactivatedAfterEdit() && cfv>0.f){
                  float hh=bw*0.5f; bewe_mod_edit_ch(v, r.station.c_str(), r.ch, r.mode, cfv-hh, cfv+hh); } }
            // ── BW(kHz) 편집 → lo/hi 갱신(Center 유지) ──
            ImGui::TableSetColumnIndex(2);
            { float bwk=bw*1000.f; ImGui::SetNextItemWidth(-1);
              ImGui::InputFloat("##bw",&bwk,0,0,"%.1f");
              if(ImGui::IsItemDeactivatedAfterEdit() && bwk>0.f){
                  float nb=bwk/1000.f; bewe_mod_edit_ch(v, r.station.c_str(), r.ch, r.mode, cf-nb*0.5f, cf+nb*0.5f); } }
            // ── Mode 편집 ──
            ImGui::TableSetColumnIndex(3);
            { int cm=r.mode<3?r.mode:0; ImGui::SetNextItemWidth(-1);
              if(ImGui::BeginCombo("##md", mode_name((uint8_t)cm))){
                  for(int k=0;k<3;k++){ bool s=(k==cm);
                      if(ImGui::Selectable(mode_name((uint8_t)k),s) && !s)
                          bewe_mod_edit_ch(v, r.station.c_str(), r.ch, k, r.lo, r.hi); }
                  ImGui::EndCombo(); } }
            // ── Channel: SDR 가시대역 안=Active / 밖=Holding. dem_paused 가 소스
            //    (LOCAL=v.channels, 원격=Central CH_LIST 의 hold 바이트). 어디서 보든 동기화 ──
            ImGui::TableSetColumnIndex(4);
            if(r.hold){ ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f,0.70f,0.30f,1.f)); cell_ctr("Holding"); ImGui::PopStyleColor(); }
            else      { ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.40f,0.80f,0.95f,1.f)); cell_ctr("Active");  ImGui::PopStyleColor(); }
            // ── State: 행별 동작 표시 (RUN 초록 / idle 회색) ──
            ImGui::TableSetColumnIndex(5);
            if(r.running>=0){ ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f,0.95f,0.45f,1.f)); cell_ctr("RUN");  ImGui::PopStyleColor(); }
            else            { ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f,0.5f,0.56f,1.f));   cell_ctr("idle"); ImGui::PopStyleColor(); }
            // ── Data: 누적 수신 메시지 (###,###msg) — HOST 측정값 (Central 경유, 전 뷰어 동일) ──
            ImGui::TableSetColumnIndex(6);
            if(r.running>=0){
                if(r.dcount>0){
                    char nb[24]; fmt_commas((long)r.dcount, nb, sizeof(nb));
                    char b[32]; snprintf(b,sizeof(b),"%smsg", nb);
                    ImGui::PushStyleColor(ImGuiCol_Text,ImVec4(0.55f,0.85f,0.6f,1.f)); cell_ctr(b); ImGui::PopStyleColor();
                } else { ImGui::PushStyleColor(ImGuiCol_Text,ImVec4(0.6f,0.6f,0.45f,1.f)); cell_ctr("waiting"); ImGui::PopStyleColor(); }
            } else { ImGui::PushStyleColor(ImGuiCol_Text,ImVec4(0.45f,0.45f,0.5f,1.f)); cell_ctr("-"); ImGui::PopStyleColor(); }
            // ── Run Time: HOST 측정 decode 경과 (HH:MM:SS) + 폴링 사이 로컬 보간 ──
            ImGui::TableSetColumnIndex(7);
            { std::string rk2 = r.station + "#" + std::to_string(r.ch);
              if(r.running>=0){
                  RunAnchor& ra = s_run[rk2];
                  if(ra.last != r.druns){ ra.last = r.druns; ra.anchor = now_ms - (int64_t)r.druns*1000; }
                  char b[16]; fmt_hms((now_ms-ra.anchor)/1000, b, sizeof(b));
                  ImGui::PushStyleColor(ImGuiCol_Text,ImVec4(0.70f,0.80f,0.92f,1.f)); cell_ctr(b); ImGui::PopStyleColor();
              } else { s_run.erase(rk2);
                  ImGui::PushStyleColor(ImGuiCol_Text,ImVec4(0.45f,0.45f,0.5f,1.f)); cell_ctr("-"); ImGui::PopStyleColor(); }
            }
            // ── Decode 콤보 ──
            ImGui::TableSetColumnIndex(8);
            const char* cur = r.running>=0 ? mods[r.running].label : "None";
            ImGui::SetNextItemWidth(-1);
            ImVec4 lblc = r.running>=0 ? ImVec4(0.45f,0.95f,0.55f,1.f) : ImVec4(0.80f,0.80f,0.85f,1.f);
            ImGui::PushStyleColor(ImGuiCol_Text, lblc);
            bool open = ImGui::BeginCombo("##dec", cur);
            ImGui::PopStyleColor();   // Text — 버튼 라벨만 색, 팝업 항목은 기본색
            if(open){
                // 항목 간격 + 상/하단 여백 (None 위·마지막 아래 답답함 완화)
                ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 5));
                ImGui::Dummy(ImVec2(0, 3));
                if(ImGui::Selectable("None", r.running<0) && r.running>=0)
                    bewe_mod_set_target(v, mods[r.running].id, r.station.c_str(), r.ch, false);
                for(int mi:dm){
                    bool is_cur=(mi==r.running);
                    if(ImGui::Selectable(mods[mi].label, is_cur) && !is_cur){
                        if(r.running>=0) bewe_mod_set_target(v, mods[r.running].id, r.station.c_str(), r.ch, false);
                        bewe_mod_set_target(v, mods[mi].id, r.station.c_str(), r.ch, true);
                    }
                }
                if(dm.empty()) ImGui::TextDisabled("(no decoder installed)");
                ImGui::Dummy(ImVec2(0, 3));
                ImGui::PopStyleVar();
                ImGui::EndCombo();
            }
            ImGui::PopStyleColor(5);  // FrameBg,FrameBgHovered,PopupBg,Header,HeaderHovered
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
            // sel_mod 보정 (좌측 목록 하이라이트용 — 우측 통합 테이블은 선택과 무관)
            if(!shown(sel_mod)){ sel_mod=-1; for(size_t i=0;i<mods.size();i++) if(shown((int)i)){ sel_mod=(int)i; break; } }
            ImGui::BeginChild("##mod_detail", ImVec2(0, 0), false);
            draw_targets(v);   // 통합 채널 타깃 테이블 (전 스테이션 1목록 + Decode 콤보)
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
