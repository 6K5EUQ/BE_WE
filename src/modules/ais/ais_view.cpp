// ── AIS 모듈 GUI: DEMOD 데이터 뷰 (공용 modview 컴포넌트로 통일) ────────────
#include "ais_module.hpp"
#include "../modview.hpp"
#include "../common/modview_map.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <set>
#include <deque>
#include <array>
#include <unordered_map>
#include <ctime>

namespace ais_mod {

namespace {
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
// 0 Time 1 MMSI 2 Type 3 Name 4 Lat 5 Lon 6 SOG 7 COG 8 Info
bool match(const AisRecord& m, const char* f){
    if(!f||!f[0]) return true;
    char ts[12]; hms(m.t_ms,ts);
    char mm[16]; snprintf(mm,sizeof(mm),"%u",m.mmsi);
    char ty[8]; snprintf(ty,sizeof(ty),"%d",m.msg_type);
    return modview::ci_find(ts,f)||modview::ci_find(mm,f)||modview::ci_find(ty,f)
        || modview::ci_find(m.name,f)||modview::ci_find(m.callsign,f)||modview::ci_find(m.station,f)
        || modview::ci_find(ais_mid_country(m.mmsi),f)||modview::ci_find(ais_navstatus(m.nav_status),f);
}
int col_cmp(int c, const AisRecord& a, const AisRecord& b){
    switch(c){
        case 0:  return a.t_ms<b.t_ms?-1:(a.t_ms>b.t_ms?1:0);
        case 1:  return a.mmsi<b.mmsi?-1:(a.mmsi>b.mmsi?1:0);
        case 2:  return a.msg_type-b.msg_type;
        case 3:  return strcmp(a.name,b.name);
        case 4:  return a.lat<b.lat?-1:(a.lat>b.lat?1:0);
        case 5:  return a.lon<b.lon?-1:(a.lon>b.lon?1:0);
        case 6:  return a.sog<b.sog?-1:(a.sog>b.sog?1:0);
        case 7:  return a.cog<b.cog?-1:(a.cog>b.cog?1:0);
        default: return a.nav_status-b.nav_status;
    }
}
std::string msg_key(const AisRecord& m){
    char b[48]; snprintf(b,sizeof(b),"%lld|%u|%d",(long long)m.t_ms,m.mmsi,m.msg_type);
    return std::string(b);
}
// Info 컬럼/복사용 한 줄 요약
void info_str(const AisRecord& m, char* o, size_t n){
    if(m.nav_status>=0) snprintf(o,n,"%s",ais_navstatus(m.nav_status));
    else if(m.ship_type>0){ const char* st=ais_shiptype(m.ship_type);
        if(m.callsign[0]) snprintf(o,n,"%s  %s",m.callsign,st); else snprintf(o,n,"%s",st); }
    else if(m.callsign[0]) snprintf(o,n,"%s",m.callsign);
    else o[0]=0;
}
// MMSI별 최신 head + 항적 꼬리 (지도용)
struct AisTrack { AisRecord head; std::deque<std::array<float,3>> trail; int ship_type=0; char name[24]={0}; };  // trail: {lat, lon, t_ms_초}
// ── MMSI 그룹 (표: 동일 MMSI 묶음, 최신값 + Up/Down/Cnt) ──
struct AisGrp {
    uint32_t mmsi;
    int      cnt;          // 수신 누계 (레코드수 집계; auth_cnt>0 이면 그 값으로 대체)
    uint32_t auth_cnt=0;   // Central 요약이 실어보낸 실제 누계 (요약 rx_cnt) — 있으면 권위
    int64_t  first;        // Up — 최초 수신(불변)
    int64_t  last;         // Down — 최근 수신(갱신)
    AisRecord latest;      // last 시점의 최신 레코드 (Type/Lat/Lon/SOG/COG/Info)
    char     name[21]={};  // 최신 비공백 선박명
    uint8_t  spoof=0;      // RF 지문 판정 (max spoof_flag)
    uint32_t match_mmsi=0; // 지문 최근접 MMSI (최신 버스트)
    float    match_conf=0.f;
};
// 컬럼: 0 Up 1 Down 2 MMSI 3 Type 4 Name 5 Country 6 Lat 7 Lon 8 SOG 9 COG 10 Cnt 11 Match 12 Info
int grp_cmp(int c, const AisGrp& a, const AisGrp& b){
    switch(c){
        case 0:  return a.first<b.first?-1:(a.first>b.first?1:0);
        case 1:  return a.last<b.last?-1:(a.last>b.last?1:0);
        case 2:  return a.mmsi<b.mmsi?-1:(a.mmsi>b.mmsi?1:0);
        case 3:  return a.latest.msg_type-b.latest.msg_type;
        case 4:  return strcmp(a.name,b.name);
        case 5:  return strcmp(ais_mid_country(a.mmsi),ais_mid_country(b.mmsi));
        case 6:  return a.latest.lat<b.latest.lat?-1:(a.latest.lat>b.latest.lat?1:0);
        case 7:  return a.latest.lon<b.latest.lon?-1:(a.latest.lon>b.latest.lon?1:0);
        case 8:  return a.latest.sog<b.latest.sog?-1:(a.latest.sog>b.latest.sog?1:0);
        case 9:  return a.latest.cog<b.latest.cog?-1:(a.latest.cog>b.latest.cog?1:0);
        case 10: return a.cnt-b.cnt;
        case 11: return (int)a.match_mmsi-(int)b.match_mmsi;
        default: return a.latest.nav_status-b.latest.nav_status;
    }
}
// 선종(ITU shiptype)별 마커 색 — 화물/유조/여객/어선 등 한눈 구분
ImU32 type_color(int t){
    if(t>=60&&t<=69) return IM_COL32(120,200,255,255);  // passenger 파랑
    if(t>=70&&t<=79) return IM_COL32(110,230,140,255);  // cargo 초록
    if(t>=80&&t<=89) return IM_COL32(255,160, 90,255);  // tanker 주황
    if(t>=30&&t<=39) return IM_COL32(235,215,120,255);  // fishing/특수 노랑
    if(t>=40&&t<=59) return IM_COL32(200,150,255,255);  // 고속/특수 보라
    return IM_COL32(90,200,255,255);                    // 미상 시안
}
} // anonymous

void draw_content(FFTViewer& v, bool just_opened){
    ImGuiIO& io = ImGui::GetIO();
    bool remote = bewe_mod_my_station()[0] != 0;
    if(just_opened && !remote) local_load_today(v);

    ImVec2 avail = ImGui::GetContentRegionAvail();
    float W=avail.x, H=avail.y, x0=ImGui::GetCursorPosX(), y0=ImGui::GetCursorPosY();
    bool win_focus = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    static std::set<std::string> sel; static std::string anchor;
    static AisRecord focus{}; static bool has_focus=false;
    static int sort_col=-1; static bool sort_asc=true;
    static bool atb=true; static size_t lastn=0;
    static uint32_t map_pin=0;                 // 지도에서 핀(필터)된 MMSI (0=없음)
    static uint32_t sel_mmsi=0;                // 표/지도에서 선택된 MMSI (0=없음) — 강조+full꼬리 공통
    static std::string sel_station;            // 클릭된 수신소 이름 (그 기지 수신선박 점선; 빈=없음)
    static std::set<uint32_t> vloaded;         // JOIN: 전체 이력 온디맨드 이미 요청한 MMSI (중복 요청 방지)
    // 과거조회(Hist) 모드 전환 감지 → 선택/캐시 리셋 (log 통째 교체되므로)
    bool histm = remote && bewe_mod_hist_mode("ais", nullptr);
    { static bool prev_hist=false;
      if(histm!=prev_hist){ prev_hist=histm; vloaded.clear(); sel.clear(); anchor.clear();
                            has_focus=false; sel_mmsi=0; map_pin=0; } }
    // 필터 박스를 수동으로 바꾸면 핀 해제 (지도 토글 일관성)
    if(map_pin){ char ms[16]; snprintf(ms,sizeof(ms),"%u",map_pin); if(strcmp(filter,ms)!=0) map_pin=0; }

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; vloaded.clear(); };

    // 스페이스바 Recv 토글 + Ctrl+F/Tab 필터 포커스
    modview::space_toggle_recv(v, "ais", remote, win_focus, on_clear);
    bool focus_filter = win_focus && !io.WantTextInput &&
        ((io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F,false)) || ImGui::IsKeyPressed(ImGuiKey_Tab,false));

    // Ctrl+C → 선택 행 복사 (탭 구분, 한 행당 한 줄)
    if(win_focus && !io.WantTextInput && io.KeyCtrl && !io.KeyShift &&
       ImGui::IsKeyPressed(ImGuiKey_C,false) && !sel.empty()){
        std::string out; std::lock_guard<std::mutex> lk(mtx);
        for(const AisRecord& m : log){ if(!sel.count(msg_key(m))) continue;
            char ts[12]; hms(m.t_ms,ts); char inf[64]; info_str(m,inf,sizeof(inf));
            char lat[16]={0},lon[16]={0},sog[12]={0},cog[12]={0};
            if(m.has_pos){ snprintf(lat,sizeof(lat),"%.5f",m.lat); snprintf(lon,sizeof(lon),"%.5f",m.lon); }
            if(m.sog>=0) snprintf(sog,sizeof(sog),"%.1f",m.sog);
            if(m.cog>=0) snprintf(cog,sizeof(cog),"%.0f",m.cog);
            char line[320];
            snprintf(line,sizeof(line),"%s\t%s\t%u\t%d\t%s\t%s\t%s\t%s\t%s\t%s\n",
                ts,m.station[0]?m.station:"LOCAL",m.mmsi,m.msg_type,m.name,lat,lon,sog,cog,inf);
            out+=line;
        }
        if(!out.empty()) ImGui::SetClipboardText(out.c_str());
    }

    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    // ── 뷰 네비게이션: 그룹목록 ↔ 선박이력. nav_stack[0]=0(목록), >0=해당 MMSI 이력 ──
    static std::vector<uint32_t> nav_stack(1, 0u);   // 히스토리 (0=그룹목록)
    static int nav_pos = 0;                          // 현재 위치
    uint32_t cur_view = nav_stack[nav_pos];          // 0=목록, else=MMSI 이력
    auto nav_go = [&](uint32_t mmsi){                // 새 화면 이동 (앞쪽 히스토리 버림)
        // JOIN: 배 활성화 시 그 MMSI 전체 이력 온디맨드 다운로드 (구독 요약엔 최초/최근10분만 있음)
        // Hist 모드는 제외 — 과거 아카이브는 이미 전체 데이터라 요청 불필요 (요청하면 오늘 데이터가 섞임)
        if(remote && mmsi && !histm && !vloaded.count(mmsi)){ bewe_mod_req_vessel("ais", mmsi); vloaded.insert(mmsi); }
        if(nav_stack[nav_pos]==mmsi) return;
        nav_stack.resize(nav_pos+1); nav_stack.push_back(mmsi); nav_pos=(int)nav_stack.size()-1;
    };
    bool nav_back=false, nav_fwd=false;
    modview::header_bar(v, "ais", filter, sizeof(filter), total, remote, focus_filter, on_clear,
                        true, nav_pos>0, nav_pos<(int)nav_stack.size()-1, &nav_back, &nav_fwd);
    if(nav_back && nav_pos>0) nav_pos--;
    if(nav_fwd  && nav_pos<(int)nav_stack.size()-1) nav_pos++;
    cur_view = nav_stack[nav_pos];

    // 세부패널 제거 — 표/지도 모두 헤더바(30) 아래 전체 높이 사용
    float upper_h = H - 36; if(upper_h<80) upper_h=80;   // 표 높이
    float map_h   = H - 34; if(map_h<80) map_h=80;       // 지도 높이

    // ── MMSI별 최신위치 + 항적 캐시 (signature 게이팅, 증분; FIFO/clear 안전) ──
    static std::unordered_map<uint32_t, AisTrack> tracks;
    static int64_t track_wm=-1; static size_t track_logn=(size_t)-1;
    {
        std::lock_guard<std::mutex> lk(mtx);
        size_t n=log.size(); int64_t last=n?log.back().t_ms:0;
        if(n<track_logn){ tracks.clear(); track_wm=-1; }          // Clear/리셋
        track_logn=n;
        if(last>track_wm){
            int start=(int)n-1; while(start>=0 && log[start].t_ms>track_wm) start--; start++;
            for(int j=start;j<(int)n;j++){
                const AisRecord& m=log[j];
                AisTrack& t=tracks[m.mmsi];
                if(m.ship_type>0) t.ship_type=m.ship_type;     // sticky: static msg(5/24)에서만 옴
                if(m.name[0] && !t.name[0]) { strncpy(t.name,m.name,sizeof(t.name)-1); }
                if(!m.has_pos) continue;                       // head/trail은 위치 msg만
                if(t.head.t_ms==0 || m.t_ms>=t.head.t_ms) t.head=m;
                // tt = 분 단위 상대시간 (float 정밀도 확보; 절대 epoch/1000 은 float 로 초단위 구분 불가)
                float la=(float)m.lat, lo=(float)m.lon, tt=(float)(m.t_ms/60000.0);
                if(t.trail.empty()) t.trail.push_back({la,lo,tt});
                else { auto& bk=t.trail.back();
                       float dla=la-bk[0], dlo=(lo-bk[1])*cosf(la*(float)M_PI/180.f);
                       if(dla*dla+dlo*dlo > 4.5e-5f*4.5e-5f) t.trail.push_back({la,lo,tt}); }
                while(t.trail.size()>800) t.trail.pop_front();   // 안전상한
            }
            track_wm=last;
        }
        // 꼬리 10분 창: 현재시각(최신 데이터 기준) - 10분 이전 점은 매 프레임 제거 (새 패킷 없어도 계속 줄어듦).
        // 트랙 자체는 삭제 안 함 (30분 제거 폐지) — 조용한 배도 유지되다 10분 지나면 꼬리만 빔.
        float now_min = (float)((n?log.back().t_ms:last)/60000.0);
        float cut10 = now_min - 10.f;
        for(auto& kv : tracks){
            auto& tr=kv.second.trail;
            while(!tr.empty() && tr.front()[2]<cut10) tr.pop_front();
        }
    }
    // ── 표시용 MapPoint 빌드 (락 밖, 안정 버퍼) ──
    static std::vector<modview_map::MapPoint> pts;
    static std::vector<std::vector<float>>    trailbuf;
    static std::vector<std::string>           tip1, tip2;
    pts.clear(); trailbuf.clear(); tip1.clear(); tip2.clear();
    for(auto& kv : tracks){
        const AisRecord& m=kv.second.head;
        modview_map::MapPoint mpt;
        mpt.lat=m.lat; mpt.lon=m.lon; mpt.id=m.mmsi;
        mpt.heading = (m.cog>=0.f)? m.cog : (m.heading!=511? (float)m.heading : -1.f);
        mpt.selected = (sel_mmsi==m.mmsi);   // 표/지도 클릭 공통 — 선택 배 강조+full꼬리
        int st = kv.second.ship_type>0? kv.second.ship_type : m.ship_type;
        mpt.color = mpt.selected? IM_COL32(255,210,80,255) : type_color(st);
        mpt.label = m.name[0]? m.name : (kv.second.name[0]? kv.second.name : nullptr);
        trailbuf.emplace_back();
        if(sel_mmsi==m.mmsi){
            // 선택된 배(표/지도 공통): 캐시 무시, log 전체 점을 시간순으로 모두 연결 = full 꼬리
            std::lock_guard<std::mutex> lk(mtx);
            for(const AisRecord& r : log)
                if(r.mmsi==m.mmsi && r.has_pos){ trailbuf.back().push_back((float)r.lat); trailbuf.back().push_back((float)r.lon); }
        } else
        for(auto& pr : kv.second.trail){ trailbuf.back().push_back(pr[0]); trailbuf.back().push_back(pr[1]); }
        char b1[32], b2[96];
        snprintf(b1,sizeof(b1),"%u", m.mmsi);                       // MMSI
        char sog[16],cog[16],hdg[16];
        if(m.sog>=0) snprintf(sog,sizeof(sog),"%.1f kt",m.sog); else snprintf(sog,sizeof(sog),"-");
        if(m.cog>=0) snprintf(cog,sizeof(cog),"%.1f°",m.cog);  else snprintf(cog,sizeof(cog),"-");
        if(m.heading!=511) snprintf(hdg,sizeof(hdg),"%d°",m.heading); else snprintf(hdg,sizeof(hdg),"-");
        snprintf(b2,sizeof(b2),"SOG : %s\nCOG : %s\nHDG : %s", sog,cog,hdg);
        tip1.emplace_back(b1); tip2.emplace_back(b2);
        pts.push_back(mpt);
    }
    for(size_t i=0;i<pts.size();i++){
        pts[i].trail   = trailbuf[i].empty()? nullptr : trailbuf[i].data();
        pts[i].trail_n = (int)trailbuf[i].size()/2;
        pts[i].tip_l1  = tip1[i].c_str();
        pts[i].tip_l2  = tip2[i].empty()? nullptr : tip2[i].c_str();
    }

    // ── MMSI 그룹 집계 (표: 동일 MMSI 묶음, Up/Down/Cnt + 최신값. 캐시 게이팅) ──
    static std::vector<AisGrp> grps;
    static std::set<std::string> rx_stations;   // 실제 AIS 복조한 기지 이름들 ("" = LOCAL)
    static float info_w = 60.f;   // Info 컬럼 동적폭
    {
        std::lock_guard<std::mutex> lk(mtx);
        static size_t c_n=(size_t)-1; static int64_t c_last=-1;
        static char c_filter[64]={'\xff'}; static int c_sc=-99; static bool c_asc=false;
        size_t n_now=log.size(); int64_t last_t=n_now?log.back().t_ms:0;
        if(n_now!=c_n||last_t!=c_last||strncmp(c_filter,filter,sizeof(c_filter))||c_sc!=sort_col||c_asc!=sort_asc){
            std::vector<AisGrp> g; g.reserve(256);
            std::unordered_map<uint32_t,int> idx; idx.reserve(512);
            rx_stations.clear();
            for(size_t i=0;i<n_now;i++){
                const AisRecord& m=log[i];
                if(!match(m,filter)) continue;
                rx_stations.insert(m.station);   // 복조한 기지 수집 (빈문자=LOCAL)
                auto it=idx.find(m.mmsi); int gi;
                if(it==idx.end()){ gi=(int)g.size(); idx[m.mmsi]=gi;
                    AisGrp ng{}; ng.mmsi=m.mmsi; ng.first=m.t_ms; ng.last=-1; g.push_back(ng); }
                else gi=it->second;
                AisGrp& G=g[gi]; G.cnt++;
                if(m.rx_cnt>G.auth_cnt) G.auth_cnt=m.rx_cnt;   // Central 요약 실제 누계 (권위)
                if(m.t_ms<G.first) G.first=m.t_ms;          // Up = 최초(불변)
                if(m.t_ms>=G.last){ G.last=m.t_ms; G.latest=m; G.match_mmsi=m.match_mmsi; G.match_conf=m.match_conf; }   // Down = 최근
                if(m.spoof_flag>G.spoof) G.spoof=m.spoof_flag;   // RF 판정 (최대)
                if(m.name[0] && !G.name[0]){ strncpy(G.name,m.name,sizeof(G.name)-1); G.name[sizeof(G.name)-1]=0; }
            }
            for(AisGrp& G : g) if(G.auth_cnt) G.cnt=(int)G.auth_cnt;   // 요약 상태: 실제 누계로 표시/정렬 통일
            std::stable_sort(g.begin(),g.end(),[&](const AisGrp&a,const AisGrp&b){ int c=grp_cmp(sort_col<0?0:sort_col,a,b); return (sort_col<0?true:sort_asc)? c<0:c>0; });
            grps.swap(g);
            c_n=n_now; c_last=last_t; strncpy(c_filter,filter,sizeof(c_filter)-1); c_filter[sizeof(c_filter)-1]=0; c_sc=sort_col; c_asc=sort_asc;
            // Info 폭 = 실제 데이터 최대 길이에 맞춤 (헤더 글자폭 무시)
            float mw=8.f;
            for(const AisGrp& G : grps){ char inf[64]; info_str(G.latest,inf,sizeof(inf));
                if(inf[0]){ float w=ImGui::CalcTextSize(inf).x; if(w>mw) mw=w; } }
            info_w = mw + 10.f;
        }
    }

    // ── 좌(표) | 우(지도) ──  (지도 크게보기 v.big 면 표 숨기고 지도 전폭)
    static modview_map::MapView mv;
    static float split_tw=-1.f;   // 사용자가 스플리터로 정한 표 폭(px). <0 = 미설정(컬럼합 자동)
    float tw, mapw;
    if(mv.big){ tw=0.f; mapw=W; }
    else {
        const float FIXED_COLS = 70+70+82+40+130+64+78+84+48+48+46+82;  // Up Down MMSI Type Name Country Lat Lon SOG COG Cnt Match
        // 컬럼 cell padding(~8px each) + inner border + 세로스크롤바(~14). 이만큼만 더해
        // Info 셀 뒤 빈 여백 없이 마지막 Info 가 데이터폭에서 끝나게.
        float auto_tw = FIXED_COLS + info_w + 13*8.f + 14.f;
        tw = (split_tw>0.f) ? split_tw : auto_tw;       // 드래그로 조절했으면 그 값
        float maxtw = W-50; if(tw>maxtw) tw=maxtw; if(tw<200) tw=200;
        mapw=W-tw-6; if(mapw<10)mapw=10;                // 6px = 스플리터
    }

    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(!mv.big && cur_view==0 && ImGui::BeginTable("##ais_tbl", 13, tf, ImVec2(tw, upper_h))){
        ImGui::TableSetupScrollFreeze(3,1);
        ImGui::TableSetupColumn("Up",      ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Down",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("MMSI",    ImGuiTableColumnFlags_WidthFixed, 82);
        ImGui::TableSetupColumn("Type",    ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Name",    ImGuiTableColumnFlags_WidthFixed, 130);
        ImGui::TableSetupColumn("Country", ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("Lat",     ImGuiTableColumnFlags_WidthFixed, 78);
        ImGui::TableSetupColumn("Lon",     ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("SOG",     ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("COG",     ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("Cnt",     ImGuiTableColumnFlags_WidthFixed, 46);
        ImGui::TableSetupColumn("Match",   ImGuiTableColumnFlags_WidthFixed, 82);  // 지문 예상 MMSI
        ImGui::TableSetupColumn("Info",    ImGuiTableColumnFlags_WidthStretch);  // 남는 폭 흡수 → Info 뒤 빈 칸 없음
        modview::sortable_headers(13, sort_col, sort_asc, 12);  // Info(12)만 좌측, 나머지 중앙

        ImGuiListClipper clip; clip.Begin((int)grps.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const AisGrp& G=grps[r];
            const AisRecord& m=G.latest;
            ImGui::TableNextRow();
            char up[12]; hms(G.first,up);
            bool selrow = (sel_mmsi==G.mmsi);
            if(modview::row_col0(r, selrow, up)){               // col0 = Up time
                sel_mmsi=G.mmsi; map_pin=G.mmsi;                // 지도 핀(필터링 안 함)
                focus=m; has_focus=true;
                nav_go(G.mmsi);                                 // 그 선박 전체 이력 화면으로
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); { char dn[12]; hms(G.last,dn); modview::cell(dn); }
            ImGui::TableSetColumnIndex(2); snprintf(b,sizeof(b),"%u",G.mmsi);
            if(G.spoof==2) modview::cell(b, ImVec4(1.f,0.35f,0.30f,1.f)); else modview::cell(b);  // 스푸핑 의심=빨강
            ImGui::TableSetColumnIndex(3); snprintf(b,sizeof(b),"%d",m.msg_type); modview::cell(b);
            ImGui::TableSetColumnIndex(4);
            if(G.name[0]) modview::cell(G.name); else { const char* cc=ais_mid_country(G.mmsi); if(cc[0]) modview::cell(cc, ImVec4(0.6f,0.6f,0.6f,1.f)); }
            ImGui::TableSetColumnIndex(5); { const char* cc=ais_mid_country(G.mmsi); if(cc[0]) modview::cell(cc, ImVec4(0.6f,0.6f,0.6f,1.f)); }
            ImGui::TableSetColumnIndex(6); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lat); modview::cell(b); }
            ImGui::TableSetColumnIndex(7); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lon); modview::cell(b); }
            ImGui::TableSetColumnIndex(8); if(m.sog>=0){ snprintf(b,sizeof(b),"%.1f",m.sog); modview::cell(b); }
            ImGui::TableSetColumnIndex(9); if(m.cog>=0){ snprintf(b,sizeof(b),"%.0f",m.cog); modview::cell(b); }
            ImGui::TableSetColumnIndex(10); snprintf(b,sizeof(b),"%d",G.cnt); modview::cell(b);
            ImGui::TableSetColumnIndex(11);   // Match: 고신뢰 예상 MMSI (claimed 불일치=빨강)
            if(G.match_mmsi){ snprintf(b,sizeof(b),"%u",G.match_mmsi);
                modview::cell(b, G.match_mmsi==G.mmsi? ImVec4(0.55f,0.8f,0.55f,1.f):ImVec4(1.f,0.5f,0.4f,1.f)); }
            else modview::cell("-", ImVec4(0.4f,0.4f,0.4f,1.f));
            ImGui::TableSetColumnIndex(12); { char inf[64]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        ImGui::EndTable();
    }
    // ── 선박 이력 뷰: 선택 MMSI 의 Up~Down 사이 모든 메시지 (시간순) ──
    else if(!mv.big && cur_view!=0 && ImGui::BeginTable("##ais_hist", 7, tf, ImVec2(tw, upper_h))){
        ImGui::TableSetupScrollFreeze(1,1);
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Lat",  ImGuiTableColumnFlags_WidthFixed, 78);
        ImGui::TableSetupColumn("Lon",  ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("SOG",  ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("COG",  ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("HDG",  ImGuiTableColumnFlags_WidthFixed, 44);
        ImGui::TableSetupColumn("Info", ImGuiTableColumnFlags_WidthStretch);
        static int hsc=-1; static bool hsa=true;
        modview::sortable_headers(7, hsc, hsa, 6);   // Info(6)만 좌측, 나머지(Time/Lat/Lon/SOG/COG/HDG) 중앙

        std::lock_guard<std::mutex> lk(mtx);
        static std::vector<int> hvis; hvis.clear();
        for(int i=0;i<(int)log.size();i++) if(log[i].mmsi==cur_view) hvis.push_back(i);

        ImGuiListClipper hc; hc.Begin((int)hvis.size());
        while(hc.Step()) for(int r=hc.DisplayStart;r<hc.DisplayEnd;r++){
            const AisRecord& m=log[hvis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            char b[24];
            ImGui::TableSetColumnIndex(0); modview::cell(ts);   // Time 중앙정렬
            ImGui::TableSetColumnIndex(1); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lat); modview::cell(b); }
            ImGui::TableSetColumnIndex(2); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lon); modview::cell(b); }
            ImGui::TableSetColumnIndex(3); if(m.sog>=0){ snprintf(b,sizeof(b),"%.1f",m.sog); modview::cell(b); }
            ImGui::TableSetColumnIndex(4); if(m.cog>=0){ snprintf(b,sizeof(b),"%.0f",m.cog); modview::cell(b); }
            ImGui::TableSetColumnIndex(5); if(m.heading!=511){ snprintf(b,sizeof(b),"%d",m.heading); modview::cell(b); }
            ImGui::TableSetColumnIndex(6); { char inf[64]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        modview::tail_follow(atb, false);
        ImGui::EndTable();
    }

    // ── 수신소(기지) 마커: 지구본과 동일하게 discovered_stations 전부 + 내 위치(HOST) 오버레이 ──
    static std::vector<modview_map::MapStation> stns;
    static std::vector<std::string> stn_names;   // 라벨 문자열 수명 보관
    stns.clear(); stn_names.clear();
    {
        // 한국 운용 전제 좌표 정규화 — 부호반전/음수 lon 보정 (서경으로 저장된 값 방어)
        auto norm=[](float& la, float& lo){
            if(lo<0.f) lo=-lo;       // 서경(-128) → 동경(128)
            if(la<0.f) la=-la;
        };
        // AIS 를 실제 복조한 기지(rx_stations)만, station_geo_cache(로그인때 받아 보관)서 좌표 조회.
        std::lock_guard<std::mutex> ck(v.station_geo_cache_mtx);
        for(const std::string& sn : rx_stations){
            std::string key = (sn.empty()||sn=="LOCAL") ? v.station_name : sn;
            float la=0.f, lo=0.f; bool got=false;
            auto it=v.station_geo_cache.find(key);
            if(it!=v.station_geo_cache.end()){ la=it->second.first; lo=it->second.second; got=true; }
            else if((sn.empty()||sn=="LOCAL") && (v.station_lat!=0.f||v.station_lon!=0.f)){
                la=v.station_lat; lo=v.station_lon; got=true;   // 내 위치 폴백
            }
            if(!got || (la==0.f&&lo==0.f)) continue;
            norm(la,lo);
            stn_names.push_back(key.empty()? sn : key);
            modview_map::MapStation ms; ms.lat=la; ms.lon=lo;
            ms.selected = (!sel_station.empty() && (key==sel_station || sn==sel_station));
            stns.push_back(ms);
        }
        for(size_t i=0;i<stns.size();i++) stns[i].name=stn_names[i].c_str();
    }

    // 선택 기지가 수신한 선박 점선 (기지좌표 → 각 선박 head)
    static std::vector<modview_map::MapLink> links;
    links.clear();
    if(!sel_station.empty()){
        // 선택 기지 좌표
        double slat=0,slon=0; bool sgot=false;
        for(size_t i=0;i<stns.size();i++) if(stns[i].selected){ slat=stns[i].lat; slon=stns[i].lon; sgot=true; break; }
        if(sgot){
            std::lock_guard<std::mutex> lk(mtx);
            std::set<uint32_t> rxmmsi;   // 그 기지가 수신한 MMSI
            for(const AisRecord& m : log){
                std::string st = (m.station[0] && strcmp(m.station,"LOCAL")) ? m.station : v.station_name;
                if(st==sel_station) rxmmsi.insert(m.mmsi);
            }
            for(const auto& mp : pts)
                if(rxmmsi.count((uint32_t)mp.id)){
                    double la=mp.lat, lo=mp.lon; if(lo<0)lo=-lo; if(la<0)la=-la;
                    links.push_back({slat,slon, la,lo});
                }
        }
    }

    // ── 스플리터 (표↔지도 크기 조절; 크게보기 아닐 때만) ──
    if(!mv.big){
        ImGui::SameLine(0,0);
        ImGui::InvisibleButton("##ais_split", ImVec2(6, map_h));
        bool sphov=ImGui::IsItemHovered(), spact=ImGui::IsItemActive();
        if(spact){ float ntw=(split_tw>0.f?split_tw:tw)+io.MouseDelta.x;
                   if(ntw<200)ntw=200; if(ntw>W-50)ntw=W-50; split_tw=ntw; }
        if(sphov||spact){
            ImVec2 a=ImGui::GetItemRectMin(), b=ImGui::GetItemRectMax();
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2((a.x+b.x)*0.5f-1,a.y),ImVec2((a.x+b.x)*0.5f+1,b.y),IM_COL32(120,140,160,200));
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        }
        ImGui::SameLine(0,0);
    } else {
        ImGui::SetCursorPosX(x0);
    }
    auto mres = modview_map::draw_map("##ais_map", mv, pts, ImVec2(mapw, map_h), just_opened, &stns, &links);
    if(mres.clicked_station>=0 && mres.clicked_station<(int)stn_names.size()){
        // 기지 아이콘 클릭 → 그 기지 수신선박 점선 토글
        const std::string& cs = stn_names[mres.clicked_station];
        sel_station = (sel_station==cs) ? std::string() : cs;
    }
    if(mres.clicked_id){
        uint32_t id=(uint32_t)mres.clicked_id;
        if(sel_mmsi==id){                               // 활성 배 재클릭 → 비활성 (목록 복귀)
            sel_mmsi=0; map_pin=0; has_focus=false; nav_go(0);
        } else {
            sel_mmsi=id; map_pin=id;
            { std::lock_guard<std::mutex> lk(mtx);
              for(const AisRecord& m : log) if(m.mmsi==id && m.has_pos) focus=m; }
            has_focus=true;
            nav_go(id);                                 // 표 행 클릭과 동일 — 그 선박 전체 이력 화면으로
        }
    }

    // 선택 배 focus 를 최신 수신 레코드로 매 프레임 갱신 (GPS 등 계속 업데이트)
    if(has_focus && sel_mmsi){
        std::lock_guard<std::mutex> lk(mtx);
        AisRecord nm{}; bool got=false;
        // 최신 레코드 = 동적(위치/속도) 베이스. 정적(이름/호출/선종/IMO/목적지/ETA/흘수)은
        // 그 필드를 가진 가장 최근 레코드에서 채움 (정적은 Type5/19/24 에만 있어 위치msg엔 없음).
        for(auto it=log.rbegin(); it!=log.rend(); ++it){
            if(it->mmsi!=sel_mmsi) continue;
            const AisRecord& r=*it;
            if(!got){ nm=r; got=true; }
            if(!nm.name[0]&&r.name[0]) strncpy(nm.name,r.name,sizeof(nm.name)-1);
            if(!nm.callsign[0]&&r.callsign[0]) strncpy(nm.callsign,r.callsign,sizeof(nm.callsign)-1);
            if(nm.ship_type<=0&&r.ship_type>0) nm.ship_type=r.ship_type;
            if(!nm.imo&&r.imo) nm.imo=r.imo;
            if(!nm.dest[0]&&r.dest[0]) strncpy(nm.dest,r.dest,sizeof(nm.dest)-1);
            if(nm.draught<0&&r.draught>=0) nm.draught=r.draught;
            if(!nm.eta_mon&&r.eta_mon){ nm.eta_mon=r.eta_mon; nm.eta_day=r.eta_day; nm.eta_hour=r.eta_hour; nm.eta_min=r.eta_min; }
            if(nm.name[0]&&nm.imo&&nm.dest[0]&&nm.draught>=0&&nm.eta_mon) break;   // 다 채우면 조기 종료
        }
        if(got) focus=nm;
    }
    // ── 선택 선박 정보: 일반 모드=하단 세부패널 / 전체화면=우상단 카드 (flightradar 스타일) ──
    auto draw_detail_body=[&](){
        ImVec4 V(0.85f,0.85f,0.9f,1.f), K(0.55f,0.62f,0.72f,1.f);
        auto row=[&](const char* k, const char* val, ImVec4 vc){    // 라벨/값 한 줄, 값 잘림 방지
            ImGui::TextColored(K,"%s",k); ImGui::SameLine(); ImGui::TextColored(vc,"%s",val);
        };
        { char mm[16]; snprintf(mm,sizeof(mm),"%u",focus.mmsi); row("MMSI", mm, ImVec4(0.5f,0.8f,1.f,1.f)); }
        if(focus.name[0]) row("Name", focus.name, V);
        if(focus.callsign[0]) row("Call", focus.callsign, V);
        row("Country", ais_mid_country(focus.mmsi), ImVec4(0.6f,0.6f,0.6f,1.f));
        if(focus.ship_type>0) row("Ship", ais_shiptype(focus.ship_type), ImVec4(0.62f,0.7f,0.62f,1.f));
        if(focus.imo){ char s[16]; snprintf(s,sizeof(s),"%u",focus.imo); row("IMO", s, V); }
        ImGui::Separator();
        if(focus.has_pos){ char p[40]; snprintf(p,sizeof(p),"%.5f",focus.lat); row("Lat", p, V);
                           snprintf(p,sizeof(p),"%.5f",focus.lon); row("Lon", p, V); }
        if(focus.sog>=0){ char s[16]; snprintf(s,sizeof(s),"%.1f kt",focus.sog); row("SOG", s, V); }
        if(focus.cog>=0){ char s[16]; snprintf(s,sizeof(s),"%.1f°",focus.cog); row("COG", s, V); }
        if(focus.heading!=511){ char s[16]; snprintf(s,sizeof(s),"%d°",focus.heading); row("HDG", s, V); }
        if(focus.nav_status>=0) row("Status", ais_navstatus(focus.nav_status), V);
        if(focus.dest[0]) row("Dest", focus.dest, ImVec4(0.82f,0.76f,0.55f,1.f));
        if(focus.draught>=0){ char s[16]; snprintf(s,sizeof(s),"%.1f m",focus.draught); row("Draught", s, V); }
        if(focus.eta_mon){ char s[24]; snprintf(s,sizeof(s),"%02d-%02d %02d:%02d",focus.eta_mon,focus.eta_day,focus.eta_hour,focus.eta_min); row("ETA", s, V); }
        // ── RF 지문 (스푸핑 검증 / Match) ──
        if(focus.has_rf){
            ImGui::Separator();
            { char s[24]; snprintf(s,sizeof(s),"%.0f Hz",focus.cfo_hz); row("CFO", s, V); }
            if(focus.spoof_flag==2)      row("RF ALERT","2nd TX 의심", ImVec4(1.f,0.4f,0.35f,1.f));
            else if(focus.spoof_flag==1) row("RF","서명 정상", ImVec4(0.55f,0.85f,0.55f,1.f));
            else                          row("RF","확립 중…", ImVec4(0.6f,0.6f,0.6f,1.f));
            if(focus.match_mmsi){ char s[32]; snprintf(s,sizeof(s),"%u (%.0f%%)",focus.match_mmsi,focus.match_conf*100.f);
                row("Match", s, focus.match_mmsi==focus.mmsi? ImVec4(0.55f,0.8f,0.55f,1.f):ImVec4(1.f,0.5f,0.4f,1.f)); }
        }
    };
    if(has_focus && mv.big){
        // 전체화면: 지도 우상단 박스 카드. 상단은 헤더바(30) 아래 + 여백, 우측 여백 동일.
        const float CW=150.f, MX=16.f;   // 폭은 데이터 최대(MMSI/"SOG 10.2 kt")에 맞춤, 높이 자동
        ImGui::SetCursorPos(ImVec2(x0 + W-CW-MX, y0+30.f+MX));  // 헤더바(30) 아래 + MX, 우측도 MX
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.06f,0.09f,0.13f,0.94f));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12,10));
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.f);
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8,6));
        ImGui::BeginChild("##ais_card", ImVec2(CW,0), ImGuiChildFlags_Borders|ImGuiChildFlags_AutoResizeY,
                          ImGuiWindowFlags_NoScrollbar);   // 높이=내용맞춤 (Status 에서 끝, 아래 여백 없음)
        draw_detail_body();
        ImGui::EndChild();
        ImGui::PopStyleVar(3); ImGui::PopStyleColor();
    }
}

} // namespace ais_mod
