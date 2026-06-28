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
struct AisTrack { AisRecord head; std::deque<std::array<float,2>> trail; int ship_type=0; char name[24]={0}; };
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
    float W=avail.x, H=avail.y, x0=ImGui::GetCursorPosX();
    bool win_focus = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);

    static std::set<std::string> sel; static std::string anchor;
    static AisRecord focus{}; static bool has_focus=false;
    static int sort_col=-1; static bool sort_asc=true;
    static bool atb=true; static size_t lastn=0;
    static uint32_t map_pin=0;                 // 지도에서 핀(필터)된 MMSI (0=없음)
    // 필터 박스를 수동으로 바꾸면 핀 해제 (지도 토글 일관성)
    if(map_pin){ char ms[16]; snprintf(ms,sizeof(ms),"%u",map_pin); if(strcmp(filter,ms)!=0) map_pin=0; }

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; };

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
    modview::header_bar(v, "ais", filter, sizeof(filter), total, remote, focus_filter, on_clear);

    float detail_h = has_focus ? 132.f : 0.f;
    float upper_h = H - 30 - detail_h - 16; if(upper_h<80) upper_h=80;

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
                float la=(float)m.lat, lo=(float)m.lon;
                if(t.trail.empty()) t.trail.push_back({la,lo});
                else { auto& bk=t.trail.back();
                       float dla=la-bk[0], dlo=(lo-bk[1])*cosf(la*(float)M_PI/180.f);
                       if(dla*dla+dlo*dlo > 4.5e-5f*4.5e-5f) t.trail.push_back({la,lo}); }
                while(t.trail.size()>12) t.trail.pop_front();
            }
            int64_t cutoff=last-30LL*60*1000;                     // 30분 지난 트랙 제거
            for(auto it=tracks.begin(); it!=tracks.end();)
                if(it->second.head.t_ms<cutoff) it=tracks.erase(it); else ++it;
            track_wm=last;
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
        mpt.selected = (has_focus && focus.mmsi==m.mmsi);
        int st = kv.second.ship_type>0? kv.second.ship_type : m.ship_type;
        mpt.color = mpt.selected? IM_COL32(255,210,80,255) : type_color(st);
        mpt.label = m.name[0]? m.name : (kv.second.name[0]? kv.second.name : nullptr);
        trailbuf.emplace_back();
        if(has_focus && focus.mmsi==m.mmsi){
            // 선택된 배: 캐시(12점/5m 게이팅) 무시, log 전체 점을 시간순으로 모두 연결
            std::lock_guard<std::mutex> lk(mtx);
            for(const AisRecord& r : log)
                if(r.mmsi==m.mmsi && r.has_pos){ trailbuf.back().push_back((float)r.lat); trailbuf.back().push_back((float)r.lon); }
        } else
        for(auto& pr : kv.second.trail){ trailbuf.back().push_back(pr[0]); trailbuf.back().push_back(pr[1]); }
        char b1[80], b2[64];
        snprintf(b1,sizeof(b1),"MMSI %u  %s", m.mmsi, ais_mid_country(m.mmsi));
        if(m.sog>=0&&m.cog>=0) snprintf(b2,sizeof(b2),"SOG %.1f kt  COG %.0f",m.sog,m.cog);
        else if(m.sog>=0) snprintf(b2,sizeof(b2),"SOG %.1f kt",m.sog);
        else if(m.name[0]) snprintf(b2,sizeof(b2),"%s",m.name); else b2[0]=0;
        tip1.emplace_back(b1); tip2.emplace_back(b2);
        pts.push_back(mpt);
    }
    for(size_t i=0;i<pts.size();i++){
        pts[i].trail   = trailbuf[i].empty()? nullptr : trailbuf[i].data();
        pts[i].trail_n = (int)trailbuf[i].size()/2;
        pts[i].tip_l1  = tip1[i].c_str();
        pts[i].tip_l2  = tip2[i].empty()? nullptr : tip2[i].c_str();
    }

    // ── 좌(표) | 스플리터 | 우(지도) ──
    static float split=0.58f;
    float tw;
    if(W < 200+220+6) tw=W*0.5f;                          // 너무 좁으면 반반
    else { tw=W*split; if(tw<200)tw=200; if(tw>W-226)tw=W-226; }
    float mapw=W-tw-6; if(mapw<10)mapw=10; if(tw<10)tw=10;

    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##ais_tbl", 9, tf, ImVec2(tw, upper_h))){
        ImGui::TableSetupScrollFreeze(2,1);
        ImGui::TableSetupColumn("Time",   ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("MMSI",   ImGuiTableColumnFlags_WidthFixed, 82);
        ImGui::TableSetupColumn("Type",   ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Name",   ImGuiTableColumnFlags_WidthFixed, 130);
        ImGui::TableSetupColumn("Lat",    ImGuiTableColumnFlags_WidthFixed, 78);
        ImGui::TableSetupColumn("Lon",    ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("SOG",    ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("COG",    ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("Info",   ImGuiTableColumnFlags_WidthFixed, 220);
        modview::sortable_headers(9, sort_col, sort_asc, 8);

        std::lock_guard<std::mutex> lk(mtx);
        // vis 캐시 (개수/마지막t/필터/정렬 시그니처 게이팅)
        static std::vector<int> vis;
        static size_t c_n=(size_t)-1; static int64_t c_last=-1;
        static char c_filter[64]={'\xff'}; static int c_sc=-99; static bool c_asc=false;
        size_t n_now=log.size(); int64_t last_t=n_now?log.back().t_ms:0;
        if(n_now!=c_n||last_t!=c_last||strncmp(c_filter,filter,sizeof(c_filter))||c_sc!=sort_col||c_asc!=sort_asc){
            vis.clear(); for(int i=0;i<(int)n_now;i++) if(match(log[i],filter)) vis.push_back(i);
            modview::sort_vis(vis, sort_col, sort_asc, [&](int c,int a,int b){ return col_cmp(c,log[a],log[b]); });
            c_n=n_now; c_last=last_t; strncpy(c_filter,filter,sizeof(c_filter)-1); c_filter[sizeof(c_filter)-1]=0; c_sc=sort_col; c_asc=sort_asc;
        }

        ImGuiListClipper clip; clip.Begin((int)vis.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const AisRecord& m = log[vis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            std::string k=msg_key(m);
            if(modview::row_col0(vis[r], sel.count(k)>0, ts)){
                modview::apply_click(sel, anchor, k, r, [&](int i){return msg_key(log[vis[i]]);}, (int)vis.size(), io.KeyCtrl, io.KeyShift);
                focus=m; has_focus=!sel.empty();
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); snprintf(b,sizeof(b),"%u",m.mmsi); modview::cell(b);
            ImGui::TableSetColumnIndex(2); snprintf(b,sizeof(b),"%d",m.msg_type); modview::cell(b);
            ImGui::TableSetColumnIndex(3);
            if(m.name[0]) modview::cell(m.name); else { const char* cc=ais_mid_country(m.mmsi); if(cc[0]) modview::cell(cc, ImVec4(0.6f,0.6f,0.6f,1.f)); }
            ImGui::TableSetColumnIndex(4); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lat); modview::cell(b); }
            ImGui::TableSetColumnIndex(5); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lon); modview::cell(b); }
            ImGui::TableSetColumnIndex(6); if(m.sog>=0){ snprintf(b,sizeof(b),"%.1f",m.sog); modview::cell(b); }
            ImGui::TableSetColumnIndex(7); if(m.cog>=0){ snprintf(b,sizeof(b),"%.0f",m.cog); modview::cell(b); }
            ImGui::TableSetColumnIndex(8); { char inf[64]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        bool grew = log.size()>lastn; lastn=log.size();
        modview::tail_follow(atb, grew && sort_col<0);
        ImGui::EndTable();
    }

    // ── 스플리터 ──
    ImGui::SameLine(0,0);
    ImGui::InvisibleButton("##ais_split", ImVec2(6, upper_h));
    if(ImGui::IsItemActive()){ split += io.MouseDelta.x/(W>1?W:1); if(split<0.2f)split=0.2f; if(split>0.8f)split=0.8f; }
    if(ImGui::IsItemHovered()||ImGui::IsItemActive()){
        ImVec2 a=ImGui::GetItemRectMin(), b=ImGui::GetItemRectMax();
        ImGui::GetWindowDrawList()->AddRectFilled(ImVec2((a.x+b.x)*0.5f-1,a.y),ImVec2((a.x+b.x)*0.5f+1,b.y),IM_COL32(120,140,160,200));
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }
    // ── 지도 (우측) ──
    ImGui::SameLine(0,0);
    static modview_map::MapView mv;
    auto mres = modview_map::draw_map("##ais_map", mv, pts, ImVec2(mapw, upper_h), just_opened);
    if(mres.clicked_id){
        uint32_t id=(uint32_t)mres.clicked_id;
        if(map_pin==id){ map_pin=0; filter[0]=0; }                    // 같은 배 재클릭 → 목록 필터 해제
        else { map_pin=id; snprintf(filter,sizeof(filter),"%u",id); } // 그 MMSI 만 목록에 표시
        std::lock_guard<std::mutex> lk(mtx);
        for(const AisRecord& m : log) if(m.mmsi==id && m.has_pos) focus=m;
        std::string fk=msg_key(focus); sel.clear(); sel.insert(fk); anchor=fk; has_focus=true;
    }

    // ── 세부 패널 (전체 폭 하단) ──
    if(has_focus){
        modview::detail_begin("ais", x0, W, detail_h);
        char tsd[12]; hms(focus.t_ms,tsd);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);
        ImGui::Text("Time:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",tsd);
        modview::kv("Station:", focus.station[0]?focus.station:"LOCAL", ImVec4(0.85f,0.75f,0.5f,1.f));
        { char mm[16]; snprintf(mm,sizeof(mm),"%u",focus.mmsi); modview::kv("MMSI:", mm, ImVec4(0.5f,0.8f,1.f,1.f)); }
        { char ty[8]; snprintf(ty,sizeof(ty),"%d",focus.msg_type); modview::kv("Type:", ty, V); }
        modview::kv("Country:", ais_mid_country(focus.mmsi), ImVec4(0.6f,0.6f,0.6f,1.f));
        modview::kv("Name:", focus.name, V);
        modview::kv("Call:", focus.callsign, V);
        if(focus.ship_type>0) modview::kv("Ship:", ais_shiptype(focus.ship_type), ImVec4(0.62f,0.7f,0.62f,1.f));
        ImGui::Separator();
        if(focus.has_pos){ char p[40]; snprintf(p,sizeof(p),"%.5f",focus.lat); ImGui::Text("Lat:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",p);
                           snprintf(p,sizeof(p),"%.5f",focus.lon); modview::kv("Lon:", p, V); }
        else { ImGui::Text("Lat:"); ImGui::SameLine(); ImGui::TextDisabled("-"); }
        if(focus.sog>=0){ char s[12]; snprintf(s,sizeof(s),"%.1f kt",focus.sog); modview::kv("SOG:", s, V); }
        if(focus.cog>=0){ char s[12]; snprintf(s,sizeof(s),"%.1f",focus.cog); modview::kv("COG:", s, V); }
        if(focus.heading!=511){ char s[12]; snprintf(s,sizeof(s),"%d",focus.heading); modview::kv("HDG:", s, V); }
        if(focus.nav_status>=0) modview::kv("Status:", ais_navstatus(focus.nav_status), V);
        modview::detail_end();
    }
}

} // namespace ais_mod
