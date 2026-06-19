// ── ADS-B 모듈 GUI: DEMOD 데이터 뷰 (공용 modview 컴포넌트로 통일) ───────────
// ais_view.cpp 미러. ADS-B 는 정보가 메시지별로 쪼개져 옴(식별/위치/속도) →
// 지도는 ICAO 별로 최신 필드를 병합한 항적으로 그린다.
#include "adsb_module.hpp"
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

namespace adsb_mod {

namespace {
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
void icao_str(uint32_t a, char* o){ snprintf(o,8,"%06X",a&0xFFFFFF); }

// 0 Time 1 Station 2 ICAO 3 Callsign 4 Alt 5 Lat 6 Lon 7 Spd 8 Trk 9 Info
bool match(const AdsbRecord& m, const char* f){
    if(!f||!f[0]) return true;
    char ts[12]; hms(m.t_ms,ts);
    char ic[8]; icao_str(m.icao,ic);
    return modview::ci_find(ts,f)||modview::ci_find(ic,f)||modview::ci_find(m.callsign,f)
        || modview::ci_find(m.station,f)||modview::ci_find(adsb_df_name(m.df),f)
        || modview::ci_find(adsb_tc_name(m.tc),f);
}
int col_cmp(int c, const AdsbRecord& a, const AdsbRecord& b){
    switch(c){
        case 0:  return a.t_ms<b.t_ms?-1:(a.t_ms>b.t_ms?1:0);
        case 1:  return strcmp(a.station,b.station);
        case 2:  return a.icao<b.icao?-1:(a.icao>b.icao?1:0);
        case 3:  return strcmp(a.callsign,b.callsign);
        case 4:  return a.altitude-b.altitude;
        case 5:  return a.lat<b.lat?-1:(a.lat>b.lat?1:0);
        case 6:  return a.lon<b.lon?-1:(a.lon>b.lon?1:0);
        case 7:  return a.speed<b.speed?-1:(a.speed>b.speed?1:0);
        case 8:  return a.track<b.track?-1:(a.track>b.track?1:0);
        default: return a.df-b.df;
    }
}
std::string msg_key(const AdsbRecord& m){
    char b[48]; snprintf(b,sizeof(b),"%lld|%u|%d|%d",(long long)m.t_ms,m.icao,m.df,m.tc);
    return std::string(b);
}
void info_str(const AdsbRecord& m, char* o, size_t n){
    const char* dn=adsb_df_name(m.df);
    if(m.df==17||m.df==18){ const char* tn=adsb_tc_name(m.tc);
        if(m.callsign[0]) snprintf(o,n,"%s",m.callsign);
        else if(m.has_vel) snprintf(o,n,"%.0f kt", m.speed);
        else snprintf(o,n,"%s",tn[0]?tn:dn); }
    else snprintf(o,n,"%s",dn);
}
// ICAO별 최신 병합 head + 항적 꼬리 (지도용)
struct AdsbTrack { AdsbRecord head; std::deque<std::array<float,2>> trail; };
// 고도대별 마커 색 (저공=초록 → 고공=파랑)
ImU32 alt_color(bool ha, int alt){
    if(!ha) return IM_COL32(150,150,160,255);
    if(alt<5000)   return IM_COL32(120,230,140,255);
    if(alt<15000)  return IM_COL32(235,215,120,255);
    if(alt<30000)  return IM_COL32(255,170, 90,255);
    return IM_COL32(120,200,255,255);
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
    static AdsbRecord focus{}; static bool has_focus=false;
    static int sort_col=-1; static bool sort_asc=true;
    static bool atb=true; static size_t lastn=0;
    static uint32_t map_pin=0;                 // 지도에서 핀(필터)된 ICAO (0=없음)
    if(map_pin){ char ms[8]; icao_str(map_pin,ms); if(strcmp(filter,ms)!=0) map_pin=0; }

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; };

    modview::space_toggle_recv(v, "adsb", remote, win_focus, on_clear);
    bool focus_filter = win_focus && !io.WantTextInput &&
        ((io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F,false)) || ImGui::IsKeyPressed(ImGuiKey_Tab,false));

    // Ctrl+C → 선택 행 복사 (탭 구분)
    if(win_focus && !io.WantTextInput && io.KeyCtrl && !io.KeyShift &&
       ImGui::IsKeyPressed(ImGuiKey_C,false) && !sel.empty()){
        std::string out; std::lock_guard<std::mutex> lk(mtx);
        for(const AdsbRecord& m : log){ if(!sel.count(msg_key(m))) continue;
            char ts[12]; hms(m.t_ms,ts); char ic[8]; icao_str(m.icao,ic); char inf[64]; info_str(m,inf,sizeof(inf));
            char alt[12]={0},lat[16]={0},lon[16]={0},spd[12]={0},trk[12]={0};
            if(m.has_alt) snprintf(alt,sizeof(alt),"%d",m.altitude);
            if(m.has_pos){ snprintf(lat,sizeof(lat),"%.5f",m.lat); snprintf(lon,sizeof(lon),"%.5f",m.lon); }
            if(m.has_vel){ snprintf(spd,sizeof(spd),"%.0f",m.speed); snprintf(trk,sizeof(trk),"%.0f",m.track); }
            char line[320];
            snprintf(line,sizeof(line),"%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n",
                ts,m.station[0]?m.station:"LOCAL",ic,m.callsign,alt,lat,lon,spd,trk,inf);
            out+=line;
        }
        if(!out.empty()) ImGui::SetClipboardText(out.c_str());
    }

    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    modview::header_bar(v, "adsb", filter, sizeof(filter), total, remote, focus_filter, on_clear);

    float detail_h = has_focus ? 132.f : 0.f;
    float upper_h = H - 30 - detail_h - 16; if(upper_h<80) upper_h=80;

    // ── ICAO별 병합 head + 항적 캐시 (signature 게이팅, 증분) ──
    static std::unordered_map<uint32_t, AdsbTrack> tracks;
    static int64_t track_wm=-1; static size_t track_logn=(size_t)-1;
    {
        std::lock_guard<std::mutex> lk(mtx);
        size_t n=log.size(); int64_t last=n?log.back().t_ms:0;
        if(n<track_logn){ tracks.clear(); track_wm=-1; }
        track_logn=n;
        if(last>track_wm){
            int start=(int)n-1; while(start>=0 && log[start].t_ms>track_wm) start--; start++;
            for(int j=start;j<(int)n;j++){
                const AdsbRecord& m=log[j];
                AdsbTrack& t=tracks[m.icao];
                t.head.icao=m.icao;
                if(m.t_ms>=t.head.t_ms){ t.head.t_ms=m.t_ms; strncpy(t.head.station,m.station,sizeof(t.head.station)-1); t.head.df=m.df; t.head.tc=m.tc; }
                if(m.callsign[0]){ strncpy(t.head.callsign,m.callsign,sizeof(t.head.callsign)-1); }
                if(m.has_alt){ t.head.altitude=m.altitude; t.head.has_alt=true; }
                if(m.has_vel){ t.head.speed=m.speed; t.head.track=m.track; t.head.has_vel=true; }
                if(m.has_pos){
                    t.head.lat=m.lat; t.head.lon=m.lon; t.head.has_pos=true;
                    float la=(float)m.lat, lo=(float)m.lon;
                    if(t.trail.empty()) t.trail.push_back({la,lo});
                    else { auto& bk=t.trail.back();
                           float dla=la-bk[0], dlo=(lo-bk[1])*cosf(la*(float)M_PI/180.f);
                           if(dla*dla+dlo*dlo > 5e-4f*5e-4f) t.trail.push_back({la,lo}); }
                    while(t.trail.size()>30) t.trail.pop_front();
                }
            }
            int64_t cutoff=last-5LL*60*1000;                  // 5분 지난 트랙 제거
            for(auto it=tracks.begin(); it!=tracks.end();)
                if(it->second.head.t_ms<cutoff) it=tracks.erase(it); else ++it;
            track_wm=last;
        }
    }
    // ── 표시용 MapPoint 빌드 (락 밖) ──
    static std::vector<modview_map::MapPoint> pts;
    static std::vector<std::vector<float>>    trailbuf;
    static std::vector<std::string>           tip1, tip2;
    pts.clear(); trailbuf.clear(); tip1.clear(); tip2.clear();
    for(auto& kv : tracks){
        const AdsbRecord& m=kv.second.head;
        if(!m.has_pos) continue;
        modview_map::MapPoint mpt;
        mpt.lat=m.lat; mpt.lon=m.lon; mpt.id=m.icao;
        mpt.heading = m.has_vel? m.track : -1.f;
        mpt.selected = (has_focus && focus.icao==m.icao);
        mpt.color = mpt.selected? IM_COL32(255,210,80,255) : alt_color(m.has_alt,m.altitude);
        mpt.label = m.callsign[0]? m.callsign : nullptr;
        trailbuf.emplace_back();
        for(auto& pr : kv.second.trail){ trailbuf.back().push_back(pr[0]); trailbuf.back().push_back(pr[1]); }
        char b1[80], b2[64], ic[8]; icao_str(m.icao,ic);
        snprintf(b1,sizeof(b1),"%s  %s", ic, m.callsign[0]?m.callsign:"");
        if(m.has_alt && m.has_vel) snprintf(b2,sizeof(b2),"%d ft  %.0f kt",m.altitude,m.speed);
        else if(m.has_alt) snprintf(b2,sizeof(b2),"%d ft",m.altitude);
        else if(m.has_vel) snprintf(b2,sizeof(b2),"%.0f kt",m.speed);
        else b2[0]=0;
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
    if(W < 200+220+6) tw=W*0.5f;
    else { tw=W*split; if(tw<200)tw=200; if(tw>W-226)tw=W-226; }
    float mapw=W-tw-6; if(mapw<10)mapw=10; if(tw<10)tw=10;

    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##adsb_tbl", 10, tf, ImVec2(tw, upper_h))){
        ImGui::TableSetupScrollFreeze(2,1);
        ImGui::TableSetupColumn("Time",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Station", ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("ICAO",    ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("Call",    ImGuiTableColumnFlags_WidthFixed, 80);
        ImGui::TableSetupColumn("Alt",     ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("Lat",     ImGuiTableColumnFlags_WidthFixed, 78);
        ImGui::TableSetupColumn("Lon",     ImGuiTableColumnFlags_WidthFixed, 84);
        ImGui::TableSetupColumn("Spd",     ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("Trk",     ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("Info",    ImGuiTableColumnFlags_WidthFixed, 160);
        modview::sortable_headers(10, sort_col, sort_asc, 9);

        std::lock_guard<std::mutex> lk(mtx);
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
            const AdsbRecord& m = log[vis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            std::string k=msg_key(m);
            if(modview::row_col0(vis[r], sel.count(k)>0, ts)){
                modview::apply_click(sel, anchor, k, r, [&](int i){return msg_key(log[vis[i]]);}, (int)vis.size(), io.KeyCtrl, io.KeyShift);
                focus=m; has_focus=!sel.empty();
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); modview::cell(m.station[0]?m.station:"LOCAL", ImVec4(0.85f,0.75f,0.5f,1.f));
            ImGui::TableSetColumnIndex(2); { char ic[8]; icao_str(m.icao,ic); modview::cell(ic, ImVec4(0.5f,0.8f,1.f,1.f)); }
            ImGui::TableSetColumnIndex(3); if(m.callsign[0]) modview::cell(m.callsign);
            ImGui::TableSetColumnIndex(4); if(m.has_alt){ snprintf(b,sizeof(b),"%d",m.altitude); modview::cell(b); }
            ImGui::TableSetColumnIndex(5); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lat); modview::cell(b); }
            ImGui::TableSetColumnIndex(6); if(m.has_pos){ snprintf(b,sizeof(b),"%.5f",m.lon); modview::cell(b); }
            ImGui::TableSetColumnIndex(7); if(m.has_vel){ snprintf(b,sizeof(b),"%.0f",m.speed); modview::cell(b); }
            ImGui::TableSetColumnIndex(8); if(m.has_vel){ snprintf(b,sizeof(b),"%.0f",m.track); modview::cell(b); }
            ImGui::TableSetColumnIndex(9); { char inf[64]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        bool grew = log.size()>lastn; lastn=log.size();
        modview::tail_follow(atb, grew && sort_col<0);
        ImGui::EndTable();
    }

    // ── 스플리터 ──
    ImGui::SameLine(0,0);
    ImGui::InvisibleButton("##adsb_split", ImVec2(6, upper_h));
    if(ImGui::IsItemActive()){ split += io.MouseDelta.x/(W>1?W:1); if(split<0.2f)split=0.2f; if(split>0.8f)split=0.8f; }
    if(ImGui::IsItemHovered()||ImGui::IsItemActive()){
        ImVec2 a=ImGui::GetItemRectMin(), b=ImGui::GetItemRectMax();
        ImGui::GetWindowDrawList()->AddRectFilled(ImVec2((a.x+b.x)*0.5f-1,a.y),ImVec2((a.x+b.x)*0.5f+1,b.y),IM_COL32(120,140,160,200));
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }
    // ── 지도 (우측) ──
    ImGui::SameLine(0,0);
    static modview_map::MapView mv;
    auto mres = modview_map::draw_map("##adsb_map", mv, pts, ImVec2(mapw, upper_h), just_opened);
    if(mres.clicked_id){
        uint32_t id=(uint32_t)mres.clicked_id;
        if(map_pin==id){ map_pin=0; filter[0]=0; }
        else { map_pin=id; icao_str(id,filter); }
        std::lock_guard<std::mutex> lk(mtx);
        for(const AdsbRecord& m : log) if(m.icao==id) focus=m;
        std::string fk=msg_key(focus); sel.clear(); sel.insert(fk); anchor=fk; has_focus=true;
    }

    // ── 세부 패널 ──
    if(has_focus){
        modview::detail_begin("adsb", x0, W, detail_h);
        char tsd[12]; hms(focus.t_ms,tsd);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);
        ImGui::Text("Time:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",tsd);
        modview::kv("Station:", focus.station[0]?focus.station:"LOCAL", ImVec4(0.85f,0.75f,0.5f,1.f));
        { char ic[8]; icao_str(focus.icao,ic); modview::kv("ICAO:", ic, ImVec4(0.5f,0.8f,1.f,1.f)); }
        modview::kv("Call:", focus.callsign, V);
        modview::kv("DF:", adsb_df_name(focus.df), ImVec4(0.6f,0.6f,0.6f,1.f));
        if(focus.df==17||focus.df==18) modview::kv("Msg:", adsb_tc_name(focus.tc), ImVec4(0.6f,0.7f,0.62f,1.f));
        ImGui::Separator();
        if(focus.has_alt){ char s[16]; snprintf(s,sizeof(s),"%d ft",focus.altitude); ImGui::Text("Alt:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",s); }
        else { ImGui::Text("Alt:"); ImGui::SameLine(); ImGui::TextDisabled("-"); }
        if(focus.has_pos){ char p[40]; snprintf(p,sizeof(p),"%.5f",focus.lat); modview::kv("Lat:", p, V);
                           snprintf(p,sizeof(p),"%.5f",focus.lon); modview::kv("Lon:", p, V); }
        if(focus.has_vel){ char s[16]; snprintf(s,sizeof(s),"%.0f kt",focus.speed); modview::kv("Spd:", s, V);
                           snprintf(s,sizeof(s),"%.0f",focus.track); modview::kv("Trk:", s, V); }
        if(focus.has_vr){ char s[16]; snprintf(s,sizeof(s),"%+d ft/min",focus.vert_rate); modview::kv("VRate:", s, V); }
        modview::detail_end();
    }
}

} // namespace adsb_mod
