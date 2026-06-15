// ── WiFi 모듈 GUI: DEMOD 데이터 뷰 (공용 modview 컴포넌트로 통일) ───────────
#include "wifi_module.hpp"
#include "../modview.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <set>
#include <ctime>

namespace wifi_mod {

namespace {
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
// 0 Time 1 Freq 2 SSID 3 Ch 4 Sec 5 Peak 6 Bursts/SR
bool match(const WifiRecord& m, const char* f){
    if(!f||!f[0]) return true;
    char fq[16]; snprintf(fq,sizeof(fq),"%.3f",m.freq);
    return modview::ci_find(fq,f)||modview::ci_find(m.ssid,f)||modview::ci_find(m.sec,f)||modview::ci_find(m.phy,f);
}
int col_cmp(int c, const WifiRecord& a, const WifiRecord& b){
    switch(c){
        case 0:  return a.t_ms<b.t_ms?-1:(a.t_ms>b.t_ms?1:0);
        case 1:  return a.freq<b.freq?-1:(a.freq>b.freq?1:0);
        case 2:  return strcmp(a.ssid,b.ssid);
        case 3:  return a.wch-b.wch;
        case 4:  return strcmp(a.sec,b.sec);
        case 5:  return a.peak_dbfs<b.peak_dbfs?-1:(a.peak_dbfs>b.peak_dbfs?1:0);
        default: return a.burst_count-b.burst_count;
    }
}
std::string msg_key(const WifiRecord& m){
    char b[48]; snprintf(b,sizeof(b),"%lld|%d|%d",(long long)m.t_ms,m.ch,m.burst_count);
    return std::string(b);
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
    static WifiRecord focus{}; static bool has_focus=false;
    static int sort_col=-1; static bool sort_asc=true;
    static bool atb=true; static size_t lastn=0;

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; };

    modview::space_toggle_recv(v, "wifi", remote, win_focus, on_clear);
    bool focus_filter = win_focus && !io.WantTextInput &&
        ((io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F,false)) || ImGui::IsKeyPressed(ImGuiKey_Tab,false));

    if(win_focus && !io.WantTextInput && io.KeyCtrl && !io.KeyShift &&
       ImGui::IsKeyPressed(ImGuiKey_C,false) && !sel.empty()){
        std::string out; std::lock_guard<std::mutex> lk(mtx);
        for(const WifiRecord& m : log){ if(!sel.count(msg_key(m))) continue;
            char ts[12]; hms(m.t_ms,ts);
            char line[256];
            snprintf(line,sizeof(line),"%s\t%.3f\t%s\t%d\t%s\t%.0f\t%d / %uM\n",
                ts,m.freq,m.ssid[0]?m.ssid:"(capturing)",m.wch,m.sec,m.peak_dbfs,m.burst_count,m.out_sr/1000000);
            out+=line;
        }
        if(!out.empty()) ImGui::SetClipboardText(out.c_str());
    }

    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    modview::header_bar(v, "wifi", filter, sizeof(filter), total, remote, focus_filter, on_clear);

    float detail_h = has_focus ? 110.f : 0.f;
    float table_h = H - 30 - detail_h - 16; if(table_h<60) table_h=60;

    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##wifi_tbl", 7, tf, ImVec2(W, table_h))){
        ImGui::TableSetupScrollFreeze(2,1);
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Freq", ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("SSID", ImGuiTableColumnFlags_WidthFixed, 150);
        ImGui::TableSetupColumn("Ch",   ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Sec",  ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Peak", ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("Bursts/SR", ImGuiTableColumnFlags_WidthStretch);
        modview::sortable_headers(7, sort_col, sort_asc, 6);

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
            const WifiRecord& m = log[vis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            std::string k=msg_key(m);
            if(modview::row_col0(vis[r], sel.count(k)>0, ts)){
                modview::apply_click(sel, anchor, k, r, [&](int i){return msg_key(log[vis[i]]);}, (int)vis.size(), io.KeyCtrl, io.KeyShift);
                focus=m; has_focus=!sel.empty();
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); snprintf(b,sizeof(b),"%.3f",m.freq); modview::cell(b);
            ImGui::TableSetColumnIndex(2);
            if(m.ssid[0]) modview::cell(m.ssid); else modview::cell("(capturing)", ImVec4(0.5f,0.5f,0.5f,1.f));
            ImGui::TableSetColumnIndex(3); if(m.wch){ snprintf(b,sizeof(b),"%d",m.wch); modview::cell(b); }
            ImGui::TableSetColumnIndex(4); if(m.sec[0]) modview::cell(m.sec);
            ImGui::TableSetColumnIndex(5); snprintf(b,sizeof(b),"%.0f dB",m.peak_dbfs); modview::cell(b);
            ImGui::TableSetColumnIndex(6); { char s[32]; snprintf(s,sizeof(s),"%d / %u M",m.burst_count,m.out_sr/1000000); ImGui::TextUnformatted(s); }
        }
        bool grew = log.size()>lastn; lastn=log.size();
        modview::tail_follow(atb, grew && sort_col<0);
        ImGui::EndTable();
    }

    // ── 세부 패널 ──
    if(has_focus){
        modview::detail_begin("wifi", x0, W, detail_h);
        char tsd[12]; hms(focus.t_ms,tsd);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);
        ImGui::Text("Time:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",tsd);
        { char fq[16]; snprintf(fq,sizeof(fq),"%.3f",focus.freq); modview::kv("Freq:", fq, V); }
        modview::kv("SSID:", focus.ssid[0]?focus.ssid:"(capturing)", ImVec4(0.5f,0.8f,1.f,1.f));
        modview::kv("BSSID:", focus.bssid, V);
        if(focus.wch){ char c[8]; snprintf(c,sizeof(c),"%d",focus.wch); modview::kv("Ch:", c, V); }
        modview::kv("Sec:", focus.sec, V);
        modview::kv("Phy:", focus.phy, V);
        ImGui::Separator();
        { char p[16]; snprintf(p,sizeof(p),"%.0f dB",focus.peak_dbfs); ImGui::Text("Peak:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",p); }
        { char s[16]; snprintf(s,sizeof(s),"%d",focus.burst_count); modview::kv("Bursts:", s, V); }
        { char s[16]; snprintf(s,sizeof(s),"%u M",focus.out_sr/1000000); modview::kv("SR:", s, V); }
        if(focus.beacon_ms){ char s[12]; snprintf(s,sizeof(s),"%d ms",focus.beacon_ms); modview::kv("Beacon:", s, V); }
        modview::detail_end();
    }
}

} // namespace wifi_mod
