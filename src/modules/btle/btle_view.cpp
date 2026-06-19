// ── BLE 모듈 GUI: DEMOD 데이터 뷰 (패킷 로그 + 세부창). acars_view.cpp 미러 ──────
// 1 패킷 = 1 행. MAC 으로 정렬하면 장치별로 묶임. CONNECT_IND 행은 세부창에서
// 연결 파라미터(AA/Interval/Hop/ChannelMap) 표시.
#include "btle_module.hpp"
#include "../modview.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "kst_time.hpp"
#include <imgui.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include <set>
#include <string>

namespace btle_mod {

namespace {
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
void mac_str(const uint8_t* m, char* o){ // o[18]
    snprintf(o,18,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]);
}
// 광고 채널번호 → 정규 중심주파수 MHz (측정 freq 아님: 37=2402 / 38=2426 / 39=2480)
float adv_chan_mhz(int c){ return c==38?2426.f:(c==39?2480.f:2402.f); }
// RSSI(dBFS) → 신호세기 색 (강/중/약)
ImVec4 rssi_col(float d){
    if(d>-45.f) return ImVec4(0.45f,0.85f,0.45f,1.f);
    if(d>-65.f) return ImVec4(0.85f,0.82f,0.45f,1.f);
    return ImVec4(0.80f,0.55f,0.45f,1.f);
}
// 표시용 한줄 정보 (이름 우선, 없으면 info/회사)
void info_str(const BtleRecord& m, char* o, size_t n){
    if(m.is_connect){ snprintf(o,n,"hop=%d iv=%.1fms",m.hop,m.interval*1.25); return; }
    if(m.info[0]) snprintf(o,n,"%s",m.info);
    else o[0]=0;
}
bool match(const BtleRecord& m, const char* f){
    if(!f||!f[0]) return true;
    char ts[12]; hms(m.t_ms,ts);
    char mc[18]; mac_str(m.mac,mc);
    char inf[64]; info_str(m,inf,sizeof(inf));
    return modview::ci_find(ts,f)||modview::ci_find(mc,f)||modview::ci_find(m.name,f)
        || modview::ci_find(m.station,f)||modview::ci_find(btle_pdu_name(m.pdu_type),f)
        || modview::ci_find(inf,f);
}
int col_cmp(int c, const BtleRecord& a, const BtleRecord& b){
    switch(c){
        case 0:  return a.t_ms<b.t_ms?-1:(a.t_ms>b.t_ms?1:0);
        case 1:  return strcmp(a.station,b.station);
        case 2:  return a.adv_chan-b.adv_chan;
        case 3:  return a.adv_chan-b.adv_chan;             // Freq = 채널 1:1 매핑
        case 4:  return a.pdu_type-b.pdu_type;
        case 5:  return memcmp(a.mac,b.mac,6);
        case 6:  return a.addr_type-b.addr_type;
        case 7:  return a.rssi<b.rssi?-1:(a.rssi>b.rssi?1:0);
        case 8:  return a.cfo_hz<b.cfo_hz?-1:(a.cfo_hz>b.cfo_hz?1:0);
        case 9:  return strcmp(a.name,b.name);
        case 10: return (int)a.crc_ok-(int)b.crc_ok;
        default: return strcmp(a.info,b.info);
    }
}
std::string msg_key(const BtleRecord& m){
    char mc[18]; mac_str(m.mac,mc);
    char b[48]; snprintf(b,sizeof(b),"%lld|%s|%d",(long long)m.t_ms,mc,m.pdu_type);
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
    static BtleRecord focus{}; static bool has_focus=false;
    static int sort_col=-1; static bool sort_asc=true;
    static size_t lastn=0; static bool atb=true;

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; };

    modview::space_toggle_recv(v, "btle", remote, win_focus, on_clear);
    bool focus_filter = win_focus && !io.WantTextInput &&
        ((io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F,false)) || ImGui::IsKeyPressed(ImGuiKey_Tab,false));

    // Ctrl+C → 선택 행 복사 (탭 구분)
    if(win_focus && !io.WantTextInput && io.KeyCtrl && !io.KeyShift &&
       ImGui::IsKeyPressed(ImGuiKey_C,false) && !sel.empty()){
        std::string out; std::lock_guard<std::mutex> lk(mtx);
        for(const BtleRecord& m : log){ if(!sel.count(msg_key(m))) continue;
            char ts[12]; hms(m.t_ms,ts); char mc[18]; mac_str(m.mac,mc); char inf[64]; info_str(m,inf,sizeof(inf));
            char line[360];
            snprintf(line,sizeof(line),"%s\t%s\t%d\t%.0fMHz\t%s\t%s\t%s\t%.0f\t%+.1f\t%s\t%s\t%s\n",
                ts,m.station[0]?m.station:"LOCAL",m.adv_chan,adv_chan_mhz(m.adv_chan),
                btle_pdu_name(m.pdu_type),mc,m.addr_type?"R":"P",
                m.rssi,m.cfo_hz/1000.f,m.name,m.crc_ok?"OK":"FAIL",inf);
            out+=line;
        }
        if(!out.empty()) ImGui::SetClipboardText(out.c_str());
    }

    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    modview::header_bar(v, "btle", filter, sizeof(filter), total, remote, focus_filter, on_clear);

    float detail_h = has_focus ? (focus.is_connect ? 150.f : 114.f) : 0.f;
    float table_h  = H - 30 - detail_h - 16; if(table_h<60) table_h=60;

    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##btle_tbl", 12, tf, ImVec2(W, table_h))){
        ImGui::TableSetupScrollFreeze(2,1);
        ImGui::TableSetupColumn("Time",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Station", ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("Ch",      ImGuiTableColumnFlags_WidthFixed, 36);
        ImGui::TableSetupColumn("Freq",    ImGuiTableColumnFlags_WidthFixed, 80);
        ImGui::TableSetupColumn("Type",    ImGuiTableColumnFlags_WidthFixed, 130);
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_WidthFixed, 130);
        ImGui::TableSetupColumn("AT",      ImGuiTableColumnFlags_WidthFixed, 32);
        ImGui::TableSetupColumn("RSSI",    ImGuiTableColumnFlags_WidthFixed, 52);
        ImGui::TableSetupColumn("CFO",     ImGuiTableColumnFlags_WidthFixed, 66);
        ImGui::TableSetupColumn("Name",    ImGuiTableColumnFlags_WidthFixed, 150);
        ImGui::TableSetupColumn("CRC",     ImGuiTableColumnFlags_WidthFixed, 44);
        ImGui::TableSetupColumn("Info",    ImGuiTableColumnFlags_WidthFixed, 240);
        modview::sortable_headers(12, sort_col, sort_asc, 11);

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
            const BtleRecord& m = log[vis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            std::string k=msg_key(m);
            if(modview::row_col0(vis[r], sel.count(k)>0, ts)){
                modview::apply_click(sel, anchor, k, r, [&](int i){return msg_key(log[vis[i]]);}, (int)vis.size(), io.KeyCtrl, io.KeyShift);
                focus=m; has_focus=!sel.empty();
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); modview::cell(m.station[0]?m.station:"LOCAL", ImVec4(0.85f,0.75f,0.5f,1.f));
            ImGui::TableSetColumnIndex(2); snprintf(b,sizeof(b),"%d",m.adv_chan); modview::cell(b);
            ImGui::TableSetColumnIndex(3); snprintf(b,sizeof(b),"%.0f MHz",adv_chan_mhz(m.adv_chan)); modview::cell(b, ImVec4(0.72f,0.72f,0.55f,1.f));
            ImGui::TableSetColumnIndex(4); modview::cell(btle_pdu_name(m.pdu_type),
                m.is_connect?ImVec4(1.f,0.7f,0.4f,1.f):ImVec4(0.7f,0.8f,0.9f,1.f));
            ImGui::TableSetColumnIndex(5); { char mc[18]; mac_str(m.mac,mc); modview::cell(mc, ImVec4(0.5f,0.8f,1.f,1.f)); }
            ImGui::TableSetColumnIndex(6); modview::cell(m.addr_type?"R":"P", ImVec4(0.6f,0.6f,0.6f,1.f));
            ImGui::TableSetColumnIndex(7); if(m.rssi!=0.f){ snprintf(b,sizeof(b),"%.0f",m.rssi); modview::cell(b, rssi_col(m.rssi)); } else modview::cell("-", ImVec4(0.4f,0.4f,0.4f,1.f));
            ImGui::TableSetColumnIndex(8); snprintf(b,sizeof(b),"%+.1f",m.cfo_hz/1000.f); modview::cell(b, ImVec4(0.72f,0.66f,0.86f,1.f));
            ImGui::TableSetColumnIndex(9); if(m.name[0]) ImGui::TextUnformatted(m.name);
            ImGui::TableSetColumnIndex(10); modview::cell(m.crc_ok?"OK":"FAIL",
                m.crc_ok?ImVec4(0.4f,0.85f,0.4f,1.f):ImVec4(0.85f,0.5f,0.35f,1.f));
            ImGui::TableSetColumnIndex(11); { char inf[64]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        bool grew = log.size()>lastn; lastn=log.size();
        modview::tail_follow(atb, grew && sort_col<0);
        ImGui::EndTable();
    }

    // ── 세부 패널 ──
    if(has_focus){
        modview::detail_begin("btle", x0, W, detail_h);
        char tsd[12]; hms(focus.t_ms,tsd);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);
        ImGui::Text("Time:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",tsd);
        modview::kv("Station:", focus.station[0]?focus.station:"LOCAL", ImVec4(0.85f,0.75f,0.5f,1.f));
        { char b[8]; snprintf(b,sizeof(b),"%d",focus.adv_chan); modview::kv("Ch:", b, V); }
        modview::kv("Type:", btle_pdu_name(focus.pdu_type), ImVec4(0.7f,0.8f,0.9f,1.f));
        { char mc[18]; mac_str(focus.mac,mc); modview::kv("Addr:", mc, ImVec4(0.5f,0.8f,1.f,1.f)); }
        modview::kv("AddrType:", focus.addr_type?"Random":"Public", ImVec4(0.6f,0.6f,0.6f,1.f));
        modview::kv("CRC:", focus.crc_ok?"OK":"FAIL", focus.crc_ok?ImVec4(0.4f,0.85f,0.4f,1.f):ImVec4(0.85f,0.5f,0.35f,1.f));
        { char b[16]; snprintf(b,sizeof(b),"%.0f MHz",adv_chan_mhz(focus.adv_chan)); modview::kv("Freq:", b, ImVec4(0.72f,0.72f,0.55f,1.f), false); }
        if(focus.rssi!=0.f){ char b[16]; snprintf(b,sizeof(b),"%.1f dBFS",focus.rssi); modview::kv("RSSI:", b, rssi_col(focus.rssi)); }
        { char b[20]; snprintf(b,sizeof(b),"%+.2f kHz",focus.cfo_hz/1000.f); modview::kv("CFO:", b, ImVec4(0.72f,0.66f,0.86f,1.f)); }
        ImGui::Separator();
        if(focus.name[0]) modview::kv("Name:", focus.name, V, false);
        if(focus.flags>=0){ char b[12]; snprintf(b,sizeof(b),"0x%02X",focus.flags & 0xFF); modview::kv("Flags:", b, V, !(focus.name[0]==0)); }
        if(focus.company!=0xFFFF){ const char* cn=btle_company_name(focus.company);
            char b[40]; if(cn[0]) snprintf(b,sizeof(b),"%s (0x%04X)",cn,focus.company); else snprintf(b,sizeof(b),"0x%04X",focus.company);
            modview::kv("Company:", b, ImVec4(0.62f,0.7f,0.62f,1.f)); }
        if(focus.info[0]) modview::kv("Info:", focus.info, V);
        if(focus.is_connect){
            ImGui::Separator();
            char b[40];
            { char im[18]; mac_str(focus.init_mac,im); modview::kv("Init:", im, ImVec4(0.5f,0.8f,1.f,1.f), false); }
            snprintf(b,sizeof(b),"0x%08X",focus.access_addr); modview::kv("AA:", b, V);
            snprintf(b,sizeof(b),"0x%06X",focus.crc_init);    modview::kv("CRCInit:", b, V);
            snprintf(b,sizeof(b),"%.2f ms",focus.interval*1.25); modview::kv("Interval:", b, V);
            snprintf(b,sizeof(b),"%d ms",focus.timeout*10);   modview::kv("Timeout:", b, V);
            snprintf(b,sizeof(b),"%d",focus.hop);             modview::kv("Hop:", b, V);
            snprintf(b,sizeof(b),"0x%010llX",(unsigned long long)focus.chan_map); modview::kv("ChMap:", b, V);
        }
        modview::detail_end();
    }
}

} // namespace btle_mod
