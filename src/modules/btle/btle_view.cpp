// ── BLE 모듈 GUI: master-detail 뷰 (ais_view.cpp 분할 미러) ─────────────────
// 좌=AdvA(MAC)별 그룹표(누계/최신 RSSI·CFO·info), 클릭 → 우=그 MAC 패킷 라이브
// (시간순 tail-follow). 하단=포커스 패킷 세부(애플 디코드/신호/CONNECT_IND 파라미터).
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
#include <unordered_map>
#include <algorithm>

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
// ── MAC 그룹 (좌측: 동일 AdvA 묶음, 최신값 + 누계) ──────────────────────────
struct Grp {
    uint8_t  mac[6];
    int      at;       // addr_type (최신)
    int      ch;       // adv_chan (최신)
    int      cnt;      // 패킷 누계
    int64_t  first;    // 최초 수신 t_ms (Up — 불변, 정렬키)
    int64_t  last;     // 최근 수신 t_ms (Down — 갱신)
    char     name[24]; // 최신 비공백 이름
    char     info[48]; // 최신 info(제조사/애플 디코드)
    float    rssi;     // 최신
    float    cfo;      // 최신
};
// 컬럼: 0 Up 1 Down 2 Address 3 Cnt 4 AT 5 RSSI 6 CFO 7 Name 8 Info
int grp_cmp(int c, const Grp& a, const Grp& b){
    switch(c){
        case 0:  return a.first<b.first?-1:(a.first>b.first?1:0);    // Up (최초)
        case 1:  return a.last<b.last?-1:(a.last>b.last?1:0);        // Down (최근)
        case 2:  return memcmp(a.mac,b.mac,6);                       // Address
        case 3:  return a.cnt-b.cnt;                                // Cnt
        case 4:  return a.at-b.at;                                  // AT
        case 5:  return a.rssi<b.rssi?-1:(a.rssi>b.rssi?1:0);       // RSSI
        case 6:  return a.cfo<b.cfo?-1:(a.cfo>b.cfo?1:0);           // CFO
        case 7:  return strcmp(a.name,b.name);                      // Name
        default: return strcmp(a.info,b.info);                      // Info
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
    static uint8_t sel_mac[6]={}; static bool has_sel=false;       // 좌측 선택된 MAC 그룹
    static int gsort_col=0; static bool gsort_asc=true;            // 그룹 정렬: 기본 Up 오름차순(불변→순서고정)
    static size_t lastn=0; static bool atb=true;

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; has_sel=false; };

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
    float upper_h  = H - 30 - detail_h - 16; if(upper_h<80) upper_h=80;

    // ── 좌(MAC 그룹) | 스플리터 | 우(선택 MAC 패킷 라이브) ──
    static float split=0.46f;
    float tw;
    if(W < 240+220+6) tw=W*0.5f;
    else { tw=W*split; if(tw<240)tw=240; if(tw>W-220)tw=W-220; }
    float rightw=W-tw-6; if(rightw<10)rightw=10; if(tw<10)tw=10;

    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;

    // ── 그룹 집계 (MAC별, 캐시: 개수/마지막t/필터/정렬 게이팅) ──
    static std::vector<Grp> grps;
    {
        std::lock_guard<std::mutex> lk(mtx);
        static size_t c_n=(size_t)-1; static int64_t c_last=-1; static char c_filter[64]={'\xff'};
        static int c_sc=-99; static bool c_asc=false;
        size_t n_now=log.size(); int64_t last_t=n_now?log.back().t_ms:0;
        if(n_now!=c_n||last_t!=c_last||strncmp(c_filter,filter,sizeof(c_filter))||c_sc!=gsort_col||c_asc!=gsort_asc){
            std::vector<Grp> g; g.reserve(256);
            std::unordered_map<uint64_t,int> idx; idx.reserve(512);
            for(size_t i=0;i<n_now;i++){
                const BtleRecord& m=log[i];
                if(!match(m,filter)) continue;
                uint64_t key=0; for(int b=0;b<6;b++) key=(key<<8)|m.mac[b];
                auto it=idx.find(key); int gi;
                if(it==idx.end()){ gi=(int)g.size(); idx[key]=gi; Grp ng{}; memcpy(ng.mac,m.mac,6); ng.last=-1; ng.first=m.t_ms; g.push_back(ng); }
                else gi=it->second;
                Grp& G=g[gi]; G.cnt++;
                if(m.t_ms<G.first) G.first=m.t_ms;        // Up = 최초(불변)
                if(m.t_ms>=G.last){
                    G.last=m.t_ms; G.ch=m.adv_chan; G.rssi=m.rssi; G.cfo=m.cfo_hz; G.at=m.addr_type;
                    if(m.name[0]){ strncpy(G.name,m.name,sizeof(G.name)-1); G.name[sizeof(G.name)-1]=0; }
                    char inf[48]; info_str(m,inf,sizeof(inf)); strncpy(G.info,inf,sizeof(G.info)-1); G.info[sizeof(G.info)-1]=0;
                } else if(!G.name[0] && m.name[0]){ strncpy(G.name,m.name,sizeof(G.name)-1); G.name[sizeof(G.name)-1]=0; }
            }
            std::stable_sort(g.begin(),g.end(),[&](const Grp&a,const Grp&b){ int c=grp_cmp(gsort_col,a,b); return gsort_asc? c<0:c>0; });
            grps.swap(g);
            c_n=n_now; c_last=last_t; strncpy(c_filter,filter,sizeof(c_filter)-1); c_filter[sizeof(c_filter)-1]=0; c_sc=gsort_col; c_asc=gsort_asc;
        }
    }

    // ── 좌측: MAC 그룹 표 (클릭 → 우측에 그 MAC 패킷 라이브) ──
    ImGui::SetCursorPosX(x0);
    if(ImGui::BeginTable("##btle_grp", 9, tf, ImVec2(tw, upper_h))){
        ImGui::TableSetupScrollFreeze(3,1);
        ImGui::TableSetupColumn("Up",      ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Down",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Address", ImGuiTableColumnFlags_WidthFixed, 132);
        ImGui::TableSetupColumn("Cnt",     ImGuiTableColumnFlags_WidthFixed, 46);
        ImGui::TableSetupColumn("AT",      ImGuiTableColumnFlags_WidthFixed, 30);
        ImGui::TableSetupColumn("RSSI",    ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("CFO",     ImGuiTableColumnFlags_WidthFixed, 58);
        ImGui::TableSetupColumn("Name",    ImGuiTableColumnFlags_WidthFixed, 120);
        ImGui::TableSetupColumn("Info",    ImGuiTableColumnFlags_WidthFixed, 170);
        modview::sortable_headers(9, gsort_col, gsort_asc, 8);   // Info 좌측정렬

        ImGuiListClipper clip; clip.Begin((int)grps.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const Grp& G=grps[r];
            ImGui::TableNextRow();
            char up[12]; hms(G.first,up);
            bool selrow = has_sel && memcmp(G.mac,sel_mac,6)==0;
            if(modview::row_col0(r, selrow, up)){                // col0 = Up time
                memcpy(sel_mac,G.mac,6); has_sel=true; atb=true;
                std::lock_guard<std::mutex> lk(mtx);
                for(auto it=log.rbegin(); it!=log.rend(); ++it)
                    if(memcmp(it->mac,sel_mac,6)==0){ focus=*it; has_focus=true; break; }
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); { char dn[12]; hms(G.last,dn); modview::cell(dn, ImVec4(0.62f,0.62f,0.62f,1.f)); }
            ImGui::TableSetColumnIndex(2); { char mc[18]; mac_str(G.mac,mc); modview::cell(mc, ImVec4(0.5f,0.8f,1.f,1.f)); }
            ImGui::TableSetColumnIndex(3); snprintf(b,sizeof(b),"%d",G.cnt); modview::cell(b);
            ImGui::TableSetColumnIndex(4); modview::cell(G.at?"R":"P", ImVec4(0.6f,0.6f,0.6f,1.f));
            ImGui::TableSetColumnIndex(5); if(G.rssi!=0.f){ snprintf(b,sizeof(b),"%.0f",G.rssi); modview::cell(b, rssi_col(G.rssi)); } else modview::cell("-", ImVec4(0.4f,0.4f,0.4f,1.f));
            ImGui::TableSetColumnIndex(6); snprintf(b,sizeof(b),"%+.1f",G.cfo/1000.f); modview::cell(b, ImVec4(0.72f,0.66f,0.86f,1.f));
            ImGui::TableSetColumnIndex(7); if(G.name[0]) ImGui::TextUnformatted(G.name);
            ImGui::TableSetColumnIndex(8); if(G.info[0]) ImGui::TextUnformatted(G.info);
        }
        ImGui::EndTable();
    }

    // ── 스플리터 ──
    ImGui::SameLine(0,0);
    ImGui::InvisibleButton("##btle_split", ImVec2(6, upper_h));
    if(ImGui::IsItemActive()){ split += io.MouseDelta.x/(W>1?W:1); if(split<0.25f)split=0.25f; if(split>0.75f)split=0.75f; }
    if(ImGui::IsItemHovered()||ImGui::IsItemActive()){
        ImVec2 a=ImGui::GetItemRectMin(), b=ImGui::GetItemRectMax();
        ImGui::GetWindowDrawList()->AddRectFilled(ImVec2((a.x+b.x)*0.5f-1,a.y),ImVec2((a.x+b.x)*0.5f+1,b.y),IM_COL32(120,140,160,200));
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    // ── 우측: 선택 MAC 패킷 라이브 (시간순, tail-follow) ──
    ImGui::SameLine(0,0);
    if(ImGui::BeginTable("##btle_pkt", 7, tf, ImVec2(rightw, upper_h))){
        ImGui::TableSetupScrollFreeze(1,1);
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 124);
        ImGui::TableSetupColumn("Freq", ImGuiTableColumnFlags_WidthFixed, 64);
        ImGui::TableSetupColumn("Sta",  ImGuiTableColumnFlags_WidthFixed, 54);
        ImGui::TableSetupColumn("RSSI", ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("CFO",  ImGuiTableColumnFlags_WidthFixed, 58);
        ImGui::TableSetupColumn("Info", ImGuiTableColumnFlags_WidthFixed, 220);
        ImGui::TableHeadersRow();

        std::lock_guard<std::mutex> lk(mtx);
        static std::vector<int> rvis; rvis.clear();
        if(has_sel) for(int i=0;i<(int)log.size();i++) if(memcmp(log[i].mac,sel_mac,6)==0) rvis.push_back(i);

        ImGuiListClipper rc; rc.Begin((int)rvis.size());
        while(rc.Step()) for(int r=rc.DisplayStart;r<rc.DisplayEnd;r++){
            const BtleRecord& m=log[rvis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            std::string k=msg_key(m);
            if(modview::row_col0(rvis[r], sel.count(k)>0, ts)){
                modview::apply_click(sel, anchor, k, r, [&](int i){return msg_key(log[rvis[i]]);}, (int)rvis.size(), io.KeyCtrl, io.KeyShift);
                focus=m; has_focus=!sel.empty();
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); modview::cell(btle_pdu_name(m.pdu_type),
                m.is_connect?ImVec4(1.f,0.7f,0.4f,1.f):ImVec4(0.7f,0.8f,0.9f,1.f));
            ImGui::TableSetColumnIndex(2); snprintf(b,sizeof(b),"%.0f",adv_chan_mhz(m.adv_chan)); modview::cell(b, ImVec4(0.72f,0.72f,0.55f,1.f));
            ImGui::TableSetColumnIndex(3); modview::cell(m.station[0]?m.station:"LOCAL", ImVec4(0.85f,0.75f,0.5f,1.f));
            ImGui::TableSetColumnIndex(4); if(m.rssi!=0.f){ snprintf(b,sizeof(b),"%.0f",m.rssi); modview::cell(b, rssi_col(m.rssi)); } else modview::cell("-", ImVec4(0.4f,0.4f,0.4f,1.f));
            ImGui::TableSetColumnIndex(5); snprintf(b,sizeof(b),"%+.1f",m.cfo_hz/1000.f); modview::cell(b, ImVec4(0.72f,0.66f,0.86f,1.f));
            ImGui::TableSetColumnIndex(6); { char inf[64]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        bool grew = rvis.size()>lastn; lastn=rvis.size();
        modview::tail_follow(atb, grew);
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
