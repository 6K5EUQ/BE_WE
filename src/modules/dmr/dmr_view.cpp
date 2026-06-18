// ── DMR 모듈 GUI: DEMOD 데이터 뷰 (공용 modview 컴포넌트) ───────────────────
#include "dmr_module.hpp"
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

namespace dmr_mod {

namespace {
void hms(int64_t ms, char* o){
    time_t t=(time_t)(ms/1000); struct tm tv; KST::to_tm(t,tv);
    strftime(o,12,"%H:%M:%S",&tv);
}
// Type 컬럼 문자열
void type_str(const DmrRecord& m, char* o, size_t n){
    if(m.csbko>=0) snprintf(o,n,"CSBK %d",m.csbko);
    else if(m.flco>=0){ const char* fn=dmr_flco_name(m.flco); snprintf(o,n,"%s",fn[0]?fn:"LC"); }
    else if(m.data_type>=0) snprintf(o,n,"%s",dmr_datatype_name(m.data_type));
    else if(m.is_voice) snprintf(o,n,"Voice");
    else o[0]=0;
}
void info_str(const DmrRecord& m, char* o, size_t n){
    const char* ct = dmr_calltype_name(m.call_type);
    snprintf(o,n,"%s%s%s", ct[0]?ct:"", (ct[0]&&m.crc_ok)?"  ":"", m.crc_ok?"CRC OK":"");
}
bool match(const DmrRecord& m, const char* f){
    if(!f||!f[0]) return true;
    char ts[12]; hms(m.t_ms,ts);
    char src[16],dst[16],cc[8],ty[24];
    snprintf(src,sizeof(src),"%u",m.src_id); snprintf(dst,sizeof(dst),"%u",m.dst_id);
    snprintf(cc,sizeof(cc),"%d",m.color_code); type_str(m,ty,sizeof(ty));
    return modview::ci_find(ts,f)||modview::ci_find(src,f)||modview::ci_find(dst,f)
        || modview::ci_find(cc,f)||modview::ci_find(ty,f)||modview::ci_find(m.station,f);
}
int col_cmp(int c, const DmrRecord& a, const DmrRecord& b){
    switch(c){
        case 0: return a.t_ms<b.t_ms?-1:(a.t_ms>b.t_ms?1:0);
        case 1: return strcmp(a.station,b.station);
        case 2: return a.color_code-b.color_code;
        case 3: return a.slot-b.slot;
        case 5: return a.src_id<b.src_id?-1:(a.src_id>b.src_id?1:0);
        case 6: return a.dst_id<b.dst_id?-1:(a.dst_id>b.dst_id?1:0);
        case 7: return a.call_type-b.call_type;
        default:return 0;
    }
}
std::string msg_key(const DmrRecord& m){
    char b[64]; snprintf(b,sizeof(b),"%lld|%u|%u|%d|%d",(long long)m.t_ms,m.src_id,m.dst_id,m.csbko,m.flco);
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
    static DmrRecord focus{}; static bool has_focus=false;
    static int sort_col=-1; static bool sort_asc=true;
    static bool atb=true; static size_t lastn=0;

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; };
    modview::space_toggle_recv(v, "dmr", remote, win_focus, on_clear);
    bool focus_filter = win_focus && !io.WantTextInput &&
        ((io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F,false)) || ImGui::IsKeyPressed(ImGuiKey_Tab,false));

    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    modview::header_bar(v, "dmr", filter, sizeof(filter), total, remote, focus_filter, on_clear);

    float detail_h = has_focus ? 92.f : 0.f;
    float upper_h = H - 30 - detail_h - 16; if(upper_h<80) upper_h=80;

    ImGui::SetCursorPosX(x0);
    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;
    if(ImGui::BeginTable("##dmr_tbl", 9, tf, ImVec2(W, upper_h))){
        ImGui::TableSetupScrollFreeze(2,1);
        ImGui::TableSetupColumn("Time",   ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Station",ImGuiTableColumnFlags_WidthFixed, 60);
        ImGui::TableSetupColumn("CC",     ImGuiTableColumnFlags_WidthFixed, 34);
        ImGui::TableSetupColumn("Slot",   ImGuiTableColumnFlags_WidthFixed, 38);
        ImGui::TableSetupColumn("Type",   ImGuiTableColumnFlags_WidthFixed, 110);
        ImGui::TableSetupColumn("Src",    ImGuiTableColumnFlags_WidthFixed, 90);
        ImGui::TableSetupColumn("Dst/TG", ImGuiTableColumnFlags_WidthFixed, 90);
        ImGui::TableSetupColumn("Call",   ImGuiTableColumnFlags_WidthFixed, 80);
        ImGui::TableSetupColumn("Info",   ImGuiTableColumnFlags_WidthStretch);
        modview::sortable_headers(9, sort_col, sort_asc, 8);

        std::lock_guard<std::mutex> lk(mtx);
        static std::vector<int> vis;
        static size_t c_n=(size_t)-1; static int64_t c_last=-1;
        static char c_filter[64]={'\xff'}; static int c_sc=-99; static bool c_asc=false;
        size_t n_now=log.size(); int64_t last_t=n_now?log.back().t_ms:0;
        if(n_now!=c_n||last_t!=c_last||strncmp(c_filter,filter,sizeof(c_filter))||c_sc!=sort_col||c_asc!=sort_asc){
            vis.clear(); for(int i=0;i<(int)n_now;i++) if(match(log[i],filter)) vis.push_back(i);
            modview::sort_vis(vis, sort_col, sort_asc, [&](int c,int a,int b){ return col_cmp(c,log[a],log[b]); });
            c_n=n_now; c_last=last_t; strncpy(c_filter,filter,sizeof(c_filter)-1); c_filter[63]=0; c_sc=sort_col; c_asc=sort_asc;
        }

        ImGuiListClipper clip; clip.Begin((int)vis.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const DmrRecord& m = log[vis[r]];
            ImGui::TableNextRow();
            char ts[12]; hms(m.t_ms,ts);
            std::string k=msg_key(m);
            if(modview::row_col0(vis[r], sel.count(k)>0, ts)){
                modview::apply_click(sel, anchor, k, r, [&](int i){return msg_key(log[vis[i]]);}, (int)vis.size(), io.KeyCtrl, io.KeyShift);
                focus=m; has_focus=!sel.empty();
            }
            char b[32];
            ImGui::TableSetColumnIndex(1); modview::cell(m.station[0]?m.station:"LOCAL", ImVec4(0.85f,0.75f,0.5f,1.f));
            ImGui::TableSetColumnIndex(2); if(m.color_code>=0){ snprintf(b,sizeof(b),"%d",m.color_code); modview::cell(b); }
            ImGui::TableSetColumnIndex(3); if(m.slot>0){ snprintf(b,sizeof(b),"%d",m.slot); modview::cell(b); }
            ImGui::TableSetColumnIndex(4); { char ty[24]; type_str(m,ty,sizeof(ty)); if(ty[0]) modview::cell(ty, m.crc_ok?ImVec4(0.6f,0.85f,0.6f,1.f):ImVec4(0.8f,0.8f,0.85f,1.f)); }
            ImGui::TableSetColumnIndex(5); if(m.src_id){ snprintf(b,sizeof(b),"%u",m.src_id); modview::cell(b); }
            ImGui::TableSetColumnIndex(6); if(m.dst_id){ snprintf(b,sizeof(b),"%u",m.dst_id); modview::cell(b, ImVec4(0.5f,0.8f,1.f,1.f)); }
            ImGui::TableSetColumnIndex(7); { const char* ct=dmr_calltype_name(m.call_type); if(ct[0]) modview::cell(ct); }
            ImGui::TableSetColumnIndex(8); { char inf[48]; info_str(m,inf,sizeof(inf)); if(inf[0]) ImGui::TextUnformatted(inf); }
        }
        bool grew = log.size()>lastn; lastn=log.size();
        modview::tail_follow(atb, grew && sort_col<0);
        ImGui::EndTable();
    }

    if(has_focus){
        modview::detail_begin("dmr", x0, W, detail_h);
        char tsd[12]; hms(focus.t_ms,tsd);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);
        ImGui::Text("Time:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",tsd);
        modview::kv("Station:", focus.station[0]?focus.station:"LOCAL", ImVec4(0.85f,0.75f,0.5f,1.f));
        { char b[8]; snprintf(b,sizeof(b),"%d",focus.color_code); modview::kv("CC:", b, V); }
        if(focus.slot>0){ char b[8]; snprintf(b,sizeof(b),"%d",focus.slot); modview::kv("Slot:", b, V); }
        { char ty[24]; type_str(focus,ty,sizeof(ty)); modview::kv("Type:", ty, V); }
        modview::kv("CRC:", focus.crc_ok?"OK":"-", focus.crc_ok?ImVec4(0.6f,0.85f,0.6f,1.f):ImVec4(0.6f,0.6f,0.6f,1.f));
        ImGui::Separator();
        { char b[16]; snprintf(b,sizeof(b),"%u",focus.src_id); modview::kv("Src:", b, ImVec4(0.5f,0.8f,1.f,1.f), false); }
        { char b[16]; snprintf(b,sizeof(b),"%u",focus.dst_id); modview::kv("Dst/TG:", b, ImVec4(0.5f,0.8f,1.f,1.f)); }
        { const char* ct=dmr_calltype_name(focus.call_type); modview::kv("Call:", ct[0]?ct:"-", V); }
        if(focus.flco>=0) modview::kv("FLCO:", dmr_flco_name(focus.flco), V);
        modview::detail_end();
    }
}

} // namespace dmr_mod
