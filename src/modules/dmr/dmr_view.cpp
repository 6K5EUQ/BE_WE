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
#include <unordered_map>
#include <algorithm>

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
bool match(const DmrRecord& m, const char* f){
    if(!f||!f[0]) return true;
    char ts[12]; hms(m.t_ms,ts);
    char src[16],dst[16],cc[8],ty[24];
    snprintf(src,sizeof(src),"%u",m.src_id); snprintf(dst,sizeof(dst),"%u",m.dst_id);
    snprintf(cc,sizeof(cc),"%d",m.color_code); type_str(m,ty,sizeof(ty));
    return modview::ci_find(ts,f)||modview::ci_find(src,f)||modview::ci_find(dst,f)
        || modview::ci_find(cc,f)||modview::ci_find(ty,f)||modview::ci_find(m.station,f);
}
// ── 통화 세션 (B): 같은 (dst,src,slot) 연속 버스트(gap<2s)를 한 통화로 접음 ──
struct Sess {
    uint32_t tg=0, src=0; int slot=0, cc=-1, ctype=-1;
    int64_t  up=0, down=0;        // 키온/키오프 t_ms
    int      bursts=0;
    int      flco=-1, csbko=-1, dtype=-1;  // 대표(최신) 타입
    bool     voice=false, enc=false;
    uint64_t rec_id=0;                     // 통화 녹음 식별자 (Stage2 재생 fetch 키)
};
// ── TG 그룹 (A): 세션을 dst(TG/상대) 별로 묶음 ──
struct Tg {
    uint32_t tg; int ctype;
    int      calls;              // 세션 수
    int64_t  up, down;
    uint32_t last_src; int cc, slot; bool voice, enc;
};
// 컬럼: 0 Up 1 Down 2 TG 3 Calls 4 Src 5 CC 6 Slot
int tg_cmp(int c, const Tg& a, const Tg& b){
    switch(c){
        case 0:  return a.up<b.up?-1:(a.up>b.up?1:0);
        case 1:  return a.down<b.down?-1:(a.down>b.down?1:0);
        case 2:  return a.tg<b.tg?-1:(a.tg>b.tg?1:0);
        case 3:  return a.calls-b.calls;
        case 4:  return a.last_src<b.last_src?-1:(a.last_src>b.last_src?1:0);
        case 5:  return a.cc-b.cc;
        case 6:  return a.slot-b.slot;
        default: return (int)a.voice-(int)b.voice;   // Voice
    }
}
// 세션 선택/포커스 키
std::string sess_key(const Sess& s){
    char b[72]; snprintf(b,sizeof(b),"%u|%u|%d|%lld",s.tg,s.src,s.slot,(long long)s.up);
    return std::string(b);
}
// DmrRecord → 세션 대표 타입 복사
static void sess_type_from(Sess& s, const DmrRecord& m){
    if(m.flco>=0)      s.flco=m.flco;
    if(m.csbko>=0)     s.csbko=m.csbko;
    if(m.data_type>=0) s.dtype=m.data_type;
    if(m.is_voice)     s.voice=true;
}
// 세션 → Type 문자열 (type_str 재사용 위해 임시 레코드 구성)
static void sess_type_str(const Sess& s, char* o, size_t n){
    DmrRecord t{}; t.flco=s.flco; t.csbko=s.csbko; t.data_type=s.dtype; t.is_voice=s.voice;
    type_str(t,o,n);
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
    static Sess focus{}; static bool has_focus=false;
    static uint32_t sel_tg=0; static bool has_sel=false;          // 선택된 TG 그룹
    static int gsort_col=0; static bool gsort_asc=true;           // 기본 Up 오름차순(불변→순서 고정)
    static bool atb=true; static size_t lastn=0;

    auto on_clear=[&](){ std::lock_guard<std::mutex> lk(mtx); log.clear(); sel.clear(); anchor.clear(); has_focus=false; has_sel=false; };
    modview::space_toggle_recv(v, "dmr", remote, win_focus, on_clear);
    bool focus_filter = win_focus && !io.WantTextInput &&
        ((io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_F,false)) || ImGui::IsKeyPressed(ImGuiKey_Tab,false));

    int total; { std::lock_guard<std::mutex> lk(mtx); total=(int)log.size(); }
    modview::header_bar(v, "dmr", filter, sizeof(filter), total, remote, focus_filter, on_clear);

    float detail_h = has_focus ? 100.f : 0.f;
    float upper_h = H - 30 - detail_h - 16; if(upper_h<80) upper_h=80;

    static float split=0.46f;
    float tw;
    if(W < 260+220+6) tw=W*0.5f;
    else { tw=W*split; if(tw<260)tw=260; if(tw>W-220)tw=W-220; }
    float rightw=W-tw-6; if(rightw<10)rightw=10; if(tw<10)tw=10;

    ImGuiTableFlags tf = ImGuiTableFlags_ScrollY|ImGuiTableFlags_ScrollX|ImGuiTableFlags_RowBg|
        ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_Resizable;

    // ── 통화 세션화(B) + TG 그룹(A) 집계 (캐시: n/last_t/filter/정렬 게이팅) ──
    static std::vector<Sess> sess;
    static std::vector<Tg>   tgs;
    {
        std::lock_guard<std::mutex> lk(mtx);
        static size_t c_n=(size_t)-1; static int64_t c_last=-1; static char c_filter[64]={'\xff'};
        static int c_sc=-99; static bool c_asc=false;
        size_t n_now=log.size(); int64_t last_t=n_now?log.back().t_ms:0;
        if(n_now!=c_n||last_t!=c_last||strncmp(c_filter,filter,sizeof(c_filter))||c_sc!=gsort_col||c_asc!=gsort_asc){
            const int64_t GAP=2000;                       // 연속 버스트 판정 (2s)
            std::vector<Sess> sv; sv.reserve(256);
            std::unordered_map<uint64_t,int> open; open.reserve(512);
            for(size_t i=0;i<n_now;i++){
                const DmrRecord& m=log[i];
                if(m.dst_id==0) continue;                 // TG/상대주소 없으면 그룹 불가 → 제외
                if(!match(m,filter)) continue;
                uint64_t key=((uint64_t)m.dst_id<<32) ^ ((uint64_t)m.src_id<<8) ^ (uint64_t)(m.slot&0xFF);
                auto it=open.find(key); int si=-1;
                if(it!=open.end() && m.t_ms - sv[it->second].down <= GAP) si=it->second;
                if(si<0){
                    Sess S; S.tg=m.dst_id; S.src=m.src_id; S.slot=m.slot;
                    S.cc=m.color_code; S.ctype=m.call_type; S.up=S.down=m.t_ms;
                    si=(int)sv.size(); sv.push_back(S); open[key]=si;
                }
                Sess& S=sv[si]; S.down=m.t_ms; S.bursts++;
                if(m.color_code>=0) S.cc=m.color_code;
                if(m.slot>0) S.slot=m.slot;
                if(m.call_type>=0) S.ctype=m.call_type;
                if(m.enc) S.enc=true;
                if(m.rec_id) S.rec_id=m.rec_id;
                sess_type_from(S, m);
            }
            std::vector<Tg> tv; tv.reserve(128);
            std::unordered_map<uint32_t,int> tidx; tidx.reserve(256);
            for(auto& S : sv){
                auto it=tidx.find(S.tg); int gi;
                if(it==tidx.end()){ Tg T{}; T.tg=S.tg; T.ctype=S.ctype; T.up=S.up; T.down=-1; gi=(int)tv.size(); tidx[S.tg]=gi; tv.push_back(T); }
                else gi=it->second;
                Tg& T=tv[gi]; T.calls++; T.voice = T.voice || S.voice; T.enc = T.enc || S.enc;
                if(S.up<T.up) T.up=S.up;
                if(S.down>=T.down){ T.down=S.down; T.last_src=S.src; T.cc=S.cc; T.slot=S.slot; T.ctype=S.ctype; }
            }
            std::stable_sort(tv.begin(),tv.end(),[&](const Tg&a,const Tg&b){ int c=tg_cmp(gsort_col,a,b); return gsort_asc? c<0:c>0; });
            sess.swap(sv); tgs.swap(tv);
            c_n=n_now; c_last=last_t; strncpy(c_filter,filter,sizeof(c_filter)-1); c_filter[63]=0; c_sc=gsort_col; c_asc=gsort_asc;
        }
    }

    // ── 좌측: TG 그룹 표 (클릭 → 우측에 그 TG 통화 세션 라이브) ──
    ImGui::SetCursorPosX(x0);
    if(ImGui::BeginTable("##dmr_tg", 8, tf, ImVec2(tw, upper_h))){
        ImGui::TableSetupScrollFreeze(3,1);
        ImGui::TableSetupColumn("Up",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Down",  ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("TG",    ImGuiTableColumnFlags_WidthFixed, 100);
        ImGui::TableSetupColumn("Calls", ImGuiTableColumnFlags_WidthFixed, 46);
        ImGui::TableSetupColumn("Src",   ImGuiTableColumnFlags_WidthFixed, 90);
        ImGui::TableSetupColumn("CC",    ImGuiTableColumnFlags_WidthFixed, 34);
        ImGui::TableSetupColumn("Slot",  ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Voice", ImGuiTableColumnFlags_WidthFixed, 50);
        modview::sortable_headers(8, gsort_col, gsort_asc, -1);

        ImGuiListClipper clip; clip.Begin((int)tgs.size());
        while(clip.Step()) for(int r=clip.DisplayStart;r<clip.DisplayEnd;r++){
            const Tg& T=tgs[r];
            ImGui::TableNextRow();
            char up[12]; hms(T.up,up);
            bool selrow = has_sel && T.tg==sel_tg;
            if(modview::row_col0(r, selrow, up)){              // col0 = Up time
                sel_tg=T.tg; has_sel=true; atb=true;
                for(auto it=sess.rbegin(); it!=sess.rend(); ++it) if(it->tg==sel_tg){ focus=*it; has_focus=true; break; }
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); { char dn[12]; hms(T.down,dn); modview::cell(dn, ImVec4(0.62f,0.62f,0.62f,1.f)); }
            ImGui::TableSetColumnIndex(2); { snprintf(b,sizeof(b),"%s%u", T.ctype==1?"->":"", T.tg);
                modview::cell(b, T.ctype==1?ImVec4(0.85f,0.75f,0.95f,1.f):ImVec4(0.5f,0.8f,1.f,1.f)); }
            ImGui::TableSetColumnIndex(3); snprintf(b,sizeof(b),"%d",T.calls); modview::cell(b);
            ImGui::TableSetColumnIndex(4); if(T.last_src){ snprintf(b,sizeof(b),"%u",T.last_src); modview::cell(b); }
            ImGui::TableSetColumnIndex(5); if(T.cc>=0){ snprintf(b,sizeof(b),"%d",T.cc); modview::cell(b); }
            ImGui::TableSetColumnIndex(6); if(T.slot>0){ snprintf(b,sizeof(b),"%d",T.slot); modview::cell(b); }
            ImGui::TableSetColumnIndex(7);
            if(T.enc)        modview::cell("ENC", ImVec4(0.95f,0.55f,0.30f,1.f));
            else if(T.voice) modview::cell("YES", ImVec4(0.40f,0.90f,0.50f,1.f));
        }
        ImGui::EndTable();
    }

    // ── 스플리터 ──
    ImGui::SameLine(0,0);
    ImGui::InvisibleButton("##dmr_split", ImVec2(6, upper_h));
    if(ImGui::IsItemActive()){ split += io.MouseDelta.x/(W>1?W:1); if(split<0.25f)split=0.25f; if(split>0.75f)split=0.75f; }
    if(ImGui::IsItemHovered()||ImGui::IsItemActive()){
        ImVec2 a=ImGui::GetItemRectMin(), b=ImGui::GetItemRectMax();
        ImGui::GetWindowDrawList()->AddRectFilled(ImVec2((a.x+b.x)*0.5f-1,a.y),ImVec2((a.x+b.x)*0.5f+1,b.y),IM_COL32(120,140,160,200));
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    // ── 우측: 선택 TG 의 통화 세션 (시간순 라이브, tail-follow) ──
    ImGui::SameLine(0,0);
    if(ImGui::BeginTable("##dmr_sess", 7, tf, ImVec2(rightw, upper_h))){
        ImGui::TableSetupScrollFreeze(1,1);
        ImGui::TableSetupColumn("Up",    ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Down",  ImGuiTableColumnFlags_WidthFixed, 70);
        ImGui::TableSetupColumn("Dur",   ImGuiTableColumnFlags_WidthFixed, 56);
        ImGui::TableSetupColumn("Src",   ImGuiTableColumnFlags_WidthFixed, 90);
        ImGui::TableSetupColumn("Slot",  ImGuiTableColumnFlags_WidthFixed, 40);
        ImGui::TableSetupColumn("Voice", ImGuiTableColumnFlags_WidthFixed, 48);
        ImGui::TableSetupColumn("Type",  ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        static std::vector<int> rvis; rvis.clear();
        if(has_sel) for(int i=0;i<(int)sess.size();i++) if(sess[i].tg==sel_tg) rvis.push_back(i);

        ImGuiListClipper rc; rc.Begin((int)rvis.size());
        while(rc.Step()) for(int r=rc.DisplayStart;r<rc.DisplayEnd;r++){
            const Sess& S=sess[rvis[r]];
            ImGui::TableNextRow();
            char up[12]; hms(S.up,up);
            std::string k=sess_key(S);
            if(modview::row_col0(rvis[r], sel.count(k)>0, up)){
                modview::apply_click(sel, anchor, k, r, [&](int i){return sess_key(sess[rvis[i]]);}, (int)rvis.size(), io.KeyCtrl, io.KeyShift);
                focus=S; has_focus=!sel.empty();
            }
            char b[24];
            ImGui::TableSetColumnIndex(1); { char dn[12]; hms(S.down,dn); modview::cell(dn, ImVec4(0.62f,0.62f,0.62f,1.f)); }
            ImGui::TableSetColumnIndex(2); { snprintf(b,sizeof(b),"%.1fs",(S.down-S.up)/1000.0); modview::cell(b); }
            ImGui::TableSetColumnIndex(3); if(S.src){ snprintf(b,sizeof(b),"%u",S.src); modview::cell(b); }
            ImGui::TableSetColumnIndex(4); if(S.slot>0){ snprintf(b,sizeof(b),"%d",S.slot); modview::cell(b); }
            ImGui::TableSetColumnIndex(5);
            if(S.enc)        modview::cell("ENC", ImVec4(0.95f,0.55f,0.30f,1.f));
            else if(S.voice) modview::cell("YES", ImVec4(0.40f,0.90f,0.50f,1.f));
            ImGui::TableSetColumnIndex(6); { char ty[24]; sess_type_str(S,ty,sizeof(ty)); if(ty[0]) ImGui::TextUnformatted(ty); }
        }
        bool grew = rvis.size()>lastn; lastn=rvis.size();
        modview::tail_follow(atb, grew);
        ImGui::EndTable();
    }

    // ── 세부: 포커스 통화 세션 ──
    if(has_focus){
        modview::detail_begin("dmr", x0, W, detail_h);
        ImVec4 V(0.85f,0.85f,0.9f,1.f);
        char up[12]; hms(focus.up,up); char dn[12]; hms(focus.down,dn);
        ImGui::Text("Up:"); ImGui::SameLine(); ImGui::TextColored(V,"%s",up);
        modview::kv("Down:", dn, V);
        { char b[16]; snprintf(b,sizeof(b),"%.1fs",(focus.down-focus.up)/1000.0); modview::kv("Dur:", b, V); }
        { char b[16]; snprintf(b,sizeof(b),"%d",focus.bursts); modview::kv("Bursts:", b, V); }
        { char b[16]; snprintf(b,sizeof(b),"%s%u", focus.ctype==1?"->":"", focus.tg); modview::kv("TG:", b, ImVec4(0.5f,0.8f,1.f,1.f)); }
        { const char* ct=dmr_calltype_name(focus.ctype); if(ct[0]) modview::kv("Call:", ct, V); }
        ImGui::Separator();
        { char b[16]; snprintf(b,sizeof(b),"%u",focus.src); modview::kv("Src:", b, ImVec4(0.5f,0.8f,1.f,1.f), false); }
        if(focus.cc>=0){ char b[16]; snprintf(b,sizeof(b),"%d",focus.cc); modview::kv("CC:", b, V); }
        if(focus.slot>0){ char b[16]; snprintf(b,sizeof(b),"%d",focus.slot); modview::kv("Slot:", b, V); }
        { char ty[24]; sess_type_str(focus,ty,sizeof(ty)); modview::kv("Type:", ty, V); }
        modview::kv("Voice:", focus.voice?"YES":"-", focus.voice?ImVec4(0.40f,0.90f,0.50f,1.f):ImVec4(0.6f,0.6f,0.6f,1.f));
        modview::kv("Enc:", focus.enc?"YES":"-", focus.enc?ImVec4(0.95f,0.55f,0.30f,1.f):ImVec4(0.6f,0.6f,0.6f,1.f));
        if(focus.flco>=0) modview::kv("FLCO:", dmr_flco_name(focus.flco), V);
        modview::detail_end();
    }
}

} // namespace dmr_mod
