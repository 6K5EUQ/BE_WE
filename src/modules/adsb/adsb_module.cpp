// ── ADS-B 모듈 본체: 등록, 워커 관리, 일 단위 아카이브, framework 연동 ────────
// ais_module.cpp 구조 미러 (위치형). 제어/전송은 전부 framework(module_api) 경유.
#include "adsb_module.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <sys/stat.h>

namespace adsb_mod {

std::mutex              mtx;
std::vector<AdsbRecord> log;
char                    filter[64] = {};

// ── 워커 슬롯 ──────────────────────────────────────────────────────────────
struct ChWork {
    std::atomic<bool>   on{false};
    std::atomic<bool>   stop{false};
    std::atomic<size_t> rp{0};
    std::thread         thr;
};
static ChWork g_w[MAX_CHANNELS];
static std::mutex g_mgmt;

std::atomic<size_t>& worker_rp(int ch){ return g_w[ch].rp; }
bool worker_stop_req(int ch){ return g_w[ch].stop.load(std::memory_order_relaxed); }
void worker_natural_exit(FFTViewer& v, int ch){
    g_w[ch].on.store(false);
    bewe_mod_host_mask_clear(v, "adsb", ch);
}

static bool host_start(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return false;
    std::lock_guard<std::mutex> lk(g_mgmt);
    ChWork& w = g_w[ch];
    if(!w.on.load() && w.thr.joinable()) w.thr.join();
    if(w.on.load()) return true;
    if(!v.channels[ch].filter_active) return false;   // 복조 모드 무관 — 워커가 IQ ring 직접 탭
    w.stop.store(false);
    w.on.store(true);
    w.thr = std::thread(worker, std::ref(v), ch);
    return true;
}
static void host_stop(FFTViewer& v, int ch){
    (void)v;
    if(ch<0 || ch>=MAX_CHANNELS) return;
    std::lock_guard<std::mutex> lk(g_mgmt);
    ChWork& w = g_w[ch];
    if(w.on.load()){
        w.stop.store(true);
        if(w.thr.joinable()) w.thr.join();
        w.on.store(false);
    } else if(w.thr.joinable()) w.thr.join();
}
static void on_ch_stop(FFTViewer& v, int ch){
    if((bewe_mod_host_mask("adsb")>>ch)&1){
        host_stop(v, ch);
        bewe_mod_host_mask_clear(v, "adsb", ch);
    }
}

// ── 표시 로그 append (dedup: 히스토리/라이브 경계 중복 도달 대비) ───────────
void append_log(const AdsbRecord& m){
    std::lock_guard<std::mutex> lk(mtx);
    int n=(int)log.size();
    for(int i=n-1;i>=0 && i>=n-64;i--)
        if(log[i].t_ms==m.t_ms && log[i].icao==m.icao && log[i].df==m.df && log[i].tc==m.tc) return;
    if(n>=LOG_MAX) log.erase(log.begin());
    log.push_back(m);
}

// station_id ("DGS-2_DGS-2") → 표시명 ("DGS-2")
static void station_disp(const char* sid, char* out, size_t cap){
    size_t o=0;
    for(const char* p=sid; *p && *p!='_' && o+1<cap; ++p) out[o++]=*p;
    if(o==0 && cap>5){ strncpy(out,"LOCAL",cap-1); out[cap-1]=0; return; }
    out[o]=0;
}

// ── 일 단위 JSONL 아카이브 ────────────────────────────────────────────────
static std::string store_dir(){ return BEWEPaths::data_dir() + "/modules/adsb"; }
static std::string store_path(int64_t t_ms){
    struct tm tmv; KST::to_tm((time_t)(t_ms/1000), tmv);
    char d[16]; strftime(d,sizeof(d),"%Y%m%d",&tmv);
    return store_dir() + "/adsb_" + d + ".jsonl";
}
static void json_escape(const char* s, char* out, size_t cap){
    size_t o=0;
    for(const char* p=s; *p && o+6<cap; ++p){
        unsigned char c=(unsigned char)*p;
        if(c=='"'||c=='\\'){ out[o++]='\\'; out[o++]=(char)c; }
        else if(c<0x20){ o+=snprintf(out+o,cap-o,"\\u%04x",c); }
        else out[o++]=(char)c;
    }
    out[o]=0;
}
void store_append(const AdsbRecord& m){
    mkdir((BEWEPaths::data_dir()+"/modules").c_str(),0755);
    mkdir(store_dir().c_str(),0755);
    FILE* f=fopen(store_path(m.t_ms).c_str(),"ab"); if(!f) return;
    char cs[24]; json_escape(m.callsign,cs,sizeof(cs));
    fprintf(f,"{\"t\":%lld,\"ch\":%d,\"f\":%.4f,\"crc\":%d,\"df\":%d,\"icao\":%u,\"tc\":%d,"
              "\"cs\":\"%s\",\"cat\":%d,\"ha\":%d,\"alt\":%d,\"hp\":%d,\"lat\":%.6f,\"lon\":%.6f,"
              "\"hv\":%d,\"spd\":%.1f,\"trk\":%.1f,\"hvr\":%d,\"vr\":%d}\n",
        (long long)m.t_ms,m.ch,m.freq,m.crc_ok?1:0,m.df,m.icao,m.tc,
        cs,m.category,m.has_alt?1:0,m.altitude,m.has_pos?1:0,m.lat,m.lon,
        m.has_vel?1:0,m.speed,m.track,m.has_vr?1:0,m.vert_rate);
    fclose(f);
}
bool store_read_today(std::string& out){
    int64_t now=(int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    FILE* f=fopen(store_path(now).c_str(),"rb"); if(!f) return false;
    char buf[8192]; size_t n;
    while((n=fread(buf,1,sizeof(buf),f))>0) out.append(buf,n);
    fclose(f); return true;
}
static bool jstr(const char* l,const char* k,char* o,size_t cap){
    const char* p=strstr(l,k); if(!p) return false; p+=strlen(k);
    size_t i=0; while(*p&&*p!='"'&&i+1<cap){ if(*p=='\\'){ ++p; if(*p) o[i++]=*p++; continue; } o[i++]=*p++; } o[i]=0; return true;
}
static long long jll(const char* l,const char* k){ const char* p=strstr(l,k); return p?atoll(p+strlen(k)):0; }
static double    jf (const char* l,const char* k){ const char* p=strstr(l,k); return p?atof (p+strlen(k)):0.0; }
void store_parse_jsonl(const char* data, size_t n, std::vector<AdsbRecord>& out){
    size_t i=0; std::string line;
    while(i<n){
        size_t e=i; while(e<n && data[e]!='\n') e++;
        line.assign(data+i,e-i); i=e+1;
        if(line.size()<8) continue;
        const char* l=line.c_str();
        AdsbRecord m{};
        m.t_ms=jll(l,"\"t\":"); m.ch=(int)jll(l,"\"ch\":"); m.freq=(float)jf(l,"\"f\":");
        m.crc_ok=jll(l,"\"crc\":")!=0; m.df=(int)jll(l,"\"df\":"); m.icao=(uint32_t)jll(l,"\"icao\":");
        m.tc=(int)jll(l,"\"tc\":"); jstr(l,"\"cs\":\"",m.callsign,sizeof(m.callsign));
        m.category=(int)jll(l,"\"cat\":");
        m.has_alt=jll(l,"\"ha\":")!=0; m.altitude=(int)jll(l,"\"alt\":");
        m.has_pos=jll(l,"\"hp\":")!=0; m.lat=jf(l,"\"lat\":"); m.lon=jf(l,"\"lon\":");
        m.has_vel=jll(l,"\"hv\":")!=0; m.speed=(float)jf(l,"\"spd\":"); m.track=(float)jf(l,"\"trk\":");
        m.has_vr=jll(l,"\"hvr\":")!=0; m.vert_rate=(int)jll(l,"\"vr\":");
        out.push_back(m);
    }
}

// ── HOST: 워커 → 스탬프 + 아카이브 + framework emit ────────────────────────
void host_emit(FFTViewer& v, AdsbRecord m){
    if(m.ch>=0 && m.ch<MAX_CHANNELS && v.channels[m.ch].filter_active)
        m.freq = (v.channels[m.ch].s + v.channels[m.ch].e)/2.0f;
    m.t_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    store_append(m);
    AdsbWireMsg w; adsb_msg_to_wire(m, w);
    bewe_mod_emit(v, "adsb", &w, sizeof(w));
}

static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v;
    if(n < sizeof(AdsbWireMsg)) return;
    AdsbRecord m; adsb_wire_to_msg(*reinterpret_cast<const AdsbWireMsg*>(d), m);
    bewe_mod_stat_bump("adsb", station, m.ch, m.t_ms);
    station_disp(station, m.station, sizeof(m.station));
    append_log(m);
}

// ── 과거조회(Hist): 라이브 log 대피/복원 + 과거 JSONL 오버레이 ──────────────
static std::vector<AdsbRecord> g_stash;   // Hist 진입 시 라이브 log 대피 버퍼
static void log_stash(){
    std::lock_guard<std::mutex> lk(mtx);
    g_stash = std::move(log); log.clear();   // move: 이전 잔여 버퍼 폐기
}
static void log_restore(){
    std::lock_guard<std::mutex> lk(mtx);
    log = std::move(g_stash); g_stash.clear();
}
// 과거 날짜 아카이브 기지별 JSONL 1개 → 파싱·기지명 태그·시간순 병합 (기지 수만큼 호출)
static void on_hist_file(const char* station, const char* data, size_t n){
    std::vector<AdsbRecord> parsed;
    store_parse_jsonl(data, n, parsed);
    for(auto& m : parsed){ strncpy(m.station, station, sizeof(m.station)-1); m.station[sizeof(m.station)-1]=0; }
    std::lock_guard<std::mutex> lk(mtx);
    log.insert(log.end(), parsed.begin(), parsed.end());
    std::stable_sort(log.begin(), log.end(),
                     [](const AdsbRecord& a, const AdsbRecord& b){ return a.t_ms < b.t_ms; });
    if((int)log.size() > LOG_MAX) log.erase(log.begin(), log.end()-LOG_MAX);
}

#ifndef BEWE_HEADLESS
void local_load_today(FFTViewer& v){
    (void)v;
    std::string body;
    if(!store_read_today(body)) return;
    std::vector<AdsbRecord> parsed;
    store_parse_jsonl(body.data(), body.size(), parsed);
    for(auto& m : parsed) strncpy(m.station, "LOCAL", sizeof(m.station)-1);
    std::lock_guard<std::mutex> lk(mtx);
    if(!parsed.empty()) log = std::move(parsed);
    if((int)log.size() > LOG_MAX) log.erase(log.begin(), log.end()-LOG_MAX);
}
#endif

// ── 모듈 등록 (static-init) ────────────────────────────────────────────────
static bool s_registered = [](){
    BeweModule m{};
    m.id    = "adsb";
    m.label = "ADS-B";
    m.planned = false;
    m.target_modes = (uint8_t)(1u << Channel::DM_AM);   // OOK/AM 계열 → AM 채널필터에 노출
#ifndef BEWE_HEADLESS
    m.draw_content = &draw_content;
#endif
    m.host_start = &host_start;
    m.host_stop  = &host_stop;
    m.on_ch_stop = &on_ch_stop;
    m.on_data    = &on_data;
    m.log_stash    = &log_stash;
    m.log_restore  = &log_restore;
    m.on_hist_file = &on_hist_file;
    bewe_register_module(m);
    return true;
}();

} // namespace adsb_mod
