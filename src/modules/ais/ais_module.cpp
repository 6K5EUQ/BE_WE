// ── AIS 모듈 본체: 등록, 워커 관리, 일 단위 아카이브, framework 연동 ─────────
// FM(GMSK) 채널필터에 활성화 → IQ ring 독립 탭 → 자체 FM 판별 복조 → HDLC 디코드.
// 제어/전송은 전부 framework(module_api) 경유. 코어 변경 없음 (ACARS 와 동형 격리).
#include "ais_module.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <algorithm>
#include <vector>
#include <sys/stat.h>

namespace ais_mod {

std::mutex             mtx;
std::vector<AisRecord> log;
char                   filter[64] = {};

// ── 워커 슬롯 ──────────────────────────────────────────────────────────────
struct ChWork {
    std::atomic<bool>   on{false};
    std::atomic<bool>   stop{false};
    std::atomic<size_t> rp{0};
    std::thread         thr;
};
static ChWork g_w[MAX_CHANNELS];
static std::mutex g_mgmt;            // start/stop 직렬화 (ACARS 와 동일 race 대비)

std::atomic<size_t>& worker_rp(int ch){ return g_w[ch].rp; }
bool worker_stop_req(int ch){ return g_w[ch].stop.load(std::memory_order_relaxed); }
void worker_natural_exit(FFTViewer& v, int ch){
    g_w[ch].on.store(false);
    bewe_mod_host_mask_clear(v, "ais", ch);
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
    if((bewe_mod_host_mask("ais")>>ch)&1){
        host_stop(v, ch);
        bewe_mod_host_mask_clear(v, "ais", ch);
    }
}

// ── 표시 로그 append (dedup: 히스토리/라이브 경계 중복 도달 대비) ───────────
void append_log(const AisRecord& m){
    std::lock_guard<std::mutex> lk(mtx);
    int n=(int)log.size();
    for(int i=n-1;i>=0 && i>=n-64;i--)
        if(log[i].t_ms==m.t_ms && log[i].mmsi==m.mmsi && log[i].msg_type==m.msg_type) return;
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
static std::string store_dir(){ return BEWEPaths::data_dir() + "/modules/ais"; }
static std::string store_path(int64_t t_ms){
    struct tm tmv; KST::to_tm((time_t)(t_ms/1000), tmv);
    char d[16]; strftime(d,sizeof(d),"%Y%m%d",&tmv);
    return store_dir() + "/ais_" + d + ".jsonl";
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
void store_append(const AisRecord& m){
    mkdir((BEWEPaths::data_dir()+"/modules").c_str(),0755);
    mkdir(store_dir().c_str(),0755);
    FILE* f=fopen(store_path(m.t_ms).c_str(),"ab"); if(!f) return;
    char nm[64], cs[24], dt[64]; json_escape(m.name,nm,sizeof(nm)); json_escape(m.callsign,cs,sizeof(cs)); json_escape(m.dest,dt,sizeof(dt));
    fprintf(f,"{\"t\":%lld,\"ch\":%d,\"f\":%.4f,\"crc\":%d,\"ty\":%d,\"mmsi\":%u,"
              "\"hp\":%d,\"lat\":%.6f,\"lon\":%.6f,\"sog\":%.1f,\"cog\":%.1f,"
              "\"hdg\":%d,\"ns\":%d,\"nm\":\"%s\",\"cs\":\"%s\",\"st\":%d,"
              "\"imo\":%u,\"dst\":\"%s\",\"dr\":%.1f,\"em\":%d,\"ed\":%d,\"eh\":%d,\"ei\":%d}\n",
        (long long)m.t_ms,m.ch,m.freq,m.crc_ok?1:0,m.msg_type,m.mmsi,
        m.has_pos?1:0,m.lat,m.lon,m.sog,m.cog,m.heading,m.nav_status,nm,cs,m.ship_type,
        m.imo,dt,m.draught,m.eta_mon,m.eta_day,m.eta_hour,m.eta_min);
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
void store_parse_jsonl(const char* data, size_t n, std::vector<AisRecord>& out){
    size_t i=0; std::string line;
    while(i<n){
        size_t e=i; while(e<n && data[e]!='\n') e++;
        line.assign(data+i,e-i); i=e+1;
        if(line.size()<8) continue;
        const char* l=line.c_str();
        AisRecord m{};
        m.t_ms=jll(l,"\"t\":"); m.ch=(int)jll(l,"\"ch\":"); m.freq=(float)jf(l,"\"f\":");
        m.crc_ok=jll(l,"\"crc\":")!=0; m.msg_type=(int)jll(l,"\"ty\":"); m.mmsi=(uint32_t)jll(l,"\"mmsi\":");
        m.has_pos=jll(l,"\"hp\":")!=0; m.lat=jf(l,"\"lat\":"); m.lon=jf(l,"\"lon\":");
        m.sog=(float)jf(l,"\"sog\":"); m.cog=(float)jf(l,"\"cog\":");
        m.heading=(int)jll(l,"\"hdg\":"); m.nav_status=(int)jll(l,"\"ns\":");
        jstr(l,"\"nm\":\"",m.name,sizeof(m.name)); jstr(l,"\"cs\":\"",m.callsign,sizeof(m.callsign));
        m.ship_type=(int)jll(l,"\"st\":");
        m.imo=(uint32_t)jll(l,"\"imo\":"); jstr(l,"\"dst\":\"",m.dest,sizeof(m.dest));
        if(strstr(l,"\"dr\":")) m.draught=(float)jf(l,"\"dr\":");    // 없으면 기본 -1(n/a) 유지
        m.eta_mon=(uint8_t)jll(l,"\"em\":"); m.eta_day=(uint8_t)jll(l,"\"ed\":");
        m.eta_hour=(uint8_t)jll(l,"\"eh\":"); m.eta_min=(uint8_t)jll(l,"\"ei\":");
        out.push_back(m);
    }
}

// ── HOST: 워커 → 스탬프 + 아카이브 + framework emit ────────────────────────
void host_emit(FFTViewer& v, AisRecord m){
    if(m.ch>=0 && m.ch<MAX_CHANNELS && v.channels[m.ch].filter_active)
        m.freq = (v.channels[m.ch].s + v.channels[m.ch].e)/2.0f;
    m.t_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    store_append(m);
    AisWireMsg w; ais_msg_to_wire(m, w);
    bewe_mod_emit(v, "ais", &w, sizeof(w));
}

static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v;
    if(n < sizeof(AisWireMsg)) return;
    AisRecord m; ais_wire_to_msg(*reinterpret_cast<const AisWireMsg*>(d), m);
    bewe_mod_stat_bump("ais", station, m.ch, m.t_ms);
    station_disp(station, m.station, sizeof(m.station));
    append_log(m);
}

// JOIN: 단일 MMSI 온디맨드 전체 이력 도착 → 그 MMSI 기존 log 레코드(구독 요약분)를
// 이 전체 세트로 교체. 스트림 = u32 len + (MpData + AisWireMsg) 반복 (시간순).
static void on_vessel_hist(FFTViewer& v, const uint8_t* d, size_t n){
    (void)v;
    std::vector<AisRecord> recs; uint32_t key = 0;
    size_t off = 0;
    while(off + 4 <= n){
        uint32_t rl; memcpy(&rl, d+off, 4); off += 4;
        if(off + rl > n) break;
        const uint8_t* rec = d + off; off += rl;
        if(rl < sizeof(MpData) + sizeof(AisWireMsg)){ continue; }
        AisWireMsg w; memcpy(&w, rec + sizeof(MpData), sizeof(AisWireMsg));
        AisRecord m; ais_wire_to_msg(w, m);
        char stn[25] = {}; memcpy(stn, rec, 24);       // MpData.station (raw station_id)
        station_disp(stn, m.station, sizeof(m.station));
        key = m.mmsi;
        recs.push_back(m);
    }
    if(recs.empty()) return;
    std::lock_guard<std::mutex> lk(mtx);
    log.erase(std::remove_if(log.begin(), log.end(),
        [key](const AisRecord& r){ return r.mmsi == key; }), log.end());
    for(const auto& m : recs) log.push_back(m);
    if((int)log.size() > LOG_MAX) log.erase(log.begin(), log.end()-LOG_MAX);
}

#ifndef BEWE_HEADLESS
void local_load_today(FFTViewer& v){
    (void)v;
    std::string body;
    if(!store_read_today(body)) return;
    std::vector<AisRecord> parsed;
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
    m.id    = "ais";
    m.label = "AIS";
    m.planned = false;
    m.target_modes = (uint8_t)(1u << Channel::DM_FM);   // GMSK → FM 채널필터에 활성화
#ifndef BEWE_HEADLESS
    m.draw_content = &draw_content;
#endif
    m.host_start = &host_start;
    m.host_stop  = &host_stop;
    m.on_ch_stop = &on_ch_stop;
    m.on_data    = &on_data;
    m.on_vessel_hist = &on_vessel_hist;
    bewe_register_module(m);
    return true;
}();

} // namespace ais_mod
