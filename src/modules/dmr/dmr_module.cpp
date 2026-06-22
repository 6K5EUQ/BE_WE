// ── DMR 모듈 본체: 등록, 워커 관리, 일 단위 아카이브, framework 연동 ─────────
// acars_module.cpp 구조 미러. 제어/전송은 전부 framework(module_api) 경유.
#include "dmr_module.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>

namespace dmr_mod {

std::mutex             mtx;
std::vector<DmrRecord> log;
char                   filter[64] = {};

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
    bewe_mod_host_mask_clear(v, "dmr", ch);
}

static bool host_start(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return false;
    std::lock_guard<std::mutex> lk(g_mgmt);
    ChWork& w = g_w[ch];
    if(!w.on.load() && w.thr.joinable()) w.thr.join();
    if(w.on.load()) return true;
    if(!v.channels[ch].filter_active) return false;
    // decode 는 IQ 직접 탭 + 자체 4FSK 판별 → 채널 audio 모드(NONE/AM/FM) 무관하게 동작
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
    if((bewe_mod_host_mask("dmr")>>ch)&1){
        host_stop(v, ch);
        bewe_mod_host_mask_clear(v, "dmr", ch);
    }
}

// ── 로그 (dedup) ────────────────────────────────────────────────────────────
void append_log(const DmrRecord& m){
    std::lock_guard<std::mutex> lk(mtx);
    int n = (int)log.size();
    for(int i=n-1; i>=0 && i>=n-64; i--){
        if(log[i].t_ms==m.t_ms && log[i].src_id==m.src_id && log[i].dst_id==m.dst_id &&
           log[i].csbko==m.csbko && log[i].flco==m.flco) return;
    }
    if(n >= LOG_MAX) log.erase(log.begin());
    log.push_back(m);
}

// station_id ("DGS-2_DGS-2") → 표시명 ("DGS-2")
static void station_disp(const char* sid, char* out, size_t cap){
    size_t o=0;
    for(const char* p=sid; *p && *p!='_' && o+1<cap; ++p) out[o++]=*p;
    if(o==0 && cap>5){ strncpy(out,"LOCAL",cap); out[cap-1]=0; return; }
    out[o]=0;
}

// ── 호스트 일 단위 JSONL 아카이브 ──────────────────────────────────────────
static std::string store_dir(){
    const char* home = getenv("HOME");
    std::string base = home ? std::string(home) : std::string(".");
    return base + "/BE_WE/modules/dmr";
}
static void mkdirs(const std::string& path){
    std::string p;
    for(size_t i=0;i<path.size();i++){ p += path[i];
        if(path[i]=='/' && p.size()>1) mkdir(p.c_str(), 0755); }
    mkdir(path.c_str(), 0755);
}
static void kst_date_of(int64_t t_ms, char out[9]){
    struct tm tmv{}; KST::to_tm((time_t)(t_ms/1000), tmv);
    snprintf(out, 9, "%04d%02d%02d", tmv.tm_year+1900, tmv.tm_mon+1, tmv.tm_mday);
}
static std::string store_path_today(){
    int64_t now = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();
    char d[9]; kst_date_of(now, d);
    return store_dir() + "/dmr_" + d + ".jsonl";
}
void store_append(const DmrRecord& m){
    static std::mutex fmtx;
    std::lock_guard<std::mutex> lk(fmtx);
    mkdirs(store_dir());
    char d[9]; kst_date_of(m.t_ms, d);
    std::string path = store_dir() + "/dmr_" + d + ".jsonl";
    FILE* f = fopen(path.c_str(), "ab");
    if(!f) return;
    fprintf(f,
        "{\"t\":%lld,\"ch\":%d,\"f\":%.4f,\"crc\":%d,\"sl\":%d,\"cc\":%d,"
        "\"dt\":%d,\"flco\":%d,\"csbko\":%d,\"src\":%u,\"dst\":%u,\"ct\":%d,\"v\":%d,\"e\":%d,\"rid\":%llu}\n",
        (long long)m.t_ms, m.ch, m.freq, m.crc_ok?1:0, m.slot, m.color_code,
        m.data_type, m.flco, m.csbko, m.src_id, m.dst_id, m.call_type, m.is_voice?1:0, m.enc?1:0,
        (unsigned long long)m.rec_id);
    fclose(f);
}
bool store_read_today(std::string& out){
    out.clear();
    FILE* f = fopen(store_path_today().c_str(), "rb");
    if(!f) return false;
    char buf[8192]; size_t n;
    while((n=fread(buf,1,sizeof(buf),f))>0) out.append(buf,n);
    fclose(f); return true;
}
static bool jget_ll(const char* l, const char* key, long long& v){
    const char* p=strstr(l,key); if(!p) return false; v=atoll(p+strlen(key)); return true; }
void store_parse_jsonl(const char* data, size_t n, std::vector<DmrRecord>& out){
    size_t i=0; std::string line;
    while(i<n){
        size_t e=i; while(e<n && data[e]!='\n') e++;
        line.assign(data+i, e-i); i=e+1;
        if(line.size()<10) continue;
        const char* l=line.c_str(); DmrRecord m{}; long long t;
        if(jget_ll(l,"\"t\":",t)) m.t_ms=t;
        if(jget_ll(l,"\"ch\":",t)) m.ch=(int)t;
        { const char* p=strstr(l,"\"f\":"); if(p) m.freq=(float)atof(p+4); }
        if(jget_ll(l,"\"crc\":",t)) m.crc_ok=t!=0;
        if(jget_ll(l,"\"sl\":",t)) m.slot=(int)t;
        if(jget_ll(l,"\"cc\":",t)) m.color_code=(int)t;
        if(jget_ll(l,"\"dt\":",t)) m.data_type=(int)t;
        if(jget_ll(l,"\"flco\":",t)) m.flco=(int)t;
        if(jget_ll(l,"\"csbko\":",t)) m.csbko=(int)t;
        if(jget_ll(l,"\"src\":",t)) m.src_id=(uint32_t)t;
        if(jget_ll(l,"\"dst\":",t)) m.dst_id=(uint32_t)t;
        if(jget_ll(l,"\"ct\":",t)) m.call_type=(int)t;
        if(jget_ll(l,"\"v\":",t)) m.is_voice=t!=0;
        if(jget_ll(l,"\"e\":",t)) m.enc=t!=0;
        if(jget_ll(l,"\"rid\":",t)) m.rec_id=(uint64_t)t;
        out.push_back(m);
    }
}

// ── HOST: 워커 → 스탬프 + 콜병합 dedup + 아카이브 + framework emit ───────────
// DMR 은 동일 CSBK/콜 정보가 ~60ms 마다 반복 → key 변경 또는 heartbeat 시만 방출.
static constexpr int64_t DEDUP_REFRESH_MS = 10000;
struct LastKey { uint64_t key=~0ull; int64_t t=0; };
static LastKey g_last[MAX_CHANNELS];
void host_emit(FFTViewer& v, DmrRecord m){
    if(m.ch>=0 && m.ch<MAX_CHANNELS && v.channels[m.ch].filter_active)
        m.freq = (v.channels[m.ch].s + v.channels[m.ch].e)/2.0f;
    m.t_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    // 콜병합: 같은 {dt,cc,csbko,flco,src,dst} 반복은 10초마다 1회만
    uint64_t key = ((uint64_t)(uint32_t)m.src_id) ^ ((uint64_t)(uint32_t)m.dst_id<<20)
                 ^ ((uint64_t)(m.csbko&0x3F)<<40) ^ ((uint64_t)(m.flco&0x3F)<<46)
                 ^ ((uint64_t)(m.color_code&0xF)<<52) ^ ((uint64_t)(m.data_type&0xF)<<56);
    if(m.ch>=0 && m.ch<MAX_CHANNELS){
        LastKey& lk=g_last[m.ch];
        if(lk.key==key && (m.t_ms-lk.t)<DEDUP_REFRESH_MS) return;
        lk.key=key; lk.t=m.t_ms;
    }
    store_append(m);
    DmrWireMsg w; dmr_msg_to_wire(m, w);
    bewe_mod_emit(v, "dmr", &w, sizeof(w));
}

// ── framework 데이터 수신 (라이브 + 히스토리 공용) ──────────────────────────
static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v;
    if(n < sizeof(DmrWireMsg)) return;
    DmrRecord m; dmr_wire_to_msg(*reinterpret_cast<const DmrWireMsg*>(d), m);
    bewe_mod_stat_bump("dmr", station, m.ch, m.t_ms);   // DEMOD 패널 Rate 갱신 (ais/acars/wifi 동일)
    station_disp(station, m.station, sizeof(m.station));
    append_log(m);
}

#ifndef BEWE_HEADLESS
void local_load_today(FFTViewer& v){
    (void)v;
    std::string body;
    if(!store_read_today(body)) return;
    std::vector<DmrRecord> parsed;
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
    m.id    = "dmr";
    m.label = "DMR";
    m.target_modes = (uint8_t)(1u << Channel::DM_FM);
    m.planned = false;
#ifndef BEWE_HEADLESS
    m.draw_content = &draw_content;
#endif
    m.host_start = &host_start;
    m.host_stop  = &host_stop;
    m.on_ch_stop = &on_ch_stop;
    m.on_data    = &on_data;
    bewe_register_module(m);
    return true;
}();

} // namespace dmr_mod
