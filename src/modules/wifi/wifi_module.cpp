// ── WiFi 모듈 본체: 등록, 워커 관리, 일 단위 아카이브, framework 연동 ────────
// 복조 없는(DM_NONE) 채널필터에 활성화 → DDC 로 채널 sub-band 추출 → 디코드.
// 제어/전송은 전부 framework(module_api) 경유. 코어 변경은 mod_wants_ring 하나뿐.
#include "wifi_module.hpp"
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
#include <sys/stat.h>

namespace wifi_mod {

std::mutex              mtx;
std::vector<WifiRecord> log;
char                    filter[64] = {};

// ── 워커 슬롯 ──────────────────────────────────────────────────────────────
struct ChWork {
    std::atomic<bool>   on{false};
    std::atomic<bool>   stop{false};
    std::atomic<size_t> rp{0};
    std::thread         thr;
};
static ChWork g_w[MAX_CHANNELS];
static std::mutex g_mgmt;           // start/stop 직렬화 (ACARS 와 동일 race 대비)
static int        g_active = 0;     // 활성 워커 수 (mod_wants_ring refcount, g_mgmt 보호)

std::atomic<size_t>& worker_rp(int ch){ return g_w[ch].rp; }
bool worker_stop_req(int ch){ return g_w[ch].stop.load(std::memory_order_relaxed); }
void worker_natural_exit(FFTViewer& v, int ch){
    g_w[ch].on.store(false);
    { std::lock_guard<std::mutex> lk(g_mgmt); if(--g_active<=0){ g_active=0; v.mod_wants_ring.store(false); } }
    bewe_mod_host_mask_clear(v, "wifi", ch);
}

static bool host_start(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return false;
    std::lock_guard<std::mutex> lk(g_mgmt);
    ChWork& w = g_w[ch];
    if(!w.on.load() && w.thr.joinable()) w.thr.join();
    if(w.on.load()) return true;
    if(!v.channels[ch].filter_active) return false;      // 복조 모드 무관 — 필터만 있으면 OK
    w.stop.store(false);
    w.on.store(true);
    if(++g_active==1) v.mod_wants_ring.store(true);       // 첫 워커 → ring 공급 ON
    w.thr = std::thread(worker, std::ref(v), ch);
    return true;
}

static void host_stop(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    std::lock_guard<std::mutex> lk(g_mgmt);
    ChWork& w = g_w[ch];
    if(w.on.load()){
        w.stop.store(true);
        if(w.thr.joinable()) w.thr.join();
        w.on.store(false);
        if(--g_active<=0){ g_active=0; v.mod_wants_ring.store(false); }   // 마지막 워커 → ring 공급 OFF
    } else if(w.thr.joinable()) w.thr.join();
}

static void on_ch_stop(FFTViewer& v, int ch){
    if((bewe_mod_host_mask("wifi")>>ch)&1){
        host_stop(v, ch);
        bewe_mod_host_mask_clear(v, "wifi", ch);
    }
}

// ── 표시 로그 append (진단 레코드는 1/s, dedup 불필요) ──────────────────────
void append_log(const WifiRecord& m){
    std::lock_guard<std::mutex> lk(mtx);
    if((int)log.size() >= LOG_MAX) log.erase(log.begin());
    log.push_back(m);
}

// ── 일 단위 JSONL 아카이브 ────────────────────────────────────────────────
static std::string store_dir(){ return BEWEPaths::data_dir() + "/modules/wifi"; }
static std::string store_path(int64_t t_ms){
    struct tm tmv; KST::to_tm((time_t)(t_ms/1000), tmv);
    char d[16]; strftime(d,sizeof(d),"%Y%m%d",&tmv);
    return store_dir() + "/wifi_" + d + ".jsonl";
}
void store_append(const WifiRecord& m){
    mkdir((BEWEPaths::data_dir()+"/modules").c_str(),0755);
    mkdir(store_dir().c_str(),0755);
    FILE* f=fopen(store_path(m.t_ms).c_str(),"ab"); if(!f) return;
    fprintf(f,"{\"t\":%lld,\"ch\":%d,\"f\":%.4f,\"bssid\":\"%s\",\"ssid\":\"%s\","
              "\"wch\":%d,\"rssi\":%d,\"phy\":\"%s\",\"sec\":\"%s\",\"bi\":%d,"
              "\"peak\":%.1f,\"br\":%d,\"osr\":%u}\n",
        (long long)m.t_ms,m.ch,m.freq,m.bssid,m.ssid,m.wch,m.rssi,m.phy,m.sec,
        m.beacon_ms,m.peak_dbfs,m.burst_count,m.out_sr);
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
    size_t i=0; while(*p&&*p!='"'&&i+1<cap) o[i++]=*p++; o[i]=0; return true;
}
static long long jll(const char* l,const char* k){ const char* p=strstr(l,k); return p?atoll(p+strlen(k)):0; }
static double    jf (const char* l,const char* k){ const char* p=strstr(l,k); return p?atof (p+strlen(k)):0.0; }
void store_parse_jsonl(const char* data, size_t n, std::vector<WifiRecord>& out){
    size_t i=0; std::string line;
    while(i<n){
        size_t e=i; while(e<n && data[e]!='\n') e++;
        line.assign(data+i,e-i); i=e+1;
        if(line.size()<8) continue;
        const char* l=line.c_str();
        WifiRecord m{};
        m.t_ms=jll(l,"\"t\":"); m.ch=(int)jll(l,"\"ch\":"); m.freq=(float)jf(l,"\"f\":");
        jstr(l,"\"bssid\":\"",m.bssid,sizeof(m.bssid));
        jstr(l,"\"ssid\":\"",m.ssid,sizeof(m.ssid));
        m.wch=(int)jll(l,"\"wch\":"); m.rssi=(int)jll(l,"\"rssi\":");
        jstr(l,"\"phy\":\"",m.phy,sizeof(m.phy));
        jstr(l,"\"sec\":\"",m.sec,sizeof(m.sec));
        m.beacon_ms=(int)jll(l,"\"bi\":");
        m.peak_dbfs=(float)jf(l,"\"peak\":"); m.burst_count=(int)jll(l,"\"br\":");
        m.out_sr=(uint32_t)jll(l,"\"osr\":");
        out.push_back(m);
    }
}

// ── HOST: 워커 → 스탬프 + 아카이브 + framework emit ────────────────────────
void host_emit(FFTViewer& v, WifiRecord m){
    if(m.ch>=0 && m.ch<MAX_CHANNELS && v.channels[m.ch].filter_active)
        m.freq = (v.channels[m.ch].s + v.channels[m.ch].e)/2.0f;
    m.t_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    store_append(m);
    WifiWireMsg w; wifi_msg_to_wire(m, w);
    bewe_mod_emit(v, "wifi", &w, sizeof(w));
}

static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v; (void)station;
    if(n < sizeof(WifiWireMsg)) return;
    WifiRecord m; wifi_wire_to_msg(*reinterpret_cast<const WifiWireMsg*>(d), m);
    bewe_mod_stat_bump("wifi", station, m.ch, m.t_ms);
    append_log(m);
}

// ── 과거조회(Hist): 라이브 log 대피/복원 + 과거 JSONL 오버레이 ──────────────
static std::vector<WifiRecord> g_stash;   // Hist 진입 시 라이브 log 대피 버퍼
static void log_stash(){
    std::lock_guard<std::mutex> lk(mtx);
    g_stash = std::move(log); log.clear();   // move: 이전 잔여 버퍼 폐기
}
static void log_restore(){
    std::lock_guard<std::mutex> lk(mtx);
    log = std::move(g_stash); g_stash.clear();
}
// 과거 날짜 아카이브 기지별 JSONL 1개 → 파싱·시간순 병합 (기지 수만큼 호출)
static void on_hist_file(const char* station, const char* data, size_t n){
    (void)station;   // WifiRecord 에 station 필드 없음
    std::vector<WifiRecord> parsed;
    store_parse_jsonl(data, n, parsed);
    std::lock_guard<std::mutex> lk(mtx);
    log.insert(log.end(), parsed.begin(), parsed.end());
    std::stable_sort(log.begin(), log.end(),
                     [](const WifiRecord& a, const WifiRecord& b){ return a.t_ms < b.t_ms; });
    if((int)log.size() > LOG_MAX) log.erase(log.begin(), log.end()-LOG_MAX);
}

#ifndef BEWE_HEADLESS
void local_load_today(FFTViewer& v){
    (void)v;
    std::string body;
    if(!store_read_today(body)) return;
    std::vector<WifiRecord> parsed;
    store_parse_jsonl(body.data(), body.size(), parsed);
    std::lock_guard<std::mutex> lk(mtx);
    if(!parsed.empty()) log = std::move(parsed);
    if((int)log.size() > LOG_MAX) log.erase(log.begin(), log.end()-LOG_MAX);
}
#endif

// ── 모듈 등록 (static-init) ────────────────────────────────────────────────
static bool s_registered = [](){
    BeweModule m{};
    m.id    = "wifi";
    m.label = "WiFi";
    m.planned = false;
    m.target_modes = (uint8_t)(1u << Channel::DM_NONE);   // 복조 없는 채널필터에 활성화
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

} // namespace wifi_mod
