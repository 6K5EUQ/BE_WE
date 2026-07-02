// ── BLE 모듈 본체: 등록, 워커 관리, 일 단위 아카이브, framework 연동 ──────────
// adsb_module.cpp 구조 미러. 제어/전송은 전부 framework(module_api) 경유.
#include "btle_module.hpp"
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
#include <mutex>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include <vector>
#include <sys/stat.h>

namespace btle_mod {

std::mutex              mtx;
std::vector<BtleRecord> log;
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
    bewe_mod_host_mask_clear(v, "btle", ch);
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
    if((bewe_mod_host_mask("btle")>>ch)&1){
        host_stop(v, ch);
        bewe_mod_host_mask_clear(v, "btle", ch);
    }
}

// ── 표시 로그 append (dedup: 히스토리/라이브 경계 중복 대비) ──────────────────
void append_log(const BtleRecord& m){
    std::lock_guard<std::mutex> lk(mtx);
    int n=(int)log.size();
    for(int i=n-1;i>=0 && i>=n-64;i--)
        if(log[i].t_ms==m.t_ms && log[i].pdu_type==m.pdu_type
           && memcmp(log[i].mac,m.mac,6)==0) return;
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
static std::string store_dir(){ return BEWEPaths::data_dir() + "/modules/btle"; }
static std::string store_path(int64_t t_ms){
    struct tm tmv; KST::to_tm((time_t)(t_ms/1000), tmv);
    char d[16]; strftime(d,sizeof(d),"%Y%m%d",&tmv);
    return store_dir() + "/btle_" + d + ".jsonl";
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
static void mac_hex(const uint8_t* mac, char* o){ // o[13]
    snprintf(o,13,"%02x%02x%02x%02x%02x%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}
static void hex_mac(const char* h, uint8_t* mac){
    for(int i=0;i<6;i++){ unsigned b=0; sscanf(h+i*2,"%2x",&b); mac[i]=(uint8_t)b; }
}
void store_append(const BtleRecord& m){
    mkdir((BEWEPaths::data_dir()+"/modules").c_str(),0755);
    mkdir(store_dir().c_str(),0755);
    FILE* f=fopen(store_path(m.t_ms).c_str(),"ab"); if(!f) return;
    char nm[48]; json_escape(m.name,nm,sizeof(nm));
    char inf[64]; json_escape(m.info,inf,sizeof(inf));
    char mc[13]; mac_hex(m.mac,mc); char im[13]; mac_hex(m.init_mac,im);
    fprintf(f,"{\"t\":%lld,\"ch\":%d,\"f\":%.4f,\"crc\":%d,\"rssi\":%.1f,\"cfo\":%.0f,"
              "\"ac\":%d,\"pt\":%d,\"at\":%d,"
              "\"mac\":\"%s\",\"name\":\"%s\",\"fl\":%d,\"co\":%u,\"ap\":%d,\"nad\":%d,\"info\":\"%s\","
              "\"cn\":%d,\"im\":\"%s\",\"aa\":%u,\"ci\":%u,\"iv\":%d,\"to\":%d,\"lt\":%d,"
              "\"hop\":%d,\"sca\":%d,\"cm\":%llu}\n",
        (long long)m.t_ms,m.ch,m.freq,m.crc_ok?1:0,m.rssi,m.cfo_hz,
        m.adv_chan,m.pdu_type,m.addr_type,
        mc,nm,m.flags,(unsigned)m.company,m.appearance,m.n_ad,inf,
        m.is_connect?1:0,im,m.access_addr,m.crc_init,m.interval,m.timeout,m.latency,
        m.hop,m.sca,(unsigned long long)m.chan_map);
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
void store_parse_jsonl(const char* data, size_t n, std::vector<BtleRecord>& out){
    size_t i=0; std::string line;
    while(i<n){
        size_t e=i; while(e<n && data[e]!='\n') e++;
        line.assign(data+i,e-i); i=e+1;
        if(line.size()<8) continue;
        const char* l=line.c_str();
        BtleRecord m{};
        m.t_ms=jll(l,"\"t\":"); m.ch=(int)jll(l,"\"ch\":"); m.freq=(float)jf(l,"\"f\":");
        m.crc_ok=jll(l,"\"crc\":")!=0; m.adv_chan=(int)jll(l,"\"ac\":");
        m.rssi=(float)jf(l,"\"rssi\":"); m.cfo_hz=(float)jf(l,"\"cfo\":");
        m.pdu_type=(int)jll(l,"\"pt\":"); m.addr_type=(int)jll(l,"\"at\":");
        char mc[13]={0}; if(jstr(l,"\"mac\":\"",mc,sizeof(mc))) hex_mac(mc,m.mac);
        jstr(l,"\"name\":\"",m.name,sizeof(m.name));
        m.flags=(int)jll(l,"\"fl\":"); m.company=(uint16_t)jll(l,"\"co\":");
        m.appearance = strstr(l,"\"ap\":") ? (int)jll(l,"\"ap\":") : -1;   // 구파일 키없음→-1
        m.n_ad=(int)jll(l,"\"nad\":"); jstr(l,"\"info\":\"",m.info,sizeof(m.info));
        m.is_connect=jll(l,"\"cn\":")!=0;
        char im[13]={0}; if(jstr(l,"\"im\":\"",im,sizeof(im))) hex_mac(im,m.init_mac);
        m.access_addr=(uint32_t)jll(l,"\"aa\":"); m.crc_init=(uint32_t)jll(l,"\"ci\":");
        m.interval=(int)jll(l,"\"iv\":"); m.timeout=(int)jll(l,"\"to\":"); m.latency=(int)jll(l,"\"lt\":");
        m.hop=(int)jll(l,"\"hop\":"); m.sca=(int)jll(l,"\"sca\":");
        m.chan_map=(uint64_t)strtoull((strstr(l,"\"cm\":")?strstr(l,"\"cm\":")+5:"0"),nullptr,10);
        out.push_back(m);
    }
}

// ── 중복 비콘 억제 (HOST): 같은 MAC 이 동일 내용(type/name/info/company/appearance)을
//    DEDUP_MS 내 재방송하면 저장·전송 생략. BLE 비콘은 초당 수회 동일패킷 → 90%+ 가 중복.
//    내용이 바뀌면(이름 등장/모델 갱신) 즉시 통과. CONNECT_IND 는 항상 통과(희소·중요).
static constexpr int64_t DEDUP_MS = 30000;
static std::mutex                       g_dedup_mtx;
static std::unordered_map<uint64_t,std::pair<int64_t,uint64_t>> g_dedup;  // MAC → (last_t, hash)
static uint64_t btle_content_hash(const BtleRecord& m){
    uint64_t h=1469598103934665603ULL;                 // FNV-1a
    auto mix=[&](const void* p,size_t n){ const uint8_t* b=(const uint8_t*)p;
        for(size_t i=0;i<n;i++){ h^=b[i]; h*=1099511628211ULL; } };
    mix(&m.pdu_type,sizeof(m.pdu_type)); mix(&m.company,sizeof(m.company));
    mix(&m.appearance,sizeof(m.appearance)); mix(&m.flags,sizeof(m.flags));
    mix(m.name,sizeof(m.name)); mix(m.info,sizeof(m.info));
    return h;
}
static bool btle_is_dup(const BtleRecord& m){
    if(m.is_connect) return false;                     // 연결요청 항상 보존
    uint64_t mac=0; for(int b=0;b<6;b++) mac=(mac<<8)|m.mac[b];
    uint64_t hsh=btle_content_hash(m);
    std::lock_guard<std::mutex> lk(g_dedup_mtx);
    auto it=g_dedup.find(mac);
    if(it!=g_dedup.end() && it->second.second==hsh && m.t_ms-it->second.first < DEDUP_MS)
        return true;
    g_dedup[mac] = {m.t_ms, hsh};
    if(g_dedup.size()>20000) g_dedup.clear();           // 폭주 가드 (단순 비우기)
    return false;
}

// ── HOST: 워커 → 스탬프 + 중복억제 + 아카이브 + framework emit ──────────────
void host_emit(FFTViewer& v, BtleRecord m){
    if(m.ch>=0 && m.ch<MAX_CHANNELS && v.channels[m.ch].filter_active)
        m.freq = (v.channels[m.ch].s + v.channels[m.ch].e)/2.0f;
    m.t_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    if(btle_is_dup(m)) return;                          // 중복 비콘 → 저장·전송 생략
    store_append(m);
    BtleWireMsg w; btle_msg_to_wire(m, w);
    bewe_mod_emit(v, "btle", &w, sizeof(w));
}

static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v;
    if(n < sizeof(BtleWireMsg)) return;
    BtleRecord m; btle_wire_to_msg(*reinterpret_cast<const BtleWireMsg*>(d), m);
    bewe_mod_stat_bump("btle", station, m.ch, m.t_ms);
    station_disp(station, m.station, sizeof(m.station));
    append_log(m);
}

// ── 과거조회(Hist): 라이브 log 대피/복원 + 과거 JSONL 오버레이 ──────────────
static std::vector<BtleRecord> g_stash;   // Hist 진입 시 라이브 log 대피 버퍼
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
    std::vector<BtleRecord> parsed;
    store_parse_jsonl(data, n, parsed);
    for(auto& m : parsed){ strncpy(m.station, station, sizeof(m.station)-1); m.station[sizeof(m.station)-1]=0; }
    std::lock_guard<std::mutex> lk(mtx);
    log.insert(log.end(), parsed.begin(), parsed.end());
    std::stable_sort(log.begin(), log.end(),
                     [](const BtleRecord& a, const BtleRecord& b){ return a.t_ms < b.t_ms; });
    if((int)log.size() > LOG_MAX) log.erase(log.begin(), log.end()-LOG_MAX);
}

#ifndef BEWE_HEADLESS
void local_load_today(FFTViewer& v){
    (void)v;
    std::string body;
    if(!store_read_today(body)) return;
    std::vector<BtleRecord> parsed;
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
    m.id    = "btle";
    m.label = "Bluetooth LE";
    m.planned = false;
    m.target_modes = (uint8_t)(1u << Channel::DM_FM);   // GFSK → FM 채널필터에 노출
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

} // namespace btle_mod
