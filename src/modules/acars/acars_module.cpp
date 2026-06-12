// ── ACARS 모듈 본체: 등록, 워커 관리, 일 단위 아카이브, framework 연동 ──────
// GUI/CLI 공통 (BEWE_HEADLESS 가드로 GUI hook 만 분기).
// 제어/전송은 전부 framework (module_api) 경유 — 모듈은 복조 + 레코드 포맷만 소유.
#include "acars_module.hpp"
#include "acars_db.hpp"
#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>

namespace acars_mod {

std::mutex            mtx;
std::vector<AcarsMsg> msglog;
bool                  scroll = true;
char                  filter[64] = {};

// ── 워커 슬롯 ──────────────────────────────────────────────────────────────
struct ChWork {
    std::atomic<bool>   on{false};
    std::atomic<bool>   stop{false};
    std::atomic<size_t> rp{0};
    std::thread         thr;
};
static ChWork g_w[MAX_CHANNELS];
// start/stop 은 여러 스레드에서 동시 도달 가능 (Central mux 스레드 / relay client_loop
// 스레드 / GUI 렌더 스레드) — join/move-assign 직렬화 필수
static std::mutex g_mgmt;
std::atomic<size_t>& worker_rp(int ch){ return g_w[ch].rp; }
bool worker_stop_req(int ch){ return g_w[ch].stop.load(std::memory_order_relaxed); }
// 워커 자연 종료(채널 삭제/스트림 에러) — framework mask 정리 + 상태 전파
void worker_natural_exit(FFTViewer& v, int ch){
    g_w[ch].on.store(false);
    bewe_mod_host_mask_clear(v, "acars", ch);
}

static bool host_start(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return false;
    std::lock_guard<std::mutex> lk(g_mgmt);
    ChWork& w = g_w[ch];
    if(!w.on.load() && w.thr.joinable()) w.thr.join();   // 죽은 스레드 회수
    if(w.on.load()) return true;
    if(!v.channels[ch].filter_active) return false;
    if(v.channels[ch].mode != Channel::DM_AM) return false;
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

// 채널 demod 종료/모드변경 (코어 stop_dem hook)
static void on_ch_stop(FFTViewer& v, int ch){
    if((bewe_mod_host_mask("acars")>>ch)&1){
        host_stop(v, ch);
        bewe_mod_host_mask_clear(v, "acars", ch);
    }
}

// ── 로그 ──────────────────────────────────────────────────────────────────
void append_log(const AcarsMsg& m){
    std::lock_guard<std::mutex> lk(mtx);
    // dedup: 히스토리/라이브 경계·재구독에서 같은 레코드 중복 도달 가능
    int n = (int)msglog.size();
    for(int i=n-1; i>=0 && i>=n-64; i--){
        if(msglog[i].t_ms==m.t_ms && !strcmp(msglog[i].reg,m.reg) && !strcmp(msglog[i].text,m.text))
            return;
    }
    if(n >= LOG_MAX) msglog.erase(msglog.begin());
    msglog.push_back(m);
    scroll = true;
}

void msg_to_wire(const AcarsMsg& m, WireMsg& w){
    memset(&w, 0, sizeof(w));
    w.t_ms=m.t_ms; w.freq=m.freq; w.ch=(uint8_t)m.ch;
    w.crc_ok=m.crc_ok?1:0; w.downlink=m.downlink?1:0;
    w.mode=m.mode; w.block=m.block; w.ack=m.ack;
    memcpy(w.reg,m.reg,sizeof(w.reg)); memcpy(w.flight,m.flight,sizeof(w.flight));
    memcpy(w.label,m.label,sizeof(w.label)); memcpy(w.text,m.text,sizeof(w.text));
}
void wire_to_msg(const WireMsg& w, AcarsMsg& m){
    m = AcarsMsg{};
    m.t_ms=w.t_ms; m.freq=w.freq; m.ch=w.ch;
    m.crc_ok=w.crc_ok!=0; m.downlink=w.downlink!=0;
    m.mode=w.mode; m.block=w.block; m.ack=w.ack;
    memcpy(m.reg,w.reg,sizeof(m.reg)); m.reg[sizeof(m.reg)-1]=0;
    memcpy(m.flight,w.flight,sizeof(m.flight)); m.flight[sizeof(m.flight)-1]=0;
    memcpy(m.label,w.label,sizeof(m.label)); m.label[sizeof(m.label)-1]=0;
    memcpy(m.text,w.text,sizeof(m.text)); m.text[sizeof(m.text)-1]=0;
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
    return base + "/BE_WE/modules/acars";
}
static void mkdirs(const std::string& path){
    std::string p;
    for(size_t i=0;i<path.size();i++){
        p += path[i];
        if(path[i]=='/' && p.size()>1) mkdir(p.c_str(), 0755);
    }
    mkdir(path.c_str(), 0755);
}
static void kst_date_of(int64_t t_ms, char out[9]){
    struct tm tmv{};
    KST::to_tm((time_t)(t_ms/1000), tmv);
    snprintf(out, 9, "%04d%02d%02d", tmv.tm_year+1900, tmv.tm_mon+1, tmv.tm_mday);
}
static std::string store_path_today(){
    int64_t now = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();
    char d[9]; kst_date_of(now, d);
    return store_dir() + "/acars_" + d + ".jsonl";
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

void store_append(const AcarsMsg& m){
    static std::mutex fmtx;
    std::lock_guard<std::mutex> lk(fmtx);
    std::string dir = store_dir();
    mkdirs(dir);
    char d[9]; kst_date_of(m.t_ms, d);
    std::string path = dir + "/acars_" + d + ".jsonl";
    FILE* f = fopen(path.c_str(), "ab");
    if(!f) return;
    char etx[600]; json_escape(m.text, etx, sizeof(etx));
    fprintf(f,
        "{\"t\":%lld,\"ch\":%d,\"f\":%.4f,\"crc\":%d,\"dn\":%d,"
        "\"mo\":\"%c\",\"bl\":\"%c\",\"ak\":\"%c\","
        "\"reg\":\"%s\",\"fl\":\"%s\",\"lb\":\"%s\",\"tx\":\"%s\"}\n",
        (long long)m.t_ms, m.ch, m.freq, m.crc_ok?1:0, m.downlink?1:0,
        m.mode?m.mode:' ', m.block?m.block:' ', m.ack?m.ack:' ',
        m.reg, m.flight, m.label, etx);
    fclose(f);
}

bool store_read_today(std::string& out){
    out.clear();
    FILE* f = fopen(store_path_today().c_str(), "rb");
    if(!f) return false;
    char buf[8192]; size_t n;
    while((n=fread(buf,1,sizeof(buf),f))>0) out.append(buf,n);
    fclose(f);
    return true;
}

// ── JSONL 파서 (store_append 가 쓴 포맷 전용) ──
static bool jget_ll (const char* l, const char* key, long long& v){ const char* p=strstr(l,key); if(!p) return false; v=atoll(p+strlen(key)); return true; }
static bool jget_f  (const char* l, const char* key, float& v){ const char* p=strstr(l,key); if(!p) return false; v=(float)atof(p+strlen(key)); return true; }
static bool jget_str(const char* l, const char* key, char* out, size_t cap){
    const char* p=strstr(l,key); if(!p) return false;
    p += strlen(key);
    size_t o=0;
    while(*p && *p!='"' && o+1<cap){
        if(*p=='\\'){
            ++p;
            if(*p=='u'){ unsigned x=0; sscanf(p+1,"%4x",&x); out[o++]=(char)(x&0x7F); p+=5; continue; }
            if(*p) out[o++]=*p++;
            continue;
        }
        out[o++]=*p++;
    }
    out[o]=0;
    return true;
}

void store_parse_jsonl(const char* data, size_t n, std::vector<AcarsMsg>& out){
    size_t i=0;
    std::string line;
    while(i<n){
        size_t e=i; while(e<n && data[e]!='\n') e++;
        line.assign(data+i, e-i);
        i = e+1;
        if(line.size()<10) continue;
        const char* l = line.c_str();
        AcarsMsg m{};
        long long t=0; jget_ll(l,"\"t\":",t); m.t_ms=t;
        long long ch=0; jget_ll(l,"\"ch\":",ch); m.ch=(int)ch;
        jget_f(l,"\"f\":",m.freq);
        long long b=0; jget_ll(l,"\"crc\":",b); m.crc_ok=b!=0;
        b=0; jget_ll(l,"\"dn\":",b); m.downlink=b!=0;
        char one[4];
        if(jget_str(l,"\"mo\":\"",one,sizeof(one))) m.mode=one[0];
        if(jget_str(l,"\"bl\":\"",one,sizeof(one))) m.block=one[0];
        if(jget_str(l,"\"ak\":\"",one,sizeof(one))) m.ack=one[0];
        jget_str(l,"\"reg\":\"",m.reg,sizeof(m.reg));
        jget_str(l,"\"fl\":\"",m.flight,sizeof(m.flight));
        jget_str(l,"\"lb\":\"",m.label,sizeof(m.label));
        jget_str(l,"\"tx\":\"",m.text,sizeof(m.text));
        out.push_back(m);
    }
}

// ── HOST: 워커 디코드 → 스탬프 + 아카이브 + framework emit ──────────────────
// (framework 가 Central 전송 + 로컬 on_data 반영을 처리)
void host_emit(FFTViewer& v, AcarsMsg m){
    if(m.ch>=0 && m.ch<MAX_CHANNELS && v.channels[m.ch].filter_active)
        m.freq = (v.channels[m.ch].s + v.channels[m.ch].e)/2.0f;
    m.t_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    store_append(m);
    WireMsg w; msg_to_wire(m, w);
    bewe_mod_emit(v, "acars", &w, sizeof(w));
}

// ── framework 데이터 수신 (라이브 + 히스토리 공용) ──────────────────────────
static void on_data(FFTViewer& v, const char* station, const uint8_t* d, size_t n){
    (void)v;
    if(n < sizeof(WireMsg)) return;
    AcarsMsg m; wire_to_msg(*reinterpret_cast<const WireMsg*>(d), m);
    station_disp(station, m.station, sizeof(m.station));
    append_log(m);
}

#ifndef BEWE_HEADLESS
// LOCAL: 오늘 아카이브 직접 로드 (Recv 개념 없음)
void local_load_today(FFTViewer& v){
    (void)v;
    std::string body;
    if(!store_read_today(body)) return;
    std::vector<AcarsMsg> parsed;
    store_parse_jsonl(body.data(), body.size(), parsed);
    for(auto& m : parsed) strncpy(m.station, "LOCAL", sizeof(m.station)-1);
    std::lock_guard<std::mutex> lk(mtx);
    if(!parsed.empty()) msglog = std::move(parsed);
    if((int)msglog.size() > LOG_MAX) msglog.erase(msglog.begin(), msglog.end()-LOG_MAX);
    scroll = true;
}

static void init_gui(FFTViewer& v){
    (void)v;
    std::string dbp = BEWEPaths::assets_dir() + "/aircraft_db.bin";
    if(acars_db_load(dbp.c_str())) bewe_log_push(0,"[ACARS] aircraft DB loaded\n");
}
#endif

// ── 모듈 등록 (static-init) ────────────────────────────────────────────────
static bool s_registered = [](){
    BeweModule m{};
    m.id    = "acars";
    m.label = "ACARS";
    m.target_modes = (uint8_t)(1u << Channel::DM_AM);
#ifndef BEWE_HEADLESS
    m.init         = &init_gui;
    m.draw_content = &draw_content;
#endif
    m.host_start = &host_start;
    m.host_stop  = &host_stop;
    m.on_ch_stop = &on_ch_stop;
    m.on_data    = &on_data;
    bewe_register_module(m);
    return true;
}();

} // namespace acars_mod
