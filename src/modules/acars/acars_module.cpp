// ── ACARS 모듈 본체: 등록, 파이프 프로토콜, 일 단위 저장, 워커 관리 ──────────
// GUI/CLI 공통 (BEWE_HEADLESS 가드로 GUI hook 만 분기).
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
std::atomic<uint32_t> on_mask{0};

// ── 워커 슬롯 ──────────────────────────────────────────────────────────────
struct ChWork {
    std::atomic<bool>   on{false};
    std::atomic<bool>   stop{false};
    std::atomic<size_t> rp{0};
    std::thread         thr;
};
static ChWork g_w[MAX_CHANNELS];
std::atomic<size_t>& worker_rp(int ch){ return g_w[ch].rp; }
bool worker_stop_req(int ch){ return g_w[ch].stop.load(std::memory_order_relaxed); }
// 워커 자연 종료(채널 삭제 등) 시 호출 — 상태 정리 + 브로드캐스트
void worker_natural_exit(int ch){
    g_w[ch].on.store(false);
    on_mask.fetch_and(~(1u<<ch));
    WireState st{ on_mask.load() };
    bewe_mod_broadcast("acars", K_STATE, &st, sizeof(st));
}

void host_set(FFTViewer& v, int ch, bool on){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    ChWork& w = g_w[ch];
    // 죽은 스레드 회수 (자연 종료 후 joinable 잔존)
    if(!w.on.load() && w.thr.joinable()) w.thr.join();
    if(on){
        if(w.on.load()) return;
        if(!v.channels[ch].filter_active) return;
        w.stop.store(false);
        w.on.store(true);
        w.thr = std::thread(worker, std::ref(v), ch);
        on_mask.fetch_or(1u<<ch);
    } else {
        if(w.on.load()){
            w.stop.store(true);
            if(w.thr.joinable()) w.thr.join();
            w.on.store(false);
        }
        on_mask.fetch_and(~(1u<<ch));
    }
    WireState st{ on_mask.load() };
    bewe_mod_broadcast("acars", K_STATE, &st, sizeof(st));
}

static void host_stop_all(FFTViewer& v){
    for(int i=0;i<MAX_CHANNELS;i++) if(g_w[i].on.load()) host_set(v, i, false);
}

// ── 로그 ──────────────────────────────────────────────────────────────────
void append_log(const AcarsMsg& m){
    std::lock_guard<std::mutex> lk(mtx);
    if((int)msglog.size() >= LOG_MAX) msglog.erase(msglog.begin());
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

// ── 일 단위 JSONL 저장소 ───────────────────────────────────────────────────
std::string store_dir(){
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
std::string store_path_today(){
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

// ── HOST: 워커 디코드 → 스탬프 + 로그 + 저장 + 브로드캐스트 ──────────────────
void host_emit(FFTViewer& v, AcarsMsg m){
    if(m.ch>=0 && m.ch<MAX_CHANNELS && v.channels[m.ch].filter_active)
        m.freq = (v.channels[m.ch].s + v.channels[m.ch].e)/2.0f;
    m.t_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    append_log(m);
    store_append(m);
    WireMsg w; msg_to_wire(m, w);
    bewe_mod_broadcast("acars", K_MSG, &w, sizeof(w));
}

// ── 파이프 수신: HOST 측 (JOIN→HOST) ───────────────────────────────────────
static void on_pipe_host(FFTViewer& v, uint8_t kind, const uint8_t* d, size_t n){
    switch(kind){
    case K_TOGGLE: {
        if(n < sizeof(WireToggle)) break;
        auto* t = reinterpret_cast<const WireToggle*>(d);
        int ch = t->ch;
        if(ch<0 || ch>=MAX_CHANNELS) break;
        if(t->on){
            if(!v.channels[ch].filter_active) break;
            host_set(v, ch, true);
        } else host_set(v, ch, false);
        break;
    }
    case K_HIST_REQ: {
        std::string body;
        store_read_today(body);   // 없으면 빈 파일 (META 0 → JOIN 은 빈 로그)
        WireHistMeta meta{ (uint32_t)body.size() };
        bewe_mod_broadcast("acars", K_HIST_META, &meta, sizeof(meta));
        constexpr size_t CHUNK = 8192;
        for(size_t off=0; off<body.size(); off+=CHUNK){
            size_t len = std::min(CHUNK, body.size()-off);
            bewe_mod_broadcast("acars", K_HIST_CHUNK, body.data()+off, len);
        }
        bewe_mod_broadcast("acars", K_HIST_DONE, nullptr, 0);
        break;
    }
    default: break;
    }
}

// ── 파이프 수신: JOIN 측 (HOST→JOIN) ───────────────────────────────────────
#ifndef BEWE_HEADLESS
std::atomic<bool> hist_waiting{false};
std::string       hist_buf;
static std::vector<AcarsMsg> hist_pending;   // 전송 중 도착한 라이브 메시지
#endif

static void on_pipe_join(FFTViewer& v, uint8_t kind, const uint8_t* d, size_t n){
    (void)v;
#ifndef BEWE_HEADLESS
    switch(kind){
    case K_STATE: {
        if(n < sizeof(WireState)) break;
        on_mask.store(reinterpret_cast<const WireState*>(d)->mask);
        break;
    }
    case K_MSG: {
        if(n < sizeof(WireMsg)) break;
        AcarsMsg m; wire_to_msg(*reinterpret_cast<const WireMsg*>(d), m);
        if(hist_waiting.load()){ std::lock_guard<std::mutex> lk(mtx); hist_pending.push_back(m); }
        else append_log(m);
        break;
    }
    case K_HIST_META:
        if(hist_waiting.load()){ hist_buf.clear(); }
        break;
    case K_HIST_CHUNK:
        if(hist_waiting.load() && n) hist_buf.append((const char*)d, n);
        break;
    case K_HIST_DONE: {
        if(!hist_waiting.exchange(false)) break;
        std::vector<AcarsMsg> parsed;
        store_parse_jsonl(hist_buf.data(), hist_buf.size(), parsed);
        hist_buf.clear(); hist_buf.shrink_to_fit();
        std::lock_guard<std::mutex> lk(mtx);
        msglog = std::move(parsed);
        // 전송 중 도착분 합류 (파일 끝과 겹치면 t_ms+text 로 dedup)
        for(auto& m : hist_pending){
            bool dup=false;
            for(int i=(int)msglog.size()-1; i>=0 && i>=(int)msglog.size()-8; i--)
                if(msglog[i].t_ms==m.t_ms && strcmp(msglog[i].text,m.text)==0){ dup=true; break; }
            if(!dup) msglog.push_back(m);
        }
        hist_pending.clear();
        if((int)msglog.size() > LOG_MAX) msglog.erase(msglog.begin(), msglog.end()-LOG_MAX);
        scroll = true;
        break;
    }
    default: break;
    }
#else
    (void)kind; (void)d; (void)n;
#endif
}

// 채널 demod 종료/모드변경 → 워커 정리 (코어 stop_dem hook)
static void on_ch_stop(FFTViewer& v, int ch){
    if((on_mask.load()>>ch)&1) host_set(v, ch, false);
}

// ── HOST: 새 JOIN 접속 → 현재 상태 push ────────────────────────────────────
static void on_join_open(FFTViewer& v){
    (void)v;
    WireState st{ on_mask.load() };
    bewe_mod_broadcast("acars", K_STATE, &st, sizeof(st));
}

#ifndef BEWE_HEADLESS
// 히스토리 로드 트리거 (탭 열릴 때 view 가 호출)
void request_history(FFTViewer& v){
    if(v.remote_mode){
        hist_waiting.store(true);
        hist_buf.clear();
        bewe_mod_send_to_host("acars", K_HIST_REQ, nullptr, 0);
    } else {
        // LOCAL: 오늘 파일 직접 로드 (저장소가 세션 로그의 superset)
        std::string body;
        if(!store_read_today(body)) return;
        std::vector<AcarsMsg> parsed;
        store_parse_jsonl(body.data(), body.size(), parsed);
        std::lock_guard<std::mutex> lk(mtx);
        if(!parsed.empty()) msglog = std::move(parsed);
        if((int)msglog.size() > LOG_MAX) msglog.erase(msglog.begin(), msglog.end()-LOG_MAX);
        scroll = true;
    }
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
#ifndef BEWE_HEADLESS
    m.init         = &init_gui;
    m.draw_content = &draw_content;
    m.channel_ui   = &channel_ui;
#endif
    m.on_pipe_join = &on_pipe_join;
    m.on_pipe_host = &on_pipe_host;
    m.on_join_open = &on_join_open;
    m.on_ch_stop   = &on_ch_stop;
    bewe_register_module(m);
    return true;
}();

} // namespace acars_mod
