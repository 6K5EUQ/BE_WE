#include "module_api.hpp"
#include "fft_viewer.hpp"
#include <cstring>
#include <map>
#include <mutex>
#include <vector>

// ── 모듈 레지스트리 + framework (GUI/CLI 공통, 모듈 무관) ───────────────────
// Central 컨트롤플레인 구조: 타깃 목록/명령/상태/데이터/구독/히스토리는 전부
// framework kind (BEWE_MK_*) 로 코어가 처리. 모듈은 데이터 payload 포맷만 소유.

static std::vector<BeweModule>& reg(){
    static std::vector<BeweModule> v;
    return v;
}
void bewe_register_module(const BeweModule& m){ reg().push_back(m); }
const std::vector<BeweModule>& bewe_modules(){ return reg(); }

static const BeweModule* find_mod(const char* id){
    for(auto& m : reg()) if(strcmp(m.id,id)==0) return &m;
    return nullptr;
}

// ── 송신 백엔드 ──
static std::function<bool(const void*, uint32_t)> g_send_up;     // JOIN→Central
static std::function<bool(const void*, uint32_t)> g_broadcast;   // HOST→relay(Central)
void bewe_mod_set_send_up(std::function<bool(const void*, uint32_t)> f){ g_send_up = std::move(f); }
void bewe_mod_set_broadcast(std::function<bool(const void*, uint32_t)> f){ g_broadcast = std::move(f); }

static char g_my_station[24] = {};
const char* bewe_mod_my_station(){ return g_my_station; }

// ── 모듈별 framework 상태 ──
struct ModFw {
    uint32_t host_mask = 0;                  // HOST: 자기 워커 mask
    std::vector<MpChEntry> targets;          // JOIN: CH_LIST 미러
    std::map<std::string,uint32_t> masks;    // JOIN: station→mask (STATE 미러)
    bool recv = false;                       // JOIN: 구독 상태
    bool hist_loading = false;               // 구독 on → DONE 처리까지 (UI 표시 + 라이브 버퍼링)
    bool hist_started = false;               // META 수신 후에만 CHUNK 수락 (잔여 스트림 차단)
    std::string hist_buf;
    uint32_t hist_total = 0;
    std::vector<std::vector<uint8_t>> live_pending;  // hist 전송 중 도착한 라이브 레코드 (MpData+payload)
};
static std::mutex g_fw_mtx;
static std::map<std::string, ModFw> g_fw;
static ModFw& fw(const char* id){ return g_fw[id]; }   // 호출측이 g_fw_mtx 잡음

void bewe_mod_set_my_station(const char* s){
    memset(g_my_station, 0, sizeof(g_my_station));
    if(s) strncpy(g_my_station, s, sizeof(g_my_station)-1);
    // (재)접속 시점 — 이전 연결의 구독/히스토리 상태는 무효 (Central 은 conn 단위 구독)
    std::lock_guard<std::mutex> lk(g_fw_mtx);
    for(auto& kv : g_fw){ kv.second.recv=false; kv.second.hist_loading=false; kv.second.hist_buf.clear(); }
}

static std::vector<uint8_t> build_pipe(const char* mod_id, uint8_t kind, const void* d, size_t n){
    std::vector<uint8_t> buf(sizeof(PktModulePipe) + n);
    auto* h = reinterpret_cast<PktModulePipe*>(buf.data());
    memset(h, 0, sizeof(*h));
    strncpy(h->mod_id, mod_id, sizeof(h->mod_id)-1);
    h->kind = kind;
    h->data_len = (uint32_t)n;
    if(n) memcpy(buf.data()+sizeof(PktModulePipe), d, n);
    return buf;
}
static bool send_up(const char* id, uint8_t kind, const void* d, size_t n){
    if(!g_send_up) return false;
    auto b = build_pipe(id, kind, d, n);
    return g_send_up(b.data(), (uint32_t)b.size());
}
static bool bcast(const char* id, uint8_t kind, const void* d, size_t n){
    if(!g_broadcast) return false;
    auto b = build_pipe(id, kind, d, n);
    return g_broadcast(b.data(), (uint32_t)b.size());
}

// ── HOST framework ──────────────────────────────────────────────────────────
static void host_send_state(const char* id){
    MpState st{}; strncpy(st.station, g_my_station, sizeof(st.station)-1);
    { std::lock_guard<std::mutex> lk(g_fw_mtx); st.mask = fw(id).host_mask; }
    bcast(id, BEWE_MK_STATE, &st, sizeof(st));
}

uint32_t bewe_mod_host_mask(const char* id){
    std::lock_guard<std::mutex> lk(g_fw_mtx);
    return fw(id).host_mask;
}

void bewe_mod_host_announce(FFTViewer& v){
    (void)v;
    for(auto& m : reg()) if(m.target_modes) host_send_state(m.id);
}

void bewe_mod_host_mask_clear(FFTViewer& v, const char* id, int ch){
    (void)v;
    { std::lock_guard<std::mutex> lk(g_fw_mtx); fw(id).host_mask &= ~(1u<<ch); }
    host_send_state(id);
}

// HOST 가 SET 적용 (Central 라우팅 수신 / LOCAL 직접)
static void host_apply_set(FFTViewer& v, const BeweModule& m, int ch, bool on){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    bool ok = false;
    if(on){ if(m.host_start) ok = m.host_start(v, ch); }
    else  { if(m.host_stop)  m.host_stop(v, ch); }
    {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        if(on && ok)   fw(m.id).host_mask |=  (1u<<ch);
        if(!on)        fw(m.id).host_mask &= ~(1u<<ch);
    }
    host_send_state(m.id);
}

// HOST 워커 → 디코드 1건: Central 전송 + 로컬 뷰 반영
void bewe_mod_emit(FFTViewer& v, const char* id, const void* payload, size_t n){
    const BeweModule* m = find_mod(id);
    if(!m) return;
    // MpData 봉투 (station 명시 — 어느 기지의 복조 데이터인지)
    std::vector<uint8_t> body(sizeof(MpData) + n);
    auto* d = reinterpret_cast<MpData*>(body.data());
    memset(d, 0, sizeof(MpData));
    strncpy(d->station, g_my_station, sizeof(d->station)-1);
    memcpy(body.data()+sizeof(MpData), payload, n);
    bcast(id, BEWE_MK_DATA, body.data(), body.size());
    // 로컬 뷰 (HOST GUI / LOCAL): 자기 데이터는 즉시 표시
    if(m->on_data) m->on_data(v, g_my_station, (const uint8_t*)payload, n);
}

// ── JOIN/뷰어 framework ─────────────────────────────────────────────────────
bool bewe_mod_recv(const char* id){
    std::lock_guard<std::mutex> lk(g_fw_mtx);
    return fw(id).recv;
}
bool bewe_mod_hist_loading(const char* id){
    std::lock_guard<std::mutex> lk(g_fw_mtx);
    return fw(id).hist_loading;
}
void bewe_mod_set_recv(FFTViewer& v, const char* id, bool on){
    (void)v;
    {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        f.recv = on;
        if(on){ f.hist_loading = true; f.hist_started = false; f.hist_buf.clear(); f.hist_total = 0; f.live_pending.clear(); }
        else { f.hist_loading = false; f.hist_started = false; f.hist_buf.clear(); f.live_pending.clear(); }
    }
    MpRecv r{ (uint8_t)(on?1:0) };
    if(!send_up(id, BEWE_MK_RECV, &r, sizeof(r))){
        // LOCAL: 구독 개념 없음 — hist_loading 해제
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        fw(id).hist_loading = false;
    }
}
void bewe_mod_req_ch_list(const char* id){
    send_up(id, BEWE_MK_CH_LIST_REQ, nullptr, 0);
}

std::vector<MpChEntry> bewe_mod_targets(FFTViewer& v, const char* id){
    if(g_send_up && v.remote_mode){
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        return fw(id).targets;   // Central 집계본
    }
    // LOCAL / HOST GUI: 로컬 채널에서 직접 구성
    std::vector<MpChEntry> out;
    uint32_t mask;
    { std::lock_guard<std::mutex> lk(g_fw_mtx); mask = fw(id).host_mask; }
    for(int i=0;i<MAX_CHANNELS;i++){
        Channel& ch = v.channels[i];
        if(!ch.filter_active) continue;
        MpChEntry e{};
        strncpy(e.station, g_my_station[0]?g_my_station:"LOCAL", sizeof(e.station)-1);
        e.ch = (uint8_t)i; e.mode = (uint8_t)ch.mode;
        e.decode_on = ((mask>>i)&1) ? 1 : 0;
        e.lo = ch.s; e.hi = ch.e;
        out.push_back(e);
    }
    return out;
}

void bewe_mod_set_target(FFTViewer& v, const char* id, const char* station, int ch, bool on){
    const BeweModule* m = find_mod(id);
    if(!m) return;
    if(g_send_up && v.remote_mode){
        MpSet st{}; strncpy(st.station, station, sizeof(st.station)-1);
        st.ch = (uint8_t)ch; st.on = on?1:0;
        send_up(id, BEWE_MK_SET, &st, sizeof(st));
        return;
    }
    // LOCAL / HOST GUI: 직접 적용 (+상태 브로드캐스트는 host_apply_set 내부)
    host_apply_set(v, *m, ch, on);
}

// ── 수신 라우팅 ─────────────────────────────────────────────────────────────
void bewe_mod_route(FFTViewer& v, bool host_side, const uint8_t* payload, size_t len){
    if(len < sizeof(PktModulePipe)) return;
    auto* h = reinterpret_cast<const PktModulePipe*>(payload);
    if(sizeof(PktModulePipe) + h->data_len > len) return;
    char id[9]={}; memcpy(id, h->mod_id, 8);
    const uint8_t* d = payload + sizeof(PktModulePipe);
    size_t n = h->data_len;
    const BeweModule* m = find_mod(id);
    if(!m) return;   // 미설치 모듈 → 조용히 무시

    if(host_side){
        // HOST: Central 이 라우팅한 SET 만 처리
        if(h->kind == BEWE_MK_SET && n >= sizeof(MpSet)){
            auto* st = reinterpret_cast<const MpSet*>(d);
            char stn[25]={}; memcpy(stn, st->station, 24);
            if(g_my_station[0] && strncmp(stn, g_my_station, 24)!=0) return; // 다른 기지 명령
            host_apply_set(v, *m, st->ch, st->on!=0);
        }
        return;
    }

    // JOIN/뷰어 측
    switch(h->kind){
    case BEWE_MK_CH_LIST: {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        f.targets.clear();
        size_t cnt = n / sizeof(MpChEntry);
        auto* e = reinterpret_cast<const MpChEntry*>(d);
        for(size_t i=0;i<cnt;i++) f.targets.push_back(e[i]);
        break;
    }
    case BEWE_MK_STATE: {
        if(n < sizeof(MpState)) break;
        auto* st = reinterpret_cast<const MpState*>(d);
        char stn[25]={}; memcpy(stn, st->station, 24);
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        f.masks[stn] = st->mask;
        for(auto& t : f.targets)
            if(strncmp(t.station, st->station, 24)==0)
                t.decode_on = ((st->mask>>t.ch)&1) ? 1 : 0;
        break;
    }
    case BEWE_MK_DATA: {
        if(n < sizeof(MpData)) break;
        {   // 히스토리 전송 중엔 버퍼링 → DONE 후 히스토리 뒤에 시간순으로 합류
            std::lock_guard<std::mutex> lk(g_fw_mtx);
            ModFw& f = fw(id);
            if(f.hist_loading){ f.live_pending.emplace_back(d, d+n); break; }
        }
        auto* md = reinterpret_cast<const MpData*>(d);
        char stn[25]={}; memcpy(stn, md->station, 24);
        if(m->on_data) m->on_data(v, stn, d+sizeof(MpData), n-sizeof(MpData));
        break;
    }
    case BEWE_MK_HIST_META: {
        if(n < sizeof(MpHistMeta)) break;
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        if(!f.hist_loading) break;   // 구독 안 한 상태의 잔여 스트림 무시
        f.hist_total = reinterpret_cast<const MpHistMeta*>(d)->total_bytes;
        f.hist_buf.clear(); f.hist_started = true;
        break;
    }
    case BEWE_MK_HIST_CHUNK: {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        if(f.hist_loading && f.hist_started && n) f.hist_buf.append((const char*)d, n);
        break;
    }
    case BEWE_MK_HIST_DONE: {
        std::string buf;
        std::vector<std::vector<uint8_t>> pending;
        {
            std::lock_guard<std::mutex> lk(g_fw_mtx);
            ModFw& f = fw(id);
            if(!f.hist_loading || !f.hist_started) break;   // META 없이 온 잔여 DONE 무시
            buf.swap(f.hist_buf);
            pending.swap(f.live_pending);
            f.hist_loading = false; f.hist_started = false;
        }
        // 레코드 스트림: u32 len + (MpData + module payload) 반복
        auto dispatch = [&](const uint8_t* rec, size_t rl){
            if(rl < sizeof(MpData)) return;
            auto* md = reinterpret_cast<const MpData*>(rec);
            char stn[25]={}; memcpy(stn, md->station, 24);
            if(m->on_data) m->on_data(v, stn, rec+sizeof(MpData), rl-sizeof(MpData));
        };
        size_t off = 0;
        while(off + 4 <= buf.size()){
            uint32_t rl; memcpy(&rl, buf.data()+off, 4); off += 4;
            if(off + rl > buf.size()) break;
            dispatch((const uint8_t*)buf.data()+off, rl);
            off += rl;
        }
        // 히스토리 후 라이브 버퍼 합류 (모듈측 append dedup 이 경계 중복 제거)
        for(auto& r : pending) dispatch(r.data(), r.size());
        break;
    }
    default: break;
    }
}
