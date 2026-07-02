#include "module_api.hpp"
#include "fft_viewer.hpp"
#include "net_server.hpp"   // CH_EDIT 적용 시 broadcast_channel_sync
#include "kst_time.hpp"     // 오늘 누적 디코드수 시드 (저장 JSONL = KST 일자)
#include "login.hpp"        // CH_ADD owner = login_get_id()
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <array>
#include <map>
#include <mutex>
#include <vector>
#include <zlib.h>
#include <chrono>
#include <utility>
#include <deque>
#include <string>
#include <thread>
#include <atomic>
#include <unistd.h>

static int64_t mod_now_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ── HOST 디코드 통계 (decode 켠 시각 기록 + 누적 카운트 리셋; ChSyncEntry 로 송출) ──
static void host_decstat_start(const char* id, int ch);  // decode on:  start=now, count=0
static void host_decstat_stop(int ch);                   // decode off: start=0

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
    uint64_t host_mask = 0;                  // HOST: 자기 워커 mask (ch 0~63 비트)
    uint64_t want_mask = 0;                  // HOST: 사용자가 켠 복조 의도 — 필터 깜빡임/SDR에러로
                                             //   워커가 죽어 host_mask 가 비어도 유지 → reconcile 자동 재장전
    std::vector<MpChEntry> targets;          // JOIN: CH_LIST 미러
    std::map<std::string,uint64_t> masks;    // JOIN: station→mask (STATE 미러)
    // JOIN: 진행 중 낙관적 SET (key={station,ch} → {want_on, ts_ms}). 폴링 CH_LIST 가
    // SET 왕복 전 옛 decode_on 으로 덮어써 버튼 깜빡이는 것 방지. 권위 STATE 도착/만료 시 제거.
    std::map<std::pair<std::string,int>, std::pair<uint8_t,int64_t>> pending;
    bool recv = false;                       // JOIN: 구독 상태
    bool hist_loading = false;               // 구독 on → DONE 처리까지 (UI 표시 + 라이브 버퍼링)
    bool hist_started = false;               // META 수신 후에만 CHUNK 수락 (잔여 스트림 차단)
    std::string hist_buf;
    uint32_t hist_total = 0;
    uint32_t hist_raw = 0;                    // META.raw_bytes (>0 → hist_buf 는 zlib 압축본)
    std::vector<std::vector<uint8_t>> live_pending;  // hist 전송 중 도착한 라이브 레코드 (MpData+payload)
    // 단일 대상 온디맨드 이력 (stream_kind=1) — 구독 스트림과 독립. 라이브 버퍼링 안 함.
    bool     vhist_active = false;
    std::string vhist_buf;
    uint32_t vhist_raw = 0;
    // 과거 날짜 아카이브 조회 (Hist 버튼; stream_kind=2)
    bool     hist_mode = false;              // 과거조회 모드 (라이브 log 는 모듈측 버퍼에 대피)
    char     hist_date[9] = {};              // 조회 중 날짜 "YYYYMMDD"
    bool     hist2_fetching = false;         // HFETCH 응답 대기/수신 중
    bool     hist2_active = false;           // META(kind2) 수신 후 CHUNK 수락
    std::string hist2_buf;
    uint32_t hist2_raw = 0;
    uint32_t hist2_gen = 0;                  // fetch 세대 — stale(옛 날짜) 응답 스트림 폐기용
    bool     hlist_loading = false;          // 날짜 목록 요청 중
    std::vector<BeweHistDate> hlist;         // 수신된 날짜 목록
    bool     hist_resub_skip = false;        // Hist 종료 재구독 시 오늘 히스토리(kind0) 무시 (버퍼 복원분과 중복 방지)
};
static std::mutex g_fw_mtx;
static std::map<std::string, ModFw> g_fw;
static ModFw& fw(const char* id){ return g_fw[id]; }   // 호출측이 g_fw_mtx 잡음

void bewe_mod_set_my_station(const char* s){
    memset(g_my_station, 0, sizeof(g_my_station));
    if(s) strncpy(g_my_station, s, sizeof(g_my_station)-1);
    // (재)접속 시점 — 이전 연결의 구독/히스토리 + 타깃/마스크 잔여 상태 전부 무효
    // (다른 기지로 전환 시 이전 기지 채널/RUN 상태가 남아 어긋나 보이는 것 방지)
    std::vector<std::string> was_hist;   // Hist 모드였던 모듈 — 락 밖에서 log_restore (모듈 mtx 는 다른 락)
    {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        for(auto& kv : g_fw){
            if(kv.second.hist_mode) was_hist.push_back(kv.first);
            kv.second.recv=false; kv.second.hist_loading=false; kv.second.hist_started=false;
            kv.second.hist_buf.clear(); kv.second.live_pending.clear();
            kv.second.targets.clear(); kv.second.masks.clear(); kv.second.pending.clear();
            kv.second.hist_mode=false; kv.second.hist_date[0]=0;
            kv.second.hist2_fetching=false; kv.second.hist2_active=false; kv.second.hist2_buf.clear(); kv.second.hist2_raw=0;
            kv.second.hlist_loading=false; kv.second.hlist.clear(); kv.second.hist_resub_skip=false;
        }
    }
    // Hist 모드에서 재접속/기지전환 → 대피(stash)된 라이브 log 복원 (안 하면 과거 데이터가 라이브로 오인 + 버퍼 유실)
    for(const std::string& id : was_hist){
        const BeweModule* m = find_mod(id.c_str());
        if(m && m->log_restore) m->log_restore();
    }
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

uint64_t bewe_mod_host_mask(const char* id){
    std::lock_guard<std::mutex> lk(g_fw_mtx);
    return fw(id).host_mask;
}

void bewe_mod_host_announce(FFTViewer& v){
    (void)v;
    for(auto& m : reg()) if(m.target_modes) host_send_state(m.id);
}

void bewe_mod_host_mask_clear(FFTViewer& v, const char* id, int ch){
    (void)v;
    { std::lock_guard<std::mutex> lk(g_fw_mtx); fw(id).host_mask &= ~(1ull<<ch); }
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
        if(on){ fw(m.id).want_mask |= (1ull<<ch); if(ok) fw(m.id).host_mask |= (1ull<<ch); }  // 의도는 시작성공 무관
        else  { fw(m.id).want_mask &= ~(1ull<<ch); fw(m.id).host_mask &= ~(1ull<<ch); }
    }
    if(on && ok) host_decstat_start(m.id, ch);   // decode 시작: 런타임 시각 + 카운트 리셋
    if(!on)      host_decstat_stop(ch);
    host_send_state(m.id);
}

// ── 복조 의도(want) ↔ 실제동작(host_mask) 재조정 ─────────────────────────────
// 필터 깜빡임/SDR 스트림에러로 워커가 죽어 host_mask 가 비어도, 사용자가 켠 want 가 남으면
// 채널이 다시 active 가 됐을 때 자동 재시작. HOST 주기 호출(cli_host/ui). SDR 에러 중엔 보류.
// ── HOST: 전일 JSONL 아카이브 자동 push (00시 KST 넘으면 어제 파일 → Central) ──
// 미션 on/off 무관 자동. 지난 7일 중 미push 파일도 회수 (기지 다운 후 복구 대비).
// 완료 마커 = <파일>.pushed (fire-and-forget; Central 다운 중 push 는 손실 가능 — 마커 지우면 재push).
static std::atomic<bool> g_arch_pushing{false};
static void host_arch_push_file(std::string id, std::string path, std::string date8){
    std::string body;
    { FILE* f=fopen(path.c_str(),"rb"); if(!f){ g_arch_pushing=false; return; }
      char b[8192]; size_t n; while((n=fread(b,1,sizeof(b),f))>0) body.append(b,n); fclose(f); }
    // zlib 압축 (JSONL 반복 텍스트 — 5~10× 흔함). 실패/팽창 시 비압축(raw=0).
    std::string wire = body; uint32_t raw = 0;
    if(!body.empty()){
        uLongf bound = compressBound((uLong)body.size());
        std::string z; z.resize(bound);
        if(compress2((Bytef*)z.data(), &bound, (const Bytef*)body.data(),
                     (uLong)body.size(), Z_BEST_SPEED)==Z_OK && bound < body.size()){
            z.resize(bound); wire.swap(z); raw=(uint32_t)body.size();
        }
    }
    MpArchMeta meta{}; memcpy(meta.date, date8.c_str(), 8);
    meta.total_bytes=(uint32_t)wire.size(); meta.raw_bytes=raw;
    if(!bcast(id.c_str(), BEWE_MK_ARCH_META, &meta, sizeof(meta))){ g_arch_pushing=false; return; }
    constexpr size_t CHUNK = 8192;
    for(size_t off=0; off<wire.size(); off+=CHUNK){
        size_t len = wire.size()-off; if(len>CHUNK) len=CHUNK;
        if(!bcast(id.c_str(), BEWE_MK_ARCH_CHUNK, wire.data()+off, len)){ g_arch_pushing=false; return; }
        usleep(10000);   // ~800KB/s — FFT 스트림/송신큐 보호
    }
    bcast(id.c_str(), BEWE_MK_ARCH_DONE, nullptr, 0);
    FILE* mk=fopen((path+".pushed").c_str(),"wb");
    if(mk){ fputs(date8.c_str(), mk); fclose(mk); }
    g_arch_pushing=false;
}
static void host_arch_push_check(){
    if(!g_broadcast) return;                          // HOST 송신로 없으면 스킵
    static int64_t last=0; int64_t now=mod_now_ms();
    if(now-last < 60000) return; last=now;            // 1분 주기
    if(g_arch_pushing.load()) return;                 // 한 번에 한 파일
    const char* home = getenv("HOME");
    std::string base = home ? std::string(home) : std::string(".");
    time_t tnow = time(nullptr);
    for(int back=1; back<=7; back++){                 // 어제~7일 전 (오늘 제외 — 파일 진행 중)
        struct tm tv{}; KST::to_tm(tnow - (time_t)back*86400, tv);
        char d[9]; snprintf(d,sizeof(d),"%04d%02d%02d",tv.tm_year+1900,tv.tm_mon+1,tv.tm_mday);
        for(auto& m : reg()){
            if(!m.target_modes) continue;
            std::string path = base + "/BE_WE/modules/" + m.id + "/" + m.id + "_" + d + ".jsonl";
            FILE* f=fopen(path.c_str(),"rb"); if(!f) continue; fclose(f);
            FILE* mk=fopen((path+".pushed").c_str(),"rb"); if(mk){ fclose(mk); continue; }
            g_arch_pushing=true;
            std::thread(host_arch_push_file, std::string(m.id), path, std::string(d)).detach();
            return;                                    // 이번 틱 1파일 — 다음 틱에 다음 파일
        }
    }
}

void bewe_mod_reconcile(FFTViewer& v){
    host_arch_push_check();                            // 00시 롤오버/미push 회수 (SDR 에러와 무관)
    if(v.sdr_stream_error.load()) return;
    for(auto& m : reg()){
        if(!m.target_modes) continue;
        uint64_t need;
        { std::lock_guard<std::mutex> lk(g_fw_mtx); need = fw(m.id).want_mask & ~fw(m.id).host_mask; }
        if(!need) continue;
        for(int ch=0; ch<MAX_CHANNELS; ch++){
            if(!((need>>ch)&1)) continue;
            if(!v.channels[ch].filter_active) continue;   // 채널 아직 비활성 → 다음 틱 대기
            host_apply_set(v, m, ch, true);               // 재장전(start+mask+stat+state)
        }
    }
}
// 채널 진짜 삭제/정지(stop_dem stop_decoders) 시 그 ch 의 모든 모듈 want 해제 — 깜빡임과 구분.
void bewe_mod_want_clear_ch(int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    std::lock_guard<std::mutex> lk(g_fw_mtx);
    for(auto& kv : g_fw) kv.second.want_mask &= ~(1ull<<ch);
}

// ── 온디맨드 녹음(WAV) 페치 ──────────────────────────────────────────────────
// JOIN: 그 기지 HOST 에 rec_id WAV 요청 (Central 이 SET 처럼 라우팅).
void bewe_mod_rec_request(const char* id, const char* station, int ch, uint64_t rec_id){
    MpRecReq r{}; strncpy(r.station, station, sizeof(r.station)-1);
    r.ch=(uint8_t)ch; r.rec_id=rec_id;
    send_up(id, BEWE_MK_REC_REQ, &r, sizeof(r));
}
// HOST: WAV 바이트 청크 회신 (Central 이 구독 JOIN 으로 팬아웃).
void bewe_mod_rec_send(const char* id, uint64_t rec_id, uint32_t total, uint32_t off, const void* b, uint32_t n){
    std::vector<uint8_t> buf(sizeof(MpRecData)+n);
    auto* h=reinterpret_cast<MpRecData*>(buf.data());
    h->rec_id=rec_id; h->total=total; h->offset=off; h->n=n;
    if(n) memcpy(buf.data()+sizeof(MpRecData), b, n);
    bcast(id, BEWE_MK_REC_DATA, buf.data(), buf.size());
}

// ── CH_EDIT 로컬 적용 (HOST): geometry/mode 변경 + 디코더/오디오데모드 재시작 + 동기화 ──
// 동작 중 디코더 정지 → s/e/mode 변경 → 오디오 데모드 재시작 → CHANNEL_SYNC 브로드캐스트
// (Central 캐시 → 전 JOIN 의 다음 CH_LIST 폴링에 새 geometry 반영) → 디코더 새 band 재시작.
static void apply_ch_edit_local(FFTViewer& v, int ch, int mode, float lo, float hi){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    if(!v.channels[ch].filter_active) return;
    if(lo>hi){ float t=lo; lo=hi; hi=t; }
    if(hi-lo < 0.001f) hi = lo + 0.001f;                  // 최소 폭 1 kHz 가드
    std::vector<const BeweModule*> wasrun;
    for(auto& mm : reg())
        if(mm.target_modes && ((bewe_mod_host_mask(mm.id)>>ch)&1)) wasrun.push_back(&mm);
    for(auto* mm : wasrun) host_apply_set(v, *mm, ch, false);   // 디코더 정지 (mask off)
    v.channels[ch].s = lo; v.channels[ch].e = hi;
    if(mode>=0 && mode<=2){
        auto dm=(Channel::DemodMode)mode;
        if((int)v.channels[ch].mode != mode){
            v.stop_dem(ch); v.channels[ch].mode = dm;
            if(dm!=Channel::DM_NONE && v.channels[ch].filter_active) v.start_dem(ch, dm);
        } else if(v.channels[ch].dem_run.load()){ v.stop_dem(ch); v.start_dem(ch, dm); }
    } else if(v.channels[ch].dem_run.load()){
        Channel::DemodMode md=v.channels[ch].mode; v.stop_dem(ch); v.start_dem(ch, md);
    }
    v.update_dem_by_freq(v.header.center_frequency/1e6f);
    if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    for(auto* mm : wasrun) host_apply_set(v, *mm, ch, true);    // 새 band 로 재시작
}

// 새 채널 생성 (LOCAL/HOST) — 빈 슬롯 선택 후 lo/hi/mode 적용. on_create_ch 미러.
static void apply_ch_add_local(FFTViewer& v, int mode, float lo, float hi){
    if(lo>hi){ float t=lo; lo=hi; hi=t; }
    if(hi-lo < 0.001f) hi = lo + 0.001f;                       // 최소 폭 1 kHz 가드
    int slot=-1;
    for(int i=0;i<MAX_CHANNELS;i++) if(!v.channels[i].filter_active){ slot=i; break; }
    if(slot<0) return;                                          // 빈 슬롯 없음
    v.stop_dem(slot);
    v.channels[slot].reset_slot();
    v.channels[slot].s=lo; v.channels[slot].e=hi;
    v.channels[slot].filter_active=true;
    strncpy(v.channels[slot].owner, login_get_id(), 31);
    v.channels[slot].audio_mask.store(0xFFFFFFFFu & ~0x1u);
    v.local_ch_out[slot] = 3;
    v.update_dem_by_freq(v.header.center_frequency/1e6f);
    if(mode>=1 && mode<=2){                                     // AM/FM 만 디코더 기동
        auto dm=(Channel::DemodMode)mode;
        v.channels[slot].mode = dm;
        if(v.channels[slot].filter_active) v.start_dem(slot, dm);
    }
    if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
}

// 채널 삭제 (LOCAL/HOST) — on_delete_ch 미러.
static void apply_ch_del_local(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    if(!v.channels[ch].filter_active) return;
    if(v.channels[ch].audio_rec_on.load()) v.stop_audio_rec(ch);
    std::vector<const BeweModule*> wasrun;
    for(auto& mm : reg())
        if(mm.target_modes && ((bewe_mod_host_mask(mm.id)>>ch)&1)) wasrun.push_back(&mm);
    for(auto* mm : wasrun) host_apply_set(v, *mm, ch, false);   // 디코더 정지 (mask off)
    v.stop_dem(ch);
    v.channels[ch].reset_slot();
    v.local_ch_out[ch] = 1;
    if(v.net_srv) v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
}

// ── 채널별 디코드 레이트 통계 (id|station|ch → 최근 60s 타임스탬프 deque) ──
static std::mutex g_stat_mtx;
static std::map<std::string, std::deque<int64_t>> g_stat;
static std::map<std::string, long> g_stat_total;   // 누적 수신 건수 (트림 안 함 — Data 컬럼)
static std::string stat_key(const char* id, const char* station, int ch){
    const char* s = (station && station[0]) ? station : "LOCAL";   // 빈 station = LOCAL (타깃표와 일치)
    std::string k = id; k += '|'; k += s; k += '|'; k += std::to_string(ch);
    return k;
}
static void stat_rollover_locked();                // 아래 정의 (KST 자정 통계 리셋)
void bewe_mod_stat_bump(const char* id, const char* station, int ch, int64_t t_ms){
    if(!id) return;
    std::lock_guard<std::mutex> lk(g_stat_mtx);
    stat_rollover_locked();                        // KST 자정 넘으면 통계 0
    std::string k = stat_key(id, station, ch);
    auto& dq = g_stat[k];
    dq.push_back(t_ms);
    int64_t cut = t_ms - 60000;
    while(!dq.empty() && dq.front() < cut) dq.pop_front();
    if(dq.size() > 4096) dq.pop_front();           // 폭주 가드
    g_stat_total[k]++;                              // 누적 (Data 컬럼)
}
// ── HOST 디코드 통계: decode 동작 시각(steady ms) + 누적건수 ──
// runtime = active(Active=가시대역 안) 구간만 누적. Holding(범위밖) 중엔 freeze.
//   g_host_decstart>0: 현재 active 구간 시작 steady-ms. accum: 직전 구간들 합(ms).
static int64_t g_host_decstart[MAX_CHANNELS] = {};   // 0=정지/Holding, >0=현 active 구간 시작 steady-ms
static int64_t g_host_decaccum[MAX_CHANNELS] = {};   // 동결된 누적 active 시간(ms)
// KST 자정 리셋: 날짜(yyyymmdd) 바뀌면 msg/runtime 0 으로. -1=미초기화.
static int      g_stat_day = -1;
static int kst_day_now(){
    int64_t now = (int64_t)std::chrono::duration_cast<std::chrono::seconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();
    struct tm tmv{}; KST::to_tm((time_t)now, tmv);
    return (tmv.tm_year+1900)*10000 + (tmv.tm_mon+1)*100 + tmv.tm_mday;
}
// 자정 경계 감지 → count(g_stat_total) + runtime(accum/start) 전 채널 0. g_stat_mtx 잡고 호출.
static void stat_rollover_locked(){
    int d = kst_day_now();
    if(g_stat_day == d) return;
    g_stat_day = d;
    g_stat_total.clear();
    for(int i=0;i<MAX_CHANNELS;i++){
        g_host_decaccum[i] = 0;
        if(g_host_decstart[i] > 0) g_host_decstart[i] = mod_now_ms();   // active 채널은 지금부터 재시작
    }
}
// 오늘(KST) 저장 JSONL 에서 채널 ch 디코드 건수 — Data 시드(히스토리 포함 총수).
// store_append 은 emit(=stat_bump) 과 1:1 이라 JSONL 라인수 = 누적 디코드수.
static long host_today_count(const char* id, int ch){
    const char* home = getenv("HOME");
    std::string base = home ? std::string(home) : std::string(".");
    int64_t now = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch()).count();
    struct tm tmv{}; KST::to_tm((time_t)(now/1000), tmv);
    char d[9]; snprintf(d,sizeof(d),"%04d%02d%02d",tmv.tm_year+1900,tmv.tm_mon+1,tmv.tm_mday);
    std::string path = base + "/BE_WE/modules/" + id + "/" + id + "_" + d + ".jsonl";
    FILE* f = fopen(path.c_str(),"rb"); if(!f) return 0;
    long cnt=0; char line[1024];
    while(fgets(line,sizeof(line),f)){
        const char* p = strstr(line,"\"ch\":");
        if(p && atoi(p+5)==ch) cnt++;
    }
    fclose(f); return cnt;
}
// 오늘 jsonl 을 1회 파싱해 전 채널 카운트 배열로 캐시 (30초 TTL, 모듈 id 별).
//  decstat 폴링이 채널마다 파일을 재파싱하지 않도록 — 파일 1회 read 로 ch0~ 전부.
static long host_today_count_cached(const char* id, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return 0;
    static std::mutex m;
    static std::map<std::string, std::pair<int64_t,std::array<long,MAX_CHANNELS>>> cache;
    std::lock_guard<std::mutex> lk(m);
    int64_t now = mod_now_ms();
    auto& e = cache[id];
    if(now - e.first >= 30000 || e.first==0){
        e.first = now; e.second.fill(0);
        const char* home = getenv("HOME");
        std::string base = home ? std::string(home) : std::string(".");
        int64_t s = (int64_t)std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
        struct tm tmv{}; KST::to_tm((time_t)s, tmv);
        char d[9]; snprintf(d,sizeof(d),"%04d%02d%02d",tmv.tm_year+1900,tmv.tm_mon+1,tmv.tm_mday);
        std::string path = base + "/BE_WE/modules/" + id + "/" + id + "_" + d + ".jsonl";
        FILE* f = fopen(path.c_str(),"rb");
        if(f){ char line[1024];
            while(fgets(line,sizeof(line),f)){
                const char* p = strstr(line,"\"ch\":");
                if(p){ int c=atoi(p+5); if(c>=0 && c<MAX_CHANNELS) e.second[c]++; }
            }
            fclose(f);
        }
    }
    return e.second[ch];
}
static void host_decstat_start(const char* id, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    g_host_decstart[ch] = mod_now_ms();
    g_host_decaccum[ch] = 0;                          // 새 decode 세션 → runtime 0 부터
    long seed = host_today_count(id, ch);             // 오늘 누적(히스토리)으로 시드
    std::lock_guard<std::mutex> lk(g_stat_mtx);
    g_stat_total[stat_key(id, g_my_station, ch)] = seed;
}
static void host_decstat_stop(int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    g_host_decstart[ch] = 0;
    g_host_decaccum[ch] = 0;
}
// HOST: Holding 진입(true)/이탈(false) 시 runtime 누적 freeze/resume. dmr_decode 워커가 전환 edge 에서 호출.
//   Holding 진입: 흐르던 active 구간을 accum 에 적립 후 start=0(정지). 이탈: start 재설정(이어서 누적).
void bewe_mod_host_ch_hold(int ch, bool holding){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    if(holding){
        int64_t st = g_host_decstart[ch];
        if(st>0){ g_host_decaccum[ch] += mod_now_ms()-st; g_host_decstart[ch] = 0; }
    } else {
        if(g_host_decstart[ch]==0) g_host_decstart[ch] = mod_now_ms();
    }
}
// HOST: 채널 ch 에서 도는 디코더의 (누적건수, 동작경과초). net_server 가 ChSyncEntry 채울 때 사용.
//   runtime = accum + (active 면 현 구간). Holding 중엔 start=0 이라 freeze.
void bewe_mod_host_ch_decstat(int ch, uint32_t& count, uint32_t& runtime_s){
    count = 0; runtime_s = 0;
    if(ch<0 || ch>=MAX_CHANNELS) return;
    { std::lock_guard<std::mutex> lk(g_stat_mtx); stat_rollover_locked(); }   // 폴링도 자정 인지
    int64_t st = g_host_decstart[ch];
    int64_t ms = g_host_decaccum[ch] + (st>0 ? mod_now_ms()-st : 0);
    if(ms>0) runtime_s = (uint32_t)(ms/1000);
    // 이 채널서 도는 모듈 찾아 누적건수 조회 (한 채널=한 디코더)
    std::string id;
    { std::lock_guard<std::mutex> lk(g_fw_mtx);
      for(auto& kv : g_fw) if((kv.second.host_mask>>ch)&1){ id=kv.first; break; } }
    if(id.empty()) return;
    // 라이브 누적(stat_total: 켠 뒤 emit) 과 오늘 저장본(jsonl 총수=DMR 뷰 누적) 중 큰 값.
    //  → reconcile 재장전/자정 직후처럼 stat_total 이 비어도 화면 Data 가 뷰와 일치.
    //  jsonl 은 채널별 30초 캐시 (폴링마다 파일 파싱 방지).
    long live = 0;
    { std::lock_guard<std::mutex> lk(g_stat_mtx);
      auto it = g_stat_total.find(stat_key(id.c_str(), g_my_station, ch));
      if(it!=g_stat_total.end() && it->second>0) live = it->second; }
    long today = host_today_count_cached(id.c_str(), ch);
    long c = live>today ? live : today;
    count = (uint32_t)(c<0?0:c);
}
void bewe_mod_ch_stat(const char* id, const char* station, int ch, int64_t now_ms,
                      int& cnt60, int64_t& last_ms){
    cnt60 = 0; last_ms = 0;
    if(!id) return;
    std::lock_guard<std::mutex> lk(g_stat_mtx);
    auto it = g_stat.find(stat_key(id, station, ch));
    if(it == g_stat.end()) return;
    auto& dq = it->second;
    int64_t cut = now_ms - 60000;
    while(!dq.empty() && dq.front() < cut) dq.pop_front();
    cnt60 = (int)dq.size();
    if(!dq.empty()) last_ms = dq.back();
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
void bewe_mod_req_vessel(const char* id, uint32_t key){
    MpVHistReq r{ key };
    send_up(id, BEWE_MK_VHIST_REQ, &r, sizeof(r));
}

// ── 과거 데이터 조회 (Hist) ─────────────────────────────────────────────────
bool bewe_mod_hist_supported(const char* id){
    const BeweModule* m = find_mod(id);
    return m && m->log_stash && m->log_restore && m->on_hist_file;
}
void bewe_mod_hist_list_req(const char* id){
    { std::lock_guard<std::mutex> lk(g_fw_mtx); fw(id).hlist_loading=true; fw(id).hlist.clear(); }
    if(!send_up(id, BEWE_MK_HLIST_REQ, nullptr, 0)){
        std::lock_guard<std::mutex> lk(g_fw_mtx); fw(id).hlist_loading=false;   // LOCAL: 목록 없음
    }
}
std::vector<BeweHistDate> bewe_mod_hist_dates(const char* id){
    std::lock_guard<std::mutex> lk(g_fw_mtx); return fw(id).hlist;
}
bool bewe_mod_hist_list_loading(const char* id){
    std::lock_guard<std::mutex> lk(g_fw_mtx); return fw(id).hlist_loading;
}
void bewe_mod_hist_fetch(FFTViewer& v, const char* id, const char* date8){
    const BeweModule* m = find_mod(id);
    if(!m || !m->log_stash || !date8 || strlen(date8)<8) return;
    bool already;
    { std::lock_guard<std::mutex> lk(g_fw_mtx); already = fw(id).hist_mode; }
    // 구독 중이면 OFF (라이브 수신 중단; clear 없음 — stash 가 대피)
    if(bewe_mod_recv(id)) bewe_mod_set_recv(v, id, false);
    if(already){
        // 날짜 전환: restore(버퍼→log) 후 재-stash = 현 과거 데이터만 버리고 버퍼 보존
        if(m->log_restore) m->log_restore();
    }
    m->log_stash();
    uint32_t gen;
    {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        f.hist_mode=true; memcpy(f.hist_date, date8, 8); f.hist_date[8]=0;
        gen = ++f.hist2_gen;                 // 새 세대 — 이전 날짜의 늦게 온 스트림은 폐기됨
        f.hist2_fetching=true; f.hist2_active=false; f.hist2_buf.clear(); f.hist2_raw=0;
    }
    MpHFetch q{}; memcpy(q.date, date8, 8); q.gen = gen;
    if(!send_up(id, BEWE_MK_HFETCH, &q, sizeof(q))){
        std::lock_guard<std::mutex> lk(g_fw_mtx); fw(id).hist2_fetching=false;
    }
}
void bewe_mod_hist_exit(FFTViewer& v, const char* id){
    const BeweModule* m = find_mod(id);
    if(!m) return;
    {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        if(!f.hist_mode) return;
        f.hist_mode=false; f.hist_date[0]=0;
        ++f.hist2_gen;                        // 진행 중 과거 스트림 무효화
        f.hist2_fetching=false; f.hist2_active=false; f.hist2_buf.clear(); f.hist2_raw=0;
        f.hist_resub_skip=true;               // 곧 오는 재구독 히스토리(kind0)는 버림 — 복원 log 와 중복 방지
    }
    if(m->log_restore) m->log_restore();    // 과거 데이터 폐기 + 라이브 버퍼 복원
    bewe_mod_set_recv(v, id, true);         // 재구독 (라이브만 재개; 오늘 히스토리는 resub_skip 으로 폐기)
}
bool bewe_mod_hist_mode(const char* id, char* date8_out){
    std::lock_guard<std::mutex> lk(g_fw_mtx);
    ModFw& f = fw(id);
    if(date8_out){ memcpy(date8_out, f.hist_date, 9); }
    return f.hist_mode;
}
bool bewe_mod_hist_fetching(const char* id){
    std::lock_guard<std::mutex> lk(g_fw_mtx); return fw(id).hist2_fetching;
}

std::vector<MpChEntry> bewe_mod_targets(FFTViewer& v, const char* id){
    if(g_send_up && v.remote_mode){
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        return fw(id).targets;   // Central 집계본
    }
    // LOCAL / HOST GUI: 로컬 채널에서 직접 구성
    std::vector<MpChEntry> out;
    uint64_t mask;
    { std::lock_guard<std::mutex> lk(g_fw_mtx); mask = fw(id).host_mask; }
    float cf_mhz = (float)(v.header.center_frequency / 1e6);   // 기지 하드웨어 튜닝 (표시/편집용)
    float sr_msps = (float)(v.header.sample_rate / 1e6);
    for(int i=0;i<MAX_CHANNELS;i++){
        Channel& ch = v.channels[i];
        if(!ch.filter_active) continue;
        MpChEntry e{};
        strncpy(e.station, g_my_station[0]?g_my_station:"LOCAL", sizeof(e.station)-1);
        e.ch = (uint8_t)i; e.mode = (uint8_t)ch.mode;
        e.decode_on = ((mask>>i)&1) ? 1 : 0;
        e.hold = ch.dem_paused.load() ? 1 : 0;
        e.dnum = (uint8_t)v.freq_sorted_display_num(i);   // State창/스펙트럼 라벨과 동일 번호
        e.lo = ch.s; e.hi = ch.e;
        e.cf_mhz = cf_mhz; e.sr_msps = sr_msps;
        { uint32_t dc=0,dr=0; bewe_mod_host_ch_decstat(i,dc,dr); e.dec_count=dc; e.dec_runtime_s=dr; }  // HOST GUI: 자기 측정 통계
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
        // 낙관적 갱신: STATE 왕복(JOIN→Central→HOST→STATE) 전 즉시 [RUN] 반영.
        // 권위 STATE 가 곧 도착해 확정/정정 → HOST 와 동일한 즉각 반응감.
        {
            std::lock_guard<std::mutex> lk(g_fw_mtx);
            for(auto& t : fw(id).targets)
                if(t.ch==ch && strncmp(t.station, station, sizeof(t.station))==0)
                    t.decode_on = on?1:0;
            // in-flight 표시 → 다음 폴링 CH_LIST 가 옛 값으로 되돌리지 못하게
            fw(id).pending[{std::string(station), ch}] = { (uint8_t)(on?1:0), mod_now_ms() };
        }
        return;
    }
    // LOCAL / HOST GUI: 직접 적용 (+상태 브로드캐스트는 host_apply_set 내부)
    host_apply_set(v, *m, ch, on);
}

// 채널 geometry/mode 변경 (어느 기지든). 원격은 Central→해당 HOST, LOCAL/HOST 는 즉시.
void bewe_mod_edit_ch(FFTViewer& v, const char* station, int ch, int mode, float lo, float hi){
    if(g_send_up && v.remote_mode){
        MpChEdit e{}; strncpy(e.station, station?station:"", sizeof(e.station)-1);
        e.ch=(uint8_t)ch; e.mode=(uint8_t)(mode&0xFF); e.lo=lo; e.hi=hi;
        send_up("*", BEWE_MK_CH_EDIT, &e, sizeof(e));   // mod_id 무관 (채널 op)
        return;
    }
    apply_ch_edit_local(v, ch, mode, lo, hi);
}

// 채널 생성 (어느 기지든). 원격은 Central→해당 HOST 가 빈 슬롯 선택, LOCAL/HOST 는 즉시.
void bewe_mod_add_ch(FFTViewer& v, const char* station, int mode, float lo, float hi){
    if(g_send_up && v.remote_mode){
        MpChEdit e{}; strncpy(e.station, station?station:"", sizeof(e.station)-1);
        e.ch=0; e.mode=(uint8_t)(mode&0xFF); e.lo=lo; e.hi=hi;   // ch 무시 (HOST 가 빈 슬롯 결정)
        send_up("*", BEWE_MK_CH_ADD, &e, sizeof(e));
        return;
    }
    apply_ch_add_local(v, mode, lo, hi);
}

// 채널 삭제 (어느 기지든). 원격은 Central→해당 HOST, LOCAL/HOST 는 즉시.
void bewe_mod_del_ch(FFTViewer& v, const char* station, int ch){
    if(g_send_up && v.remote_mode){
        MpChEdit e{}; strncpy(e.station, station?station:"", sizeof(e.station)-1);
        e.ch=(uint8_t)ch; e.mode=0; e.lo=0; e.hi=0;
        send_up("*", BEWE_MK_CH_DEL, &e, sizeof(e));
        return;
    }
    apply_ch_del_local(v, ch);
}

// HOST: 채널 geometry/SR 변경 → 그 채널서 도는 디코더만 새 band 로 재시작 (mask 유지).
// 디코더 워커는 시작 시 ch.s/e/bw/SR 을 고정 캡처하므로, 변경 반영엔 워커 재시작이 필요.
// audio 모드(NONE/AM/FM) 무관 — 디코더는 IQ 직접 탭. 채널에 디코더 없으면 no-op.
void bewe_mod_ch_retune(FFTViewer& v, int ch){
    if(ch<0 || ch>=MAX_CHANNELS) return;
    for(auto& mm : reg()){
        if(!mm.target_modes) continue;
        if(!((bewe_mod_host_mask(mm.id)>>ch)&1)) continue;   // 이 채널서 도는 디코더만
        if(mm.host_stop)  mm.host_stop(v, ch);               // 옛 band 워커 종료(join)
        if(mm.host_start) mm.host_start(v, ch);              // 새 geometry/SR 로 재시작, mask 유지
    }
}

// 기지 하드웨어 CF/SR 적용 (HOST 로컬). sr 먼저(재초기화) → cf. 0 인 필드는 건너뜀.
static void apply_tune_local(FFTViewer& v, float cf_mhz, float sr_msps){
    if(sr_msps > 0.f){ v.pending_sr_msps = sr_msps; v.sr_change_req = true; }
    if(cf_mhz  > 0.f){ v.set_frequency(cf_mhz); }
}
// 기지 CF/SR 변경 (어느 기지든). 원격은 Central→해당 HOST, LOCAL/HOST 는 즉시.
void bewe_mod_tune(FFTViewer& v, const char* station, float cf_mhz, float sr_msps){
    if(g_send_up && v.remote_mode){
        MpTune t{}; strncpy(t.station, station?station:"", sizeof(t.station)-1);
        t.cf_mhz=cf_mhz; t.sr_msps=sr_msps;
        send_up("*", BEWE_MK_TUNE, &t, sizeof(t));   // mod_id 무관 (기지 op)
        return;
    }
    apply_tune_local(v, cf_mhz, sr_msps);
}

// ── 수신 라우팅 ─────────────────────────────────────────────────────────────
void bewe_mod_route(FFTViewer& v, bool host_side, const uint8_t* payload, size_t len){
    if(len < sizeof(PktModulePipe)) return;
    auto* h = reinterpret_cast<const PktModulePipe*>(payload);
    if(sizeof(PktModulePipe) + h->data_len > len) return;
    char id[9]={}; memcpy(id, h->mod_id, 8);
    const uint8_t* d = payload + sizeof(PktModulePipe);
    size_t n = h->data_len;
    // CH_EDIT 는 모듈 무관(채널 op) — find_mod 전에 처리
    if(host_side && h->kind==BEWE_MK_CH_EDIT && n>=sizeof(MpChEdit)){
        auto* e=reinterpret_cast<const MpChEdit*>(d);
        char stn[25]={}; memcpy(stn, e->station, 24);
        if(g_my_station[0] && strncmp(stn, g_my_station, 24)!=0) return;  // 다른 기지 명령
        apply_ch_edit_local(v, e->ch, e->mode, e->lo, e->hi);
        return;
    }
    // CH_ADD / CH_DEL 도 모듈 무관(채널 op) — find_mod 전에 처리
    if(host_side && h->kind==BEWE_MK_CH_ADD && n>=sizeof(MpChEdit)){
        auto* e=reinterpret_cast<const MpChEdit*>(d);
        char stn[25]={}; memcpy(stn, e->station, 24);
        if(g_my_station[0] && strncmp(stn, g_my_station, 24)!=0) return;  // 다른 기지 명령
        apply_ch_add_local(v, e->mode, e->lo, e->hi);
        return;
    }
    if(host_side && h->kind==BEWE_MK_CH_DEL && n>=sizeof(MpChEdit)){
        auto* e=reinterpret_cast<const MpChEdit*>(d);
        char stn[25]={}; memcpy(stn, e->station, 24);
        if(g_my_station[0] && strncmp(stn, g_my_station, 24)!=0) return;  // 다른 기지 명령
        apply_ch_del_local(v, e->ch);
        return;
    }
    // TUNE 도 모듈 무관(기지 op) — find_mod 전에 처리
    if(host_side && h->kind==BEWE_MK_TUNE && n>=sizeof(MpTune)){
        auto* t=reinterpret_cast<const MpTune*>(d);
        char stn[25]={}; memcpy(stn, t->station, 24);
        if(g_my_station[0] && strncmp(stn, g_my_station, 24)!=0) return;  // 다른 기지 명령
        apply_tune_local(v, t->cf_mhz, t->sr_msps);
        return;
    }
    const BeweModule* m = find_mod(id);
    if(!m) return;   // 미설치 모듈 → 조용히 무시

    if(host_side){
        // HOST: Central 이 라우팅한 SET 만 처리
        if(h->kind == BEWE_MK_SET && n >= sizeof(MpSet)){
            auto* st = reinterpret_cast<const MpSet*>(d);
            char stn[25]={}; memcpy(stn, st->station, 24);
            if(g_my_station[0] && strncmp(stn, g_my_station, 24)!=0) return; // 다른 기지 명령
            host_apply_set(v, *m, st->ch, st->on!=0);
        } else if(h->kind == BEWE_MK_REC_REQ && n >= sizeof(MpRecReq)){     // 녹음 WAV 요청
            auto* r = reinterpret_cast<const MpRecReq*>(d);
            char stn[25]={}; memcpy(stn, r->station, 24);
            if(g_my_station[0] && strncmp(stn, g_my_station, 24)!=0) return;
            if(m->on_rec_req) m->on_rec_req(v, r->ch, r->rec_id);
        }
        return;
    }

    // JOIN/뷰어 측
    switch(h->kind){
    case BEWE_MK_REC_DATA: {                          // 녹음 WAV 청크 회신
        if(n < sizeof(MpRecData)) break;
        auto* r = reinterpret_cast<const MpRecData*>(d);
        if(sizeof(MpRecData)+r->n > n) break;
        if(m->on_rec_data) m->on_rec_data(v, r->rec_id, r->total, r->offset,
                                          d+sizeof(MpRecData), r->n);
        break;
    }
    case BEWE_MK_CH_LIST: {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        f.targets.clear();
        size_t cnt = n / sizeof(MpChEntry);
        auto* e = reinterpret_cast<const MpChEntry*>(d);
        for(size_t i=0;i<cnt;i++) f.targets.push_back(e[i]);
        // 라이브 STATE(푸시) 우선: 폴링 스냅샷보다 신선한 권위 mask 로 decode_on 덮어씀
        // (STATE 가 첫 CH_LIST 전에 와도 여기서 반영 — H4 누락 방지).
        for(auto& t : f.targets){
            auto mit = f.masks.find(std::string(t.station, strnlen(t.station, sizeof(t.station))));
            if(mit != f.masks.end()) t.decode_on = ((mit->second >> t.ch) & 1) ? 1 : 0;
        }
        // 진행 중 낙관적 SET 우선: 왕복 완료(STATE) 또는 만료(4s) 전까지 유지 → 버튼 깜빡임 제거
        int64_t tn = mod_now_ms();
        for(auto& t : f.targets){
            auto pit = f.pending.find({std::string(t.station, strnlen(t.station, sizeof(t.station))), (int)t.ch});
            if(pit == f.pending.end()) continue;
            if(tn - pit->second.second > 4000) f.pending.erase(pit);
            else t.decode_on = pit->second.first;
        }
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
        // 권위 STATE 도착 → 이 기지 낙관적 pending 확정/제거
        for(auto it=f.pending.begin(); it!=f.pending.end(); ){
            if(it->first.first == std::string(stn)) it = f.pending.erase(it);
            else ++it;
        }
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
        auto* hm = reinterpret_cast<const MpHistMeta*>(d);
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        if(hm->stream_kind == 1){                 // 단일 대상 온디맨드 이력 (구독과 독립)
            f.vhist_active = true; f.vhist_raw = hm->raw_bytes; f.vhist_buf.clear();
            break;
        }
        if(hm->stream_kind == 2){                 // 과거 날짜 아카이브 (HFETCH 응답)
            if(!f.hist_mode || hm->req_id != f.hist2_gen) break;  // 옛 날짜의 늦게 온(stale) 스트림 폐기
            f.hist2_active = true; f.hist2_raw = hm->raw_bytes; f.hist2_buf.clear();
            break;
        }
        if(!f.hist_loading) break;   // 구독 안 한 상태의 잔여 스트림 무시
        f.hist_total = hm->total_bytes;
        f.hist_raw   = hm->raw_bytes;
        f.hist_buf.clear(); f.hist_started = true;
        break;
    }
    case BEWE_MK_HIST_CHUNK: {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        if(f.hist2_active){ if(n) f.hist2_buf.append((const char*)d, n); break; }
        if(f.vhist_active){ if(n) f.vhist_buf.append((const char*)d, n); break; }
        if(f.hist_loading && f.hist_started && n) f.hist_buf.append((const char*)d, n);
        break;
    }
    case BEWE_MK_HLIST: {
        std::lock_guard<std::mutex> lk(g_fw_mtx);
        ModFw& f = fw(id);
        f.hlist.clear();
        size_t cnt = n / sizeof(MpHistDate);
        auto* e = reinterpret_cast<const MpHistDate*>(d);
        for(size_t i=0;i<cnt;i++){
            BeweHistDate b{}; memcpy(b.date, e[i].date, 8); b.date[8]=0;
            b.bytes=e[i].bytes; b.stations=e[i].stations;
            f.hlist.push_back(b);
        }
        f.hlist_loading = false;
        break;
    }
    case BEWE_MK_HIST_DONE: {
        std::string buf;
        std::vector<std::vector<uint8_t>> pending;
        uint32_t raw = 0;
        bool vessel = false, arch = false, skip_hist = false;
        {
            std::lock_guard<std::mutex> lk(g_fw_mtx);
            ModFw& f = fw(id);
            if(f.hist2_active){                     // 과거 날짜 아카이브 (Hist 모드 오버레이)
                arch = true; buf.swap(f.hist2_buf); raw = f.hist2_raw;
                f.hist2_active = false; f.hist2_raw = 0; f.hist2_fetching = false;
            } else if(f.vhist_active){              // 단일 대상 이력 — 즉시 append (라이브 병합 없음)
                vessel = true; buf.swap(f.vhist_buf); raw = f.vhist_raw;
                f.vhist_active = false; f.vhist_raw = 0;
            } else {
                if(!f.hist_loading || !f.hist_started) break;   // META 없이 온 잔여 DONE 무시
                buf.swap(f.hist_buf);
                pending.swap(f.live_pending);
                raw = f.hist_raw;
                f.hist_loading = false; f.hist_started = false; f.hist_raw = 0;
                skip_hist = f.hist_resub_skip; f.hist_resub_skip = false;   // Hist 종료 재구독: 히스토리 버림, 라이브만
            }
        }
        // raw>0 → buf 는 zlib 압축본. 풀어서 레코드 스트림 복원. 실패 시 빈 히스토리.
        if(raw > 0 && !buf.empty()){
            std::string out; out.resize(raw);
            uLongf dlen = raw;
            if(uncompress((Bytef*)out.data(), &dlen, (const Bytef*)buf.data(),
                          (uLong)buf.size()) == Z_OK && dlen == raw) buf.swap(out);
            else buf.clear();
        }
        // 과거 아카이브: [station 24B][u32 len][JSONL] 블록 반복 → 모듈이 파싱해 log 오버레이
        if(arch){
            if(m->on_hist_file){
                size_t off = 0;
                while(off + 28 <= buf.size()){
                    char stn[25]={}; memcpy(stn, buf.data()+off, 24);
                    uint32_t fl; memcpy(&fl, buf.data()+off+24, 4); off += 28;
                    if(off + fl > buf.size()) break;
                    m->on_hist_file(stn, buf.data()+off, fl);
                    off += fl;
                }
            }
            break;
        }
        // 단일 대상 전체 이력: 배치 훅으로 그 대상 레코드 통째 교체 (요약↔전체 중복 방지).
        if(vessel && m->on_vessel_hist){ m->on_vessel_hist(v, (const uint8_t*)buf.data(), buf.size()); break; }
        // 레코드 스트림: u32 len + (MpData + module payload) 반복
        auto dispatch = [&](const uint8_t* rec, size_t rl){
            if(rl < sizeof(MpData)) return;
            auto* md = reinterpret_cast<const MpData*>(rec);
            char stn[25]={}; memcpy(stn, md->station, 24);
            if(m->on_data) m->on_data(v, stn, rec+sizeof(MpData), rl-sizeof(MpData));
        };
        // skip_hist(Hist 종료 재구독): 오늘 히스토리는 복원된 log 와 중복 → 버림. 라이브(pending)만 반영.
        if(!skip_hist){
            size_t off = 0;
            while(off + 4 <= buf.size()){
                uint32_t rl; memcpy(&rl, buf.data()+off, 4); off += 4;
                if(off + rl > buf.size()) break;
                dispatch((const uint8_t*)buf.data()+off, rl);
                off += rl;
            }
        }
        // 히스토리 후 라이브 버퍼 합류 (모듈측 append dedup 이 경계 중복 제거)
        for(auto& r : pending) dispatch(r.data(), r.size());
        break;
    }
    default: break;
    }
}
