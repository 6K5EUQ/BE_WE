// ── Live constellation (성상도) — DM_CONST worker + SA LIVE feed ───────────
// 채널필터를 DM_CONST 모드로 전환하면 (i 키) 오디오 복조 대신 채널 full-BW baseband
// IQ 를 처리해 "성상도 데이터" (int8 IQ snapshot) 만 만든다.
//  - host-local viewer (const_mask bit0): eid_ch_i/q 로 직접 push (dev HOST=GUI).
//  - remote viewer (const_mask bit_i):    int8 양자화 후 send_const → Central → JOIN.
// dem 슬롯(dem_thr/dem_rp/dem_stop_req)을 재사용하므로 start_dem/stop_dem/update_dem_by_freq
// 무변경. dem_worker/iq_only_worker hot path 는 건드리지 않음.
#include "fft_viewer.hpp"
#include "net_server.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>

// ── 스트림 파라미터 ────────────────────────────────────────────────────────
static constexpr int   CONST_FPS         = 20;     // 시각 갱신 cadence (frames/sec)
static constexpr int   CONST_FRAME_SAMPS = 2048;   // 프레임당 최대 complex pair (대역폭 상한)
static constexpr float CONST_TARGET_MULT = 1.25f;  // target_sr = bw*1.25 (full BW, iq_only 와 동일)
static constexpr float CONST_MIN_SR      = 8000.f;

void FFTViewer::con_worker(int ch_idx){
    Channel& ch = channels[ch_idx];
    uint32_t msr = header.sample_rate;
    uint64_t init_cf = live_cf_hz.load(std::memory_order_acquire);
    float ch_cf_mhz = (ch.s + ch.e) * 0.5f;
    float bw_hz = fabsf(ch.e - ch.s) * 1e6f;
    float off_hz = (ch_cf_mhz - (float)(init_cf / 1e6f)) * 1e6f;

    // full-BW decimation (iq_only_worker 와 동일)
    float target_sr = bw_hz * CONST_TARGET_MULT;
    if(target_sr < CONST_MIN_SR) target_sr = CONST_MIN_SR;
    uint32_t decim = (uint32_t)ceilf((float)msr / target_sr);
    if(decim < 1) decim = 1;
    uint32_t actual_sr = msr / decim;
    ch.con_sr = actual_sr;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;

    // BW LPF cascade (4-stage IIR1) — pre-decim anti-alias
    float cn = (bw_hz * 0.5f) / (float)msr;
    if(cn > 0.45f) cn = 0.45f;
    IIR1 lpi[4], lpq[4];
    for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); }

    double acc_i=0, acc_q=0; int acc_cnt=0;
    const size_t MAX_LAG = (size_t)(msr * 0.08);
    const size_t BATCH   = std::max((size_t)4096, (size_t)decim * 256);

    // snapshot ring (최근 CONST_FRAME_SAMPS) + 고정 cadence emit
    const int FN = CONST_FRAME_SAMPS;
    std::vector<float> rb_i(FN), rb_q(FN);
    size_t rb_w = 0, rb_n = 0;
    uint32_t emit_interval = std::max(1u, actual_sr / (uint32_t)CONST_FPS);
    uint32_t since_emit = 0;
    std::vector<float>  oi, oq; oi.reserve(FN); oq.reserve(FN);
    std::vector<int8_t> q8;     q8.reserve((size_t)FN*2);

    bewe_log("CON[%d] start: cf=%.4fMHz off=%.0fHz bw=%.0fHz decim=%u sr=%u\n",
             ch_idx, ch_cf_mhz, off_hz, bw_hz, decim, actual_sr);

    auto do_emit = [&](){
        uint32_t cm = ch.const_mask.load(std::memory_order_relaxed);
        if(cm == 0 || rb_n == 0) return;
        size_t n = rb_n;
        size_t start = rb_w - n;
        oi.clear(); oq.clear();
        for(size_t k=0;k<n;k++){ size_t idx=(start+k)%(size_t)FN; oi.push_back(rb_i[idx]); oq.push_back(rb_q[idx]); }
        // host-local viewer (bit0) → eid_ch_i/q 직접 push
        if(cm & 0x1u)
            eid_live_push(ch_idx, actual_sr, oi.data(), oq.data(), (int)n);
        // remote viewers (bit_i, i>=1) → int8 양자화 후 send_const
        if(net_srv && (cm & ~0x1u)){
            float scale = 1e-6f;
            for(size_t k=0;k<n;k++){ float a=fabsf(oi[k]), b=fabsf(oq[k]); if(a>scale)scale=a; if(b>scale)scale=b; }
            float inv = 127.0f / scale;
            q8.clear();
            for(size_t k=0;k<n;k++){
                int qi=(int)lrintf(oi[k]*inv); qi=std::max(-127,std::min(127,qi));
                int qq=(int)lrintf(oq[k]*inv); qq=std::max(-127,std::min(127,qq));
                q8.push_back((int8_t)qi); q8.push_back((int8_t)qq);
            }
            net_srv->send_const(cm>>1, (uint8_t)ch_idx, scale, actual_sr, q8.data(), (uint32_t)n);
        }
    };

    while(!ch.dem_stop_req.load(std::memory_order_relaxed) && !sdr_stream_error.load()){
        // center freq 변경 → 오실레이터 재설정
        uint64_t cur_cf = live_cf_hz.load(std::memory_order_acquire);
        if(cur_cf != prev_cf){
            off_hz = (ch_cf_mhz - (float)(cur_cf / 1e6f)) * 1e6f;
            osc.set_freq((double)off_hz, (double)msr);
            prev_cf = cur_cf;
        }
        size_t wp = ring_wp.load(std::memory_order_acquire);
        size_t rp = ch.dem_rp.load(std::memory_order_relaxed);
        size_t lag = (wp - rp) & IQ_RING_MASK;
        if(lag > MAX_LAG){
            size_t keep = (size_t)(msr * 0.02);
            rp = (wp - keep) & IQ_RING_MASK;
            ch.dem_rp.store(rp, std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            acc_i=acc_q=0; acc_cnt=0; rb_n=0; since_emit=0;
            lag = (wp - rp) & IQ_RING_MASK;
        }
        if(lag == 0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail = std::min(lag, BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos = (rp + s) & IQ_RING_MASK;
            float si = ring[pos*2]   / hw.iq_scale;
            float sq = ring[pos*2+1] / hw.iq_scale;
            float mi, mq; osc.mix(si, sq, mi, mq);
            mi = lpi[0].p(mi); mi = lpi[1].p(mi); mi = lpi[2].p(mi); mi = lpi[3].p(mi);
            mq = lpq[0].p(mq); mq = lpq[1].p(mq); mq = lpq[2].p(mq); mq = lpq[3].p(mq);
            acc_i += mi; acc_q += mq; acc_cnt++;
            if(acc_cnt < (int)decim) continue;
            float fi = (float)(acc_i / acc_cnt);
            float fq = (float)(acc_q / acc_cnt);
            acc_i = acc_q = 0; acc_cnt = 0;
            rb_i[rb_w % FN] = fi; rb_q[rb_w % FN] = fq; rb_w++;
            if(rb_n < (size_t)FN) rb_n++;
            since_emit++;
            if(since_emit >= emit_interval){ do_emit(); rb_n = 0; since_emit = 0; }
        }
        ch.dem_rp.store((rp + avail) & IQ_RING_MASK, std::memory_order_release);
    }
    bewe_log("CON[%d] worker exited\n", ch_idx);
}

// ── thread-safe inbound: con_worker(host-local) 또는 net CONST_FRAME 수신 스레드 → pending ──
void FFTViewer::eid_live_push(int ch_idx, uint32_t sr, const float* i, const float* q, int n){
    if(n <= 0) return;
    std::lock_guard<std::mutex> lk(live_const_mtx);
    if(live_const_ch != ch_idx){
        live_const_i.clear(); live_const_q.clear();
        live_const_ch = ch_idx;
    }
    live_const_sr = sr;
    live_const_i.insert(live_const_i.end(), i, i+n);
    live_const_q.insert(live_const_q.end(), q, q+n);
    if(live_const_i.size() > LIVE_CONST_CAP){           // pending 무한증가 방지
        size_t drop = live_const_i.size() - LIVE_CONST_CAP;
        live_const_i.erase(live_const_i.begin(), live_const_i.begin()+drop);
        live_const_q.erase(live_const_q.begin(), live_const_q.begin()+drop);
    }
}

// ── UI thread: pending → eid_ch_i/q rolling (eid_const_playing=follow 일 때만 누적) ──
void FFTViewer::eid_live_drain(){
    std::vector<float> pi, pq; uint32_t psr=0; int pch=-1;
    {
        std::lock_guard<std::mutex> lk(live_const_mtx);
        if(live_const_i.empty()) return;
        pi.swap(live_const_i); pq.swap(live_const_q);
        psr = live_const_sr; pch = live_const_ch;
    }
    if(eid_source != EID_LIVE || eid_live_ch != pch) return;  // 채널 전환 등 stale
    if(!eid_const_playing) return;   // 일시정지(freeze): 누적 중단 → 버퍼 인덱스 고정 (스크럽용)
    if(psr) eid_sample_rate = psr;
    size_t add = pi.size();
    for(size_t k=0;k<add;k++){
        float iv=pi[k], qv=pq[k];
        eid_ch_i.push_back(iv); eid_ch_q.push_back(qv);
        float env=sqrtf(iv*iv+qv*qv);
        float ph =atan2f(qv,iv);
        eid_envelope.push_back(env);
        eid_phase.push_back(ph);
        float prevph = (eid_phase.size()>=2)? eid_phase[eid_phase.size()-2] : ph;
        float df = ph - prevph;
        while(df >  (float)M_PI) df -= 2.f*(float)M_PI;
        while(df < -(float)M_PI) df += 2.f*(float)M_PI;
        eid_inst_freq.push_back(df);
    }
    if(eid_ch_i.size() > LIVE_CONST_CAP){               // rolling trim
        size_t drop = eid_ch_i.size() - LIVE_CONST_CAP;
        eid_ch_i.erase(eid_ch_i.begin(), eid_ch_i.begin()+drop);
        eid_ch_q.erase(eid_ch_q.begin(), eid_ch_q.begin()+drop);
        eid_envelope.erase(eid_envelope.begin(), eid_envelope.begin()+drop);
        eid_phase.erase(eid_phase.begin(), eid_phase.begin()+drop);
        eid_inst_freq.erase(eid_inst_freq.begin(), eid_inst_freq.begin()+drop);
    }
    eid_total_samples = (int64_t)eid_ch_i.size();
    eid_view_t0 = 0.0;
    eid_view_t1 = (double)eid_total_samples;
}

// ── SA 패널을 LIVE(성상도) 모드로 바인딩 (UI thread) ──
void FFTViewer::eid_open_live(int ch_idx){
    {
        std::lock_guard<std::mutex> lk(live_const_mtx);
        live_const_i.clear(); live_const_q.clear();
        live_const_ch = ch_idx; live_const_sr = 0;
    }
    eid_ch_i.clear(); eid_ch_q.clear();
    eid_envelope.clear(); eid_phase.clear(); eid_inst_freq.clear();
    eid_total_samples = 0;
    eid_view_t0 = 0.0; eid_view_t1 = 0.0;
    eid_sample_rate    = channels[ch_idx].con_sr ? channels[ch_idx].con_sr : 0;
    eid_center_freq_hz = (uint64_t)(((double)(channels[ch_idx].s+channels[ch_idx].e)*0.5)*1e6);
    eid_source         = EID_LIVE;
    eid_live_ch        = ch_idx;
    eid_view_mode      = 5;       // 성상도
    eid_const_zoom     = 0.0f;
    eid_const_pos      = 0;
    eid_const_playing  = true;    // follow live
    eid_phase_detrend_hz = 0.0f;
    eid_data_ready.store(true);
}

// ── LIVE 해제 → 파일 모드로 (UI thread) ──
void FFTViewer::eid_close_live(){
    eid_source  = EID_FILE;
    eid_live_ch = -1;
    eid_data_ready.store(false);
    eid_total_samples = 0;
    eid_ch_i.clear(); eid_ch_q.clear();
    eid_envelope.clear(); eid_phase.clear(); eid_inst_freq.clear();
    std::lock_guard<std::mutex> lk(live_const_mtx);
    live_const_i.clear(); live_const_q.clear(); live_const_ch = -1;
}
