// ── Live constellation (성상도) — DM_CONST worker + SA LIVE feed ───────────
// 채널필터를 DM_CONST 모드로 전환하면 (i 키) 오디오 복조 대신 채널 full-BW baseband
// IQ 를 처리해 "성상도 데이터" 만 만든다.
//  - raw 모드(con_sync_on=false): 디시메이트된 baseband 스냅샷(점=샘플).
//  - sync 모드(con_sync_on=true) : PSK 수신기 체인으로 복원한 심볼(점=심볼) — 깨끗한 성상도.
//      AGC → RRC matched filter → Gardner 타이밍 복원 → Costas M-PSK 반송파 복원.
//  - host-local viewer(const_mask bit0): eid_ch_i/q 로 직접 push (dev HOST=GUI).
//  - remote viewer(const_mask bit_i)   : int8 양자화 후 send_const → Central → JOIN.
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
static constexpr int   CONST_FRAME_SAMPS = 2048;   // 프레임당 최대 점 (대역폭 상한)
static constexpr float CONST_TARGET_MULT = 1.25f;  // raw target_sr = bw*1.25 (full BW)
static constexpr float CONST_MIN_SR      = 8000.f;
static constexpr int   SYNC_OSF          = 4;      // sync 모드 oversampling (samples/symbol)
static constexpr int   RRC_SPAN          = 8;      // RRC matched filter span (symbols)

// ── RRC matched filter (complex FIR) ──────────────────────────────────────
namespace {
struct RRC {
    std::vector<float> h, bi, bq;
    int n=0, wp=0;
    static float tap(float t, float b){   // root-raised-cosine impulse, t in symbols
        if(fabsf(t) < 1e-5f)
            return 1.0f + b*(4.0f/(float)M_PI - 1.0f);
        if(b > 1e-6f && fabsf(fabsf(t) - 1.0f/(4.0f*b)) < 1e-4f){
            float p = (float)M_PI/(4.0f*b);
            return (b/sqrtf(2.0f)) *
                   ((1.0f+2.0f/(float)M_PI)*sinf(p) + (1.0f-2.0f/(float)M_PI)*cosf(p));
        }
        float num = sinf((float)M_PI*t*(1.0f-b)) + 4.0f*b*t*cosf((float)M_PI*t*(1.0f+b));
        float den = (float)M_PI*t*(1.0f - (4.0f*b*t)*(4.0f*b*t));
        return num/den;
    }
    void design(float beta, int sps, int span){
        int N = sps*span; if((N&1)==0) N++;       // odd length
        h.assign(N,0.f); bi.assign(N,0.f); bq.assign(N,0.f); n=N; wp=0;
        int mid=N/2; double e=0;
        for(int i=0;i<N;i++){ h[i]=tap((float)(i-mid)/(float)sps, beta); e+=(double)h[i]*h[i]; }
        float s=(float)sqrt(e); if(s>0) for(float& v:h) v/=s;   // unit energy
    }
    void push(float i,float q,float& oi,float& oq){
        bi[wp]=i; bq[wp]=q;
        float ai=0,aq=0; int idx=wp;
        for(int k=0;k<n;k++){ ai+=h[k]*bi[idx]; aq+=h[k]*bq[idx]; if(--idx<0) idx=n-1; }
        oi=ai; oq=aq; if(++wp>=n) wp=0;
    }
};

// ── PSK 심볼 동기: MF → Gardner 타이밍 → Costas 반송파 ──────────────────────
struct SymbolSync {
    RRC   mf;
    float sps=4.f;            // 입력 samples/symbol (= fs/baud)
    int   modM=2;
    // 타이밍 (Gardner + 선형보간, 2차 루프)
    double k=0;               // 입력 샘플 인덱스
    double t_on=0, t_half=0;  // 다음 on/half 심볼 strobe 시각(입력샘플 단위)
    double Tsym=4, Tnom=4;    // 추정/공칭 symbol period
    double tt_int=0;          // 타이밍 루프 적분기
    float  tKp=0.f, tKi=0.f;
    float  mp_i=0, mp_q=0;    // 직전 MF 샘플 (선형보간용)
    float  h_i=0, h_q=0; bool have_half=false;
    float  py_i=0, py_q=0; bool have_prev=false;
    // 반송파 (Costas M-PSK, 2차 PLL)
    float  cph=0, cfr=0, cKp=0.f, cKi=0.f;
    // AGC
    float  agc=1.0f, agc_p=1.0f;

    void configure(uint32_t fs, float baud, float roll, int mod, int osf){
        sps  = (float)fs/baud;
        Tnom = Tsym = (double)sps;
        modM = (mod==4||mod==8)?mod:2;
        mf.design(roll, osf, RRC_SPAN);
        // 루프 대역폭 (보수적): timing ~0.5% , carrier ~1% of symbol rate
        float zeta=1.0f;
        { float bw=0.005f, th=bw/(zeta+0.25f/zeta);
          tKp=(4.0f*zeta*th)/(1.0f+2.0f*zeta*th+th*th);
          tKi=(4.0f*th*th)/(1.0f+2.0f*zeta*th+th*th); }
        { float bw=0.01f, th=bw/(zeta+0.25f/zeta);
          cKp=(4.0f*zeta*th)/(1.0f+2.0f*zeta*th+th*th);
          cKi=(4.0f*th*th)/(1.0f+2.0f*zeta*th+th*th); }
        k=0; t_half=0.5*Tnom; t_on=Tnom; tt_int=0;
        have_half=have_prev=false; mp_i=mp_q=0;
        cph=cfr=0; agc=1.0f; agc_p=1.0f;
    }
    static float psk_err(float i,float q,int M){    // sin(M*theta) = Im(y^M)/|y|^M
        float ai=i, aq=q;
        for(int m=1;m<M;m++){ float ni=ai*i-aq*q, nq=ai*q+aq*i; ai=ni; aq=nq; }
        float mag=powf(sqrtf(i*i+q*q),(float)M)+1e-9f;
        return aq/mag;
    }
    // MF + 타이밍 + 반송파. 심볼 1개 나오면 true + oi,oq.
    bool process(float ii, float qq, float& oi, float& oq){
        float mi, mq; mf.push(ii,qq, mi,mq);
        k += 1.0;
        bool got=false;
        // half-symbol strobe (Gardner 중간점)
        if(t_half <= k && t_half > k-1.0){
            float fr=(float)(t_half-(k-1.0));
            h_i = mp_i + fr*(mi-mp_i); h_q = mp_q + fr*(mq-mp_q);
            have_half=true;
        }
        // on-symbol strobe
        if(t_on <= k && t_on > k-1.0){
            float fr=(float)(t_on-(k-1.0));
            float yi = mp_i + fr*(mi-mp_i);
            float yq = mp_q + fr*(mq-mp_q);
            if(have_prev && have_half){      // Gardner TED → 2차 루프 → Tsym 보정
                // 진폭 무관하게 정규화 (agc_p = 직전 심볼 전력 추정)
                float e = (h_i*(yi-py_i) + h_q*(yq-py_q)) / (agc_p+1e-6f);
                e = std::max(-1.0f,std::min(1.0f,e));
                tt_int += (double)tKi*e;
                double corr = (double)tKp*e + tt_int;
                Tsym = Tnom*(1.0 - corr);
                if(Tsym < Tnom*0.9) Tsym=Tnom*0.9;
                if(Tsym > Tnom*1.1) Tsym=Tnom*1.1;
            }
            py_i=yi; py_q=yq; have_prev=true;
            t_half = t_on + Tsym*0.5;
            t_on   = t_on + Tsym;
            // AGC (avg power → 1)
            float p = yi*yi+yq*yq;
            agc_p += 0.01f*(p-agc_p);
            agc = 1.0f/sqrtf(agc_p+1e-9f);
            float gi=yi*agc, gq=yq*agc;
            // Costas M-PSK 반송파 복원
            float c=cosf(cph), s=sinf(cph);
            float ri =  gi*c + gq*s;     // derotate by -cph
            float rq = -gi*s + gq*c;
            // e2 = sin(M·theta) > 0 일 때 cph 증가 → theta→0 (PLL 추적)
            float e2 = psk_err(ri,rq,modM);
            cph += cfr + cKp*e2;
            cfr += cKi*e2;
            if(cph >  (float)M_PI) cph -= 2.0f*(float)M_PI;
            if(cph < -(float)M_PI) cph += 2.0f*(float)M_PI;
            oi=ri; oq=rq; got=true;
        }
        mp_i=mi; mp_q=mq;
        return got;
    }
};
} // namespace

void FFTViewer::con_worker(int ch_idx){
    Channel& ch = channels[ch_idx];
    uint32_t msr = header.sample_rate;
    uint64_t init_cf = live_cf_hz.load(std::memory_order_acquire);
    float ch_cf_mhz = (ch.s + ch.e) * 0.5f;
    float bw_hz = fabsf(ch.e - ch.s) * 1e6f;
    float off_hz = (ch_cf_mhz - (float)(init_cf / 1e6f)) * 1e6f;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;

    // ── 재설정 가능한 front-end (raw vs sync) ─────────────────────────────
    uint32_t decim=1, actual_sr=0, emit_interval=1;
    IIR1 lpi[4], lpq[4];
    SymbolSync sync;
    bool  sync_mode=false;
    bool  cfg_sync=false; float cfg_baud=-1, cfg_roll=-1; uint8_t cfg_mod=0;
    double acc_i=0, acc_q=0; int acc_cnt=0;
    const int FN = CONST_FRAME_SAMPS;
    std::vector<float> rb_i(FN), rb_q(FN);
    size_t rb_w=0, rb_n=0; uint32_t since_emit=0;

    auto reconfigure = [&](){
        bool  son  = ch.con_sync_on.load(std::memory_order_relaxed);
        float baud = ch.con_sym_rate.load(std::memory_order_relaxed);
        float roll = ch.con_rolloff.load(std::memory_order_relaxed);
        uint8_t mod= ch.con_mod_order.load(std::memory_order_relaxed);
        sync_mode = (son && baud > 1.0f);
        if(sync_mode){
            float tgt = baud*(float)SYNC_OSF;
            decim = std::max(1u,(uint32_t)lroundf((float)msr/tgt));
            actual_sr = msr/decim;
            float cut = std::min(bw_hz*0.5f, baud*(1.0f+roll)*0.6f);
            float cn = cut/(float)msr; if(cn>0.45f)cn=0.45f; if(cn<1e-5f)cn=1e-5f;
            for(int kk=0;kk<4;kk++){ lpi[kk].set(cn); lpq[kk].set(cn); lpi[kk].s=lpq[kk].s=0; }
            sync.configure(actual_sr, baud, roll, (int)mod, SYNC_OSF);
            emit_interval = std::max(1u,(uint32_t)(baud/(float)CONST_FPS));
        } else {
            float tgt = bw_hz*CONST_TARGET_MULT; if(tgt<CONST_MIN_SR) tgt=CONST_MIN_SR;
            decim = std::max(1u,(uint32_t)ceilf((float)msr/tgt));
            actual_sr = msr/decim;
            float cn=(bw_hz*0.5f)/(float)msr; if(cn>0.45f)cn=0.45f;
            for(int kk=0;kk<4;kk++){ lpi[kk].set(cn); lpq[kk].set(cn); lpi[kk].s=lpq[kk].s=0; }
            emit_interval = std::max(1u,(uint32_t)((float)actual_sr/(float)CONST_FPS));
        }
        ch.con_sr = actual_sr;
        acc_i=acc_q=0; acc_cnt=0; rb_n=0; since_emit=0;
        cfg_sync=son; cfg_baud=baud; cfg_roll=roll; cfg_mod=mod;
    };
    reconfigure();

    const size_t MAX_LAG = (size_t)(msr * 0.08);
    const size_t BATCH   = std::max((size_t)4096, (size_t)decim * 256);
    std::vector<float>  oi, oq; oi.reserve(FN); oq.reserve(FN);
    std::vector<int8_t> q8;     q8.reserve((size_t)FN*2);

    bewe_log("CON[%d] start: cf=%.4fMHz off=%.0fHz bw=%.0fHz decim=%u sr=%u sync=%d\n",
             ch_idx, ch_cf_mhz, off_hz, bw_hz, decim, actual_sr, (int)sync_mode);

    auto do_emit = [&](){
        uint32_t cm = ch.const_mask.load(std::memory_order_relaxed);
        if(cm == 0 || rb_n == 0) return;
        size_t n = rb_n;
        size_t start = rb_w - n;
        oi.clear(); oq.clear();
        for(size_t kk=0;kk<n;kk++){ size_t idx=(start+kk)%(size_t)FN; oi.push_back(rb_i[idx]); oq.push_back(rb_q[idx]); }
        if(cm & 0x1u)
            eid_live_push(ch_idx, (uint32_t)(sync_mode? ch.con_sym_rate.load():actual_sr), oi.data(), oq.data(), (int)n);
        if(net_srv && (cm & ~0x1u)){
            float scale = 1e-6f;
            for(size_t kk=0;kk<n;kk++){ float a=fabsf(oi[kk]), b=fabsf(oq[kk]); if(a>scale)scale=a; if(b>scale)scale=b; }
            float inv = 127.0f/scale;
            q8.clear();
            for(size_t kk=0;kk<n;kk++){
                int qi=(int)lrintf(oi[kk]*inv); qi=std::max(-127,std::min(127,qi));
                int qq=(int)lrintf(oq[kk]*inv); qq=std::max(-127,std::min(127,qq));
                q8.push_back((int8_t)qi); q8.push_back((int8_t)qq);
            }
            net_srv->send_const(cm>>1, (uint8_t)ch_idx, scale,
                                (uint32_t)(sync_mode? ch.con_sym_rate.load():actual_sr), q8.data(), (uint32_t)n);
        }
    };
    auto push_pt = [&](float i, float q){
        rb_i[rb_w % FN]=i; rb_q[rb_w % FN]=q; rb_w++;
        if(rb_n < (size_t)FN) rb_n++;
        if(++since_emit >= emit_interval){ do_emit(); rb_n=0; since_emit=0; }
    };

    while(!ch.dem_stop_req.load(std::memory_order_relaxed) && !sdr_stream_error.load()){
        // sync 파라미터 변경 감지 → front-end 재설정
        if(ch.con_sync_on.load(std::memory_order_relaxed)!=cfg_sync ||
           ch.con_sym_rate.load(std::memory_order_relaxed)!=cfg_baud ||
           ch.con_rolloff.load(std::memory_order_relaxed)!=cfg_roll ||
           ch.con_mod_order.load(std::memory_order_relaxed)!=cfg_mod)
            reconfigure();
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
            for(int kk=0;kk<4;kk++){ lpi[kk].s=lpq[kk].s=0; }
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
            if(sync_mode){
                float yi, yq;
                if(sync.process(fi, fq, yi, yq)) push_pt(yi, yq);   // 점 = 복원 심볼
            } else {
                push_pt(fi, fq);                                    // 점 = raw 샘플
            }
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
    // 심볼동기는 raw 로 시작 (사용자가 Sync 체크 시 켬)
    eid_sync_on=false; eid_sync_baud=0.f; eid_sync_mod=2; eid_sync_rolloff=0.35f;
    channels[ch_idx].con_sync_on.store(false);
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
