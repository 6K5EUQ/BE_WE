// ── Blind CP-OFDM demod (성상도) — DM_OFDM worker ──────────────────────────
// con_worker 와 동일한 dem 슬롯 + front-end(Oscillator mix → IIR LPF → decimate)를
// 재사용하되, 단일반송파 PSK 동기 대신 "표준 무관 블라인드 CP-OFDM 수신기"를 돌린다:
//   CP 자기상관으로 FFT크기(N)·CP길이(L)·심볼타이밍·fractional CFO 추정(자동) 또는 수동값,
//   심볼별 CP 제거 → CFO 보정 → FFT → 부반송파 점유 검출 → 블라인드 부반송파 등화(진폭+
//   M-th power 위상) → 등화된 데이터 부반송파 심볼을 점으로 모아 con_worker 와 동일한
//   eid_live_push / net_srv->send_const(CONST_FRAME) 경로로 그대로 emit.
// 출력이 성상점 스트림이라 네트워크 RX·표시·구독 게이트(const_mask)는 전부 무변경 재사용.
#include "fft_viewer.hpp"
#include "net_server.hpp"
#include <fftw3.h>
#include <cmath>
#include <complex>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>

// FFTW plan 생성/파괴는 thread-safe 가 아니다 (fftwf_execute 는 서로 다른 plan 이면 안전).
// 다중 OFDM 채널 워커가 동시에 plan 을 만들 때의 경합 방지용 전역 락.
static std::mutex g_fftw_plan_mtx;

using cf = std::complex<float>;

namespace {
constexpr int   OFDM_FRAME_SAMPS = 2048;   // emit 프레임당 최대 점
constexpr int   OFDM_FPS         = 20;     // emit cadence (frames/sec) — 심볼레이트 무관 throttle
constexpr float OFDM_TARGET_MULT = 1.25f;  // front-end 디시메이션 목표 SR = bw*1.25 (full BW)
constexpr float OFDM_MIN_SR      = 8000.f;
constexpr int   ANALYZE_LEN      = 16384;  // 블라인드 추정용 디시메이트 샘플 윈도우
// CP 상관 품질 q=|R|/P 는 Cauchy-Schwarz 로 [0,1] (완전 정렬 시 ~1.0). 하한은 1 미만이어야 함.
constexpr float HOLD_THRESH      = 0.5f;   // lock 중 q 하한 (이하 K심볼 연속 → 재탐색)
constexpr int   HOLD_FAIL_MAX    = 16;
constexpr float N_PEAK_RATIO     = 2.0f;   // 자기상관 peak/mean — 이 이상이어야 N 후보 채택
constexpr float FOLD_RATIO       = 1.6f;   // CP fold peak/mean — 이 이상이어야 lock
}

void FFTViewer::ofdm_worker(int ch_idx){
    Channel& ch = channels[ch_idx];
    uint32_t msr = header.sample_rate;
    uint64_t init_cf = live_cf_hz.load(std::memory_order_acquire);
    float ch_cf_mhz = (ch.s + ch.e) * 0.5f;
    float bw_hz = fabsf(ch.e - ch.s) * 1e6f;
    float off_hz = (ch_cf_mhz - (float)(init_cf / 1e6f)) * 1e6f;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;

    // ── front-end: 채널 full-BW 를 담도록 디시메이션 ──
    float tgt = bw_hz * OFDM_TARGET_MULT; if(tgt < OFDM_MIN_SR) tgt = OFDM_MIN_SR;
    uint32_t decim = std::max(1u, (uint32_t)ceilf((float)msr / tgt));
    uint32_t actual_sr = msr / decim;
    IIR1 lpi[4], lpq[4];
    { float cn=(bw_hz*0.5f)/(float)msr; if(cn>0.45f)cn=0.45f; if(cn<1e-5f)cn=1e-5f;
      for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); lpi[k].s=lpq[k].s=0; } }
    ch.con_sr = actual_sr;
    double acc_i=0, acc_q=0; int acc_cnt=0;

    // ── 디시메이트 샘플 rolling 버퍼 (buf[0] 의 전역 인덱스 = buf_base) ──
    std::vector<cf> buf; buf.reserve((size_t)ANALYZE_LEN*2);
    int64_t buf_base = 0;

    // ── 추정기 상태 ──
    bool    locked = false;
    int     N = 0, L = 0;       // FFT size, CP length
    int64_t sym_start = 0;      // 다음 처리할 심볼의 전역 시작 인덱스 (CP 시작)
    double  cfo = 0.0;          // cycles/sample (fractional)
    int     lost = 0;
    int64_t last_search_gidx = INT64_MIN/2;
    int     Mord = 4;           // 등화용 변조차수

    // ── FFT plan + 등화 상태 (N 크기) ──
    int planN=0; fftwf_plan plan=nullptr; fftwf_complex *fin=nullptr, *fout=nullptr;
    std::vector<float> Pbin;    // 부반송파별 running 전력
    std::vector<cf>    Acc;     // 부반송파별 M-power 위상 누적
    bool eq_init=false;
    auto free_plan=[&](){
        std::lock_guard<std::mutex> lk(g_fftw_plan_mtx);
        if(plan){ fftwf_destroy_plan(plan); plan=nullptr; }
        if(fin){ fftwf_free(fin); fin=nullptr; }
        if(fout){ fftwf_free(fout); fout=nullptr; }
    };
    auto ensure_plan=[&](int n){
        if(planN==n && plan) return;
        free_plan();
        { std::lock_guard<std::mutex> lk(g_fftw_plan_mtx);
          fin =(fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*n);
          fout=(fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*n);
          plan=fftwf_plan_dft_1d(n, fin, fout, FFTW_FORWARD, FFTW_ESTIMATE); }
        planN=n; Pbin.assign(n,0.f); Acc.assign(n, cf(0,0)); eq_init=true;
    };

    // ── emit (con_worker 와 동일 패턴) ──
    const int FN = OFDM_FRAME_SAMPS;
    std::vector<float> rb_i(FN), rb_q(FN); size_t rb_w=0, rb_n=0;
    int emit_every_syms=1, sym_emit_cnt=0;   // ~OFDM_FPS 로 throttle (publish_lock 에서 설정)
    std::vector<float> oi, oq; oi.reserve(FN); oq.reserve(FN);
    std::vector<int8_t> q8; q8.reserve((size_t)FN*2);
    auto do_emit=[&](){
        uint32_t cm = ch.const_mask.load(std::memory_order_relaxed);
        if(cm==0 || rb_n==0) return;
        size_t n = rb_n, start = rb_w - n;
        oi.clear(); oq.clear();
        for(size_t kk=0;kk<n;kk++){ size_t idx=(start+kk)%(size_t)FN; oi.push_back(rb_i[idx]); oq.push_back(rb_q[idx]); }
        if(cm & 0x1u) eid_live_push(ch_idx, actual_sr, oi.data(), oq.data(), (int)n);
        if(net_srv && (cm & ~0x1u)){
            float scale=1e-6f;
            for(size_t kk=0;kk<n;kk++){ float a=fabsf(oi[kk]),b=fabsf(oq[kk]); if(a>scale)scale=a; if(b>scale)scale=b; }
            float inv=127.0f/scale; q8.clear();
            for(size_t kk=0;kk<n;kk++){
                int qi=(int)lrintf(oi[kk]*inv); qi=std::max(-127,std::min(127,qi));
                int qq=(int)lrintf(oq[kk]*inv); qq=std::max(-127,std::min(127,qq));
                q8.push_back((int8_t)qi); q8.push_back((int8_t)qq);
            }
            net_srv->send_const(cm>>1, (uint8_t)ch_idx, scale, actual_sr, q8.data(), (uint32_t)n);
        }
    };
    auto push_pt=[&](float i, float q){
        rb_i[rb_w%FN]=i; rb_q[rb_w%FN]=q; rb_w++;
        if(rb_n<(size_t)FN) rb_n++;   // emit 는 심볼 단위로 throttle (locked 루프에서 호출)
    };

    // ── 파라미터 변경 감지 캐시 ──
    bool     cfg_auto = ch.ofdm_auto.load(std::memory_order_relaxed);
    uint16_t cfg_fft  = ch.ofdm_fft_size.load(std::memory_order_relaxed);
    uint16_t cfg_cp   = ch.ofdm_cp_len.load(std::memory_order_relaxed);
    uint8_t  cfg_mod  = ch.ofdm_mod_order.load(std::memory_order_relaxed);
    auto reset_est=[&](){
        locked=false; lost=0;
        ch.ofdm_locked.store(false);
        ch.ofdm_est_fft.store(0); ch.ofdm_est_cp.store(0); ch.ofdm_est_cfo_hz.store(0.f);
    };
    auto publish_lock=[&](float score){
        Mord = (cfg_mod==2)?2:4;
        ensure_plan(N);
        // 심볼레이트에 맞춰 emit 주기를 ~OFDM_FPS 로 (고심볼레이트 신호의 네트워크 폭주 방지)
        int srate = (int)(actual_sr / (uint32_t)std::max(1, N+L));
        emit_every_syms = std::max(1, srate / OFDM_FPS);
        sym_emit_cnt = 0;
        locked=true; lost=0;
        ch.ofdm_locked.store(true);
        ch.ofdm_est_fft.store((uint16_t)N);
        ch.ofdm_est_cp.store((uint16_t)L);
        ch.ofdm_est_cfo_hz.store((float)(cfo*actual_sr));
        bewe_log("OFDM[%d] LOCK N=%d CP=%d score=%.2f cfo=%.0fHz sr=%u\n",
                 ch_idx, N, L, score, cfo*actual_sr, actual_sr);
    };

    // CFO: 윈도우 시작 d0(윈도우 좌표)에서 reps 심볼 평균한 CP 상관의 위상.
    // R=Σ x[n]·conj(x[n+N]) ≈ |s|²·e^{-jθN} (θ=2π·Δf) → Δf = -arg(R)/(2πN).
    // cfo 는 실제 신호 오프셋 Δf(cycles/sample) 로 보관, derotate 는 e^{-j2π·cfo·n}.
    auto est_cfo=[&](const cf* x, int W, int d0, int n, int l, int S)->double{
        cf Rsum(0,0); int reps=0;
        for(int rep=0; rep<4; rep++){ int dd=d0+rep*S; if(dd+l+n>W) break;
            cf rr(0,0); for(int k=0;k<l;k++) rr += x[dd+k]*std::conj(x[dd+k+n]);
            Rsum+=rr; reps++; }
        return (reps>0)? -(double)(std::arg(Rsum)/(2.0*M_PI*(double)n)) : 0.0;
    };

    // ── 블라인드 획득: (1) 자기상관 peak 로 N(임의값) 추정 → (2) CP길이/타이밍 fold ──
    // A(τ)=|Σ x[n]·conj(x[n+τ])| 는 CP 가 매 심볼 x[n]≈x[n+N] 을 만들어 τ=N 에서 peak
    // (CFO 는 lag-N 위상이 일정하므로 peak 위치 불변). N 후보를 2의 거듭제곱에 한정하지 않음.
    int dbg_cnt=0;
    auto blind_acquire=[&](){
        int W = (int)std::min((size_t)ANALYZE_LEN, buf.size());
        if(W < 4096) return;
        const cf* x = buf.data() + (buf.size()-W);
        int tmin=32, tmax=std::min(2048, W/3);
        int CL=std::min(W - tmax, 8192); if(CL<1024) return;
        // (1) N 추정
        float bestA=0, sumA=0; int peak_t=0, nA=0;
        for(int t=tmin; t<=tmax; t++){
            cf acc(0,0);
            for(int n=0; n+t<CL; n+=2) acc += x[n]*std::conj(x[n+t]);
            float a=std::abs(acc); sumA+=a; nA++;
            if(a>bestA){ bestA=a; peak_t=t; }
        }
        float ratioA = bestA/(sumA/std::max(1,nA)+1e-9f);
        bool n_ok = (peak_t>=tmin && ratioA>=N_PEAK_RATIO);
        // (2) N 고정 → CP 길이/타이밍 fold (M(d)=|Σ_{k<L} c|/P, c=x·conj(x+N))
        float best=0; int bL=0, bd=0;
        if(n_ok){
            int n=peak_t, clen=W-n;
            std::vector<cf> c(clen);
            for(int d=0; d<clen; d++) c[d]=x[d]*std::conj(x[d+n]);
            const int divs[]={4,6,8,16,32,64};
            for(int ci=0; ci<6; ci++){
                int l=n/divs[ci]; if(l<2) continue;
                int S=n+l; if(clen < 3*S) continue;
                std::vector<float> folded(S,0.f); std::vector<int> cnt(S,0);
                cf rsum(0,0); double psum=0;
                for(int k=0;k<l;k++){ rsum+=c[k]; psum+=0.5*(std::norm(x[k])+std::norm(x[k+n])); }
                int dmax=clen-l;
                for(int d=0; d<=dmax; d++){
                    float m=std::abs(rsum)/((float)psum+1e-9f);
                    int ph=d%S; folded[ph]+=m; cnt[ph]++;
                    if(d<dmax){ rsum-=c[d]; rsum+=c[d+l];
                        psum-=0.5*(std::norm(x[d])+std::norm(x[d+n]));
                        psum+=0.5*(std::norm(x[d+l])+std::norm(x[d+l+n])); }
                }
                float mean=0; for(int p=0;p<S;p++){ if(cnt[p])folded[p]/=cnt[p]; mean+=folded[p]; } mean/=S;
                int pk=0; float pkv=0; for(int p=0;p<S;p++) if(folded[p]>pkv){pkv=folded[p];pk=p;}
                float sc=pkv/(mean+1e-9f);
                if(sc>best){ best=sc; bL=l; bd=pk; }
            }
        }
        if((++dbg_cnt % 12)==1)
            bewe_log("OFDM[%d] search: N~%d Aratio=%.2f L=%d foldsc=%.2f (need A>=%.1f fold>=%.1f)\n",
                     ch_idx, peak_t, ratioA, bL, best, N_PEAK_RATIO, FOLD_RATIO);
        if(n_ok && best>=FOLD_RATIO && bL>0){
            N=peak_t; L=bL; int S=N+L;
            int base_off=(int)buf.size()-W;
            sym_start = buf_base + base_off + bd;
            cfo = est_cfo(x, W, bd, N, L, S);
            publish_lock(best);
        }
    };

    // ── 수동 획득: N,L 고정, 타이밍만 1패스 탐색 ──
    auto manual_acquire=[&](){
        int n = cfg_fft;
        if(n<8){ float sp=ch.ofdm_subcarrier_hz.load(std::memory_order_relaxed);
                 if(sp>1.f) n=(int)lroundf((float)actual_sr/sp); }
        if(n<8 || n>8192) return;
        int l = cfg_cp; if(l<1) l=n/8; if(l<1) l=1;
        int S=n+l;
        int W = (int)std::min((size_t)ANALYZE_LEN, buf.size());
        if(W < 3*S) return;
        const cf* x = buf.data() + (buf.size()-W);
        int clen=W-n;
        std::vector<float> folded(S,0.f); std::vector<int> cnt(S,0);
        cf rsum(0,0); double psum=0;
        for(int k=0;k<l;k++){ rsum+=x[k]*std::conj(x[k+n]); psum+=0.5*(std::norm(x[k])+std::norm(x[k+n])); }
        int dmax=clen-l;
        for(int d=0; d<=dmax; d++){
            float m=std::abs(rsum)/((float)psum+1e-9f);
            int ph=d%S; folded[ph]+=m; cnt[ph]++;
            if(d<dmax){
                rsum-=x[d]*std::conj(x[d+n]); rsum+=x[d+l]*std::conj(x[d+l+n]);
                psum-=0.5*(std::norm(x[d])+std::norm(x[d+n]));
                psum+=0.5*(std::norm(x[d+l])+std::norm(x[d+l+n]));
            }
        }
        int pk=0; float pkv=0; for(int p=0;p<S;p++){ if(cnt[p]) folded[p]/=cnt[p]; if(folded[p]>pkv){pkv=folded[p];pk=p;} }
        N=n; L=l;
        int base_off=(int)buf.size()-W;
        sym_start = buf_base + base_off + pk;
        cfo = est_cfo(x, W, pk, N, L, S);
        publish_lock(99.f);   // 수동: score 미사용
    };

    // ── 심볼 1개 처리: CP 제거 → CFO 보정 → FFT → 점유 → 등화 → push ──
    auto process_symbol=[&](int off){
        int S=N+L;
        // CP 상관 품질 + CFO 추적 (off 의 CP 영역 사용)
        cf R(0,0); double P=0;
        for(int k=0;k<L;k++){ cf a=buf[off+k], b=buf[off+k+N]; R+=a*std::conj(b); P+=0.5*(std::norm(a)+std::norm(b)); }
        float q = std::abs(R)/((float)P+1e-9f);
        // 신호 품질이 좋을 때만 CFO 추적 (상실 중 잡음값으로 끌려가지 않게)
        if(q >= HOLD_THRESH){
            double inst = -std::arg(R)/(2.0*M_PI*(double)N);   // Δf = -arg(R)/(2πN)
            cfo += 0.05*(inst - cfo); lost=0;
        } else lost++;

        // CP 제거 + CFO derotate → FFT 입력
        int64_t g0 = buf_base + off + L;
        for(int k=0;k<N;k++){
            cf v = buf[off+L+k];
            // 정수 사이클 제거 후 cos/sin (g0 가 1e9 까지 커져도 인자 정밀도 보존)
            double cyc = cfo*(double)(g0+k); cyc -= floor(cyc);
            double ph = -2.0*M_PI*cyc;
            float cphf=(float)cos(ph), sphf=(float)sin(ph);
            fin[k][0]= v.real()*cphf - v.imag()*sphf;
            fin[k][1]= v.real()*sphf + v.imag()*cphf;
        }
        fftwf_execute(plan);

        // 부반송파 전력 running 평균 (점유 판정용). 첫 심볼은 직접 세팅(transient 방지).
        const float a_occ=0.05f;
        double sum=0;
        for(int k=0;k<N;k++){
            float p = fout[k][0]*fout[k][0] + fout[k][1]*fout[k][1];
            if(eq_init) Pbin[k]=p; else Pbin[k]+=a_occ*(p-Pbin[k]);
            sum += Pbin[k];
        }
        float thr = (float)(0.15*sum/(double)N);   // 점유 임계 (평균의 15%)

        // 블라인드 등화: 진폭 정규화 + M-th power 위상 de-rotate
        const float beta=0.02f;
        for(int k=0;k<N;k++){
            if(k==0) continue;            // DC 제거
            if(Pbin[k] <= thr) continue;  // null/guard 제거
            float A = sqrtf(Pbin[k]) + 1e-12f;
            cf Y(fout[k][0]/A, fout[k][1]/A);
            cf ym = Y; for(int m=1;m<Mord;m++) ym *= Y;     // Y^M
            if(eq_init) Acc[k]=ym; else Acc[k]+= beta*(ym-Acc[k]);
            float phi = std::arg(Acc[k])/(float)Mord;
            float c2=cosf(-phi), s2=sinf(-phi);
            push_pt(Y.real()*c2 - Y.imag()*s2, Y.real()*s2 + Y.imag()*c2);
        }
        eq_init=false;

        if(lost > HOLD_FAIL_MAX){ reset_est(); }   // 신호 상실 → 재탐색
        else { ch.ofdm_est_cfo_hz.store((float)(cfo*actual_sr)); }
    };

    bewe_log("OFDM[%d] start: cf=%.4fMHz off=%.0fHz bw=%.0fHz decim=%u sr=%u\n",
             ch_idx, ch_cf_mhz, off_hz, bw_hz, decim, actual_sr);

    const size_t MAX_LAG = (size_t)(msr * 0.08);
    const size_t BATCH   = std::max((size_t)4096, (size_t)decim * 256);

    while(!ch.dem_stop_req.load(std::memory_order_relaxed) && !sdr_stream_error.load()){
        // 파라미터 변경 → 추정기 리셋
        if(ch.ofdm_auto.load(std::memory_order_relaxed)!=cfg_auto ||
           ch.ofdm_fft_size.load(std::memory_order_relaxed)!=cfg_fft ||
           ch.ofdm_cp_len.load(std::memory_order_relaxed)!=cfg_cp ||
           ch.ofdm_mod_order.load(std::memory_order_relaxed)!=cfg_mod){
            cfg_auto=ch.ofdm_auto.load(); cfg_fft=ch.ofdm_fft_size.load();
            cfg_cp=ch.ofdm_cp_len.load(); cfg_mod=ch.ofdm_mod_order.load();
            reset_est();
        }
        // center freq 변경 → 오실레이터 재설정 + 추정기 리셋
        uint64_t cur_cf = live_cf_hz.load(std::memory_order_acquire);
        if(cur_cf != prev_cf){
            off_hz=(ch_cf_mhz-(float)(cur_cf/1e6f))*1e6f;
            osc.set_freq((double)off_hz,(double)msr); prev_cf=cur_cf;
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            buf.clear(); buf_base=0; sym_start=0; reset_est();
        }

        size_t wp = ring_wp.load(std::memory_order_acquire);
        size_t rp = ch.dem_rp.load(std::memory_order_relaxed);
        size_t lag = (wp - rp) & IQ_RING_MASK;
        if(lag > MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK; ch.dem_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            acc_i=acc_q=0; acc_cnt=0; buf.clear(); buf_base=0; sym_start=0; reset_est();
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag == 0){ std::this_thread::sleep_for(std::chrono::microseconds(80)); continue; }

        size_t avail = std::min(lag, BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/hw.iq_scale, sq=ring[pos*2+1]/hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            mi=lpi[0].p(mi); mi=lpi[1].p(mi); mi=lpi[2].p(mi); mi=lpi[3].p(mi);
            mq=lpq[0].p(mq); mq=lpq[1].p(mq); mq=lpq[2].p(mq); mq=lpq[3].p(mq);
            acc_i+=mi; acc_q+=mq; acc_cnt++;
            if(acc_cnt < (int)decim) continue;
            buf.push_back(cf((float)(acc_i/acc_cnt),(float)(acc_q/acc_cnt)));
            acc_i=acc_q=0; acc_cnt=0;
        }
        ch.dem_rp.store((rp+avail)&IQ_RING_MASK, std::memory_order_release);

        // 획득 (미lock 시, 80ms 마다)
        if(!locked){
            int64_t gnow = buf_base + (int64_t)buf.size();
            if((int)buf.size() >= ANALYZE_LEN &&
               gnow - last_search_gidx >= (int64_t)(actual_sr*0.08)){
                last_search_gidx = gnow;
                if(cfg_auto) blind_acquire(); else manual_acquire();
            }
        }
        // lock 시: 버퍼 내 완전한 심볼들 처리
        if(locked){
            while(true){
                int64_t off = sym_start - buf_base;
                if(off < 0){ sym_start=buf_base; off=0; }
                if(off + L + N > (int64_t)buf.size()) break;
                process_symbol((int)off);
                sym_start += (int64_t)(N + L);
                if(!locked) break;   // process_symbol 에서 재탐색 전환됨
                if(++sym_emit_cnt >= emit_every_syms){ do_emit(); rb_n=0; sym_emit_cnt=0; }
            }
        }
        // 소비분 trim
        int64_t consume=0;
        if(locked) consume = sym_start - buf_base;
        else if((int64_t)buf.size() > (int64_t)ANALYZE_LEN*2) consume = (int64_t)buf.size() - ANALYZE_LEN;
        if(consume > 0){
            if(consume > (int64_t)buf.size()) consume=(int64_t)buf.size();
            buf.erase(buf.begin(), buf.begin()+consume);
            buf_base += consume;
        }
    }
    free_plan();
    reset_est();
    bewe_log("OFDM[%d] worker exited\n", ch_idx);
}
