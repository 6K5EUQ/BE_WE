// ── AIS HOST 워커: IQ ring 독립 read-ptr 탭 → 채널 DDC → FM 판별기 → GMSK 비트동기
//    → NRZI → HDLC 프레임 디코드. demod_worker/오디오 ring 과 완전 분리.
//    결과는 host_emit() 으로 로그+일단위 저장+전 JOIN 브로드캐스트 (ACARS/WiFi 와 동형).
#include "fft_viewer.hpp"
#include "ais_module.hpp"
#include "ais_decode.hpp"
#include "ais_fp.hpp"
#include "module_api.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <vector>

namespace ais_mod {

static int64_t now_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// ── 버스트 RF 지문 누산기 (워커 hot loop → on_record 서 finalize) ───────────
// 마지막 emit 이후 구간을 한 버스트로 간주 (상대비교라 preamble 노이즈 공통상쇄).
struct BurstAcc {
    double sum_d=0, sumsq_d=0, sum_mag2=0, sum_dw=0, sum_pll=0;  // sum_dw=Σ d·|z|² (magnitude-weighted CFO)
    long   n_d=0, n_bits=0;
    std::vector<float> series;   // 옵션 raw 캡처 (BEWE_AIS_FPCAP; off면 비움)
    void reset(){ sum_d=sumsq_d=sum_mag2=sum_dw=sum_pll=0; n_d=n_bits=0; series.clear(); }
};
// raw 캡처 플래그 (env BEWE_AIS_FPCAP=1). 1회 평가 캐시.
static bool fpcap_enabled(){
    static int c=-1; if(c<0){ const char* e=getenv("BEWE_AIS_FPCAP"); c=(e&&e[0]=='1')?1:0; }
    return c==1;
}

// GMSK 정합 가우시안 LPF (9600 bps @ ~48 kHz, 5 sps). GNU AIS receiver.c 계수.
static const float GMSK_FIR[36] = {
   2.5959e-55f,2.9479e-49f,1.4741e-43f,3.2462e-38f,3.1480e-33f,
   1.3443e-28f,2.5280e-24f,2.0934e-20f,7.6339e-17f,1.2259e-13f,
   8.6690e-11f,2.6996e-08f,3.7020e-06f,2.2355e-04f,5.9448e-03f,
   6.9616e-02f,3.5899e-01f,8.1522e-01f,8.1522e-01f,3.5899e-01f,
   6.9616e-02f,5.9448e-03f,2.2355e-04f,3.7020e-06f,2.6996e-08f,
   8.6690e-11f,1.2259e-13f,7.6339e-17f,2.0934e-20f,2.5280e-24f,
   1.3443e-28f,3.1480e-33f,3.2462e-38f,1.4741e-43f,2.9479e-49f,
   2.5959e-55f };

void worker(FFTViewer& v, int ch_idx){
    Channel& ch = v.channels[ch_idx];
    uint32_t msr = v.header.sample_rate;
    const float inv_scale=1.0f/v.hw.iq_scale;  // ÷ → ×
    uint64_t init_cf = v.live_cf_hz.load(std::memory_order_acquire);
    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(init_cf/1e6f)) * 1e6f;
    float bw_hz  = fabsf(ch.e-ch.s) * 1e6f;

    // ── DDC: ~48 kHz 정수배 데시메이트 (9600 bps → ~5 sps) ──
    uint32_t decim  = std::max(1u, (uint32_t)llround((double)msr / 48000.0));
    uint32_t out_sr = msr / decim;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;
    // 데시메이션 전 anti-alias LPF: cutoff = min(채널BW/2, out_sr*0.45)
    IIR1 lpi[4], lpq[4];
    { float cut = std::min(bw_hz*0.5f, out_sr*0.45f);
      float cn = cut/(float)msr; if(cn>0.45f)cn=0.45f; if(cn<0.005f)cn=0.005f;
      for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); } }
    double dec_i=0, dec_q=0; uint32_t dec_cnt=0;

    // FM 판별기 상태 + GMSK 정합 FIR 딜레이라인
    float prev_i=0, prev_q=0;
    float fir[36]={}; int fir_pos=0;

    // DPLL 비트동기 (GNU AIS 구조): pllinc 1비트 = out_sr/9600 샘플
    const uint32_t PLLINC = (uint32_t)llround(65536.0*9600.0/(double)out_sr);
    uint32_t pll=0; int prev_zc=0; uint8_t lastbit=0;

    AisDecoder dec; dec.reset_all();
    long frames=0;
    BurstAcc acc; acc.reset();
    const bool cap = fpcap_enabled();
    // RF 지문 게이팅: 페이로드(ST_DATA) 구간만 누산 (프리앰블/플래그/탐색노이즈 제외 → CFO 잡음↓).
    bool acc_gate=false; int acc_skip=0;
    dec.on_gate = [&](bool on){ if(on){ acc.reset(); acc_gate=true; acc_skip=6; }   // FIR/DPLL 지연 6심볼 건너뜀
                                else   { acc_gate=false; } };
    dec.on_record = [&](const AisRecord& r){
        frames++;
        AisRecord m=r; m.ch=ch_idx;
        // ── RF 지문 finalize (이 버스트 acc → 레코드) ──
        if(acc.n_d>2 && acc.sum_mag2>0){
            double inv=1.0/acc.n_d, hz=(double)out_sr/(2.0*M_PI);
            double mean_w=acc.sum_dw/acc.sum_mag2;                // magnitude-weighted CFO (fade/noise 가중↓)
            double mean=acc.sum_d*inv;                            // fdev 용 unweighted 평균
            double var =acc.sumsq_d*inv - mean*mean; if(var<0) var=0;
            m.cfo_hz      = (float)(mean_w*hz);
            m.fdev_std_hz = (float)(std::sqrt(var)*hz);
            m.rssi_db     = (float)(10.0*std::log10(acc.sum_mag2*inv + 1e-20));
            m.dur_ms      = (float)(acc.n_d*1000.0/out_sr);
            m.clk_ppm     = acc.n_bits ? (float)(acc.sum_pll/acc.n_bits*1e6/(double)PLLINC) : 0.f;
            m.fp_ver      = ais_fp::FP_VER;
            m.has_rf      = true;
            if(cap && !acc.series.empty()) host_fpcap(m.mmsi, acc.series.data(), (int)acc.series.size());
        }
        host_emit(v, m);
        acc.reset(); acc_gate=false;
    };

    bewe_log_push(0,"AIS[%d] start: %.4f MHz  BW=%.1f kHz  station=%u  decim=%u out=%u Hz (%.2f sps)\n",
        ch_idx,(ch.s+ch.e)/2.0f, bw_hz/1000.f, msr, decim, out_sr, (double)out_sr/9600.0);

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =std::max<size_t>(4096, msr/50);
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());
    int64_t last_diag=now_ms(); long diag_bits=0;

    bool hold_prev=false;
    while(!worker_stop_req(ch_idx) && !v.sdr_stream_error.load() && ch.filter_active){
        // 가시대역 밖(Holding) → 복조 불가 → DDC 정지(연산/배터리 절약). 진입 edge 에서 상태 리셋
        // (HDLC framing/FIR/DPLL 잔류 → 복귀 시 false frame 방지) + runtime 누적 freeze.
        bool hold = ch.dem_paused.load(std::memory_order_relaxed);
        if(hold!=hold_prev){
            bewe_mod_host_ch_hold(ch_idx, hold);
            if(hold){ for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; } dec_i=dec_q=0; dec_cnt=0; prev_i=prev_q=0;
                      std::fill(fir,fir+36,0.f); fir_pos=0; pll=0; prev_zc=0; lastbit=0; dec.reset_all(); acc.reset(); acc_gate=false; }
            hold_prev=hold;
        }
        if(hold){
            my_rp.store(v.ring_wp.load(std::memory_order_acquire), std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        { uint64_t cur=v.live_cf_hz.load(std::memory_order_acquire);
          if(cur!=prev_cf){
              off_hz=(((ch.s+ch.e)/2.0f)-(float)(cur/1e6f))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur;
          }
        }
        size_t wp=v.ring_wp.load(std::memory_order_acquire);
        size_t rp=my_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        if(lag>MAX_LAG){                                   // 과부하 → 경계 점프 + 상태 리셋
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK; my_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            dec_i=dec_q=0; dec_cnt=0; prev_i=prev_q=0; acc.reset(); acc_gate=false;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(1000)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]*inv_scale, sq=v.ring[pos*2+1]*inv_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            mi=lpi[0].p(mi); mi=lpi[1].p(mi); mi=lpi[2].p(mi); mi=lpi[3].p(mi);
            mq=lpq[0].p(mq); mq=lpq[1].p(mq); mq=lpq[2].p(mq); mq=lpq[3].p(mq);
            dec_i+=mi; dec_q+=mq;
            if(++dec_cnt < decim) continue;
            float oi=(float)(dec_i/dec_cnt), oq=(float)(dec_q/dec_cnt);
            dec_i=dec_q=0; dec_cnt=0;

            // FM 판별: arg(z * conj(prev)) — 순시주파수 (GMSK mark/space). 부호모호성은 NRZI 가 흡수.
            float d = atan2f(oq*prev_i - oi*prev_q, oi*prev_i + oq*prev_q + 1e-20f);
            prev_i=oi; prev_q=oq;
            // RF 지문 누산 (페이로드 게이트 열림 + skip 경과 후만; on_record 서 finalize)
            if(acc_gate){
                if(acc_skip>0) acc_skip--;
                else {
                    double mag2=(double)oi*oi + (double)oq*oq;
                    acc.sum_d += d; acc.sumsq_d += (double)d*d;
                    acc.sum_dw += d*mag2; acc.sum_mag2 += mag2; acc.n_d++;
                    if(cap && acc.series.size()<512) acc.series.push_back(d);   // 옵션 raw 시리즈
                }
            }

            // GMSK 정합 가우시안 FIR
            fir[fir_pos]=d;
            float out=0; int idx=fir_pos;
            for(int k=0;k<36;k++){ out+=GMSK_FIR[k]*fir[idx]; if(--idx<0) idx=35; }
            fir_pos=(fir_pos+1)%36;

            // DPLL: 영교차마다 위상 보정, wrap 마다 1심볼 샘플 → slice → NRZI → 디코더
            int curr=(out>0);
            if((curr^prev_zc)==1){ int corr = (pll<0x8000)? +(int)(PLLINC/16) : -(int)(PLLINC/16);
                                   pll += corr; if(acc_gate) acc.sum_pll += (double)corr; }  // clk_ppm 페이로드 한정
            prev_zc=curr;
            pll+=PLLINC;
            if(pll>0xFFFF){
                uint8_t bit=(out>0)?1:0;
                uint8_t b=(uint8_t)!(bit^lastbit);          // NRZI
                dec.feed_bit(b);
                lastbit=bit; pll&=0xFFFF; diag_bits++; if(acc_gate) acc.n_bits++;
            }
        }
        my_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);

        int64_t t=now_ms();
        if(t-last_diag>=10000){                              // 10초마다 진단 (콘솔만)
            bewe_log_push(0,"AIS[%d] diag: bits/10s=%ld frames=%ld\n", ch_idx, diag_bits, frames);
            last_diag=t; diag_bits=0;
        }
    }
    if(!worker_stop_req(ch_idx)) worker_natural_exit(v, ch_idx);
    bewe_log_push(0,"AIS[%d] stop\n",ch_idx);
}

} // namespace ais_mod
