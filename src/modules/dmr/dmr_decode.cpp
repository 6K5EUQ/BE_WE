// ── DMR HOST 워커: IQ ring 독립 read-ptr 탭 → 채널 DDC → FM 판별기 → boxcar
//    정합필터 → DmrDecoder(심볼동기/sync/Slot Type Golay/BPTC/CSBK·LC). AIS 와 동형.
//    결과는 host_emit() → 로그+일단위 저장+전 JOIN 브로드캐스트.
#include "fft_viewer.hpp"
#include "dmr_module.hpp"
#include "dmr_decode.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

namespace dmr_mod {

static int64_t now_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void worker(FFTViewer& v, int ch_idx){
    Channel& ch = v.channels[ch_idx];
    uint32_t msr = v.header.sample_rate;
    uint64_t init_cf = v.live_cf_hz.load(std::memory_order_acquire);
    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(init_cf/1e6f)) * 1e6f;
    float bw_hz  = fabsf(ch.e-ch.s) * 1e6f;

    // ── DDC: ~48 kHz 정수배 데시메이트 (4800 sym/s → ~10 sps) ──
    uint32_t decim  = std::max(1u, (uint32_t)llround((double)msr / 48000.0));
    uint32_t out_sr = msr / decim;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;
    IIR1 lpi[4], lpq[4];
    { float cut = std::min(bw_hz*0.5f, out_sr*0.45f);
      float cn = cut/(float)msr; if(cn>0.45f)cn=0.45f; if(cn<0.005f)cn=0.005f;
      for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); } }
    double dec_i=0, dec_q=0; uint32_t dec_cnt=0;

    // FM 판별기 상태 + boxcar 정합필터 (1심볼 = round(out_sr/4800) 샘플)
    float prev_i=0, prev_q=0;
    int   W = std::max(1, (int)llround((double)out_sr/4800.0));
    std::vector<float> box(W,0.f); int box_pos=0; double box_sum=0;

    DmrDecoder dec; dec.configure((double)out_sr);
    long frames=0;
    dec.on_record = [&](const DmrRecord& r){
        frames++; DmrRecord m=r; m.ch=ch_idx; host_emit(v, m);
    };

    bewe_log_push(0,"DMR[%d] start: %.4f MHz  BW=%.1f kHz  decim=%u out=%u Hz (%.2f sps)\n",
        ch_idx,(ch.s+ch.e)/2.0f, bw_hz/1000.f, decim, out_sr, (double)out_sr/4800.0);

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =std::max<size_t>(4096, msr/50);
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());
    int64_t last_diag=now_ms();

    while(!worker_stop_req(ch_idx) && !v.sdr_stream_error.load() && ch.filter_active){
        { uint64_t cur=v.live_cf_hz.load(std::memory_order_acquire);
          if(cur!=prev_cf){
              off_hz=(((ch.s+ch.e)/2.0f)-(float)(cur/1e6f))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur;
          }
        }
        size_t wp=v.ring_wp.load(std::memory_order_acquire);
        size_t rp=my_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        if(lag>MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK; my_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            dec_i=dec_q=0; dec_cnt=0; prev_i=prev_q=0;
            std::fill(box.begin(),box.end(),0.f); box_sum=0; box_pos=0;
            dec.reset();
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]/v.hw.iq_scale, sq=v.ring[pos*2+1]/v.hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            mi=lpi[0].p(mi); mi=lpi[1].p(mi); mi=lpi[2].p(mi); mi=lpi[3].p(mi);
            mq=lpq[0].p(mq); mq=lpq[1].p(mq); mq=lpq[2].p(mq); mq=lpq[3].p(mq);
            dec_i+=mi; dec_q+=mq;
            if(++dec_cnt < decim) continue;
            float oi=(float)(dec_i/dec_cnt), oq=(float)(dec_q/dec_cnt);
            dec_i=dec_q=0; dec_cnt=0;

            // FM 판별 (순시주파수) → boxcar 정합필터(이동평균) → 디코더
            float dft = atan2f(oq*prev_i - oi*prev_q, oi*prev_i + oq*prev_q + 1e-20f);
            prev_i=oi; prev_q=oq;
            box_sum += dft - box[box_pos]; box[box_pos]=dft;
            if(++box_pos>=W) box_pos=0;
            dec.feed((float)(box_sum/W));
        }
        my_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);

        int64_t t=now_ms();
        if(t-last_diag>=10000){
            bewe_log_push(0,"DMR[%d] diag: records=%ld\n", ch_idx, frames);
            last_diag=t;
        }
    }
    if(!worker_stop_req(ch_idx)) worker_natural_exit(v, ch_idx);
    bewe_log_push(0,"DMR[%d] stop\n",ch_idx);
}

} // namespace dmr_mod
