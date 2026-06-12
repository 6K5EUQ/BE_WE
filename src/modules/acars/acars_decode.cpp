// ── ACARS HOST 워커: IQ ring 독립 read-ptr 탭 → AM 포락선 → 스트리밍 디코더 ──
// demod_worker / 오디오 ring 과 완전 분리. 디코드 결과는 host_emit() 으로
// 로그 + 일 단위 저장 + 전 JOIN 브로드캐스트.
#include "fft_viewer.hpp"
#include "acars_module.hpp"
#include "acars_decode.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

namespace acars_mod {

// acars_module.cpp 의 워커 슬롯 접근자
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

void worker(FFTViewer& v, int ch_idx){
    Channel& ch=v.channels[ch_idx];
    uint32_t msr=v.header.sample_rate;
    uint64_t init_cf=v.live_cf_hz.load(std::memory_order_acquire);
    float off_hz=(((ch.s+ch.e)/2.0f)-(float)(init_cf/1e6f))*1e6f;
    float bw_hz=fabsf(ch.e-ch.s)*1e6f;

    uint32_t inter_sr,audio_decim,cap_decim;
    demod_rates(msr,bw_hz,inter_sr,audio_decim,cap_decim);
    uint32_t actual_inter=msr/cap_decim;
    uint32_t actual_ad=std::max(1u,(uint32_t)round((double)actual_inter/AUDIO_SR));
    uint32_t actual_asr=actual_inter/actual_ad;

    Oscillator osc; osc.set_freq((double)off_hz,(double)msr);
    uint64_t prev_cf=init_cf;
    double cap_i=0,cap_q=0; int cap_cnt=0;
    IIR1 lpi[4],lpq[4];
    { float cn=(bw_hz*0.5f)/(float)msr; if(cn>0.45f)cn=0.45f;
      for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); } }
    float am_dc=0;
    float am_dc_alpha=1.0f-expf(-2.0f*M_PI*30.0f/(float)actual_inter);
    IIR1 alf; alf.set(std::min(3000.0f, bw_hz*0.5f) / (float)actual_inter); // ~3 kHz audio LPF (ACARS tones <=2400 Hz)
    double aac=0; int acnt=0;

    AcarsDecoder dec;
    dec.on_record=[&v](const AcarsMsg& m){ host_emit(v, m); };
    dec.reset((float)actual_asr, ch_idx);
    bewe_log_push(0,"ACARS[%d] start: %.4f MHz  BW=%.1fkHz  asr=%u\n",
        ch_idx,(ch.s+ch.e)/2.0f,bw_hz/1000.f,actual_asr);

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =(size_t)cap_decim*actual_asr/50;
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());

    while(!worker_stop_req(ch_idx) && !v.sdr_stream_error.load() && ch.filter_active){
        { uint64_t cur=v.live_cf_hz.load(std::memory_order_acquire);
          if(cur!=prev_cf){
              off_hz=(((ch.s+ch.e)/2.0f)-(float)(cur/1e6))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur;
          }
        }
        size_t wp=v.ring_wp.load(std::memory_order_acquire);
        size_t rp=my_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        if(lag>MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK;
            my_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            am_dc=0; cap_i=cap_q=0; cap_cnt=0;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]/v.hw.iq_scale, sq=v.ring[pos*2+1]/v.hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            float mi_aa=mi, mq_aa=mq;
            mi_aa=lpi[0].p(mi_aa); mi_aa=lpi[1].p(mi_aa); mi_aa=lpi[2].p(mi_aa); mi_aa=lpi[3].p(mi_aa);
            mq_aa=lpq[0].p(mq_aa); mq_aa=lpq[1].p(mq_aa); mq_aa=lpq[2].p(mq_aa); mq_aa=lpq[3].p(mq_aa);
            cap_i+=mi_aa; cap_q+=mq_aa; cap_cnt++;
            if(cap_cnt<(int)cap_decim) continue;
            float fi=(float)(cap_i/cap_cnt), fq=(float)(cap_q/cap_cnt);
            cap_i=cap_q=0; cap_cnt=0;
            // AM envelope (amplitude-independent decode -> no AGC needed)
            float env=sqrtf(fi*fi+fq*fq);
            am_dc+=am_dc_alpha*(env-am_dc);
            float audio=alf.p(env-am_dc);
            aac+=audio; acnt++;
            if(acnt>=(int)actual_ad){
                float a=(float)(aac/acnt); aac=0; acnt=0;
                dec.feed(a);
            }
        }
        my_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    // 정지 요청 없이 끝났으면 (채널 삭제/스트림 에러) 상태 정리 + 브로드캐스트
    if(!worker_stop_req(ch_idx)) worker_natural_exit(v, ch_idx);
    bewe_log_push(0,"ACARS[%d] stop\n",ch_idx);
}

} // namespace acars_mod
