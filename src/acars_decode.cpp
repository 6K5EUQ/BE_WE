#include "fft_viewer.hpp"
#include "acars_decode.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

// Isolated ACARS decode worker: taps the IQ ring with its own read pointer
// (like iq_only_worker), AM-envelope demodulates, feeds the streaming decoder.
// Does NOT touch dem_worker / the audio ring. Output via bewe_log_push (HOST col).
void FFTViewer::acars_worker(int ch_idx){
    Channel& ch=channels[ch_idx];
    uint32_t msr=header.sample_rate;
    uint64_t init_cf=live_cf_hz.load(std::memory_order_acquire);
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
    IIR1 alf; alf.set(std::min(3000.0f, bw_hz*0.5f) / (float)actual_inter); // ~3 kHz audio LPF (ACARS tones ≤2400 Hz)
    double aac=0; int acnt=0;

    AcarsDecoder dec;
    dec.on_record=[this](const AcarsMsg& m){ push_acars(m); };
    dec.reset((float)actual_asr, ch_idx);
    bewe_log_push(0,"ACARS[%d] start: %.4f MHz  BW=%.1fkHz  asr=%u\n",
        ch_idx,(ch.s+ch.e)/2.0f,bw_hz/1000.f,actual_asr);

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =(size_t)cap_decim*actual_asr/50;
    ch.acars_rp.store(ring_wp.load());

    while(!ch.acars_stop_req.load(std::memory_order_relaxed) && !sdr_stream_error.load()){
        { uint64_t cur=live_cf_hz.load(std::memory_order_acquire);
          if(cur!=prev_cf){
              off_hz=(((ch.s+ch.e)/2.0f)-(float)(cur/1e6))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur;
          }
        }
        size_t wp=ring_wp.load(std::memory_order_acquire);
        size_t rp=ch.acars_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        if(lag>MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK;
            ch.acars_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            am_dc=0; cap_i=cap_q=0; cap_cnt=0;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/hw.iq_scale, sq=ring[pos*2+1]/hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            float mi_aa=mi, mq_aa=mq;
            mi_aa=lpi[0].p(mi_aa); mi_aa=lpi[1].p(mi_aa); mi_aa=lpi[2].p(mi_aa); mi_aa=lpi[3].p(mi_aa);
            mq_aa=lpq[0].p(mq_aa); mq_aa=lpq[1].p(mq_aa); mq_aa=lpq[2].p(mq_aa); mq_aa=lpq[3].p(mq_aa);
            cap_i+=mi_aa; cap_q+=mq_aa; cap_cnt++;
            if(cap_cnt<(int)cap_decim) continue;
            float fi=(float)(cap_i/cap_cnt), fq=(float)(cap_q/cap_cnt);
            cap_i=cap_q=0; cap_cnt=0;
            // AM envelope (amplitude-independent decode → no AGC needed)
            float env=sqrtf(fi*fi+fq*fq);
            am_dc+=am_dc_alpha*(env-am_dc);
            float audio=alf.p(env-am_dc);
            aac+=audio; acnt++;
            if(acnt>=(int)actual_ad){
                float a=(float)(aac/acnt); aac=0; acnt=0;
                dec.feed(a);
            }
        }
        ch.acars_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    bewe_log_push(0,"ACARS[%d] stop\n",ch_idx);
}

void FFTViewer::start_acars(int ch_idx){
    Channel& ch=channels[ch_idx];
    if(ch.acars_on.load()||!ch.filter_active) return;
    ch.acars_stop_req.store(false);
    ch.acars_on.store(true);
    ch.acars_thr=std::thread(&FFTViewer::acars_worker,this,ch_idx);
}

void FFTViewer::stop_acars(int ch_idx){
    Channel& ch=channels[ch_idx];
    if(!ch.acars_on.load()) return;
    ch.acars_stop_req.store(true);
    if(ch.acars_thr.joinable()) ch.acars_thr.join();
    ch.acars_on.store(false);
}

// 디코드된 메시지 저장 (ch 기반 freq/time 스탬프). GUI/headless 양쪽 빌드 공통.
void FFTViewer::push_acars(AcarsMsg m){
    if(m.ch>=0 && m.ch<MAX_CHANNELS && channels[m.ch].filter_active)
        m.freq = (channels[m.ch].s + channels[m.ch].e)/2.0f;
    m.t_ms = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lk(acars_mtx);
    if((int)acars_log.size() >= ACARS_MAX) acars_log.erase(acars_log.begin());
    acars_log.push_back(m);
    acars_scroll = true;
}
