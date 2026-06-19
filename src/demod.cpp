#include "fft_viewer.hpp"
#include "module_api.hpp"
#include "net_server.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>

void FFTViewer::dem_worker(int ch_idx){
    Channel& ch=channels[ch_idx];
    Channel::DemodMode mode=ch.mode;
    // 네트워크 오디오 배치 버퍼 (256샘플 단위로 서버에 전송)
    static constexpr int NET_AUDIO_BATCH = 256;
    std::vector<float> net_audio_buf;
    net_audio_buf.reserve(NET_AUDIO_BATCH);
    uint32_t msr=header.sample_rate;
    uint64_t init_cf = live_cf_hz.load(std::memory_order_acquire);
    float off_hz=(((ch.s+ch.e)/2.0f)-(float)(init_cf/1e6f))*1e6f;
    float bw_hz=fabsf(ch.e-ch.s)*1e6f;

    uint32_t inter_sr,audio_decim,cap_decim;
    demod_rates(msr,bw_hz,inter_sr,audio_decim,cap_decim);
    uint32_t actual_inter=msr/cap_decim;
    uint32_t actual_ad=std::max(1u,(uint32_t)round((double)actual_inter/AUDIO_SR));
    uint32_t actual_asr=actual_inter/actual_ad;
    bewe_log("DEM[%d]: mode=%d  cf=%.4fMHz  off=%.0fHz  cap_dec=%u  asr=%u\n",
           ch_idx,(int)mode,(ch.s+ch.e)/2.0f,off_hz,cap_decim,actual_asr);
    // 분수 리샘플러: actual_asr(=msr/cap_decim/actual_ad)는 msr이 48k의 정수배가
    // 아니면 48000과 어긋남 (3.2M: +1.01%, 2.56M: +0.63%). 생산률 > 소비율(ALSA 48k)이면
    // JOIN jitter buffer가 JITTER_MAX 도달 시마다 ~100ms를 잘라내 주기적 글리치 발생.
    // 위상 누적 + 선형보간으로 정확히 48 kHz 출력 (비율이 1 근방이라 보간 왜곡 무시 가능).
    double rs_step=(double)msr/(double)cap_decim/(double)actual_ad/(double)AUDIO_SR;
    double rs_pos=0.0; float rs_prev=0.0f;

    // ── DSP state ─────────────────────────────────────────────────────────
    Oscillator osc; osc.set_freq((double)off_hz,(double)msr);
    uint64_t prev_cf = init_cf;
    double cap_i=0,cap_q=0; int cap_cnt=0;
    // BW LPF cascade (4-stage IIR1) — pre-decim 단계에서 anti-alias
    IIR1 lpi[4],lpq[4];
    { float cn=(bw_hz*0.5f)/(float)msr; if(cn>0.45f)cn=0.45f;
      for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); } }
    float prev_i=0,prev_q=0,am_dc=0;
    float am_dc_alpha=1.0f-expf(-2.0f*M_PI*30.0f/(float)actual_inter);
    // v4.5.0 — alf 적응형: 채널 BW/2 와 12 kHz 중 작은 쪽 (좁은 채널 → 좁은 LPF → 잡음 ↓).
    // 이전엔 고정 8 kHz 라 narrowband 무전기 (BW 3 kHz) 도 8 kHz 까지 통과시켜 SNR 손해.
    float alf_cut_hz = std::min(12000.0f, bw_hz * 0.5f);
    IIR1 alf; alf.set(alf_cut_hz / (float)actual_inter);
    // v4.5.0 — FM 50 µs de-emphasis (turnover 3183 Hz). 송신측 pre-emphasis 보정 +
    // FM 의 f² 잡음 분포 cut. 한국 방송 FM 표준 (50 µs) 와 매칭. AM 은 미적용.
    IIR1 deemph; deemph.set(3183.0f / (float)actual_inter);

    // ── AM AGC ────────────────────────────────────────────────────────────
    float agc_rms=0.01f;
    const float AGC_TARGET =1.0f;
    const float AGC_ATTACK =0.001f;
    const float AGC_RELEASE=0.0001f;

    double aac=0; int acnt=0;

    // 스컬치는 UI 스레드에서 FFT 기반으로 중앙 관리 (sq_gate 읽기만)
    bool gate_open=false;

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =(size_t)cap_decim*actual_asr/50;

    while(!ch.dem_stop_req.load(std::memory_order_relaxed) && !sdr_stream_error.load()){
        // center frequency 변경 감지 → 오실레이터 재설정
        { uint64_t cur_cf=live_cf_hz.load(std::memory_order_acquire);
          if(cur_cf!=prev_cf){
              off_hz=(((ch.s+ch.e)/2.0f)-(float)(cur_cf/1e6))*1e6f;
              osc.set_freq((double)off_hz,(double)msr);
              prev_cf=cur_cf;
          }
        }
        size_t wp=ring_wp.load(std::memory_order_acquire);
        size_t rp=ch.dem_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;

        // Lag limiter: reset if too far behind
        if(lag>MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK;
            ch.dem_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            alf.s=0; prev_i=prev_q=0; am_dc=0;
            aac=0; acnt=0; cap_i=cap_q=0; cap_cnt=0;
            rs_pos=0.0; rs_prev=0.0f;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/hw.iq_scale, sq=ring[pos*2+1]/hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            // 4-stage LPF cascade (anti-alias BEFORE decimation)
            float mi_aa=mi, mq_aa=mq;
            mi_aa=lpi[0].p(mi_aa); mi_aa=lpi[1].p(mi_aa); mi_aa=lpi[2].p(mi_aa); mi_aa=lpi[3].p(mi_aa);
            mq_aa=lpq[0].p(mq_aa); mq_aa=lpq[1].p(mq_aa); mq_aa=lpq[2].p(mq_aa); mq_aa=lpq[3].p(mq_aa);
            cap_i+=mi_aa; cap_q+=mq_aa; cap_cnt++;
            if(cap_cnt<(int)cap_decim) continue;
            float fi=(float)(cap_i/cap_cnt), fq=(float)(cap_q/cap_cnt);
            cap_i=cap_q=0; cap_cnt=0;

            // 스컬치 게이트 읽기 (UI 스레드에서 FFT 기반으로 관리)
            float p_inst=fi*fi+fq*fq;
            gate_open = ch.sq_gate.load(std::memory_order_relaxed);

            // per-channel IQ recording (squelch-gated, decimated baseband)
            ch.maybe_rec_iq(fi, fq, gate_open);

            // ── Demodulate ────────────────────────────────────────────────
            {
                float samp=0;
                if(mode==Channel::DM_AM){
                    // AM: envelope detection + AGC
                    float env2=sqrtf(p_inst);
                    am_dc+=am_dc_alpha*(env2-am_dc);
                    float audio=alf.p(env2-am_dc);
                    float rms_in=audio*audio;
                    if(rms_in>agc_rms) agc_rms+=(rms_in-agc_rms)*AGC_ATTACK;
                    else               agc_rms+=(rms_in-agc_rms)*AGC_RELEASE;
                    float gain=(agc_rms>1e-9f)?(AGC_TARGET/sqrtf(agc_rms)):100.0f;
                    gain=std::min(gain,1000.0f);
                    samp=std::max(-1.0f,std::min(1.0f,audio*gain));
                } else {
                    // FM: phase discriminator
                    float cross=fi*prev_q-fq*prev_i, dot=fi*prev_i+fq*prev_q;
                    float d=atan2f(cross,dot+1e-12f); prev_i=fi; prev_q=fq;
                    // alf (적응형 채널 BW LPF) → deemph (50 µs 6dB/oct from 3.18 kHz) → 4배 gain
                    samp = deemph.p(alf.p(d)) * 4.0f;
                }
                aac+=samp; acnt++;
                if(acnt>=(int)actual_ad){
                    float raw_out=std::max(-1.0f,std::min(1.0f,(float)(aac/acnt)));
                    aac=0; acnt=0;
                    // 분수 리샘플 (actual_asr → 정확히 AUDIO_SR): rs_pos<1 동안 보간 출력
                    while(rs_pos<1.0){
                        float r=rs_prev+(float)rs_pos*(raw_out-rs_prev);
                        float out=gate_open?r:0.0f;
                        ch.maybe_rec_audio(r, gate_open);
                        // 외부 디코더(DMR 음성)가 오디오 소유 시 FM/AM 오디오 억제 (잡음 대신 무음/음성)
                        if(!ch.ext_audio.load(std::memory_order_relaxed)){
                          ch.push_audio(out);
                          // ── 네트워크 오디오 전송 (스컬치 초과 시만) ────────────────────
                          if(net_srv && gate_open && (ch.audio_mask.load() & ~0x1u)){
                            net_audio_buf.push_back(out);
                            if((int)net_audio_buf.size()>=NET_AUDIO_BATCH){
                                uint32_t mask=(ch.audio_mask.load()>>1);
                                net_srv->send_audio(mask,(uint8_t)ch_idx,(int8_t)ch.pan,
                                    net_audio_buf.data(),(uint32_t)net_audio_buf.size());
                                net_audio_buf.clear();
                            }
                          }
                        }
                        rs_pos+=rs_step;
                    }
                    rs_pos-=1.0;
                    rs_prev=raw_out;
                }
            }
        }
        ch.dem_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    bewe_log("DEM[%d] worker exited\n",ch_idx);
}

void FFTViewer::start_dem(int ch_idx, Channel::DemodMode mode){
    Channel& ch=channels[ch_idx];
    if(ch.dem_run.load()||!ch.filter_active) return;
    ch.mode=mode;
    ch.dem_rp.store(ring_wp.load());
    ch.dem_stop_req.store(false);
    ch.dem_run.store(true);
    ch.dem_thr=std::thread(&FFTViewer::dem_worker,this,ch_idx);
    const char* n[]={"NONE","AM","FM"};
    bewe_log("DEM[%d] start: %s  %.4f-%.4f MHz\n",ch_idx,n[(int)mode],ch.s,ch.e);
}

void FFTViewer::stop_dem(int ch_idx, bool stop_decoders){
    Channel& ch=channels[ch_idx];
    // 재튜닝(주파수/대역폭/모드 변경)에 의한 오디오 demod 재시작은 IQ-탭 디코더(ACARS/
    // AIS/ADS-B 등)를 건드리면 안 됨 — 디코더는 demod 와 독립. stop_decoders=false 면 보존.
    // 진짜 종료(채널 삭제/전체 stop)에서만 on_ch_stop 으로 디코더 정리.
    if(stop_decoders)
        for(auto& bm : bewe_modules())
            if(bm.on_ch_stop) bm.on_ch_stop(*this, ch_idx);
    if(!ch.dem_run.load()) return;
    ch.dem_stop_req.store(true);
    if(ch.dem_thr.joinable()) ch.dem_thr.join();
    ch.dem_run.store(false);
    ch.mode=Channel::DM_NONE;
}

void FFTViewer::stop_all_dem(){
    for(int i=0;i<MAX_CHANNELS;i++){
        stop_dem(i);
    }
}

// ── 주파수 변경 시 채널 복조 pause / resume ──────────────────────────────
// 호출 시점: 주파수 변경 직후 (캡처 스레드 내)
// new_cf_mhz: 새 center frequency
void FFTViewer::update_dem_by_freq(float new_cf_mhz){
    float eff_half = hw.sample_rate_mhz * hw.eff_bw_ratio * 0.5f;
    float vis_lo   = new_cf_mhz - eff_half;
    float vis_hi   = new_cf_mhz + eff_half;

    for(int i=0;i<MAX_CHANNELS;i++){
        Channel& ch = channels[i];
        if(!ch.filter_active) continue;  // 필터 없으면 무시

        // 채널 대역 전체가 가시 범위 안에 포함되어야 visible (full containment)
        // 일부라도 범위 밖이면 Holding — 음성 깨짐 방지
        float ch_lo = std::min(ch.s, ch.e);
        float ch_hi = std::max(ch.s, ch.e);
        bool visible = (ch_lo >= vis_lo) && (ch_hi <= vis_hi);

        if(visible){
            // 범위 안 → pause 상태였으면 해제 (+ 보존된 mode가 있으면 복조 재개)
            if(ch.dem_paused.load()){
                ch.dem_paused.store(false);
                if(ch.dem_paused_mode != Channel::DM_NONE){
                    start_dem(i, ch.dem_paused_mode);
                    ch.dem_paused_mode = Channel::DM_NONE;
                }
            }
        } else {
            // 범위 밖 → 항상 Holding 표시. 복조 중이었으면 mode 보존 + stop_dem
            if(!ch.dem_paused.load()){
                if(ch.dem_run.load()){
                    ch.dem_paused_mode = ch.mode;
                    stop_dem(i,false);   // Holding 은 오디오 demod 만 정지 — IQ-탭 디코더 보존
                }
                ch.dem_paused.store(true);
            }
        }
    }

    // JOIN에 pause 상태 즉시 반영
    if(net_srv) net_srv->broadcast_channel_sync(channels, MAX_CHANNELS);
}