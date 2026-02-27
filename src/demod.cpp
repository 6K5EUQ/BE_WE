#include "fft_viewer.hpp"
#include "net_server.hpp"
#include <cmath>
#include <algorithm>
#include <vector>
#include <chrono>

// forward declaration
static int magic_classify(const std::vector<float>&, const std::vector<float>&,
                          const std::vector<float>&, const std::vector<float>&,
                          uint32_t);

void FFTViewer::dem_worker(int ch_idx){
    Channel& ch=channels[ch_idx];
    Channel::DemodMode mode=ch.mode;
    // 네트워크 오디오 배치 버퍼 (256샘플 단위로 서버에 전송)
    static constexpr int NET_AUDIO_BATCH = 256;
    std::vector<float> net_audio_buf;
    net_audio_buf.reserve(NET_AUDIO_BATCH);
    uint32_t msr=header.sample_rate;
    float off_hz=(((ch.s+ch.e)/2.0f)-(float)(header.center_frequency/1e6f))*1e6f;
    float bw_hz=fabsf(ch.e-ch.s)*1e6f;

    uint32_t inter_sr,audio_decim,cap_decim;
    demod_rates(msr,bw_hz,inter_sr,audio_decim,cap_decim);
    uint32_t actual_inter=msr/cap_decim;
    uint32_t actual_ad=std::max(1u,(uint32_t)round((double)actual_inter/AUDIO_SR));
    uint32_t actual_asr=actual_inter/actual_ad;
    bewe_log("DEM[%d]: mode=%d  cf=%.4fMHz  off=%.0fHz  cap_dec=%u  asr=%u\n",
           ch_idx,(int)mode,(ch.s+ch.e)/2.0f,off_hz,cap_decim,actual_asr);

    // ── Magic mode state ──────────────────────────────────────────────────
    const int MAGIC_ANALYZE_SAMP=(int)(actual_inter*0.6f); // 600ms analysis window
    std::vector<float> mg_env, mg_freq, mg_ibuf, mg_qbuf;
    int  magic_det_mode=0;      // 0=analyzing, 1~5=detected
    bool magic_analyzed=false;
    Channel::DemodMode magic_active_mode=Channel::DM_NONE;
    if(mode==Channel::DM_MAGIC){
        mg_env.reserve(MAGIC_ANALYZE_SAMP);
        mg_freq.reserve(MAGIC_ANALYZE_SAMP);
        mg_ibuf.reserve(MAGIC_ANALYZE_SAMP);
        mg_qbuf.reserve(MAGIC_ANALYZE_SAMP);
        ch.magic_det.store(0,std::memory_order_relaxed);
    }

    // ── DSP state ─────────────────────────────────────────────────────────
    Oscillator osc; osc.set_freq((double)off_hz,(double)msr);
    double cap_i=0,cap_q=0; int cap_cnt=0;
    IIR1 lpi,lpq;
    { float cn=(bw_hz*0.5f)/(float)actual_inter; if(cn>0.45f)cn=0.45f; lpi.set(cn); lpq.set(cn); }
    float prev_i=0,prev_q=0,am_dc=0;
    float am_dc_alpha=1.0f-expf(-2.0f*M_PI*30.0f/(float)actual_inter);
    IIR1 alf; alf.set(8000.0/actual_inter);

    // ── AM AGC ────────────────────────────────────────────────────────────
    float agc_rms=0.01f;
    const float AGC_TARGET =1.0f;
    const float AGC_ATTACK =0.001f;
    const float AGC_RELEASE=0.0001f;

    double aac=0; int acnt=0;
    size_t aac_detect_pos=0;

    // ── Squelch ───────────────────────────────────────────────────────────
    const float SQL_ALPHA     = 0.05f;
    const int   SQL_HOLD_SAMP = 0;
    const int   CALIB_SAMP    = (int)(actual_inter * 0.500f);
    float sql_avg=-120.0f;
    std::vector<float> calib_buf;
    bool calibrated=ch.sq_calibrated.load(std::memory_order_relaxed);
    if(!calibrated) calib_buf.reserve(CALIB_SAMP);
    bool gate_open=false;
    int  gate_hold=0;
    int  sq_ui_tick=0;

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =(size_t)cap_decim*actual_asr/50;

    while(!ch.dem_stop_req.load(std::memory_order_relaxed)){
        size_t wp=ring_wp.load(std::memory_order_acquire);
        size_t rp=ch.dem_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;

        // Lag limiter: reset if too far behind
        if(lag>MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK;
            ch.dem_rp.store(rp,std::memory_order_release);
            lpi.s=lpq.s=alf.s=0; prev_i=prev_q=0; am_dc=0;
            aac=0; acnt=0; cap_i=cap_q=0; cap_cnt=0;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/2048.0f, sq=ring[pos*2+1]/2048.0f;
            float mi,mq; osc.mix(si,sq,mi,mq);
            cap_i+=mi; cap_q+=mq; cap_cnt++;
            if(cap_cnt<(int)cap_decim) continue;
            float fi=(float)(cap_i/cap_cnt), fq=(float)(cap_q/cap_cnt);
            cap_i=cap_q=0; cap_cnt=0;
            fi=lpi.p(fi); fq=lpq.p(fq);

            // ── Squelch ───────────────────────────────────────────────────
            float p_inst=fi*fi+fq*fq;
            float db_inst=(p_inst>1e-12f)?10.0f*log10f(p_inst):-120.0f;
            sql_avg=SQL_ALPHA*db_inst+(1.0f-SQL_ALPHA)*sql_avg;

            if(!calibrated){
                if((int)calib_buf.size()<CALIB_SAMP) calib_buf.push_back(db_inst);
                if((int)calib_buf.size()>=CALIB_SAMP){
                    std::vector<float> tmp=calib_buf;
                    size_t p20=tmp.size()/5;
                    std::nth_element(tmp.begin(),tmp.begin()+p20,tmp.end());
                    float noise_floor=tmp[p20];
                    ch.sq_threshold.store(noise_floor+10.0f,std::memory_order_relaxed);
                    calibrated=true;
                    ch.sq_calibrated.store(true,std::memory_order_relaxed);
                    calib_buf.clear(); calib_buf.shrink_to_fit();
                }
            }
            float thr=ch.sq_threshold.load(std::memory_order_relaxed);
            const float HYS=3.0f;
            if(calibrated){
                if(!gate_open&&sql_avg>=thr){ gate_open=true; gate_hold=SQL_HOLD_SAMP; }
                if( gate_open){
                    if(sql_avg>=thr-HYS) gate_hold=SQL_HOLD_SAMP;
                    else if(--gate_hold<=0){ gate_open=false; gate_hold=0; }
                }
            }
            if(++sq_ui_tick>=256){ sq_ui_tick=0;
                ch.sq_sig .store(sql_avg,  std::memory_order_relaxed);
                ch.sq_gate.store(gate_open,std::memory_order_relaxed);
            }

            // ── Demodulate ────────────────────────────────────────────────
            if(mode==Channel::DM_MAGIC){
                // ── Phase 1: analysis ─────────────────────────────────────
                if(!magic_analyzed){
                    float env=sqrtf(p_inst);
                    float cross=fi*prev_q-fq*prev_i, dot=fi*prev_i+fq*prev_q;
                    float inst_f=atan2f(cross,dot+1e-12f);
                    prev_i=fi; prev_q=fq;
                    mg_env.push_back(env);
                    mg_freq.push_back(inst_f);
                    mg_ibuf.push_back(fi);
                    mg_qbuf.push_back(fq);
                    if((int)mg_env.size()>=MAGIC_ANALYZE_SAMP){
                        magic_det_mode=magic_classify(mg_env,mg_freq,mg_ibuf,mg_qbuf,actual_asr);
                        magic_analyzed=true;
                        ch.magic_det.store(magic_det_mode,std::memory_order_relaxed);
                        mg_env.clear(); mg_freq.clear(); mg_ibuf.clear(); mg_qbuf.clear();
                        // Map detected mode
                        switch(magic_det_mode){
                            case 1: magic_active_mode=Channel::DM_AM;  break;
                            case 3: magic_active_mode=Channel::DM_AM;  break; // DSB: AM demod
                            case 4: magic_active_mode=Channel::DM_FM;  break; // SSB: FM demod (approx)
                            default: magic_active_mode=Channel::DM_FM; break; // FM, CW
                        }
                        // Reset DSP state for clean demod start
                        lpi.s=lpq.s=alf.s=0; prev_i=prev_q=0; am_dc=0;
                        aac=0; acnt=0; agc_rms=0.01f;
                        bewe_log("MAGIC[%d]: detected=%d\n",ch_idx,magic_det_mode);
                    }
                    // During analysis: silence
                    acnt++;
                    if(acnt>=(int)actual_ad){ acnt=0; ch.push_audio(0.0f); }
                } else {
                    // ── Phase 2: demodulate with detected mode ────────────
                    float samp=0;
                    if(magic_active_mode==Channel::DM_AM){
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
                        float cross=fi*prev_q-fq*prev_i, dot=fi*prev_i+fq*prev_q;
                        float d=atan2f(cross,dot+1e-12f); prev_i=fi; prev_q=fq;
                        samp=alf.p(d)*4.0f;
                    }
                    aac+=samp; acnt++;
                    if(acnt>=(int)actual_ad){
                        float out=gate_open
                                  ?std::max(-1.0f,std::min(1.0f,(float)(aac/acnt)))
                                  :0.0f;
                        aac=0; acnt=0;
                        ch.maybe_rec_audio(out);
                        ch.push_audio(out);
                        // ── 네트워크 오디오 전송 ────────────────────
                        if(net_srv && (ch.audio_mask.load() & ~0x1u)){
                            net_audio_buf.push_back(out);
                            if((int)net_audio_buf.size()>=NET_AUDIO_BATCH){
                                uint32_t mask=(ch.audio_mask.load()>>1);
                                net_srv->send_audio(mask,(uint8_t)ch_idx,(int8_t)ch.pan,
                                    net_audio_buf.data(),(uint32_t)net_audio_buf.size());
                                net_audio_buf.clear();
                            }
                        }
                    }
                }
            } else {
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
                    samp=alf.p(d)*4.0f;
                }
                aac+=samp; acnt++;
                if(acnt>=(int)actual_ad){
                    float out=gate_open
                              ?std::max(-1.0f,std::min(1.0f,(float)(aac/acnt)))
                              :0.0f;
                    aac=0; acnt=0;
                    ch.maybe_rec_audio(out);
                    ch.push_audio(out);
                    // ── 네트워크 오디오 전송 ────────────────────────
                    if(net_srv && (ch.audio_mask.load() & ~0x1u)){
                        net_audio_buf.push_back(out);
                        if((int)net_audio_buf.size()>=NET_AUDIO_BATCH){
                            uint32_t mask=(ch.audio_mask.load()>>1);
                            net_srv->send_audio(mask,(uint8_t)ch_idx,(int8_t)ch.pan,
                                net_audio_buf.data(),(uint32_t)net_audio_buf.size());
                            net_audio_buf.clear();
                        }
                    }
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
    if(mode == Channel::DM_DMR)
        ch.dem_thr=std::thread(&FFTViewer::dmr_worker,this,ch_idx);
    else
        ch.dem_thr=std::thread(&FFTViewer::dem_worker,this,ch_idx);
    const char* n[]={"NONE","AM","FM","MAGIC","DMR"};
    bewe_log("DEM[%d] start: %s  %.4f-%.4f MHz\n",ch_idx,n[(int)mode],ch.s,ch.e);
}

void FFTViewer::stop_dem(int ch_idx){
    Channel& ch=channels[ch_idx];
    if(!ch.dem_run.load()) return;
    ch.dem_stop_req.store(true);
    if(ch.dem_thr.joinable()) ch.dem_thr.join();
    ch.dem_run.store(false);
    ch.mode=Channel::DM_NONE;
}

void FFTViewer::stop_all_dem(){
    for(int i=0;i<MAX_CHANNELS;i++) stop_dem(i);
}

// ── Magic mode: modulation classifier ────────────────────────────────────
// Returns: 1=AM, 2=FM, 3=DSB, 4=SSB, 5=CW
static int magic_classify(
    const std::vector<float>& env_buf,   // envelope samples
    const std::vector<float>& freq_buf,  // inst frequency samples
    const std::vector<float>& i_buf,     // decimated I
    const std::vector<float>& q_buf,     // decimated Q
    uint32_t asr)                        // audio sample rate
{
    size_t N=env_buf.size(); if(N<64) return 1;

    // ── 1. Envelope statistics ────────────────────────────────────────────
    double env_mean=0;
    for(float v:env_buf) env_mean+=v;
    env_mean/=N;

    double env_var=0;
    for(float v:env_buf){ double d=v-env_mean; env_var+=d*d; }
    env_var/=N;

    double env_norm_var=(env_mean>1e-6f)? env_var/(env_mean*env_mean) : 0.0;

    // ── 2. Instantaneous frequency statistics ─────────────────────────────
    double freq_mean=0;
    for(float v:freq_buf) freq_mean+=v;
    freq_mean/=N;

    double freq_var=0;
    for(float v:freq_buf){ double d=v-freq_mean; freq_var+=d*d; }
    freq_var/=N;

    // ── 3. Spectral asymmetry (SSB detection) ─────────────────────────────
    // Simple: compare power of upper vs lower half of spectrum via sign of
    // mean instantaneous frequency deviation
    size_t pos_cnt=0;
    for(float v:freq_buf) if(v>0) pos_cnt++;
    double freq_asym=((double)pos_cnt/N)-0.5; // [-0.5, +0.5], |>0.3| = SSB

    // ── 4. CW detection: envelope bimodality ─────────────────────────────
    // Histogram env into low/high bins around mean
    size_t low_cnt=0, hi_cnt=0;
    double thresh=env_mean*0.5;
    for(float v:env_buf){
        if(v<thresh) low_cnt++;
        else         hi_cnt++;
    }
    double bimodal_ratio=(double)low_cnt/N; // CW: lots of near-zero (key-up)

    // ── 5. Carrier presence (AM vs DSB) ──────────────────────────────────
    // AM has strong carrier → envelope mean >> envelope AC component
    // Normalized: env_norm_var < 0.3 suggests strong carrier presence
    // (DSB suppressed carrier → higher normalized variance)
    double carrier_score = 1.0 - std::min(1.0, env_norm_var / 1.5);

    // ── Decision tree ─────────────────────────────────────────────────────
    // FM: low envelope variance, high freq variance
    if(env_norm_var < 0.15 && freq_var > 0.05)
        return 2; // FM

    // CW: bimodal envelope (many near-zero samples), low freq_var
    if(bimodal_ratio > 0.35 && freq_var < 0.1 && env_norm_var > 0.3)
        return 5; // CW

    // SSB: strong spectral asymmetry
    if(fabs(freq_asym) > 0.28 && env_norm_var > 0.1)
        return 4; // SSB

    // AM vs DSB: carrier score
    if(carrier_score > 0.5)
        return 1; // AM (carrier present)
    else
        return 3; // DSB (suppressed carrier)
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

        // 채널 범위가 보이는 주파수 범위와 겹치는지 확인
        bool visible = (ch.e > vis_lo) && (ch.s < vis_hi);

        if(visible){
            // 범위 안 → pause 상태였으면 재시작
            if(ch.dem_paused && ch.dem_paused_mode != Channel::DM_NONE){
                ch.dem_paused = false;
                start_dem(i, ch.dem_paused_mode);
            }
        } else {
            // 범위 밖 → 복조 중이면 pause
            if(ch.dem_run.load() && !ch.dem_paused){
                ch.dem_paused      = true;
                ch.dem_paused_mode = ch.mode;  // mode 보존
                stop_dem(i);                   // stop_dem이 mode=NONE으로 지움
            }
        }
    }
}