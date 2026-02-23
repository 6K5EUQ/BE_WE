#include "fft_viewer.hpp"
#include <cstring>
#include <algorithm>
#include <chrono>

bool FFTViewer::initialize_bladerf(float cf_mhz, float sr_msps){
    int s=bladerf_open(&dev,nullptr);
    if(s){ fprintf(stderr,"bladerf_open: %s\n",bladerf_strerror(s)); return false; }

    s=bladerf_set_frequency(dev,CHANNEL,(uint64_t)(cf_mhz*1e6));
    if(s){ fprintf(stderr,"set_freq: %s\n",bladerf_strerror(s)); bladerf_close(dev); return false; }

    uint32_t actual=0;
    s=bladerf_set_sample_rate(dev,CHANNEL,(uint32_t)(sr_msps*1e6),&actual);
    if(s){ fprintf(stderr,"set_sr: %s\n",bladerf_strerror(s)); bladerf_close(dev); return false; }

    uint32_t actual_bw=0;
    s=bladerf_set_bandwidth(dev,CHANNEL,(uint32_t)(sr_msps*1e6*0.8f),&actual_bw);
    if(s){ fprintf(stderr,"set_bw: %s\n",bladerf_strerror(s)); bladerf_close(dev); return false; }

    s=bladerf_set_gain(dev,CHANNEL,RX_GAIN);
    if(s){ fprintf(stderr,"set_gain: %s\n",bladerf_strerror(s)); bladerf_close(dev); return false; }

    s=bladerf_enable_module(dev,CHANNEL,true);
    if(s){ fprintf(stderr,"enable: %s\n",bladerf_strerror(s)); bladerf_close(dev); return false; }

    s=bladerf_sync_config(dev,BLADERF_RX_X1,BLADERF_FORMAT_SC16_Q11,512,16384,16,10000);
    if(s){ fprintf(stderr,"sync: %s\n",bladerf_strerror(s)); bladerf_close(dev); return false; }

    printf("BladeRF: %.2f MHz  %.2f MSPS  BW %.2f MHz\n",cf_mhz,actual/1e6f,actual_bw/1e6f);

    std::memcpy(header.magic,"FFTD",4);
    header.version=1; header.fft_size=fft_size; header.sample_rate=actual;
    header.center_frequency=(uint64_t)(cf_mhz*1e6);
    header.time_average=TIME_AVERAGE; header.power_min=-80; header.power_max=-30; header.num_ffts=0;
    fft_data.resize(MAX_FFTS_MEMORY*fft_size);
    current_spectrum.resize(fft_size,-80.0f);

    char title[256]; snprintf(title,256,"BEWE - %.2f MHz",cf_mhz);
    window_title=title; display_power_min=-80; display_power_max=0;
    fft_in =fftwf_alloc_complex(fft_size);
    fft_out=fftwf_alloc_complex(fft_size);
    fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_MEASURE);
    ring.resize(IQ_RING_CAPACITY*2,0);
    return true;
}

void FFTViewer::capture_and_process(){
    int16_t* iq=new int16_t[fft_size*2];
    std::vector<float> pacc(fft_size,0.0f); int fcnt=0;
    // 초기 안정화: 처음 N번 FFT 결과 버림
    static constexpr int WARMUP_FFTS = 30;
    int warmup_cnt = 0;

    while(is_running){
        // ── Pause (타임머신 모드) ─────────────────────────────────────────
        if(capture_pause.load(std::memory_order_relaxed)){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        // ── FFT size change ───────────────────────────────────────────────
        if(fft_size_change_req){
            fft_size_change_req=false; int ns=pending_fft_size;
            fftwf_destroy_plan(fft_plan); fftwf_free(fft_in); fftwf_free(fft_out);
            fft_size=ns; time_average=std::max(1,TIME_AVERAGE*DEFAULT_FFT_SIZE/ns);
            fft_in =fftwf_alloc_complex(fft_size);
            fft_out=fftwf_alloc_complex(fft_size);
            fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_MEASURE);
            delete[] iq; iq=new int16_t[fft_size*2];
            pacc.assign(fft_size,0.0f); fcnt=0;
            {std::lock_guard<std::mutex> lk(data_mtx);
             header.fft_size=fft_size;
             fft_data.assign(MAX_FFTS_MEMORY*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;
             autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;}
            texture_needs_recreate=true; continue;
        }

        // ── Frequency change ──────────────────────────────────────────────
        if(freq_req&&!freq_prog){
            freq_prog=true;
            int s=bladerf_set_frequency(dev,CHANNEL,(uint64_t)(pending_cf*1e6));
            if(!s){
                {std::lock_guard<std::mutex> lk(data_mtx);
                 header.center_frequency=(uint64_t)(pending_cf*1e6);}
                printf("Freq → %.2f MHz\n",pending_cf);
                autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
                // 주파수 변경 직후 ADC 과도현상 제거: warmup 리셋
                warmup_cnt=0;
            }
            freq_req=false; freq_prog=false;
        }

        // ── RX ────────────────────────────────────────────────────────────
        int status=bladerf_sync_rx(dev,iq,fft_size,nullptr,10000);
        if(status){ fprintf(stderr,"RX: %s\n",bladerf_strerror(status)); continue; }

        // ── IQ Ring write ─────────────────────────────────────────────────
        bool need_ring=rec_on.load(std::memory_order_relaxed);
        if(!need_ring) for(int i=0;i<MAX_CHANNELS;i++) if(channels[i].dem_run.load()){need_ring=true;break;}
        bool need_tm=tm_iq_on.load(std::memory_order_relaxed)&&(warmup_cnt>=WARMUP_FFTS);
        if(need_ring||need_tm){
            size_t wp=ring_wp.load(std::memory_order_relaxed);
            size_t n=(size_t)fft_size, cap=IQ_RING_CAPACITY;
            if(wp+n<=cap) memcpy(&ring[wp*2],iq,n*2*sizeof(int16_t));
            else{
                size_t p1=cap-wp, p2=n-p1;
                memcpy(&ring[wp*2],iq,p1*2*sizeof(int16_t));
                memcpy(&ring[0],iq+p1*2,p2*2*sizeof(int16_t));
            }
            ring_wp.store((wp+n)&IQ_RING_MASK,std::memory_order_release);
            // TM IQ 롤링 저장 (워밍업 이후만)
            if(need_tm) tm_iq_write(iq,(int)n);
        }

        // ── FFT ───────────────────────────────────────────────────────────
        // render_visible=false(좌측 패널 완전 숨김)이면 FFT 연산 스킵
        // 스트리밍/채널복조/롤링IQ는 위에서 이미 처리됨
        if(!render_visible.load(std::memory_order_relaxed)){
            std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            continue;
        }
        for(int i=0;i<fft_size;i++){
            fft_in[i][0]=iq[i*2]/2048.0f;
            fft_in[i][1]=iq[i*2+1]/2048.0f;
        }
        if(!spectrum_pause.load(std::memory_order_relaxed)){
            apply_hann(fft_in,fft_size);
            fftwf_execute(fft_plan);
            {
                const float scale=HANN_WINDOW_CORRECTION/((float)fft_size*(float)fft_size);
                for(int i=0;i<fft_size;i++){
                    float ms=(fft_out[i][0]*fft_out[i][0]+fft_out[i][1]*fft_out[i][1])*scale+1e-10f;
                    pacc[i]+=10.0f*log10f(ms);
                }
            }
            pacc[0]=(pacc[1]+pacc[fft_size-1])*0.5f; fcnt++;
            if(fcnt>=time_average){
                // 초기 워밍업 버리기
                if(warmup_cnt < WARMUP_FFTS){
                    warmup_cnt++;
                    std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
                    continue;
                }
                int fi=total_ffts%MAX_FFTS_MEMORY;
                int8_t* rowp=fft_data.data()+fi*fft_size;
                {std::lock_guard<std::mutex> lk(data_mtx);
                 for(int i=0;i<fft_size;i++){
                     float avg=pacc[i]/fcnt;
                     float nn=(avg-header.power_min)/(header.power_max-header.power_min);
                     rowp[i]=(int8_t)(std::max(-1.0f,std::min(1.0f,nn))*127);
                     current_spectrum[i]=avg;
                 }
                 if(autoscale_active){
                     if(!autoscale_init){
                         autoscale_accum.reserve(fft_size*200);
                         autoscale_last=std::chrono::steady_clock::now();
                         autoscale_init=true;
                     }
                     for(int i=1;i<fft_size;i++) autoscale_accum.push_back(current_spectrum[i]);
                     float el=std::chrono::duration<float>(std::chrono::steady_clock::now()-autoscale_last).count();
                     if(el>=1.0f&&!autoscale_accum.empty()){
                         size_t idx=(size_t)(autoscale_accum.size()*0.15f);
                         std::nth_element(autoscale_accum.begin(),autoscale_accum.begin()+idx,autoscale_accum.end());
                         display_power_min=autoscale_accum[idx]-10.0f;
                         autoscale_accum.clear(); autoscale_active=false; cached_sp_idx=-1;
                     }
                 }
                 total_ffts++; current_fft_idx=total_ffts-1;
                 header.num_ffts=std::min(total_ffts,MAX_FFTS_MEMORY); cached_sp_idx=-1;
                 // 이 행의 IQ 끝 위치 기록 (캡처 스레드 내에서 atomic하게)
                 row_write_pos[current_fft_idx%MAX_FFTS_MEMORY]=tm_iq_write_sample;
                 if(tm_iq_on.load(std::memory_order_relaxed))
                     tm_mark_rows(current_fft_idx%MAX_FFTS_MEMORY);
                 else
                     iq_row_avail[current_fft_idx%MAX_FFTS_MEMORY]=false;
                 tm_add_time_tag(current_fft_idx);
                }
                std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            }
        } // end !spectrum_pause
    }
    delete[] iq;
}