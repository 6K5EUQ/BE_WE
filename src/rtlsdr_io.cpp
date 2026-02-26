#include "fft_viewer.hpp"
#include "net_server.hpp"
#include <cstring>
#include <algorithm>
#include <chrono>
#include <rtl-sdr.h>

// RTL-SDR V4 고정 파라미터
static constexpr uint32_t RTL_SAMPLE_RATE = 2560000;  // 2.56 MSPS (안정 최대)
static constexpr int      RTL_BUF_COUNT   = 32;       // async 버퍼 수
static constexpr int      RTL_BUF_LEN     = 16384;    // 버퍼당 바이트 (8192 IQ샘플)

// ── RTL-SDR 초기화 ────────────────────────────────────────────────────────
bool FFTViewer::initialize_rtlsdr(float cf_mhz){
    uint32_t dev_count = rtlsdr_get_device_count();
    if(dev_count == 0){ fprintf(stderr,"RTL-SDR: no device found\n"); return false; }

    int r = rtlsdr_open(&dev_rtl, 0);
    if(r < 0){ fprintf(stderr,"RTL-SDR: open failed (%d)\n",r); return false; }

    // 샘플레이트 설정
    r = rtlsdr_set_sample_rate(dev_rtl, RTL_SAMPLE_RATE);
    if(r < 0){ fprintf(stderr,"RTL-SDR: set_sample_rate failed\n"); rtlsdr_close(dev_rtl); dev_rtl=nullptr; return false; }
    uint32_t actual_sr = rtlsdr_get_sample_rate(dev_rtl);

    // 주파수 설정
    r = rtlsdr_set_center_freq(dev_rtl, (uint32_t)(cf_mhz * 1e6));
    if(r < 0){ fprintf(stderr,"RTL-SDR: set_center_freq failed\n"); rtlsdr_close(dev_rtl); dev_rtl=nullptr; return false; }

    // 튜너 BW 자동 (0 = 샘플레이트 기준 자동)
    rtlsdr_set_tuner_bandwidth(dev_rtl, 0);

    // 수동 게인 모드, 39.6 dB
    rtlsdr_set_tuner_gain_mode(dev_rtl, 1);
    rtlsdr_set_tuner_gain(dev_rtl, RTLSDR_RX_GAIN_TENTHS);

    // RTL AGC 켜기
    rtlsdr_set_agc_mode(dev_rtl, 1);

    // 버퍼 초기화
    rtlsdr_reset_buffer(dev_rtl);

    hw = make_rtlsdr_config(actual_sr);
    gain_db = hw.gain_default;

    printf("RTL-SDR: %.2f MHz  %.3f MSPS  gain %.1f dB\n",
           cf_mhz, actual_sr/1e6, RTLSDR_RX_GAIN_TENTHS/10.0);

    // FFT 헤더
    std::memcpy(header.magic,"FFTD",4);
    header.version=1; header.fft_size=fft_size; header.sample_rate=actual_sr;
    header.center_frequency=(uint64_t)(cf_mhz*1e6);
    time_average=hw.compute_time_average(fft_size);
    header.time_average=time_average; header.power_min=-80; header.power_max=-30; header.num_ffts=0;
    fft_data.resize(MAX_FFTS_MEMORY*fft_size);
    current_spectrum.resize(fft_size,-80.0f);

    char title[256]; snprintf(title,256,"BEWE RTL-SDR - %.2f MHz",cf_mhz);
    window_title=title; display_power_min=-80; display_power_max=0;
    fft_in =fftwf_alloc_complex(fft_size);
    fft_out=fftwf_alloc_complex(fft_size);
    fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_MEASURE);
    ring.resize(IQ_RING_CAPACITY*2,0);
    return true;
}

// ── RTL-SDR 주파수 변경 ───────────────────────────────────────────────────
void FFTViewer::set_frequency(float cf_mhz){
    if(hw.type == HWType::BLADERF){
        bladerf_set_frequency(dev_blade, BLADERF_CHANNEL_RX(0), (uint64_t)(cf_mhz*1e6));
    } else if(hw.type == HWType::RTLSDR){
        rtlsdr_set_center_freq(dev_rtl, (uint32_t)(cf_mhz*1e6));
    }
    {std::lock_guard<std::mutex> lk(data_mtx);
     header.center_frequency=(uint64_t)(cf_mhz*1e6);}
    printf("Freq → %.2f MHz\n", cf_mhz);
    autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
}

// ── RTL-SDR 캡처 루프 ─────────────────────────────────────────────────────
// RTL-SDR은 uint8 IQ, center=127.5
// 동기 read 방식 사용 (async보다 지연 제어 쉬움)
void FFTViewer::capture_and_process_rtl(){
    const int n_bytes = fft_size * 2; // 각 샘플 I+Q = 2 bytes
    uint8_t*  raw     = new uint8_t[n_bytes];
    // ring에 넣을 int16 변환 버퍼 (채널 복조, TM IQ는 int16 사용)
    int16_t*  iq16    = new int16_t[fft_size * 2];

    std::vector<float> pacc(fft_size, 0.0f);
    int   fcnt      = 0;
    static constexpr int WARMUP_FFTS = 15; // RTL-SDR은 BladeRF 절반 (더 적은 버퍼)
    int   warmup_cnt = 0;
    float iq_scale   = hw.iq_scale;   // 127.5f
    float iq_offset  = hw.iq_offset;  // 127.5f

    while(is_running){
        if(capture_pause.load(std::memory_order_relaxed)){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        // FFT size 변경
        if(fft_size_change_req){
            fft_size_change_req=false; int ns=pending_fft_size;
            fftwf_destroy_plan(fft_plan); fftwf_free(fft_in); fftwf_free(fft_out);
            fft_size=ns; time_average=hw.compute_time_average(ns);
            fft_in =fftwf_alloc_complex(fft_size);
            fft_out=fftwf_alloc_complex(fft_size);
            fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_MEASURE);
            delete[] raw;  raw  = new uint8_t[fft_size*2];
            delete[] iq16; iq16 = new int16_t[fft_size*2];
            pacc.assign(fft_size,0.0f); fcnt=0;
            {std::lock_guard<std::mutex> lk(data_mtx);
             header.fft_size=fft_size;
             fft_data.assign(MAX_FFTS_MEMORY*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;
             autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;}
            texture_needs_recreate=true; continue;
        }

        // 주파수 변경
        if(freq_req && !freq_prog){
            freq_prog=true;
            rtlsdr_set_center_freq(dev_rtl, (uint32_t)(pending_cf*1e6));
            {std::lock_guard<std::mutex> lk(data_mtx);
             header.center_frequency=(uint64_t)(pending_cf*1e6);}
            printf("Freq → %.2f MHz\n", pending_cf);
            autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
            warmup_cnt=0;
            update_dem_by_freq(pending_cf);
            freq_req=false; freq_prog=false;
        }

        // ── RX ────────────────────────────────────────────────────────────
        int n_read = 0;
        int r = rtlsdr_read_sync(dev_rtl, raw, n_bytes, &n_read);
        if(r < 0 || n_read < n_bytes){
            fprintf(stderr,"RTL-SDR RX: r=%d n_read=%d\n", r, n_read);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // uint8 → int16 변환 (채널 복조/TM IQ용)
        // iq16 스케일: (raw-127.5)/127.5 * 2048 → int16
        // → 실제로는 (raw-128)*16 (근사, 빠른 정수 연산)
        for(int i=0; i<fft_size*2; i++){
            iq16[i] = (int16_t)((int)raw[i] - 128) << 4; // ×16
        }

        // ── IQ Ring write ─────────────────────────────────────────────────
        bool need_ring = rec_on.load(std::memory_order_relaxed);
        if(!need_ring) for(int i=0;i<MAX_CHANNELS;i++) if(channels[i].dem_run.load()){need_ring=true;break;}
        bool need_tm = tm_iq_on.load(std::memory_order_relaxed) && (warmup_cnt>=WARMUP_FFTS);
        if(need_ring || need_tm){
            size_t wp=ring_wp.load(std::memory_order_relaxed);
            size_t n=(size_t)fft_size, cap=IQ_RING_CAPACITY;
            if(wp+n<=cap) memcpy(&ring[wp*2],iq16,n*2*sizeof(int16_t));
            else{
                size_t p1=cap-wp, p2=n-p1;
                memcpy(&ring[wp*2],iq16,p1*2*sizeof(int16_t));
                memcpy(&ring[0],iq16+p1*2,p2*2*sizeof(int16_t));
            }
            ring_wp.store((wp+n)&IQ_RING_MASK, std::memory_order_release);
            if(need_tm) tm_iq_write(iq16,(int)n);
        }

        // ── FFT ───────────────────────────────────────────────────────────
        if(!render_visible.load(std::memory_order_relaxed)){
            std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            continue;
        }
        // uint8 → complex float 정규화
        for(int i=0;i<fft_size;i++){
            fft_in[i][0] = ((float)raw[i*2  ] - iq_offset) / iq_scale;
            fft_in[i][1] = ((float)raw[i*2+1] - iq_offset) / iq_scale;
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
                 row_write_pos[current_fft_idx%MAX_FFTS_MEMORY]=tm_iq_write_sample;
                 if(tm_iq_on.load(std::memory_order_relaxed))
                     tm_mark_rows(current_fft_idx%MAX_FFTS_MEMORY);
                 else
                     iq_row_avail[current_fft_idx%MAX_FFTS_MEMORY]=false;
                 tm_add_time_tag(current_fft_idx);
                 net_bcast_seq.fetch_add(1, std::memory_order_release);
                 net_bcast_cv.notify_one();
                }
                std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            }
        }
    }
    delete[] raw;
    delete[] iq16;
    rtlsdr_close(dev_rtl);
    dev_rtl=nullptr;
}