#include "fft_viewer.hpp"
#include "net_server.hpp"
#include "long_waterfall.hpp"
#include <volk/volk.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <rtl-sdr.h>

// RTL-SDR V4 고정 파라미터
static constexpr uint32_t RTL_SAMPLE_RATE = 3200000;  // 3.2 MSPS
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

    // Direct Sampling: HF (<24MHz) 시 Q-branch 자동 전환
    if(cf_mhz < 24.0f){
        rtlsdr_set_direct_sampling(dev_rtl, 2);  // Q-branch
        bewe_log_push(0,"RTL-SDR: Direct Sampling ON (Q-branch) for HF\n");
    } else {
        rtlsdr_set_direct_sampling(dev_rtl, 0);  // 일반 모드
    }

    // 주파수 설정
    r = rtlsdr_set_center_freq(dev_rtl, (uint32_t)(cf_mhz * 1e6));
    if(r < 0){ fprintf(stderr,"RTL-SDR: set_center_freq failed\n"); rtlsdr_close(dev_rtl); dev_rtl=nullptr; return false; }

    // 튜너 BW 자동 (0 = 샘플레이트 기준 자동)
    rtlsdr_set_tuner_bandwidth(dev_rtl, 0);

    // 수동 게인 모드, 39.6 dB
    rtlsdr_set_tuner_gain_mode(dev_rtl, 1);
    rtlsdr_set_tuner_gain(dev_rtl, RTLSDR_RX_GAIN_TENTHS);

    // RTL AGC 끄기 (Manual Gain Control만 사용)
    rtlsdr_set_agc_mode(dev_rtl, 0);

    // 버퍼 초기화
    rtlsdr_reset_buffer(dev_rtl);

    hw = make_rtlsdr_config(actual_sr);
    gain_db = hw.gain_default;

    bewe_log_push(0,"RTL-SDR: %.2f MHz  %.3f MSPS  gain %.1f dB\n",
           cf_mhz, actual_sr/1e6, RTLSDR_RX_GAIN_TENTHS/10.0);

    // FFT 헤더
    std::memcpy(header.magic,"FFTD",4);
    fft_input_size = fft_size / FFT_PAD_FACTOR;
    header.version=1; header.fft_size=fft_size; header.sample_rate=actual_sr;
    header.center_frequency=(uint64_t)(cf_mhz*1e6);
    live_cf_hz.store((uint64_t)(cf_mhz*1e6), std::memory_order_release);
    time_average=hw.compute_time_average(fft_input_size);
    header.time_average=time_average; header.power_min=-100; header.power_max=0; header.num_ffts=0;
    fft_data.resize(MAX_FFTS_MEMORY*fft_size);
    current_spectrum.resize(fft_size,-100.0f);

    char title[256]; snprintf(title,256,"BEWE (" BEWE_VERSION ")");
    (void)cf_mhz;
    window_title=title; display_power_min=-100; display_power_max=0;
    fft_in =fftwf_alloc_complex(fft_size);
    fft_out=fftwf_alloc_complex(fft_size);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex));
    fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_ESTIMATE);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex));
    // Pre-compute Nuttall window + VOLK mag_sq buffer
    if(win_buf) free(win_buf);
    win_buf=(float*)volk_malloc(fft_input_size*sizeof(float), volk_get_alignment());
    fill_nuttall_window(win_buf, fft_input_size);
    if(mag_sq_buf) volk_free(mag_sq_buf);
    mag_sq_buf=(float*)volk_malloc(fft_size*sizeof(float), volk_get_alignment());
    ring.resize(IQ_RING_CAPACITY*2,0);
    return true;
}

// ── 공통 주파수 변경 (BladeRF/RTL-SDR/Pluto) ──────────────────────────────
void FFTViewer::set_frequency(float cf_mhz){
    if(hw.type == HWType::BLADERF){
        bladerf_set_frequency(dev_blade, BLADERF_CHANNEL_RX(0), (uint64_t)(cf_mhz*1e6));
    } else if(hw.type == HWType::RTLSDR){
        // Direct Sampling 자동 전환
        if(cf_mhz < 24.0f)
            rtlsdr_set_direct_sampling(dev_rtl, 2);
        else
            rtlsdr_set_direct_sampling(dev_rtl, 0);
        rtlsdr_set_center_freq(dev_rtl, (uint32_t)(cf_mhz*1e6));
    } else if(hw.type == HWType::PLUTO){
        // Pluto는 capture_and_process_pluto 루프가 freq_req를 처리 (LO + header + log)
        pending_cf = cf_mhz;
        freq_req = true;
        autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
        return;
    }
    {std::lock_guard<std::mutex> lk(data_mtx);
     header.center_frequency=(uint64_t)(cf_mhz*1e6);}
    live_cf_hz.store((uint64_t)(cf_mhz*1e6), std::memory_order_release);
    bewe_log_push(0,"Freq > %.2f MHz\n", cf_mhz);
    autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
    // 범위 밖 채널 Holding 전환 + JOIN에 CH_SYNC 브로드캐스트
    update_dem_by_freq(cf_mhz);
    LongWaterfall::request_rotate();   // CF changed (direct path) -> new file
}

// ── RTL-SDR 캡처 루프 ─────────────────────────────────────────────────────
// RTL-SDR은 uint8 IQ, center=127.5
// 동기 read 방식 사용 (async보다 지연 제어 쉬움)
void FFTViewer::capture_and_process_rtl(){
    static constexpr int RX_MIN = 8192; // 최소 RX 청크 (USB 오버헤드 최소화)
    int rx_chunk = std::max(fft_input_size, RX_MIN);
    size_t    n_bytes = (size_t)rx_chunk * 2;
    uint8_t*  raw     = new uint8_t[n_bytes];
    int16_t*  iq16    = new int16_t[rx_chunk * 2];
    int rx_pos  = 0; // raw/iq16 내 현재 읽기 위치 (샘플 단위)
    int rx_avail = 0;

    std::vector<float> pacc(fft_size, 0.0f);
    int   fcnt      = 0;
    static constexpr int WARMUP_FFTS = 15;
    int   warmup_cnt = 0;
    float iq_scale   = hw.iq_scale;   // 127.5f
    float iq_offset  = hw.iq_offset;  // 127.5f

    while(is_running){
        if(capture_pause.load(std::memory_order_relaxed)){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            rx_avail=0; rx_pos=0;
            continue;
        }

        // FFT size 변경 — off-lock 재할당 + atomic swap (broadcast race 방지)
        if(fft_size_change_req){
            fft_size_change_req=false; int ns=pending_fft_size;
            int new_input = ns;
            int new_fft_sz = ns * FFT_PAD_FACTOR;
            // ① Off-lock: capture 전용 FFTW/VOLK 자원 재구성
            fftwf_destroy_plan(fft_plan); fftwf_free(fft_in); fftwf_free(fft_out);
            fft_in =fftwf_alloc_complex(new_fft_sz);
            fft_out=fftwf_alloc_complex(new_fft_sz);
            memset(fft_in, 0, new_fft_sz*sizeof(fftwf_complex));
            fft_plan=fftwf_plan_dft_1d(new_fft_sz,fft_in,fft_out,FFTW_FORWARD,FFTW_ESTIMATE);
            memset(fft_in, 0, new_fft_sz*sizeof(fftwf_complex));
            if(win_buf) volk_free(win_buf);
            win_buf=(float*)volk_malloc(new_input*sizeof(float), volk_get_alignment());
            fill_nuttall_window(win_buf, new_input);
            if(mag_sq_buf) volk_free(mag_sq_buf);
            mag_sq_buf=(float*)volk_malloc(new_fft_sz*sizeof(float), volk_get_alignment());
            rx_chunk = std::max(new_input, RX_MIN);
            n_bytes = (size_t)rx_chunk * 2;
            delete[] raw;  raw  = new uint8_t[n_bytes];
            delete[] iq16; iq16 = new int16_t[rx_chunk*2];
            rx_pos=0; rx_avail=0;
            pacc.assign(new_fft_sz,0.0f); fcnt=0;
            // ② 원자적 스왑: fft_size와 fft_data 동시 교체
            {std::lock_guard<std::mutex> lk(data_mtx);
             fft_input_size=new_input;
             fft_size=new_fft_sz;
             time_average=hw.compute_time_average(fft_input_size);
             header.fft_size=fft_size;
             fft_data.assign(MAX_FFTS_MEMORY*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;}
            texture_needs_recreate=true;
            LongWaterfall::request_rotate();
            continue;
        }

        // 샘플레이트 변경
        if(sr_change_req){
            sr_change_req=false;
            uint32_t new_sr = (uint32_t)(pending_sr_msps * 1e6f);

            // TM IQ 리셋: on/off 여부 관계없이 기존 롤링 파일 삭제 후 재생성
            bool tm_was_on = tm_iq_on.load(std::memory_order_relaxed);
            if(tm_was_on) tm_iq_on.store(false);
            tm_iq_close(); // fd 닫기 + 상태 초기화
            // 기존 SR 롤링 파일 삭제 (SR 불일치 방지)
            {
                std::string tm_dir = BEWEPaths::time_temp_dir();
                char old_path[256];
                snprintf(old_path, sizeof(old_path), "%s/iq_rolling_%uMSPS.wav",
                         tm_dir.c_str(), header.sample_rate/1000000);
                if(access(old_path, F_OK)==0){ remove(old_path); }
            }

            rtlsdr_set_sample_rate(dev_rtl, new_sr);
            rtlsdr_set_tuner_bandwidth(dev_rtl, 0); // 0 = SR 기준 자동 BW
            rtlsdr_reset_buffer(dev_rtl);
            uint32_t actual_sr = rtlsdr_get_sample_rate(dev_rtl);

            // RX 버퍼 크기 재계산 (rx_chunk * 2 bytes)
            rx_chunk = std::max(fft_input_size, RX_MIN);
            n_bytes = (size_t)rx_chunk * 2;
            delete[] raw;  raw  = new uint8_t[n_bytes];
            delete[] iq16; iq16 = new int16_t[rx_chunk*2];
            rx_pos=0; rx_avail=0;

            hw = make_rtlsdr_config(actual_sr);
            iq_scale  = hw.iq_scale;
            time_average = hw.compute_time_average(fft_input_size);

            {std::lock_guard<std::mutex> lk(data_mtx);
             header.sample_rate = actual_sr;
             header.time_average = time_average;
             fft_data.assign(MAX_FFTS_MEMORY*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;}
            {std::lock_guard<std::mutex> lk(wf_events_mtx);
             wf_events.clear(); last_tagged_sec=-1;}

            pacc.assign(fft_size,0.0f); fcnt=0; warmup_cnt=0;
            texture_needs_recreate=true;
            // SR 변경 > 신호 크기 스케일이 달라질 수 있어 오토스케일 재트리거
            autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
            // TM IQ가 켜져 있었으면 새 SR로 롤링 파일 재시작
            if(tm_was_on){
                tm_iq_open();
                tm_iq_on.store(true);
            }
            dem_restart_needed.store(true); // demod가 새 SR로 재초기화되도록
            bewe_log_push(0,"SR > %.3f MSPS\n", actual_sr/1e6f);
            // SR 변경으로 가시 대역폭이 달라짐 → 범위 재평가 (Holding/Active 전환)
            update_dem_by_freq(header.center_frequency/1e6f);
            continue;
        }

        // 주파수 변경 - set 후 한 사이클 쉬고 read_sync 재개 (RTL-SDR v4 USB 안정화)
        if(freq_req && !freq_prog){
            freq_prog=true;
            // Direct Sampling 자동 전환
            if(pending_cf < 24.0f)
                rtlsdr_set_direct_sampling(dev_rtl, 2);
            else
                rtlsdr_set_direct_sampling(dev_rtl, 0);
            rtlsdr_set_center_freq(dev_rtl, (uint32_t)(pending_cf*1e6));
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            rx_pos=0; rx_avail=0;
            {std::lock_guard<std::mutex> lk(data_mtx);
             header.center_frequency=(uint64_t)(pending_cf*1e6);}
            live_cf_hz.store((uint64_t)(pending_cf*1e6), std::memory_order_release);
            LongWaterfall::request_rotate();
            bewe_log_push(0,"Freq > %.2f MHz\n", pending_cf);
            autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
            warmup_cnt=0;
            update_dem_by_freq(pending_cf);
            freq_req=false; freq_prog=false;
            continue; // read_sync 호출을 다음 사이클로 미룸
        }

        // ── RX: 고정 청크(min 8192)로 읽기 > USB 오버헤드 최소화 ──────────
        if(rx_avail == 0){
            int n_read = 0;
            int r = rtlsdr_read_sync(dev_rtl, raw, (int)n_bytes, &n_read);
            if(r < 0 || n_read < (int)n_bytes){
                fprintf(stderr,"RTL-SDR RX: r=%d n_read=%d - SDR disconnected\n", r, n_read);
                sdr_stream_error.store(true);
                // 디바이스 닫고 루프 종료 > ui는 sdr_stream_error 빨간불 표시
                rtlsdr_close(dev_rtl);
                dev_rtl = nullptr;
                break;
            }
            // uint8 > int16 변환 (ring/TM IQ용)
            for(int i=0; i<rx_chunk*2; i++){
                iq16[i] = (int16_t)((int)raw[i] - 128) << 4;
            }
            // IQ Ring write: 전체 청크
            bool need_ring = rec_on.load(std::memory_order_relaxed);
            if(!need_ring) for(int i=0;i<MAX_CHANNELS;i++) if(channels[i].dem_run.load()){need_ring=true;break;}
            bool need_tm = tm_iq_on.load(std::memory_order_relaxed) && (warmup_cnt>=WARMUP_FFTS);
            if(need_ring || need_tm){
                size_t wp=ring_wp.load(std::memory_order_relaxed);
                size_t n=(size_t)rx_chunk, cap=IQ_RING_CAPACITY;
                if(wp+n<=cap) memcpy(&ring[wp*2],iq16,n*2*sizeof(int16_t));
                else{
                    size_t p1=cap-wp, p2=n-p1;
                    memcpy(&ring[wp*2],iq16,p1*2*sizeof(int16_t));
                    memcpy(&ring[0],iq16+p1*2,p2*2*sizeof(int16_t));
                }
                ring_wp.store((wp+n)&IQ_RING_MASK, std::memory_order_release);
                if(need_tm) tm_iq_write(iq16,(int)n);
            }
            rx_pos=0; rx_avail=rx_chunk;
        }

        // ── FFT: 버퍼에서 fft_input_size씩 처리 ─────────────────────────
        if(rx_avail < fft_input_size){ rx_avail=0; rx_pos=0; continue; }

        if(!render_visible.load(std::memory_order_relaxed)){
            rx_pos+=fft_input_size; rx_avail-=fft_input_size;
            std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            continue;
        }
        if(!spectrum_pause.load(std::memory_order_relaxed)){
            const uint8_t* rp = raw + rx_pos*2;
            for(int i=0;i<fft_input_size;i++){
                fft_in[i][0] = ((float)rp[i*2  ] - iq_offset) / iq_scale;
                fft_in[i][1] = ((float)rp[i*2+1] - iq_offset) / iq_scale;
            }
            // Nuttall window via VOLK SIMD
            volk_32fc_32f_multiply_32fc((lv_32fc_t*)fft_in, (lv_32fc_t*)fft_in,
                                        win_buf, fft_input_size);
            // pad region은 init/resize 시 한 번만 0 초기화
            fftwf_execute(fft_plan);
            {
                volk_32fc_magnitude_squared_32f(mag_sq_buf, (lv_32fc_t*)fft_out, fft_size);
                const float scale=NUTTALL_WINDOW_CORRECTION/((float)fft_input_size*(float)fft_input_size);
                for(int i=0;i<fft_size;i++){
                    pacc[i] += mag_sq_buf[i]*scale + 1e-10f;
                }
            }
            pacc[0]=(pacc[1]+pacc[fft_size-1])*0.5f; fcnt++;
            if(fcnt>=time_average){
                if(warmup_cnt < WARMUP_FFTS){
                    warmup_cnt++;
                    std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
                    rx_pos+=fft_input_size; rx_avail-=fft_input_size;
                    continue;
                }
                int fi=total_ffts%MAX_FFTS_MEMORY;
                float* rowp=fft_data.data()+fi*fft_size;
                {std::lock_guard<std::mutex> lk(data_mtx);
                 // current_spectrum은 UI 스레드 전용 > 캡처 쓰기 금지 (race 유발)
                 for(int i=0;i<fft_size;i++){
                     rowp[i]=10.0f*log10f(pacc[i]/fcnt);
                 }
                 if(autoscale_active){
                     if(!autoscale_init){
                         size_t cap=(size_t)fft_size*100;
                         if(autoscale_accum.size()!=cap) autoscale_accum.assign(cap,0.0f);
                         autoscale_wp=0; autoscale_buf_full=false;
                         autoscale_last=std::chrono::steady_clock::now();
                         autoscale_init=true;
                     }
                     size_t cap=autoscale_accum.size();
                     for(int i=1;i<fft_size;i++){
                         autoscale_accum[autoscale_wp]=rowp[i];
                         if(++autoscale_wp>=cap){ autoscale_wp=0; autoscale_buf_full=true; }
                     }
                     float el=std::chrono::duration<float>(std::chrono::steady_clock::now()-autoscale_last).count();
                     if(el>=1.0f&&(autoscale_buf_full||autoscale_wp>0)){
                         size_t n=autoscale_buf_full?cap:autoscale_wp;
                         std::vector<float> tmp(autoscale_accum.begin(),
                                                autoscale_accum.begin()+(ptrdiff_t)n);
                         size_t idx_lo=(size_t)(n*0.15f);
                         std::nth_element(tmp.begin(),tmp.begin()+(ptrdiff_t)idx_lo,tmp.end());
                         float noise=tmp[idx_lo];
                         float peak=*std::max_element(tmp.begin(),tmp.end());
                         display_power_min=noise-5.0f;
                         display_power_max=peak+20.0f;
                         if(display_power_max-display_power_min<20.f)
                             display_power_max=display_power_min+20.f;
                         header.power_min=display_power_min;
                         header.power_max=display_power_max;
                         bewe_log_push(0,"[autoscale] noise=%.1f peak=%.1f → pmin=%.1f pmax=%.1f\n",
                             noise, peak, display_power_min, display_power_max);
                         autoscale_active=false; autoscale_init=false;
                         autoscale_wp=0; autoscale_buf_full=false;
                         cached_sp_idx=-1;
                     }
                 }
                 total_ffts++; current_fft_idx=total_ffts-1;
                 header.num_ffts=std::min(total_ffts,MAX_FFTS_MEMORY);
                 row_write_pos[current_fft_idx%MAX_FFTS_MEMORY]=tm_iq_write_sample;
                 row_wall_ms[current_fft_idx%MAX_FFTS_MEMORY]=(int64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
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
        } // end !spectrum_pause
        rx_pos+=fft_input_size; rx_avail-=fft_input_size;
    }
    delete[] raw;
    delete[] iq16;
    if(dev_rtl){ rtlsdr_close(dev_rtl); dev_rtl=nullptr; }
}