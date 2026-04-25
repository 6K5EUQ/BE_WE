#include "fft_viewer.hpp"
#include "net_server.hpp"
#include "bewe_paths.hpp"
#include <volk/volk.h>
#include <iio.h>
#include <ad9361.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <unistd.h>

// ADALM-Pluto 고정 파라미터 (RTL-SDR/BladeRF 경로와 호환)
static constexpr uint32_t PLUTO_DEFAULT_SR = 3200000;  // 3.2 MSPS
static constexpr uint32_t PLUTO_MAX_SR     = 61440000; // 61.44 MSPS (AD9361 최대, USB2 드롭 발생)
// Pluto에서 TM IQ 롤링 허용 상한 (3.2 MSPS 이하만 허용; 10/20/40/61.44는 차단)
static constexpr uint32_t PLUTO_IQ_REC_MAX_SR = 3200000;
static constexpr int      PLUTO_BUF_MIN    = 8192;     // iio_buffer 최소 샘플 수
static constexpr int      PLUTO_BUF_SAMPS  = 8192;     // 초기 iio_buffer 샘플 수 (FFT size 변경 시 확장됨)

static struct iio_context*  g_ctx_fallback = nullptr;

// ── URI 후보 순회 (USB → IP) ─────────────────────────────────────────────
static struct iio_context* pluto_open_ctx(){
    struct iio_context* ctx = nullptr;
    ctx = iio_create_context_from_uri("usb:");              // 첫 USB 디바이스
    if(ctx) return ctx;
    ctx = iio_create_context_from_uri("ip:192.168.2.1");    // 기본 USB RNDIS IP
    if(ctx) return ctx;
    ctx = iio_create_default_context();                     // IIOD_REMOTE env 포함 폴백
    return ctx;
}

// phy 설정 헬퍼
static bool pluto_cfg_attr_ll(struct iio_channel* ch, const char* k, long long v){
    return iio_channel_attr_write_longlong(ch, k, v) == 0;
}
static bool pluto_cfg_attr_s(struct iio_channel* ch, const char* k, const char* v){
    return iio_channel_attr_write(ch, k, v) >= 0;
}

// AD9361: ~2.56 MSPS 미만은 내부 FIR 필터 활성화 필요
// FIR OFF 상태에서 낮은 SR 요청 시 드라이버가 -EINVAL 반환
static constexpr uint32_t PLUTO_FIR_THRESHOLD = 2560000;
static void pluto_set_fir(struct iio_device* phy, uint32_t sr){
    const char* en = (sr < PLUTO_FIR_THRESHOLD) ? "1" : "0";
    iio_device_attr_write(phy, "in_voltage_filter_fir_en", en);
}

// ── 초기화 ────────────────────────────────────────────────────────────────
bool FFTViewer::initialize_pluto(float cf_mhz, float sr_msps){
    struct iio_context* ctx = pluto_open_ctx();
    if(!ctx){ fprintf(stderr,"Pluto: no context\n"); return false; }

    struct iio_device* phy = iio_context_find_device(ctx, "ad9361-phy");
    struct iio_device* rxd = iio_context_find_device(ctx, "cf-ad9361-lpc");
    if(!phy || !rxd){
        fprintf(stderr,"Pluto: device not found (phy=%p rxd=%p)\n", (void*)phy,(void*)rxd);
        iio_context_destroy(ctx); return false;
    }

    // RX LO (altvoltage0)
    struct iio_channel* lo = iio_device_find_channel(phy, "altvoltage0", true);
    // RX RF 포트/gain/BW/sr (voltage0)
    struct iio_channel* v0 = iio_device_find_channel(phy, "voltage0", false);
    if(!lo || !v0){
        fprintf(stderr,"Pluto: phy ch not found (lo=%p v0=%p)\n",(void*)lo,(void*)v0);
        iio_context_destroy(ctx); return false;
    }

    uint32_t sr = (sr_msps > 0.1f) ? (uint32_t)(sr_msps * 1e6f) : PLUTO_DEFAULT_SR;
    if(sr < 521000) sr = 521000;          // AD9363 최저 근처
    if(sr > PLUTO_MAX_SR) sr = PLUTO_MAX_SR;  // 61.44 MSPS AD9361 최대 (USB2 드롭 감수)

    pluto_cfg_attr_s (v0, "rf_port_select",      "A_BALANCED");
    pluto_set_fir(phy, sr);                                       // FIR: 2.56M 미만 필수
    pluto_cfg_attr_ll(v0, "sampling_frequency",  (long long)sr);
    pluto_cfg_attr_ll(v0, "rf_bandwidth",        (long long)sr);
    pluto_cfg_attr_s (v0, "gain_control_mode",   "manual");
    pluto_cfg_attr_ll(v0, "hardwaregain",        30);
    pluto_cfg_attr_ll(lo, "frequency",           (long long)(cf_mhz * 1e6));

    // 실제 값 재읽기
    long long actual_sr_ll = sr;
    iio_channel_attr_read_longlong(v0, "sampling_frequency", &actual_sr_ll);
    uint32_t actual_sr = (uint32_t)actual_sr_ll;

    // RX IQ 채널 (voltage0=I, voltage1=Q on cf-ad9361-lpc)
    struct iio_channel* ri = iio_device_find_channel(rxd, "voltage0", false);
    struct iio_channel* rq = iio_device_find_channel(rxd, "voltage1", false);
    if(!ri || !rq){
        fprintf(stderr,"Pluto: rx ch not found\n");
        iio_context_destroy(ctx); return false;
    }
    iio_channel_enable(ri);
    iio_channel_enable(rq);

    struct iio_buffer* buf = iio_device_create_buffer(rxd, PLUTO_BUF_SAMPS, false);
    if(!buf){
        fprintf(stderr,"Pluto: iio_device_create_buffer failed (errno=%d)\n", errno);
        iio_context_destroy(ctx); return false;
    }

    pluto_ctx     = ctx;
    pluto_phy_dev = phy;
    pluto_rx_dev  = rxd;
    pluto_rx_i_ch = ri;
    pluto_rx_q_ch = rq;
    pluto_rx_buf  = buf;

    hw = make_pluto_config(actual_sr);
    gain_db = hw.gain_default;

    bewe_log_push(0,"Pluto: %.2f MHz  %.3f MSPS  gain %.0f dB\n",
           cf_mhz, actual_sr/1e6, gain_db);

    // FFT 헤더 (RTL 경로와 동일)
    std::memcpy(header.magic,"FFTD",4);
    fft_input_size = fft_size / FFT_PAD_FACTOR;
    header.version=1; header.fft_size=fft_size; header.sample_rate=actual_sr;
    header.center_frequency=(uint64_t)(cf_mhz*1e6);
    live_cf_hz.store((uint64_t)(cf_mhz*1e6), std::memory_order_release);
    time_average=hw.compute_time_average(fft_input_size);
    header.time_average=time_average; header.power_min=-100; header.power_max=0; header.num_ffts=0;
    fft_data.resize(MAX_FFTS_MEMORY*fft_size);
    current_spectrum.resize(fft_size,-100.0f);

    char title[256]; snprintf(title,256,"BEWE (" BEWE_VERSION ") Pluto - %.2f MHz",cf_mhz);
    window_title=title; display_power_min=-100; display_power_max=0;
    fft_in =fftwf_alloc_complex(fft_size);
    fft_out=fftwf_alloc_complex(fft_size);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex));
    fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_ESTIMATE);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex));
    if(win_buf) free(win_buf);
    win_buf=(float*)volk_malloc(fft_input_size*sizeof(float), volk_get_alignment());
    fill_nuttall_window(win_buf, fft_input_size);
    if(mag_sq_buf) volk_free(mag_sq_buf);
    mag_sq_buf=(float*)volk_malloc(fft_size*sizeof(float), volk_get_alignment());
    ring.resize(IQ_RING_CAPACITY*2,0);
    return true;
}

// ── 캡처 루프 ────────────────────────────────────────────────────────────
void FFTViewer::capture_and_process_pluto(){
    auto* rxd = (struct iio_device*)pluto_rx_dev;
    auto* phy = (struct iio_device*)pluto_phy_dev;
    auto* buf = (struct iio_buffer*)pluto_rx_buf;
    auto* ri  = (struct iio_channel*)pluto_rx_i_ch;
    auto* rq  = (struct iio_channel*)pluto_rx_q_ch;
    if(!rxd || !phy || !buf || !ri || !rq){ sdr_stream_error.store(true); return; }

    struct iio_channel* lo  = iio_device_find_channel(phy, "altvoltage0", true);
    struct iio_channel* v0p = iio_device_find_channel(phy, "voltage0",    false);

    // 현재 iio_buffer 샘플 용량 (FFT size 변경 시 확장)
    int cur_buf_samps = PLUTO_BUF_SAMPS;
    int16_t* iq16 = new int16_t[cur_buf_samps * 2];

    std::vector<float> pacc(fft_size, 0.0f);
    int   fcnt      = 0;
    static constexpr int WARMUP_FFTS = 30;
    int   warmup_cnt = 0;
    float iq_scale  = hw.iq_scale;   // 2048.0f

    // 내부 RX 버퍼 내 포지션
    int rx_pos = 0, rx_avail = 0;

    while(is_running){
        if(capture_pause.load(std::memory_order_relaxed)){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            rx_pos=0; rx_avail=0;
            continue;
        }

        // FFT size 변경 — off-lock 재할당 + atomic swap (broadcast race 방지)
        if(fft_size_change_req){
            fft_size_change_req=false; int ns=pending_fft_size;
            int new_input = ns;
            int new_fft_sz = ns * FFT_PAD_FACTOR;
            // iio_buffer가 fft_input_size보다 작으면 확장 (Pluto 고유: 고정 크기라 동적 재생성 필요)
            int need_buf = std::max(PLUTO_BUF_MIN, new_input);
            if(need_buf > cur_buf_samps){
                iio_buffer_destroy(buf);
                buf = iio_device_create_buffer(rxd, need_buf, false);
                if(!buf){
                    fprintf(stderr,"Pluto: iio_buffer recreate failed (size=%d errno=%d)\n", need_buf, errno);
                    sdr_stream_error.store(true); break;
                }
                pluto_rx_buf = buf;
                delete[] iq16; iq16 = new int16_t[need_buf * 2];
                cur_buf_samps = need_buf;
                rx_pos=0; rx_avail=0;
            }
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
            texture_needs_recreate=true; continue;
        }

        // 샘플레이트 변경
        if(sr_change_req){
            sr_change_req=false;
            uint32_t new_sr = (uint32_t)(pending_sr_msps * 1e6f);
            if(new_sr < 521000) new_sr = 521000;
            if(new_sr > PLUTO_MAX_SR) new_sr = PLUTO_MAX_SR;

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

            // 버퍼 재생성
            iio_buffer_destroy(buf);
            pluto_set_fir(phy, new_sr);                           // FIR: sampling_frequency 쓰기 전 설정
            bool sr_write_ok = pluto_cfg_attr_ll(v0p, "sampling_frequency", (long long)new_sr);
            bool bw_write_ok = pluto_cfg_attr_ll(v0p, "rf_bandwidth",       (long long)new_sr);
            if(!sr_write_ok)
                bewe_log_push(0,"[Pluto] sampling_frequency write FAILED for %u Hz (driver -EINVAL?)\n", new_sr);
            if(!bw_write_ok)
                bewe_log_push(0,"[Pluto] rf_bandwidth write FAILED for %u Hz\n", new_sr);
            buf = iio_device_create_buffer(rxd, PLUTO_BUF_SAMPS, false);
            pluto_rx_buf = buf;
            if(!buf){ sdr_stream_error.store(true); break; }

            long long actual_sr_ll = new_sr;
            iio_channel_attr_read_longlong(v0p, "sampling_frequency", &actual_sr_ll);
            uint32_t actual_sr = (uint32_t)actual_sr_ll;
            // 5% 이상 어긋나면 변경 거절로 간주 (write -EINVAL 또는 하드웨어 라운딩 큰 경우)
            uint32_t diff = (actual_sr > new_sr) ? (actual_sr - new_sr) : (new_sr - actual_sr);
            bool sr_rejected = (!sr_write_ok) || (diff * 20u > new_sr);
            if(sr_rejected){
                bewe_log_push(0,
                    "[Pluto] *** SR CHANGE REJECTED *** requested %.3f MSPS, hardware kept %.3f MSPS\n"
                    "        > Likely cause: FIR coefficients not loaded for sub-2.56 MSPS rates.\n",
                    new_sr/1e6f, actual_sr/1e6f);
            } else if(actual_sr != new_sr){
                bewe_log_push(0,"[Pluto] SR rounded: requested %u Hz, hardware=%u Hz (within 5%%)\n",
                              new_sr, actual_sr);
            }

            hw = make_pluto_config(actual_sr);
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
            rx_pos=0; rx_avail=0;
            texture_needs_recreate=true;
            // SR 변경 > 신호 크기 스케일이 달라질 수 있어 오토스케일 재트리거
            autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
            // TM IQ가 켜져 있었고 새 SR이 Pluto 허용 범위면 재시작. 61.44 MSPS면 차단.
            if(tm_was_on){
                if(new_sr > PLUTO_IQ_REC_MAX_SR){
                    bewe_log_push(0,"[TM IQ] disabled: Pluto SR %.2f MSPS (power spectrum only)\n",
                                  new_sr/1e6f);
                } else {
                    tm_iq_open();
                    tm_iq_on.store(true);
                }
            }
            dem_restart_needed.store(true); // demod가 새 SR로 재초기화되도록
            bewe_log_push(0,"SR > %.3f MSPS\n", actual_sr/1e6f);
            // SR 변경으로 가시 대역폭이 달라짐 → 범위 재평가 (Holding/Active 전환)
            update_dem_by_freq(header.center_frequency/1e6f);
            // (C) 즉시 STATUS 브로드캐스트 — JOIN UI/스탯이 1초 타이머를 안 기다리고
            //     실제 하드웨어 SR(거절 시 옛 값)을 즉시 보게 해서 드롭다운 자동 복원
            if(net_srv){
                uint8_t hwt = (hw.type==HWType::RTLSDR) ? 1 :
                              (hw.type==HWType::PLUTO)  ? 2 : 0;
                net_srv->broadcast_status(
                    (float)(header.center_frequency/1e6),
                    gain_db, header.sample_rate, hwt);
            }
            continue;
        }

        // 주파수 변경
        if(freq_req && !freq_prog){
            freq_prog=true;
            pluto_cfg_attr_ll(lo, "frequency", (long long)(pending_cf * 1e6));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            rx_pos=0; rx_avail=0;
            {std::lock_guard<std::mutex> lk(data_mtx);
             header.center_frequency=(uint64_t)(pending_cf*1e6);}
            live_cf_hz.store((uint64_t)(pending_cf*1e6), std::memory_order_release);
            bewe_log_push(0,"Freq > %.2f MHz\n", pending_cf);
            autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
            warmup_cnt=0;
            update_dem_by_freq(pending_cf);
            freq_req=false; freq_prog=false;
            continue;
        }

        // ── RX ──
        if(rx_avail == 0){
            ssize_t nbytes = iio_buffer_refill(buf);
            if(nbytes <= 0){
                fprintf(stderr,"Pluto RX: refill=%zd - disconnected\n", nbytes);
                sdr_stream_error.store(true);
                break;
            }
            // 인터리브 int16 I/Q 꺼내기
            ssize_t i_step = iio_buffer_step(buf);
            char*   p_end  = (char*)iio_buffer_end(buf);
            char*   p      = (char*)iio_buffer_first(buf, ri);
            int n = 0;
            for(; p < p_end && n < cur_buf_samps; p += i_step){
                int16_t si = ((int16_t*)p)[0];
                int16_t sq = ((int16_t*)p)[1];
                // libiio는 12-bit signed를 하위 12bit에 정렬; <<4로 ±32768 스케일 매칭
                iq16[n*2+0] = (int16_t)(si << 4);
                iq16[n*2+1] = (int16_t)(sq << 4);
                n++;
            }

            // IQ Ring + TM IQ 기록
            bool need_ring = rec_on.load(std::memory_order_relaxed);
            if(!need_ring) for(int i=0;i<MAX_CHANNELS;i++) if(channels[i].dem_run.load()){need_ring=true;break;}
            bool need_tm = tm_iq_on.load(std::memory_order_relaxed) && (warmup_cnt>=WARMUP_FFTS);
            if(need_ring || need_tm){
                size_t wp=ring_wp.load(std::memory_order_relaxed);
                size_t nn=(size_t)n, cap=IQ_RING_CAPACITY;
                if(wp+nn<=cap) memcpy(&ring[wp*2],iq16,nn*2*sizeof(int16_t));
                else{
                    size_t p1=cap-wp, p2=nn-p1;
                    memcpy(&ring[wp*2],iq16,p1*2*sizeof(int16_t));
                    memcpy(&ring[0],iq16+p1*2,p2*2*sizeof(int16_t));
                }
                ring_wp.store((wp+nn)&IQ_RING_MASK, std::memory_order_release);
                if(need_tm) tm_iq_write(iq16, n);
            }
            rx_pos=0; rx_avail=n;
        }

        if(rx_avail < fft_input_size){ rx_avail=0; rx_pos=0; continue; }

        if(!render_visible.load(std::memory_order_relaxed)){
            rx_pos+=fft_input_size; rx_avail-=fft_input_size;
            std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            continue;
        }
        if(!spectrum_pause.load(std::memory_order_relaxed)){
            const int16_t* rp = iq16 + rx_pos*2;
            const float inv_scale = 1.0f / iq_scale;
            for(int i=0;i<fft_input_size;i++){
                fft_in[i][0] = (float)rp[i*2+0] * inv_scale;
                fft_in[i][1] = (float)rp[i*2+1] * inv_scale;
            }
            volk_32fc_32f_multiply_32fc((lv_32fc_t*)fft_in, (lv_32fc_t*)fft_in,
                                        win_buf, fft_input_size);
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
                         size_t nn=autoscale_buf_full?cap:autoscale_wp;
                         std::vector<float> tmp(autoscale_accum.begin(),
                                                autoscale_accum.begin()+(ptrdiff_t)nn);
                         size_t idx_lo=(size_t)(nn*0.15f);
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
        }
        rx_pos+=fft_input_size; rx_avail-=fft_input_size;
    }

    delete[] iq16;
    if(pluto_rx_buf){ iio_buffer_destroy((struct iio_buffer*)pluto_rx_buf); pluto_rx_buf=nullptr; }
    if(pluto_ctx){ iio_context_destroy((struct iio_context*)pluto_ctx); pluto_ctx=nullptr; }
    pluto_phy_dev=nullptr; pluto_rx_dev=nullptr; pluto_rx_i_ch=nullptr; pluto_rx_q_ch=nullptr;
}

// ── AD9361 내부 온도 읽기 ────────────────────────────────────────────────
// 성공 시 °C, 실패 시 음수 반환
float FFTViewer::pluto_get_temp_c() const {
    auto* phy = (struct iio_device*)pluto_phy_dev;
    if(!phy) return -1.f;
    struct iio_channel* tch = iio_device_find_channel(phy, "temp0", false);
    if(!tch) return -1.f;
    // Pluto: in_temp0_input이 이미 milli-°C 단위로 환산된 값을 제공
    long long milli = 0;
    if(iio_channel_attr_read_longlong(tch, "input", &milli) == 0)
        return (float)milli / 1000.f;
    // fallback: raw + offset + scale
    long long raw = 0, off = 0;
    double scale = 1.0;
    if(iio_channel_attr_read_longlong(tch, "raw", &raw) != 0) return -1.f;
    iio_channel_attr_read_longlong(tch, "offset", &off);
    iio_channel_attr_read_double  (tch, "scale",  &scale);
    return (float)((raw + off) * scale / 1000.0);
}
