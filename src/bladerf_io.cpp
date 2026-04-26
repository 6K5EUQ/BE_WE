#include "fft_viewer.hpp"
#include <thread>
#include "net_server.hpp"
#include "long_waterfall.hpp"
#include <volk/volk.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

// ── BladeRF USB 소프트 리셋 (USBDEVFS_RESET ioctl) ───────────────────────
// vendor=2cf0, product=5250 장치를 /dev/bus/usb에서 찾아 리셋
// 효과: 물리적으로 뽑았다 꽂는 것과 동일 (드라이버 unbind>reenumerate)
// sudo 불필요 - udev rule로 plugdev 그룹에 rw 권한 부여됨
bool bladerf_usb_reset(){
    // /dev/bus/usb/NNN/MMM 파일 순회해서 vendor/product 매칭
    DIR* bus_dir = opendir("/dev/bus/usb");
    if(!bus_dir){ perror("[USBreset] opendir /dev/bus/usb"); return false; }

    struct dirent* bus_ent;
    bool found = false;
    while(!found && (bus_ent = readdir(bus_dir))){
        if(bus_ent->d_name[0] == '.') continue;
        char bus_path[64];
        snprintf(bus_path, sizeof(bus_path), "/dev/bus/usb/%s", bus_ent->d_name);
        DIR* dev_dir = opendir(bus_path);
        if(!dev_dir) continue;
        struct dirent* dev_ent;
        while(!found && (dev_ent = readdir(dev_dir))){
            if(dev_ent->d_name[0] == '.') continue;
            char dev_path[128];
            snprintf(dev_path, sizeof(dev_path), "%s/%s", bus_path, dev_ent->d_name);
            int fd = open(dev_path, O_RDWR);
            if(fd < 0) continue;

            // USB descriptor: byte 8=vendor(LE16), byte 10=product(LE16)
            uint8_t desc[18] = {};
            if(read(fd, desc, sizeof(desc)) == (ssize_t)sizeof(desc)){
                uint16_t vid = (uint16_t)(desc[8]  | (desc[9]  << 8));
                uint16_t pid = (uint16_t)(desc[10] | (desc[11] << 8));
                if(vid == 0x2cf0 && pid == 0x5250){
                    bewe_log_push(0,"[USBreset] found BladeRF at %s - issuing USBDEVFS_RESET\n", dev_path);
                    if(ioctl(fd, USBDEVFS_RESET, nullptr) == 0){
                        bewe_log_push(0,"[USBreset] reset OK\n");
                        found = true;
                    } else {
                        perror("[USBreset] ioctl USBDEVFS_RESET");
                    }
                }
            }
            close(fd);
        }
        closedir(dev_dir);
    }
    closedir(bus_dir);
    if(!found) bewe_log_push(0,"[USBreset] BladeRF not found in /dev/bus/usb\n");
    return found;
}

bool FFTViewer::initialize_bladerf(float cf_mhz, float sr_msps){
    int s=bladerf_open(&dev_blade,nullptr);
    if(s){ bewe_log("bladerf_open: %s\n",bladerf_strerror(s)); return false; }

    s=bladerf_set_frequency(dev_blade,BLADERF_CHANNEL_RX(0),(uint64_t)(cf_mhz*1e6));
    if(s){ bewe_log("set_freq: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    uint32_t actual=0;
    s=bladerf_set_sample_rate(dev_blade,BLADERF_CHANNEL_RX(0),(uint32_t)(sr_msps*1e6),&actual);
    if(s){ bewe_log("set_sr: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    uint32_t actual_bw=0;
    s=bladerf_set_bandwidth(dev_blade,BLADERF_CHANNEL_RX(0),(uint32_t)(sr_msps*1e6*0.8f),&actual_bw);
    if(s){ bewe_log("set_bw: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    // Manual Gain Control (AGC 비활성화)
    s=bladerf_set_gain_mode(dev_blade,BLADERF_CHANNEL_RX(0),BLADERF_GAIN_MGC);
    if(s) bewe_log("set_gain_mode: %s\n",bladerf_strerror(s));

    s=bladerf_set_gain(dev_blade,BLADERF_CHANNEL_RX(0),BLADERF_RX_GAIN);
    if(s){ bewe_log("set_gain: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    s=bladerf_enable_module(dev_blade,BLADERF_CHANNEL_RX(0),true);
    if(s){ bewe_log("enable: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    s=bladerf_sync_config(dev_blade,BLADERF_RX_X1,BLADERF_FORMAT_SC16_Q11,512,16384,128,5000);
    if(s){ bewe_log("sync: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    bewe_log("BladeRF: %.2f MHz  %.2f MSPS  BW %.2f MHz\n",cf_mhz,actual/1e6f,actual_bw/1e6f);

    hw = make_bladerf_config(actual);
    gain_db = hw.gain_default;
    std::memcpy(header.magic,"FFTD",4);
    fft_input_size = fft_size / FFT_PAD_FACTOR;  // fft_size is already padded in member init
    header.version=1; header.fft_size=fft_size; header.sample_rate=actual;
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
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex));  // zero-pad region
    fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_ESTIMATE);
    memset(fft_in, 0, fft_size*sizeof(fftwf_complex)); // MEASURE가 입력 파괴 > 재초기화
    // Pre-compute Nuttall window + allocate VOLK mag_sq buffer
    if(win_buf) free(win_buf);
    win_buf=(float*)volk_malloc(fft_input_size*sizeof(float), volk_get_alignment());
    fill_nuttall_window(win_buf, fft_input_size);
    if(mag_sq_buf) volk_free(mag_sq_buf);
    mag_sq_buf=(float*)volk_malloc(fft_size*sizeof(float), volk_get_alignment());
    ring.resize(IQ_RING_CAPACITY*2,0);
    return true;
}

void FFTViewer::capture_and_process(){
    // RX 버퍼: fft_size와 무관하게 최소 8192 샘플 고정 > USB 오버헤드 최소화
    static constexpr int RX_MIN = 8192;
    int rx_chunk = std::max(fft_input_size, RX_MIN);
    int16_t* iq_buf=new int16_t[rx_chunk*2];
    // FFT 처리용 오프셋 (rx_chunk 내 슬라이딩)
    int rx_pos = 0; // iq_buf 내 현재 읽기 위치 (샘플 단위)
    int rx_avail = 0; // iq_buf에 유효한 샘플 수

    std::vector<float> pacc(fft_size,0.0f); int fcnt=0;
    // 초기 안정화: 처음 N번 FFT 결과 버림
    static constexpr int WARMUP_FFTS = 30;
    int warmup_cnt = 0;

    while(is_running){
        // ── Pause (타임머신 모드) ─────────────────────────────────────────
        if(capture_pause.load(std::memory_order_relaxed)){
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            rx_avail=0; rx_pos=0;
            continue;
        }

        // ── FFT size change — off-lock 재할당 + atomic swap (broadcast race 방지)
        if(fft_size_change_req){
            fft_size_change_req=false; int ns=pending_fft_size;
            int new_input = ns;
            int new_fft_sz = ns * FFT_PAD_FACTOR;
            // demod 스레드 일시 정지: ring 접근 충돌 방지
            size_t cur_wp = ring_wp.load(std::memory_order_relaxed);
            for(int ci=0;ci<MAX_CHANNELS;ci++)
                channels[ci].dem_rp.store(cur_wp, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // ① Off-lock: capture 전용 FFTW/VOLK 자원 재구성
            fftwf_destroy_plan(fft_plan); fftwf_free(fft_in); fftwf_free(fft_out);
            fft_in =fftwf_alloc_complex(new_fft_sz);
            fft_out=fftwf_alloc_complex(new_fft_sz);
            memset(fft_in, 0, new_fft_sz*sizeof(fftwf_complex));
            fft_plan=fftwf_plan_dft_1d(new_fft_sz,fft_in,fft_out,FFTW_FORWARD,FFTW_ESTIMATE);
            memset(fft_in, 0, new_fft_sz*sizeof(fftwf_complex)); // MEASURE가 입력 파괴 > 재초기화
            if(win_buf) volk_free(win_buf);
            win_buf=(float*)volk_malloc(new_input*sizeof(float), volk_get_alignment());
            fill_nuttall_window(win_buf, new_input);
            if(mag_sq_buf) volk_free(mag_sq_buf);
            mag_sq_buf=(float*)volk_malloc(new_fft_sz*sizeof(float), volk_get_alignment());
            rx_chunk = std::max(new_input, RX_MIN);
            delete[] iq_buf; iq_buf=new int16_t[rx_chunk*2];
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
            LongWaterfall::request_rotate();   // fft_size changed → new file
            continue;
        }

        // ── Sample rate change ────────────────────────────────────────────
        if(sr_change_req){
            sr_change_req=false;
            uint32_t new_sr = (uint32_t)(pending_sr_msps * 1e6f);

            // TM IQ 리셋: on/off 여부 관계없이 기존 롤링 파일 삭제 후 재생성
            bool tm_was_on = tm_iq_on.load(std::memory_order_relaxed);
            if(tm_was_on) tm_iq_on.store(false);
            tm_iq_close(); // fd 닫기 + 상태 초기화 (이미 closed면 no-op)
            // 기존 SR 롤링 파일 삭제 (SR 불일치 방지)
            {
                std::string tm_dir = BEWEPaths::time_temp_dir();
                char old_path[256];
                snprintf(old_path, sizeof(old_path), "%s/iq_rolling_%uMSPS.wav",
                         tm_dir.c_str(), header.sample_rate/1000000);
                if(access(old_path, F_OK)==0){ remove(old_path); }
            }

            // 122.88M 이상 > SC8_Q7 (8bit) + OVERSAMPLE, 그 외 > SC16_Q11 (16bit)
            bool was_sc8 = sc8_mode;
            sc8_mode = (new_sr >= 122880000);
            bladerf_format fmt = sc8_mode ? BLADERF_FORMAT_SC8_Q7 : BLADERF_FORMAT_SC16_Q11;

            bladerf_enable_module(dev_blade,BLADERF_CHANNEL_RX(0),false);

            // OVERSAMPLE 피쳐: SC8 진입 시 enable, 복귀 시 disable
            if(sc8_mode != was_sc8){
                int fe = bladerf_enable_feature(dev_blade, BLADERF_FEATURE_OVERSAMPLE, sc8_mode);
                if(fe) bewe_log("enable_feature(OVERSAMPLE,%d): %s\n", sc8_mode, bladerf_strerror(fe));
            }

            uint32_t actual_sr=0, actual_bw=0;
            int sr_err = bladerf_set_sample_rate(dev_blade,BLADERF_CHANNEL_RX(0),new_sr,&actual_sr);
            if(sr_err) bewe_log("set_sr(%u): %s\n", new_sr, bladerf_strerror(sr_err));

            // BW: 최대한 열기 (AD9361이 지원하는 범위로 자동 클램프됨)
            uint32_t req_bw = (uint32_t)(actual_sr*0.8f);
            int bw_err = bladerf_set_bandwidth(dev_blade,BLADERF_CHANNEL_RX(0),req_bw,&actual_bw);
            if(bw_err) bewe_log("set_bw(%u): %s\n", req_bw, bladerf_strerror(bw_err));

            bladerf_enable_module(dev_blade,BLADERF_CHANNEL_RX(0),true);
            int sc_err = bladerf_sync_config(dev_blade,BLADERF_RX_X1,fmt,512,16384,128,5000);
            if(sc_err) bewe_log("sync_config(fmt=%d): %s\n", (int)fmt, bladerf_strerror(sc_err));

            bewe_log("SC8=%d req=%u actual_sr=%u actual_bw=%u\n", sc8_mode, new_sr, actual_sr, actual_bw);

            // HW 파라미터 갱신
            hw = make_bladerf_config(actual_sr);
            if(sc8_mode) hw.iq_scale = 128.0f; // SC8_Q7: 7bit > 128
            time_average = hw.compute_time_average(fft_input_size);

            {std::lock_guard<std::mutex> lk(data_mtx);
             header.sample_rate = actual_sr;
             header.time_average = time_average;
             fft_data.assign(MAX_FFTS_MEMORY*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;}
            {std::lock_guard<std::mutex> lk(wf_events_mtx);
             wf_events.clear(); last_tagged_sec=-1;}

            rx_chunk = std::max(fft_input_size, RX_MIN);
            delete[] iq_buf; iq_buf = new int16_t[rx_chunk*2];
            rx_pos=0; rx_avail=0;
            pacc.assign(fft_size,0.0f); fcnt=0; warmup_cnt=0;
            texture_needs_recreate=true;
            // SR 변경 > 신호 크기 스케일이 달라질 수 있어 오토스케일 재트리거
            autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
            // TM IQ: SC8 모드(122.88M)에서는 롤링 IQ 비활성화
            if(tm_was_on && !sc8_mode){
                tm_iq_open();
                tm_iq_on.store(true);
            }
            // SR 변경 후 게인 재적용 (BladeRF가 SR 변경 시 게인을 리셋할 수 있음)
            set_gain(gain_db);
            dem_restart_needed.store(true); // demod가 새 SR로 재초기화되도록
            bewe_log("SR > %.2f MSPS  BW > %.2f MHz\n", actual_sr/1e6f, actual_bw/1e6f);
            // SR 변경으로 가시 대역폭이 달라짐 → 범위 재평가 (Holding/Active 전환)
            update_dem_by_freq(header.center_frequency/1e6f);
            continue;
        }

        // ── Frequency change ──────────────────────────────────────────────
        if(freq_req&&!freq_prog){
            freq_prog=true;
            int s=bladerf_set_frequency(dev_blade,BLADERF_CHANNEL_RX(0),(uint64_t)(pending_cf*1e6));
            if(!s){
                {std::lock_guard<std::mutex> lk(data_mtx);
                 header.center_frequency=(uint64_t)(pending_cf*1e6);}
                live_cf_hz.store((uint64_t)(pending_cf*1e6), std::memory_order_release);
                LongWaterfall::request_rotate();   // CF changed → new file
                bewe_log("Freq > %.2f MHz\n",pending_cf);
                autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
                warmup_cnt=0;
                update_dem_by_freq(pending_cf);
            }
            freq_req=false; freq_prog=false;
        }

        // ── RX: 고정 청크(min 8192)로 읽기 > fft_size 무관 일정 throughput ──
        if(rx_avail == 0){
            int status=bladerf_sync_rx(dev_blade,iq_buf,rx_chunk,nullptr,3000);
            if(status){
                if(status==BLADERF_ERR_TIMEOUT){
                    // 타임아웃 중 장치 분리 확인
                    if(!dev_blade || !bladerf_is_fpga_configured(dev_blade)){
                        fprintf(stderr,"BladeRF: device lost during timeout\n");
                        dev_blade = nullptr;
                        sdr_stream_error.store(true);
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                bewe_log("RX error: %s\n",bladerf_strerror(status));
                fprintf(stderr,"BladeRF: fatal RX error (%d) - SDR disconnected\n", status);
                dev_blade = nullptr;
                sdr_stream_error.store(true);
                break;
            }
            // SC8_Q7: int8 샘플을 int16으로 확장 (뒤에서부터 > in-place 안전)
            if(sc8_mode){
                int8_t* i8 = (int8_t*)iq_buf;
                for(int k = rx_chunk*2 - 1; k >= 0; k--)
                    iq_buf[k] = (int16_t)i8[k];
            }
            // IQ Ring write: 전체 청크를 한 번에 ring에 추가
            bool need_ring=rec_on.load(std::memory_order_relaxed);
            if(!need_ring) for(int i=0;i<MAX_CHANNELS;i++){
                if(channels[i].dem_run.load()||channels[i].digi_run.load()){need_ring=true;break;}
            }
            bool need_tm=!sc8_mode&&tm_iq_on.load(std::memory_order_relaxed)&&(warmup_cnt>=WARMUP_FFTS);
            if(need_ring||need_tm){
                size_t wp=ring_wp.load(std::memory_order_relaxed);
                size_t n=(size_t)rx_chunk, cap=IQ_RING_CAPACITY;
                if(wp+n<=cap) memcpy(&ring[wp*2],iq_buf,n*2*sizeof(int16_t));
                else{
                    size_t p1=cap-wp, p2=n-p1;
                    memcpy(&ring[wp*2],iq_buf,p1*2*sizeof(int16_t));
                    memcpy(&ring[0],iq_buf+p1*2,p2*2*sizeof(int16_t));
                }
                ring_wp.store((wp+n)&IQ_RING_MASK,std::memory_order_release);
                if(need_tm) tm_iq_write(iq_buf,(int)n);
            }
            rx_pos=0; rx_avail=rx_chunk;
        }

        // ── FFT: 버퍼에서 fft_input_size씩 처리 ─────────────────────────
        // 남은 샘플이 fft_input_size 미만이면 다음 RX로
        if(rx_avail < fft_input_size){ rx_avail=0; rx_pos=0; continue; }

        const int16_t* iq = iq_buf + rx_pos*2;

        if(!render_visible.load(std::memory_order_relaxed)){
            rx_pos+=fft_input_size; rx_avail-=fft_input_size;
            std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            continue;
        }
        if(!spectrum_pause.load(std::memory_order_relaxed)){
            // Fill input samples (first fft_input_size), rest stays zero (zero-padding)
            for(int i=0;i<fft_input_size;i++){
                fft_in[i][0]=iq[i*2]/hw.iq_scale;
                fft_in[i][1]=iq[i*2+1]/hw.iq_scale;
            }
            // Nuttall window via VOLK SIMD (complex × real element-wise)
            volk_32fc_32f_multiply_32fc((lv_32fc_t*)fft_in, (lv_32fc_t*)fft_in,
                                        win_buf, fft_input_size);
            // pad region은 init/resize 시 한 번만 0 초기화 (out-of-place FFT > fft_in 불변)
            fftwf_execute(fft_plan);
            {
                // VOLK magnitude squared: |X[k]|² for all bins
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
                 // current_spectrum은 UI 스레드 전용(픽셀별 peak) > 캡처가 절대 쓰지 않음
                 // (과거 bin별 avg를 여기에 덮어써 UI 파워스펙트럼에 1프레임 깨짐 유발했음)
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
                         autoscale_accum[autoscale_wp]=rowp[i];  // current_spectrum 대신 rowp 직접 사용
                         if(++autoscale_wp>=cap){ autoscale_wp=0; autoscale_buf_full=true; }
                     }
                     float el=std::chrono::duration<float>(std::chrono::steady_clock::now()-autoscale_last).count();
                     if(el>=1.0f&&(autoscale_buf_full||autoscale_wp>0)){
                         size_t n=autoscale_buf_full?cap:autoscale_wp;
                         std::vector<float> tmp(autoscale_accum.begin(),
                                                autoscale_accum.begin()+(ptrdiff_t)n);
                         // 노이즈 플로어: 15% 분위수 → pmin = noise - 5dB
                         // 피크: 99% 분위수 → pmax = peak + 20dB
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
    delete[] iq_buf;
    if(dev_blade){
        bladerf_enable_module(dev_blade, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(dev_blade);
        dev_blade = nullptr;
    }
}