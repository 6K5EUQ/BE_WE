#include "fft_viewer.hpp"
#include <thread>
#include "net_server.hpp"
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
// 효과: 물리적으로 뽑았다 꽂는 것과 동일 (드라이버 unbind→reenumerate)
// sudo 불필요 — udev rule로 plugdev 그룹에 rw 권한 부여됨
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
                    printf("[USBreset] found BladeRF at %s — issuing USBDEVFS_RESET\n", dev_path);
                    if(ioctl(fd, USBDEVFS_RESET, nullptr) == 0){
                        printf("[USBreset] reset OK\n");
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
    if(!found) printf("[USBreset] BladeRF not found in /dev/bus/usb\n");
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

    s=bladerf_set_gain(dev_blade,BLADERF_CHANNEL_RX(0),BLADERF_RX_GAIN);
    if(s){ bewe_log("set_gain: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    s=bladerf_enable_module(dev_blade,BLADERF_CHANNEL_RX(0),true);
    if(s){ bewe_log("enable: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    // num_transfers=128: USB DMA 큐 깊이 확보 (네트워크 부하 시 버퍼 여유)
    s=bladerf_sync_config(dev_blade,BLADERF_RX_X1,BLADERF_FORMAT_SC16_Q11,512,16384,128,5000);
    if(s){ bewe_log("sync: %s\n",bladerf_strerror(s)); bladerf_close(dev_blade); return false; }

    bewe_log("BladeRF: %.2f MHz  %.2f MSPS  BW %.2f MHz\n",cf_mhz,actual/1e6f,actual_bw/1e6f);

    hw = make_bladerf_config(actual);
    gain_db = hw.gain_default;
    std::memcpy(header.magic,"FFTD",4);
    header.version=1; header.fft_size=fft_size; header.sample_rate=actual;
    header.center_frequency=(uint64_t)(cf_mhz*1e6);
    time_average=hw.compute_time_average(fft_size);
    header.time_average=time_average; header.power_min=-100; header.power_max=0; header.num_ffts=0;
    fft_data.resize(MAX_FFTS_MEMORY*fft_size);
    current_spectrum.resize(fft_size,-100.0f);

    char title[256]; snprintf(title,256,"BEWE - %.2f MHz",cf_mhz);
    window_title=title; display_power_min=-100; display_power_max=0;
    autoscale_active=true; autoscale_init=false;
    fft_in =fftwf_alloc_complex(fft_size);
    fft_out=fftwf_alloc_complex(fft_size);
    fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_ESTIMATE);
    ring.resize(IQ_RING_CAPACITY*2,0);
    return true;
}

void FFTViewer::capture_and_process(){
    // RX 버퍼: fft_size와 무관하게 최소 8192 샘플 고정 → USB 오버헤드 최소화
    static constexpr int RX_MIN = 8192;
    int rx_chunk = std::max(fft_size, RX_MIN);
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

        // ── FFT size change ───────────────────────────────────────────────
        if(fft_size_change_req){
            fft_size_change_req=false; int ns=pending_fft_size;
            // demod 스레드 일시 정지: ring 접근 충돌 방지
            size_t cur_wp = ring_wp.load(std::memory_order_relaxed);
            for(int ci=0;ci<MAX_CHANNELS;ci++)
                channels[ci].dem_rp.store(cur_wp, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            fftwf_destroy_plan(fft_plan); fftwf_free(fft_in); fftwf_free(fft_out);
            fft_size=ns; time_average=hw.compute_time_average(ns);
            fft_in =fftwf_alloc_complex(fft_size);
            fft_out=fftwf_alloc_complex(fft_size);
            fft_plan=fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_ESTIMATE);
            rx_chunk = std::max(fft_size, RX_MIN);
            delete[] iq_buf; iq_buf=new int16_t[rx_chunk*2];
            rx_pos=0; rx_avail=0;
            pacc.assign(fft_size,0.0f); fcnt=0;
            {std::lock_guard<std::mutex> lk(data_mtx);
             header.fft_size=fft_size;
             fft_data.assign(MAX_FFTS_MEMORY*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;
             autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;}
            texture_needs_recreate=true; continue;
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

            // 122.88M 이상 → SC8_Q7 (8bit) + OVERSAMPLE, 그 외 → SC16_Q11 (16bit)
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
            if(sc8_mode) hw.iq_scale = 128.0f; // SC8_Q7: 7bit → 128
            time_average = hw.compute_time_average(fft_size);

            {std::lock_guard<std::mutex> lk(data_mtx);
             header.sample_rate = actual_sr;
             header.time_average = time_average;
             fft_data.assign(MAX_FFTS_MEMORY*fft_size,0);
             current_spectrum.assign(fft_size,-80.0f);
             total_ffts=0; current_fft_idx=0; cached_sp_idx=-1;
             autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;}
            {std::lock_guard<std::mutex> lk(wf_events_mtx);
             wf_events.clear(); last_tagged_sec=-1;}

            rx_chunk = std::max(fft_size, RX_MIN);
            delete[] iq_buf; iq_buf = new int16_t[rx_chunk*2];
            rx_pos=0; rx_avail=0;
            pacc.assign(fft_size,0.0f); fcnt=0; warmup_cnt=0;
            texture_needs_recreate=true;
            // TM IQ: SC8 모드(122.88M)에서는 롤링 IQ 비활성화
            if(tm_was_on && !sc8_mode){
                tm_iq_open();
                tm_iq_on.store(true);
            }
            // SR 변경 후 게인 재적용 (BladeRF가 SR 변경 시 게인을 리셋할 수 있음)
            set_gain(gain_db);
            bewe_log("SR → %.2f MSPS  BW → %.2f MHz\n", actual_sr/1e6f, actual_bw/1e6f);
            continue;
        }

        // ── Frequency change ──────────────────────────────────────────────
        if(freq_req&&!freq_prog){
            freq_prog=true;
            int s=bladerf_set_frequency(dev_blade,BLADERF_CHANNEL_RX(0),(uint64_t)(pending_cf*1e6));
            if(!s){
                {std::lock_guard<std::mutex> lk(data_mtx);
                 header.center_frequency=(uint64_t)(pending_cf*1e6);}
                bewe_log("Freq → %.2f MHz\n",pending_cf);
                autoscale_accum.clear(); autoscale_init=false; autoscale_active=true;
                warmup_cnt=0;
                update_dem_by_freq(pending_cf);
            }
            freq_req=false; freq_prog=false;
        }

        // ── RX: 고정 청크(min 8192)로 읽기 → fft_size 무관 일정 throughput ──
        if(rx_avail == 0){
            int status=bladerf_sync_rx(dev_blade,iq_buf,rx_chunk,nullptr,3000);
            if(status){
                if(status==BLADERF_ERR_TIMEOUT){
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                bewe_log("RX error: %s\n",bladerf_strerror(status));
                if(status==BLADERF_ERR_IO || status==BLADERF_ERR_UNEXPECTED){
                    // USB 뽑힘 → 장치 핸들 즉시 무효화 후 루프 종료
                    // bladerf_close()는 이미 장치가 없어 블로킹되므로 호출 생략
                    fprintf(stderr,"BladeRF: fatal RX IO error — SDR disconnected\n");
                    dev_blade = nullptr;  // 핸들 무효화 (close 생략)
                    sdr_stream_error.store(true);
                    break;
                }
                continue;
            }
            // SC8_Q7: int8 샘플을 int16으로 확장 (뒤에서부터 → in-place 안전)
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

        // ── FFT: 버퍼에서 fft_size씩 처리 ───────────────────────────────
        // 남은 샘플이 fft_size 미만이면 다음 RX로
        if(rx_avail < fft_size){ rx_avail=0; rx_pos=0; continue; }

        const int16_t* iq = iq_buf + rx_pos*2;

        if(!render_visible.load(std::memory_order_relaxed)){
            rx_pos+=fft_size; rx_avail-=fft_size;
            std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
            continue;
        }
        if(!spectrum_pause.load(std::memory_order_relaxed)){
            for(int i=0;i<fft_size;i++){
                fft_in[i][0]=iq[i*2]/hw.iq_scale;
                fft_in[i][1]=iq[i*2+1]/hw.iq_scale;
            }
            apply_hann(fft_in,fft_size);
            fftwf_execute(fft_plan);
            {
                const float scale=HANN_WINDOW_CORRECTION/((float)fft_size*(float)fft_size);
                for(int i=0;i<fft_size;i++){
                    float ms=(fft_out[i][0]*fft_out[i][0]+fft_out[i][1]*fft_out[i][1])*scale+1e-10f;
                    pacc[i]+=ms;
                }
            }
            pacc[0]=(pacc[1]+pacc[fft_size-1])*0.5f; fcnt++;
            if(fcnt>=time_average){
                if(warmup_cnt < WARMUP_FFTS){
                    warmup_cnt++;
                    std::fill(pacc.begin(),pacc.end(),0.0f); fcnt=0;
                    rx_pos+=fft_size; rx_avail-=fft_size;
                    continue;
                }
                int fi=total_ffts%MAX_FFTS_MEMORY;
                int8_t* rowp=fft_data.data()+fi*fft_size;
                {std::lock_guard<std::mutex> lk(data_mtx);
                 for(int i=0;i<fft_size;i++){
                     float avg=10.0f*log10f(pacc[i]/fcnt);
                     float nn=(avg-header.power_min)/(header.power_max-header.power_min);
                     rowp[i]=(int8_t)(std::max(-1.0f,std::min(1.0f,nn))*127);
                     current_spectrum[i]=avg;
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
                         autoscale_accum[autoscale_wp]=current_spectrum[i];
                         if(++autoscale_wp>=cap){ autoscale_wp=0; autoscale_buf_full=true; }
                     }
                     float el=std::chrono::duration<float>(std::chrono::steady_clock::now()-autoscale_last).count();
                     if(el>=1.0f&&(autoscale_buf_full||autoscale_wp>0)){
                         size_t n=autoscale_buf_full?cap:autoscale_wp;
                         std::vector<float> tmp(autoscale_accum.begin(),
                                                autoscale_accum.begin()+(ptrdiff_t)n);
                         size_t idx=(size_t)(n*0.15f);
                         std::nth_element(tmp.begin(),tmp.begin()+(ptrdiff_t)idx,tmp.end());
                         display_power_min=tmp[idx]-10.0f;
                         autoscale_active=false; autoscale_init=false;
                         autoscale_wp=0; autoscale_buf_full=false;
                         cached_sp_idx=-1;
                     }
                 }
                 total_ffts++; current_fft_idx=total_ffts-1;
                 header.num_ffts=std::min(total_ffts,MAX_FFTS_MEMORY);
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
        } // end !spectrum_pause
        rx_pos+=fft_size; rx_avail-=fft_size;
    }
    delete[] iq_buf;
    if(dev_blade){
        bladerf_enable_module(dev_blade, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(dev_blade);
        dev_blade = nullptr;
    }
}