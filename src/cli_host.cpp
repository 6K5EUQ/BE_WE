// ── BE_WE CLI HOST Mode ──────────────────────────────────────────────────
// 라즈베리파이5 등 디스플레이 없는 환경에서 HOST 모드 전용 실행
// GLFW/OpenGL/ImGui 의존성 없음

#include "fft_viewer.hpp"
#include "login.hpp"
#include "bewe_paths.hpp"
#include "central_client.hpp"
#include "net_protocol.hpp"
#include "host_band_plan.hpp"
#include "host_band_categories.hpp"
#include "long_waterfall.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>
#include <csignal>
#include <ctime>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <poll.h>
#include <unistd.h>
#include <termios.h>
#include <arpa/inet.h>
#include <dirent.h>
#include <sys/stat.h>

// ── bewe_log (ui.cpp 대체) ───────────────────────────────────────────────
void bewe_log(const char* fmt, ...){
    char buf[256];
    va_list ap; va_start(ap,fmt);
    vsnprintf(buf,sizeof(buf),fmt,ap);
    va_end(ap);
    bewe_log_push(0, "%s", buf);
}

// ── bladerf_usb_reset 선언 (hw_detect.cpp) ───────────────────────────────
bool bladerf_usb_reset();

// ── 채널 스컬치 (ui.cpp에서 추출 - GUI 의존성 없음) ──────────────────────
void FFTViewer::update_channel_squelch(){
    if(total_ffts < 1 || fft_size < 1) return;
    // 호출 간격 기반 실 delta 시간 (시간 카운터용)
    static auto sq_last_tick = std::chrono::steady_clock::now();
    auto sq_now = std::chrono::steady_clock::now();
    float real_dt = std::chrono::duration<float>(sq_now - sq_last_tick).count();
    if(real_dt > 0.5f) real_dt = 0.02f; // 첫 호출/정지 이후 복귀 보정
    sq_last_tick = sq_now;

    std::lock_guard<std::mutex> lk(data_mtx);
    float cf_mhz = (float)(header.center_frequency / 1e6);
    float nyq_mhz = header.sample_rate / 2e6f;
    if(nyq_mhz < 0.001f) return;
    int hf = fft_size / 2;
    int fi = (total_ffts > 0 ? total_ffts - 1 : 0) % MAX_FFTS_MEMORY;
    const float* rowp = fft_data.data() + fi * fft_size;
    auto freq_to_bin = [&](float rel_mhz) -> int {
        int bin = (rel_mhz >= 0)
            ? (int)((rel_mhz / nyq_mhz) * hf)
            : fft_size + (int)((rel_mhz / nyq_mhz) * hf);
        return std::max(0, std::min(fft_size - 1, bin));
    };
    for(int c = 0; c < MAX_CHANNELS; c++){
        Channel& ch = channels[c];
        if(!ch.filter_active) continue;
        float s_mhz = std::min(ch.s, ch.e) - cf_mhz;
        float e_mhz = std::max(ch.s, ch.e) - cf_mhz;
        int bin_s = freq_to_bin(s_mhz);
        int bin_e = freq_to_bin(e_mhz);
        float peak_db = -120.0f;
        if(bin_s <= bin_e){
            for(int b = bin_s; b <= bin_e; b++)
                if(rowp[b] > peak_db) peak_db = rowp[b];
        } else {
            for(int b = bin_s; b < fft_size; b++)
                if(rowp[b] > peak_db) peak_db = rowp[b];
            for(int b = 0; b <= bin_e; b++)
                if(rowp[b] > peak_db) peak_db = rowp[b];
        }
        float prev = ch.sq_sig.load(std::memory_order_relaxed);
        float sig = 0.3f * peak_db + 0.7f * prev;
        ch.sq_sig.store(sig, std::memory_order_relaxed);
        if(!ch.sq_calibrated.load(std::memory_order_relaxed)){
            if(ch.sq_calib_cnt < 60)
                ch.sq_calib_buf[ch.sq_calib_cnt++] = peak_db;
            if(ch.sq_calib_cnt >= 60){
                float tmp[60];
                memcpy(tmp, ch.sq_calib_buf, sizeof(tmp));
                std::nth_element(tmp, tmp + 12, tmp + 60); // 20th percentile
                ch.sq_threshold.store(tmp[12] + 10.0f, std::memory_order_relaxed);
                ch.sq_calibrated.store(true, std::memory_order_relaxed);
                ch.sq_calib_cnt = 0;
            }
        }
        float thr = ch.sq_threshold.load(std::memory_order_relaxed);
        bool gate = ch.sq_gate.load(std::memory_order_relaxed);
        const float HYS = 3.0f;
        const int HOLD_FRAMES = 18;
        if(ch.sq_calibrated.load(std::memory_order_relaxed)){
            if(!gate && sig >= thr){
                gate = true;
                ch.sq_gate_hold = HOLD_FRAMES;
            }
            if(gate){
                if(sig >= thr - HYS)
                    ch.sq_gate_hold = HOLD_FRAMES;
                else if(--ch.sq_gate_hold <= 0)
                    gate = false;
            }
        }
        ch.sq_gate.store(gate, std::memory_order_relaxed);

        // 스컬치 누적 시간 — 실벽시계 delta 사용 (Holding 중에는 정지)
        if(ch.filter_active && !ch.dem_paused.load()){
            if(!sdr_stream_error.load()){
                ch.sq_total_time += real_dt;
                if(gate) ch.sq_active_time += real_dt;
            }
        } else if(!ch.filter_active){
            ch.sq_active_time = 0;
            ch.sq_total_time = 0;
        }
    }
}

// ── Signal handler ───────────────────────────────────────────────────────
static std::atomic<bool> g_shutdown{false};
static void sig_handler(int){ g_shutdown.store(true); }

// ── System monitor helpers (from ui.cpp) ─────────────────────────────────
static void read_cpu(long long& idle, long long& total){
    FILE* f=fopen("/proc/stat","r"); if(!f){idle=total=0;return;}
    long long u,n,s,i,iow,irq,sirq;
    if(fscanf(f,"cpu %lld %lld %lld %lld %lld %lld %lld",&u,&n,&s,&i,&iow,&irq,&sirq)!=7)
        { idle=total=0; fclose(f); return; }
    fclose(f); idle=i+iow; total=u+n+s+i+iow+irq+sirq;
}
static float read_ram(){
    FILE* f=fopen("/proc/meminfo","r"); if(!f) return 0.0f;
    long long total=0,avail=0; char key[64]; long long val;
    for(int i=0;i<10;i++){
        if(fscanf(f,"%63s %lld kB",key,&val)!=2) break;
        if(!strcmp(key,"MemTotal:"))      total=val;
        else if(!strcmp(key,"MemAvailable:")) avail=val;
    }
    fclose(f);
    return (total>0)?(float)(total-avail)/total*100.0f:0.0f;
}
static float read_ghz(){
    double sum=0; int cnt=0;
    for(int c=0;c<256;c++){
        char path[128];
        snprintf(path,sizeof(path),"/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq",c);
        FILE* f=fopen(path,"r"); if(!f) break;
        long long khz=0; if(fscanf(f,"%lld",&khz)){} fclose(f);
        sum+=khz; cnt++;
    }
    return cnt>0?(float)(sum/cnt/1e6):0.0f;
}
static int read_cpu_temp_c(){
    // 1) hwmon coretemp (Intel/AMD)
    for(int i=0;i<32;i++){
        char np[80]; snprintf(np,sizeof(np),"/sys/class/hwmon/hwmon%d/name",i);
        FILE* fn=fopen(np,"r"); if(!fn) continue;
        char name[32]={}; fgets(name,sizeof(name),fn); fclose(fn);
        if(strncmp(name,"coretemp",8)==0 || strncmp(name,"k10temp",7)==0
           || strncmp(name,"cpu_thermal",11)==0){
            char tp[80]; snprintf(tp,sizeof(tp),"/sys/class/hwmon/hwmon%d/temp1_input",i);
            FILE* ft=fopen(tp,"r"); if(!ft) continue;
            int milli=0; if(fscanf(ft,"%d",&milli)==1){ fclose(ft); return milli/1000; }
            fclose(ft);
        }
    }
    // 2) thermal_zone: x86_pkg_temp / TCPU / cpu-thermal (RPi5 등 ARM)
    for(int i=0;i<16;i++){
        char tt[80]; snprintf(tt,sizeof(tt),"/sys/class/thermal/thermal_zone%d/type",i);
        FILE* ft=fopen(tt,"r"); if(!ft) continue;
        char zt[32]={}; fgets(zt,sizeof(zt),ft); fclose(ft);
        if(strncmp(zt,"x86_pkg_temp",12)==0 || strncmp(zt,"TCPU",4)==0
           || strncmp(zt,"cpu-thermal",11)==0 || strncmp(zt,"cpu_thermal",11)==0){
            char tp[80]; snprintf(tp,sizeof(tp),"/sys/class/thermal/thermal_zone%d/temp",i);
            FILE* fv=fopen(tp,"r"); if(!fv) continue;
            int milli=0; if(fscanf(fv,"%d",&milli)==1){ fclose(fv); return milli/1000; }
            fclose(fv);
        }
    }
    // 3) fallback: thermal_zone0
    FILE* fv=fopen("/sys/class/thermal/thermal_zone0/temp","r");
    if(fv){ int milli=0; if(fscanf(fv,"%d",&milli)==1 && milli>0){ fclose(fv); return milli/1000; } fclose(fv); }
    return 0;
}
static long long read_io_ms(){
    FILE* f=fopen("/proc/diskstats","r"); if(!f) return 0;
    long long sum=0; char dev[32]; unsigned int maj,min_;
    long long f1,f2,f3,f4,f5,f6,f7,f8,f9,io_ticks;
    while(fscanf(f,"%u %u %31s %lld %lld %lld %lld %lld %lld %lld %lld %lld %lld %*[^\n]",
                 &maj,&min_,dev,&f1,&f2,&f3,&f4,&f5,&f6,&f7,&f8,&f9,&io_ticks)==13){
        if(dev[0]=='s'&&dev[2]>='a'&&dev[2]<='z'&&dev[3]=='\0') sum+=io_ticks;
        else if(dev[0]=='n'&&dev[1]=='v'&&strstr(dev,"p")==nullptr) sum+=io_ticks;
        else if(dev[0]=='v'&&dev[1]=='d'&&dev[3]=='\0') sum+=io_ticks;
    }
    fclose(f); return sum;
}

// ── Prompt helper (with default value) ───────────────────────────────────
static std::string prompt_input(const char* label, const char* def=nullptr){
    if(def && def[0])
        bewe_log_push(0,"%s [%s]: ", label, def);
    else
        bewe_log_push(0,"%s: ", label);
    fflush(stdout);
    char buf[128]={};
    if(!fgets(buf,sizeof(buf),stdin)){
        // Ctrl+C 또는 EOF (fgets가 EINTR로 중단되거나 stdin 닫힘)
        if(g_shutdown.load()) std::exit(0);
        return def ? def : "";
    }
    buf[strcspn(buf,"\r\n")]=0;
    if(buf[0]=='\0' && def) return def;
    return buf;
}

// ── stdin readline (non-blocking) ────────────────────────────────────────
static bool read_line_nb(std::string& out){
    struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
    if(poll(&pfd,1,0)<=0) return false;
    char buf[512];
    if(!fgets(buf,sizeof(buf),stdin)) return false;
    size_t len=strlen(buf);
    while(len>0 && (buf[len-1]=='\n'||buf[len-1]=='\r')) buf[--len]=0;
    out=buf;
    return len>0;
}

// ══════════════════════════════════════════════════════════════════════════
void run_cli_host(){
    // SA_RESTART 없이 등록 → fgets 등 blocking syscall이 Ctrl+C에 EINTR로 중단됨
    struct sigaction sa{};
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;  // SA_RESTART 미설정
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // ── Interactive prompts ──────────────────────────────────────────────
    bewe_log_push(0,"\n=== BE_WE CLI HOST ===\n\n");

    std::string id_str = prompt_input("ID ");
    if(id_str.empty()){ bewe_log_push(0,"Aborted.\n"); return; }

    // 패스워드 에코 숨기기
    bewe_log_push(0,"PW : "); fflush(stdout);
    struct termios old_t, new_t;
    tcgetattr(STDIN_FILENO,&old_t); new_t=old_t;
    new_t.c_lflag &= ~(ECHO);
    tcsetattr(STDIN_FILENO,TCSANOW,&new_t);
    char pw_buf[64]={};
    if(!fgets(pw_buf,sizeof(pw_buf),stdin)){
        tcsetattr(STDIN_FILENO,TCSANOW,&old_t);
        if(g_shutdown.load()){ bewe_log_push(0,"\n"); std::exit(0); }
        bewe_log_push(0,"\nAborted.\n");
        return;
    }
    tcsetattr(STDIN_FILENO,TCSANOW,&old_t);
    pw_buf[strcspn(pw_buf,"\r\n")]=0;
    bewe_log_push(0,"\n");

    int tier = atoi(prompt_input("Tier ").c_str());
    if(tier<1||tier>2){ bewe_log_push(0,"CLI HOST requires tier 1 or 2.\n"); return; }

    std::string server_str = "20.2.86.135";

    float lat = atof(prompt_input("Latitude ").c_str());
    float lon = -atof(prompt_input("Longitude").c_str()); // 양수 입력 = 동경(E), 내부 규약은 서경=양수이므로 부호 반전
    float cf  = 100.0f;

    std::string station_str = prompt_input("Station name");
    if(station_str.empty()){ bewe_log_push(0,"Aborted.\n"); return; }

    bewe_log_push(0,"\n");

    // ── Login ────────────────────────────────────────────────────────────
    cli_login(id_str.c_str(), pw_buf, tier, server_str.c_str());
    bewe_log_push(0,"[BEWE CLI] Login: %s (Tier %d)\n", login_get_id(), login_get_tier());

    // ── FFTViewer init ───────────────────────────────────────────────────
    FFTViewer v;
    extern FFTViewer* g_log_viewer;
    g_log_viewer = &v;
    v.station_name = station_str;
    v.station_lat  = lat;
    v.station_lon  = lon;
    v.station_location_set = true;
    strncpy(v.host_name, login_get_id(), 31);

    // ── SDR init ─────────────────────────────────────────────────────────
    std::thread cap;
    if(!v.initialize(cf)){
        bewe_log_push(0,"[BEWE CLI] SDR init failed - running without hardware\n");
        v.sdr_stream_error.store(true);
        // 초기 SDR 없음 → 파일 분석 모드. 주기적 재탐지 비활성화 (CPU/로그 스팸 방지)
        v.rx_stopped.store(true);
        v.fft_size = DEFAULT_FFT_SIZE * FFT_PAD_FACTOR;
        v.fft_input_size = DEFAULT_FFT_SIZE;
        v.header.fft_size  = DEFAULT_FFT_SIZE;
        v.header.power_min = -100.f;
        v.header.power_max = 0.f;
        v.display_power_min = -80.f;
        v.display_power_max = 0.f;
        v.fft_data.assign((size_t)MAX_FFTS_MEMORY * DEFAULT_FFT_SIZE * FFT_PAD_FACTOR, 0);
        v.current_spectrum.assign(DEFAULT_FFT_SIZE * FFT_PAD_FACTOR, -80.f);
        v.autoscale_active = false;
        v.create_waterfall_texture();
    } else {
        bewe_log_push(0,"[BEWE CLI] SDR: %s detected\n",
               v.hw.type==HWType::BLADERF ? "BladeRF" :
               v.hw.type==HWType::PLUTO   ? "ADALM-Pluto" : "RTL-SDR");
        if(v.hw.type == HWType::BLADERF)
            cap = std::thread(&FFTViewer::capture_and_process, &v);
        else if(v.hw.type == HWType::PLUTO)
            cap = std::thread(&FFTViewer::capture_and_process_pluto, &v);
        else
            cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
    }
    v.mix_stop.store(false);
    v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);

    // ── Long Waterfall worker (post-FFT image accumulator) ──────────────
    LongWaterfall::start_worker(&v);

    // ── NetServer ────────────────────────────────────────────────────────
    NetServer* srv = new NetServer();
    int host_port = 0;

    // Static state shared with callbacks
    std::map<std::string,std::string> pub_owners;
    std::map<std::string,std::vector<std::string>> pub_listeners;
    std::vector<std::string> shared_files, pub_iq_files, pub_audio_files;
    std::vector<std::string> rec_iq_files;
    std::atomic<bool> pending_chassis1_reset{false};
    std::atomic<bool> pending_chassis2_reset{false};
    std::atomic<bool> pending_rx_stop{false};
    std::atomic<bool> pending_rx_start{false};
    bool usb_reset_pending = false;
    std::atomic<bool> ch_sync_dirty_flag{false};

    // Central client
    CentralClient central_cli;
    char central_host[128] = {};
    strncpy(central_host, login_get_server(), 127);
    constexpr int central_port = CENTRAL_PORT;

    // ── Server callbacks (from ui.cpp 2243-2822) ─────────────────────────
    srv->cb.on_auth = [&,srv](const char* id, const char* pw,
                               uint8_t tier, uint8_t& idx) -> bool {
        static uint8_t next=1;
        idx = next++;
        if(next>MAX_OPERATORS) next=1;
        uint8_t new_idx = idx;
        std::thread([srv, new_idx, &pub_owners](){
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
            auto scan_pub = [&](const std::string& dir){
                DIR* ds = opendir(dir.c_str());
                if(!ds) return;
                struct dirent* ent;
                while((ent=readdir(ds))!=nullptr){
                    const char* n = ent->d_name;
                    size_t nl = strlen(n);
                    if(nl>4 && strcmp(n+nl-4,".wav")==0){
                        std::string fp = dir+"/"+n;
                        struct stat st{}; uint64_t fsz=0;
                        if(stat(fp.c_str(),&st)==0) fsz=(uint64_t)st.st_size;
                        std::string upl;
                        auto it=pub_owners.find(n);
                        if(it!=pub_owners.end()) upl=it->second;
                        slist.push_back({n, fsz, upl});
                    }
                }
                closedir(ds);
            };
            scan_pub(BEWEPaths::public_iq_dir());
            scan_pub(BEWEPaths::public_audio_dir());
            if(!slist.empty()) srv->send_share_list((int)new_idx, slist);
        }).detach();
        bewe_log_push(0,"[CLI] Client authenticated: idx=%d\n", idx);
        return true;
    };

    srv->cb.on_set_freq   = [&](const char* who, float cf){
        bewe_log_push(0, "[CMD:%s] Freq > %.3f MHz\n", who, cf);
        v.set_frequency(cf);
    };
    srv->cb.on_set_gain   = [&](const char* who, float db){
        bewe_log_push(0, "[CMD:%s] Gain > %.1f dB\n", who, db);
        v.gain_db=db; v.set_gain(db);
    };
    srv->cb.on_create_ch  = [&](int idx, float s, float e, const char* creator){
        if(idx<0||idx>=MAX_CHANNELS) return;
        bewe_log_push(0, "[CMD:%s] CH%d create s=%.4f e=%.4f bw=%.4f\n",
                      creator?creator:"?", idx, s, e, fabsf(e-s));
        v.stop_dem(idx); v.stop_digi(idx);
        v.channels[idx].reset_slot();
        v.channels[idx].s=s; v.channels[idx].e=e;
        v.channels[idx].filter_active=true;
        strncpy(v.channels[idx].owner, creator?creator:"", 31);
        v.channels[idx].audio_mask.store(0xFFFFFFFFu & ~0x1u);
        v.local_ch_out[idx] = 3;
        // 생성 직후 범위 판정 (범위 밖이면 Holding으로) + CH_SYNC 브로드캐스트
        v.update_dem_by_freq(v.header.center_frequency/1e6f);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_delete_ch  = [&](const char* who, int idx){
        if(idx<0||idx>=MAX_CHANNELS) return;
        bewe_log_push(0, "[CMD:%s] CH%d deleted\n", who, idx);
        if(v.channels[idx].audio_rec_on.load())
            v.stop_audio_rec(idx);
        v.stop_dem(idx); v.stop_digi(idx); v.digi_panel_on[idx]=false;
        v.channels[idx].reset_slot();
        v.local_ch_out[idx] = 1;
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_ch_mode= [&](const char* who, int idx, int mode){
        if(idx<0||idx>=MAX_CHANNELS) return;
        static const char* mn[]={"NONE","AM","FM","MAGIC"};
        bewe_log_push(0, "[CMD:%s] CH%d mode > %s\n", who, idx, mn[mode<4?mode:0]);
        v.stop_dem(idx);
        auto dm=(Channel::DemodMode)mode;
        v.channels[idx].mode=dm;
        if(dm!=Channel::DM_NONE && v.channels[idx].filter_active)
            v.start_dem(idx,dm);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_ch_audio=[&](int idx, uint32_t mask){
        if(idx<0||idx>=MAX_CHANNELS) return;
        v.channels[idx].audio_mask.store(mask);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_ch_pan =[&](int idx, int pan){
        if(idx<0||idx>=MAX_CHANNELS) return;
        v.channels[idx].pan=pan;
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_sq_thresh = [&](int idx2, float thr){
        if(idx2<0||idx2>=MAX_CHANNELS) return;
        v.channels[idx2].sq_threshold.store(thr, std::memory_order_relaxed);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_autoscale = [&](){
        v.autoscale_active=true; v.autoscale_init=false;
        v.autoscale_accum.clear();
    };
    srv->cb.on_toggle_tm_iq = [&](){
        bool cur=v.tm_iq_on.load();
        if(cur){
            v.tm_iq_on.store(false); v.tm_add_event_tag(2); v.tm_iq_was_stopped=true;
            srv->broadcast_wf_event(0,(int64_t)time(nullptr),2,"IQ Stop");
        } else {
            // Pluto에서 SR > 3.2 MSPS (10/20/40/61.44)는 파워 스펙트럼 관측 전용 → 롤링 IQ 차단
            if(v.hw.type == HWType::PLUTO && v.header.sample_rate > 3200000u){
                bewe_log_push(0,"[TM IQ] blocked: Pluto SR %.2f MSPS (power spectrum only)\n",
                              v.header.sample_rate/1e6f);
                return;
            }
            if(v.tm_iq_was_stopped){ v.tm_iq_close(); v.tm_iq_was_stopped=false; }
            v.tm_iq_open();
            if(v.tm_iq_file_ready){
                v.tm_iq_on.store(true); v.tm_add_event_tag(1);
                srv->broadcast_wf_event(0,(int64_t)time(nullptr),1,"IQ Start");
            }
        }
    };
    srv->cb.on_set_capture_pause = [&](bool pause){
        v.capture_pause.store(pause);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_set_spectrum_pause = [&](bool pause){
        v.spectrum_pause.store(pause);
    };

    // JOIN per-channel IQ recording (HOST에서 대행)
    srv->cb.on_start_iq_rec = [&](uint8_t op_idx, const char* who, uint8_t ch_idx){
        if(ch_idx >= MAX_CHANNELS) return;
        if(!v.channels[ch_idx].filter_active || !v.channels[ch_idx].dem_run.load()) return;
        float bw = fabsf(v.channels[ch_idx].e - v.channels[ch_idx].s) * 1e3f; // kHz
        if(bw > 100.f){ bewe_log_push(0,"[CMD:%s] IQ REC ch%d denied: BW=%.0fkHz>100kHz\n", who, ch_idx, bw); return; }
        if(v.channels[ch_idx].iq_rec_on.load()){ bewe_log_push(0,"[CMD:%s] IQ REC ch%d already on\n", who, ch_idx); return; }
        v.start_iq_rec(ch_idx);
        bewe_log_push(0,"[CMD:%s] IQ REC start ch%d (%.0fkHz)\n", who, ch_idx, bw);
    };
    srv->cb.on_stop_iq_rec = [&](uint8_t op_idx, const char* who, uint8_t ch_idx){
        if(ch_idx >= MAX_CHANNELS) return;
        if(!v.channels[ch_idx].iq_rec_on.load()) return;
        v.stop_iq_rec(ch_idx);
        bewe_log_push(0,"[CMD:%s] IQ REC stop ch%d\n", who, ch_idx);
        // 녹음 파일을 요청한 JOIN에게 전송
        std::string path = v.channels[ch_idx].iq_rec_path;
        if(!path.empty()){
            static std::atomic<uint32_t> g_iq_req{2000};
            uint32_t req_id = g_iq_req.fetch_add(1);
            auto* central_ptr = &central_cli;
            std::thread([path, srv, req_id, op_idx, central_ptr](){
                FILE* fp = fopen(path.c_str(), "rb");
                if(!fp) return;
                fseek(fp, 0, SEEK_END); uint64_t fsz = (uint64_t)ftell(fp); fseek(fp, 0, SEEK_SET);
                const char* fn = strrchr(path.c_str(), '/');
                fn = fn ? fn+1 : path.c_str();
                // START
                { PktIqChunkHdr ch{}; ch.req_id=req_id; ch.seq=0;
                  strncpy(ch.filename, fn, 127); ch.filesize=fsz; ch.data_len=0;
                  auto bewe=make_packet(PacketType::IQ_CHUNK, &ch, sizeof(ch));
                  if(srv->cb.on_relay_broadcast) srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), true); }
                // DATA
                const size_t CHUNK=64*1024;
                std::vector<uint8_t> buf(sizeof(PktIqChunkHdr)+CHUNK);
                uint64_t sent=0; uint32_t seq=1;
                while(true){
                    size_t n=fread(buf.data()+sizeof(PktIqChunkHdr),1,CHUNK,fp);
                    if(n==0) break;
                    auto* ch=reinterpret_cast<PktIqChunkHdr*>(buf.data());
                    ch->req_id=req_id; ch->seq=seq++; strncpy(ch->filename,fn,127);
                    ch->filesize=fsz; ch->data_len=(uint32_t)n;
                    auto bewe=make_packet(PacketType::IQ_CHUNK,buf.data(),(uint32_t)(sizeof(PktIqChunkHdr)+n));
                    if(srv->cb.on_relay_broadcast) srv->cb.on_relay_broadcast(bewe.data(),bewe.size(),true);
                    sent+=n;
                    while(central_ptr->queue_bytes()>2*1024*1024)
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                fclose(fp);
                // END
                { PktIqChunkHdr ch{}; ch.req_id=req_id; ch.seq=0xFFFFFFFF;
                  strncpy(ch.filename,fn,127); ch.filesize=fsz; ch.data_len=0;
                  auto bewe=make_packet(PacketType::IQ_CHUNK,&ch,sizeof(ch));
                  if(srv->cb.on_relay_broadcast) srv->cb.on_relay_broadcast(bewe.data(),bewe.size(),true); }
                // 전송 완료 후 HOST에서 삭제
                remove(path.c_str());
                bewe_log_push(0,"[CLI] IQ REC transferred and deleted: %s\n", path.c_str());
            }).detach();
        }
    };

    // Digital demod start/stop from JOIN
    srv->cb.on_start_digi = [&](const char* who, uint8_t ch_idx, uint8_t mode, uint8_t demod_type, float baud_rate){
        if(ch_idx >= MAX_CHANNELS) return;
        Channel& ch = v.channels[ch_idx];
        if(!ch.filter_active){ bewe_log_push(0,"[CMD:%s] DIGI ch%d not active\n",who,ch_idx); return; }
        if(ch.digi_run.load()){ v.stop_digi(ch_idx); }
        auto dm = (Channel::DigitalMode)mode;
        if(dm == Channel::DIGI_DEMOD){
            ch.digi_demod_type = demod_type;
            ch.digi_baud_rate  = baud_rate;
        }
        v.start_digi(ch_idx, dm);
        bewe_log_push(0,"[CMD:%s] DIGI start ch%d mode=%d\n",who,ch_idx,mode);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_stop_digi = [&](const char* who, uint8_t ch_idx){
        if(ch_idx >= MAX_CHANNELS) return;
        if(!v.channels[ch_idx].digi_run.load()) return;
        v.stop_digi(ch_idx);
        bewe_log_push(0,"[CMD:%s] DIGI stop ch%d\n",who,ch_idx);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };

    // Region IQ request from JOIN
    srv->cb.on_request_region = [&](uint8_t op_idx, const char* op_name,
                                     int32_t fft_top, int32_t fft_bot,
                                     float freq_lo, float freq_hi,
                                     int64_t time_start_ms, int64_t time_end_ms,
                                     int64_t samp_start, int64_t samp_end){
        std::string fname;
        {
            std::lock_guard<std::mutex> lk(v.rec_entries_mtx);
            FFTViewer::RecEntry e{};
            time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
            char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
            float cf_mhz = (freq_lo+freq_hi)/2.0f;
            char fn[128]; snprintf(fn,sizeof(fn),"IQ_%.3fMHz_%s.wav",cf_mhz,dts);
            e.filename = fn;
            e.is_region = true;
            e.req_state = FFTViewer::RecEntry::REQ_CONFIRMED;
            e.req_op_idx = op_idx;
            strncpy(e.req_op_name, op_name?op_name:"?", 31);
            e.req_fft_top=fft_top; e.req_fft_bot=fft_bot;
            e.req_freq_lo=freq_lo; e.req_freq_hi=freq_hi;
            e.req_time_start=time_start_ms/1000; e.req_time_end=time_end_ms/1000;
            e.t_start=std::chrono::steady_clock::now();
            v.rec_entries.push_back(e);
            fname = fn;
        }
        // JOIN 요청: time_start/time_end는 절대 wall_time
        // HOST에서 FFT 인덱스 변환 없이 time 기반으로 직접 샘플 위치 계산
        float fl=freq_lo, fh=freq_hi;
        uint8_t oidx=op_idx;
        std::string sid = v.station_name + "_" + std::string(login_get_id());
        static std::atomic<uint32_t> g_req_id{1000};
        uint32_t req_id_val = g_req_id.fetch_add(1);
        std::thread([&v,srv,fl,fh,time_start_ms,time_end_ms,samp_start,samp_end,oidx,fname,sid,&central_cli,req_id_val](){
            uint32_t req_id = req_id_val;
            for(int w=0;w<200&&v.rec_busy_flag.load();w++)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            v.region.fft_top=0; v.region.fft_bot=0; // 사용 안 함 (samp/time 기반)
            v.region.freq_lo=fl; v.region.freq_hi=fh;
            v.region.time_start_ms=time_start_ms;
            v.region.time_end_ms=time_end_ms;
            v.region.samp_start=samp_start;
            v.region.samp_end=samp_end;
            v.region.active=true;
            v.rec_busy_flag.store(true);
            v.rec_state = FFTViewer::REC_BUSY;
            v.rec_anim_timer = 0.0f;
            v.region.active = false;
            if(srv){
                PktIqProgress prog{};
                prog.req_id = req_id;
                strncpy(prog.filename, fname.c_str(), 127);
                prog.done=0; prog.total=0; prog.phase=0;
                srv->broadcast_iq_progress(prog);
            }
            bewe_log_push(0,"[CLI] region_save: tm_on=%d tm_write=%lld t_ms=%lld~%lld\n",
                (int)v.tm_iq_on.load(), (long long)v.tm_iq_write_sample,
                (long long)time_start_ms, (long long)time_end_ms);
            std::string path = v.do_region_save_work();
            v.rec_state = FFTViewer::REC_SUCCESS;
            v.rec_success_timer = 3.0f;
            v.rec_busy_flag.store(false);
            bewe_log_push(0,"[CLI] region_save done: path='%s'\n", path.c_str());
            if(path.empty()){
                if(srv) srv->send_region_response((int)oidx, false);
                return;
            }
            uint64_t fsz=0;
            {FILE* f=fopen(path.c_str(),"rb");if(f){fseek(f,0,SEEK_END);fsz=(uint64_t)ftell(f);fclose(f);}}
            // IQ chunk transfer via central relay
            if(srv && srv->cb.on_relay_broadcast){
                const char* fn_only2 = strrchr(path.c_str(), '/');
                fn_only2 = fn_only2 ? fn_only2+1 : path.c_str();
                bewe_log_push(0,"[CLI] IQ_CHUNK transfer start: req_id=%u file='%s' size=%.1fMB\n",
                       req_id, fn_only2, fsz/1048576.0);
                {
                    PktIqChunkHdr ch{};
                    ch.req_id = req_id; ch.seq = 0;
                    strncpy(ch.filename, fn_only2, 127);
                    ch.filesize = fsz; ch.data_len = 0;
                    auto bewe = make_packet(PacketType::IQ_CHUNK, &ch, sizeof(ch));
                    srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), true);
                }
                auto* central_ptr = &central_cli;
                std::thread([&v, fname, path, fsz, srv, req_id,
                             fn2 = std::string(fn_only2), central_ptr](){
                    FILE* fp = fopen(path.c_str(), "rb");
                    if(!fp) return;
                    const size_t CHUNK = 64 * 1024;
                    std::vector<uint8_t> buf(sizeof(PktIqChunkHdr) + CHUNK);
                    uint64_t sent = 0; uint32_t seq = 1;
                    while(true){
                        size_t n = fread(buf.data() + sizeof(PktIqChunkHdr), 1, CHUNK, fp);
                        if(n == 0) break;
                        auto* ch = reinterpret_cast<PktIqChunkHdr*>(buf.data());
                        ch->req_id = req_id; ch->seq = seq++;
                        strncpy(ch->filename, fn2.c_str(), 127);
                        ch->filesize = fsz; ch->data_len = (uint32_t)n;
                        auto bewe = make_packet(PacketType::IQ_CHUNK, buf.data(), (uint32_t)(sizeof(PktIqChunkHdr)+n));
                        if(srv->cb.on_relay_broadcast)
                            srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), true);
                        sent += n;
                        // pacing: 큐가 2MB 넘으면 sender가 따라잡을 때까지 대기
                        while(central_ptr->queue_bytes() > 2*1024*1024)
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    fclose(fp);
                    {
                        PktIqChunkHdr ch{};
                        ch.req_id = req_id; ch.seq = 0xFFFFFFFF;
                        strncpy(ch.filename, fn2.c_str(), 127);
                        ch.filesize = fsz; ch.data_len = 0;
                        auto bewe = make_packet(PacketType::IQ_CHUNK, &ch, sizeof(ch));
                        if(srv->cb.on_relay_broadcast)
                            srv->cb.on_relay_broadcast(bewe.data(), bewe.size(), true);
                    }
                    {
                        PktIqProgress prog{};
                        prog.req_id = req_id;
                        strncpy(prog.filename, fname.c_str(), 127);
                        prog.done = sent; prog.total = fsz; prog.phase = 2;
                        srv->broadcast_iq_progress(prog);
                    }
                    // 전송 완료 후 HOST 파일 삭제
                    if(remove(path.c_str()) == 0)
                        bewe_log_push(0,"[CLI] region IQ transferred and deleted: %s\n", path.c_str());
                }).detach();
            } else {
                // Direct TCP send
                srv->send_file_to((int)oidx, path.c_str(), 0);
            }
        }).detach();
    };

    srv->cb.on_toggle_recv = [&](int ch_idx, uint8_t op_idx, bool enable){
        if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
        uint32_t bit = 1u << op_idx;
        uint32_t old_mask = v.channels[ch_idx].audio_mask.load();
        uint32_t new_mask;
        do {
            new_mask = enable ? (old_mask | bit) : (old_mask & ~bit);
        } while(!v.channels[ch_idx].audio_mask.compare_exchange_weak(old_mask, new_mask));
    };
    srv->cb.on_update_ch_range = [&](int idx, float s, float e){
        if(idx<0||idx>=MAX_CHANNELS) return;
        v.channels[idx].s = s;
        v.channels[idx].e = e;
        if(v.channels[idx].dem_run.load()){
            Channel::DemodMode md = v.channels[idx].mode;
            v.stop_dem(idx); v.start_dem(idx, md);
        }
        // 리사이즈로 범위 밖/안 전환될 수 있음 → 재평가
        v.update_dem_by_freq(v.header.center_frequency/1e6f);
        srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
    };
    srv->cb.on_start_rec  = [&](int){ v.start_rec(); };
    srv->cb.on_stop_rec   = [&](){ v.stop_rec(); };
    srv->cb.on_chat       = [&](const char* from, const char* msg){
        bewe_log_push(0,"[CHAT] %s: %s\n", from, msg);
    };

    // Share download
    srv->cb.on_share_download_req = [&](uint8_t op_idx, const char* filename){
        std::string fn(filename);
        bool is_iq = (fn.size()>3 && fn.substr(0,3)=="IQ_") || (fn.size()>3 && fn.substr(0,3)=="sa_");
        std::string path = (is_iq ? BEWEPaths::public_iq_dir() : BEWEPaths::public_audio_dir()) + "/" + fn;
        struct stat st{};
        if(stat(path.c_str(),&st)!=0) return;
        uint8_t tid = v.next_transfer_id.fetch_add(1);
        int op_int = (int)op_idx;
        std::string path_copy = path;
        std::thread([srv, path_copy, op_int, tid](){
            srv->send_file_to(op_int, path_copy.c_str(), tid);
        }).detach();
    };

    // Share upload done
    srv->cb.on_share_upload_done = [&](uint8_t, const char* op_name, const char* tmp_path){
        const char* fn = strrchr(tmp_path, '/'); fn = fn ? fn+1 : tmp_path;
        if(strncmp(fn,"bewe_up_",8)==0) fn+=8;
        bool is_iq = (strlen(fn)>3 && strncmp(fn,"IQ_",3)==0)
                  || (strlen(fn)>3 && strncmp(fn,"sa_",3)==0);
        std::string pub_dir = is_iq ? BEWEPaths::public_iq_dir() : BEWEPaths::public_audio_dir();
        struct stat sd{}; if(stat(pub_dir.c_str(),&sd)!=0) mkdir(pub_dir.c_str(),0755);
        std::string dst = pub_dir + "/" + fn;
        FILE* fin = fopen(tmp_path,"rb"); FILE* fout = fopen(dst.c_str(),"wb");
        if(fin&&fout){ char buf[65536]; size_t n; while((n=fread(buf,1,sizeof(buf),fin))>0) fwrite(buf,1,n,fout); }
        if(fin) fclose(fin); if(fout) fclose(fout);
        remove(tmp_path);
        std::string fname(fn);
        pub_owners[fname] = std::string(op_name);
        if(is_iq){ bool dup=false; for(auto& sf:pub_iq_files) if(sf==fname){dup=true;break;} if(!dup) pub_iq_files.push_back(fname); }
        else     { bool dup=false; for(auto& sf:pub_audio_files) if(sf==fname){dup=true;break;} if(!dup) pub_audio_files.push_back(fname); }
        { bool dup=false; for(auto& sf:shared_files) if(sf==fname){dup=true;break;} if(!dup) shared_files.push_back(fname); }
        // Broadcast updated list
        std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
        for(auto& sf : shared_files){
            bool siq = (sf.size()>3&&sf.substr(0,3)=="IQ_")||(sf.size()>3&&sf.substr(0,3)=="sa_");
            std::string sfp = (siq?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+sf;
            struct stat sst{}; uint64_t fsz=0;
            if(stat(sfp.c_str(),&sst)==0) fsz=(uint64_t)sst.st_size;
            std::string upl; auto it=pub_owners.find(sf); if(it!=pub_owners.end()) upl=it->second;
            slist.push_back({sf,fsz,upl});
        }
        srv->send_share_list(-1, slist);
        bewe_log_push(0,"[CLI] Public upload done: %s (from %s)\n", fn, op_name);
    };

    srv->cb.on_set_fft_size = [&](const char* who, uint32_t size){
        bewe_log_push(0, "[CMD:%s] FFT size > %u\n", who, size);
        static const int valid[]={512,1024,2048,4096,8192,16384};
        for(int vs : valid)
            if((uint32_t)vs==size){ v.pending_fft_size=size; v.fft_size_change_req=true; break; }
    };
    srv->cb.on_set_sr = [&](const char* who, float msps){
        bewe_log_push(0, "[CMD:%s] SR > %.2f MSPS\n", who, msps);
        v.pending_sr_msps=msps; v.sr_change_req=true;
    };
    srv->cb.on_set_antenna = [&](const char* who, const char* antenna){
        bewe_log_push(0, "[CMD:%s] Antenna > '%s'\n", who, antenna?antenna:"");
        strncpy(v.host_antenna, antenna?antenna:"", sizeof(v.host_antenna)-1);
        v.host_antenna[sizeof(v.host_antenna)-1] = '\0';
    };

    // ── 예약 녹음 (JOIN → HOST) ───────────────────────────────────────────
    srv->cb.on_add_sched = [&](uint8_t op_idx, const char* op_name,
                                int64_t start_time, float duration_sec,
                                float freq_mhz, float bw_khz,
                                const char* target){
        if(duration_sec <= 0 || freq_mhz <= 0 || bw_khz <= 0){
            bewe_log_push(0,"[CMD:%s] SCHED add denied: invalid params\n", op_name);
            return;
        }
        time_t now = time(nullptr);
        if((time_t)start_time + (time_t)duration_sec < now){
            bewe_log_push(0,"[CMD:%s] SCHED add denied: past time\n", op_name);
            return;
        }
        {
            std::lock_guard<std::mutex> lk(v.sched_mtx);
            if(v.sched_has_overlap((time_t)start_time, duration_sec)){
                bewe_log_push(0,"[CMD:%s] SCHED add denied: overlap\n", op_name);
                return;
            }
            if((int)v.sched_entries.size() >= MAX_SCHED_ENTRIES){
                bewe_log_push(0,"[CMD:%s] SCHED add denied: list full\n", op_name);
                return;
            }
            FFTViewer::SchedEntry e;
            e.start_time   = (time_t)start_time;
            e.duration_sec = duration_sec;
            e.freq_mhz     = freq_mhz;
            e.bw_khz       = bw_khz;
            e.status       = FFTViewer::SchedEntry::WAITING;
            e.op_index     = op_idx;
            strncpy(e.operator_name, op_name?op_name:"", sizeof(e.operator_name)-1);
            strncpy(e.target,        target ?target :"", sizeof(e.target)-1);
            v.sched_entries.push_back(e);
            bewe_log_push(0,"[CMD:%s] SCHED added: %.3fMHz %.0fkHz dur=%.0fs target='%s' at %lld\n",
                          op_name, freq_mhz, bw_khz, duration_sec, e.target, (long long)start_time);
        }
        v.broadcast_sched_list();
    };
    srv->cb.on_remove_sched = [&](uint8_t op_idx, const char* op_name,
                                   int64_t start_time, float freq_mhz){
        bool removed = false;
        {
            std::lock_guard<std::mutex> lk(v.sched_mtx);
            for(auto it = v.sched_entries.begin(); it != v.sched_entries.end(); ++it){
                if((time_t)start_time != it->start_time) continue;
                if(fabsf(freq_mhz - it->freq_mhz) > 0.0001f) continue;
                // RECORDING/ARMED는 불가 (권한 검사는 누구나 가능하도록 제거)
                if(it->status == FFTViewer::SchedEntry::RECORDING ||
                   it->status == FFTViewer::SchedEntry::ARMED){
                    bewe_log_push(0,"[CMD:%s] SCHED remove denied: in progress\n", op_name);
                    return;
                }
                (void)op_idx;
                v.sched_entries.erase(it);
                removed = true;
                break;
            }
        }
        if(removed){
            bewe_log_push(0,"[CMD:%s] SCHED removed\n", op_name);
            v.broadcast_sched_list();
        }
    };

    // 예약 녹음 완료 시 자동 DB 업로드: HOST가 로컬 파일을 central DB로 전송
    // on_relay_broadcast가 central_cli에 연결되어 있으면 그 경로로, 아니면 로컬 DB 폴더에 복사
    v.sched_db_upload_fn = [&, srv](const std::string& path, const std::string& op, const std::string& info){
        FILE* fp = fopen(path.c_str(), "rb");
        if(!fp){ bewe_log_push(0,"[SCHED-DB] open failed: %s\n", path.c_str()); return; }
        fseek(fp, 0, SEEK_END); long total = ftell(fp); fseek(fp, 0, SEEK_SET);
        if(total <= 0){ fclose(fp); return; }

        const char* slash = strrchr(path.c_str(), '/');
        const char* base = slash ? slash+1 : path.c_str();

        bool relay_ok = (srv && srv->cb.on_relay_broadcast) ? true : false;

        if(relay_ok){
            PktDbSaveMeta meta{};
            strncpy(meta.filename, base, sizeof(meta.filename)-1);
            meta.total_bytes = (uint64_t)total;
            static std::atomic<uint32_t> g_tid{1};
            meta.transfer_id = (uint8_t)(g_tid.fetch_add(1) & 0xFF);
            strncpy(meta.operator_name, op.c_str(), sizeof(meta.operator_name)-1);
            strncpy(meta.info_data, info.c_str(), sizeof(meta.info_data)-1);

            auto meta_pkt = make_packet(PacketType::DB_SAVE_META, &meta, sizeof(meta));
            srv->cb.on_relay_broadcast(meta_pkt.data(), meta_pkt.size(), true);

            constexpr size_t CHUNK = 64*1024;
            std::vector<uint8_t> buf(sizeof(PktDbSaveData) + CHUNK);
            long sent = 0;
            while(sent < total){
                size_t n = (size_t)std::min<long>(CHUNK, total - sent);
                auto* d = reinterpret_cast<PktDbSaveData*>(buf.data());
                d->transfer_id = meta.transfer_id;
                d->is_last     = (sent + (long)n >= total) ? 1 : 0;
                d->chunk_bytes = (uint32_t)n;
                if(fread(buf.data() + sizeof(PktDbSaveData), 1, n, fp) != n) break;
                auto data_pkt = make_packet(PacketType::DB_SAVE_DATA, buf.data(), sizeof(PktDbSaveData)+n);
                srv->cb.on_relay_broadcast(data_pkt.data(), data_pkt.size(), true);
                sent += (long)n;
            }
            bewe_log_push(0,"[SCHED-DB] uploaded %s (%ld bytes) by %s\n", base, total, op.c_str());
        } else {
            // 로컬 DB 폴더에 복사 (central 미연결 시 폴백)
            std::string db_path = BEWEPaths::database_dir() + "/" + base;
            FILE* out = fopen(db_path.c_str(), "wb");
            if(out){
                constexpr size_t BUF = 64*1024;
                std::vector<uint8_t> tmp(BUF);
                size_t n;
                while((n = fread(tmp.data(), 1, BUF, fp)) > 0) fwrite(tmp.data(), 1, n, out);
                fclose(out);
                std::string info_path = db_path + ".info";
                FILE* fi = fopen(info_path.c_str(), "w");
                if(fi){ fputs(info.c_str(), fi); fclose(fi); }
                bewe_log_push(0,"[SCHED-DB] saved locally: %s\n", db_path.c_str());
            }
        }
        fclose(fp);
    };
    srv->cb.on_chassis_reset = [&](const char* who){ bewe_log_push(0,"[CMD:%s] /chassis 1 reset\n",who); pending_chassis1_reset.store(true); };
    srv->cb.on_net_reset     = [&](const char* who){ bewe_log_push(0,"[CMD:%s] /chassis 2 reset\n",who); pending_chassis2_reset.store(true); };
    srv->cb.on_rx_stop       = [&](const char* who){ bewe_log_push(0,"[CMD:%s] /rx stop\n",who); pending_rx_stop.store(true); };
    srv->cb.on_rx_start      = [&](const char* who){ bewe_log_push(0,"[CMD:%s] /rx start\n",who); pending_rx_start.store(true); };
    srv->cb.on_pub_delete_req = [&](const char* op_name, const char* filename){
        std::string fname(filename);
        auto oit = pub_owners.find(fname);
        if(oit == pub_owners.end() || oit->second != std::string(op_name)) return;
        bool is_iq = (fname.size()>3&&fname.substr(0,3)=="IQ_")||(fname.size()>3&&fname.substr(0,3)=="sa_");
        std::string fp = (is_iq?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+fname;
        remove(fp.c_str());
        auto rm_from = [&](std::vector<std::string>& vec){
            vec.erase(std::remove(vec.begin(),vec.end(),fname),vec.end());
        };
        rm_from(pub_iq_files); rm_from(pub_audio_files); rm_from(shared_files);
        pub_owners.erase(fname); pub_listeners.erase(fname);
        std::vector<std::tuple<std::string,uint64_t,std::string>> slist;
        for(auto& sf : shared_files){
            bool siq2 = (sf.size()>3&&sf.substr(0,3)=="IQ_")||(sf.size()>3&&sf.substr(0,3)=="sa_");
            std::string sfp = (siq2?BEWEPaths::public_iq_dir():BEWEPaths::public_audio_dir())+"/"+sf;
            struct stat sst{}; uint64_t fsz=0;
            if(stat(sfp.c_str(),&sst)==0) fsz=(uint64_t)sst.st_size;
            std::string upl; auto it=pub_owners.find(sf); if(it!=pub_owners.end()) upl=it->second;
            slist.push_back({sf,fsz,upl});
        }
        srv->send_share_list(-1, slist);
    };

    // ── Report: JOIN이 파일을 report에 추가 → 전체 브로드캐스트 ──────
    srv->cb.on_report_add = [&](uint8_t op_idx, const char* op_name,
                                 const char* filename, const char* info_summary){
        bewe_log_push(0,"[Report] %s reported '%s'\n", op_name, filename);
        // report 목록 갱신 후 브로드캐스트
        std::vector<ReportFileEntry> entries;
        auto scan_rpt = [](const std::string& dir, std::vector<ReportFileEntry>& out){
            DIR* d = opendir(dir.c_str()); if(!d) return;
            struct dirent* e;
            while((e = readdir(d))){
                if(e->d_name[0]=='.') continue;
                std::string n(e->d_name);
                if(n.size()<5 || n.substr(n.size()-4)!=".wav") continue;
                ReportFileEntry re{};
                strncpy(re.filename, n.c_str(), 127);
                std::string fp = dir+"/"+n;
                struct stat st{}; if(stat(fp.c_str(),&st)==0) re.size_bytes=(uint64_t)st.st_size;
                // .info에서 reporter 읽기
                std::string ip = fp+".info";
                FILE* fi = fopen(ip.c_str(), "r");
                if(fi){
                    char line[256];
                    while(fgets(line,sizeof(line),fi)){
                        char k[64]={},val[128]={};
                        if(sscanf(line,"%63[^:]: %127[^\n]",k,val)>=1){
                            if(strcmp(k,"Operator")==0) strncpy(re.reporter,val,31);
                        }
                    }
                    fclose(fi);
                }
                out.push_back(re);
            }
            closedir(d);
        };
        scan_rpt(BEWEPaths::report_iq_dir(), entries);
        scan_rpt(BEWEPaths::report_audio_dir(), entries);
        srv->broadcast_report_list(entries);
    };

    // ── DB Save: JOIN이 파일을 Central DB에 저장 → HOST가 대행 ──────
    static struct { FILE* fp=nullptr; std::string path; uint8_t tid=0; } db_recv;
    srv->cb.on_db_save = [&](uint8_t op_idx, const char* op_name,
                              const PktDbSaveMeta* meta, const uint8_t* data, uint32_t len){
        if(meta){
            // META: 파일 열기 (flat — operator는 .info 의 Operator: 필드로 보존)
            mkdir(BEWEPaths::database_dir().c_str(), 0755);
            std::string dst = BEWEPaths::database_dir() + "/" + std::string(meta->filename);
            bewe_log_push(0,"[DB] Save '%s' by %s (%.1fMB)\n",
                meta->filename, meta->operator_name, meta->total_bytes/1048576.0);
            if(db_recv.fp) fclose(db_recv.fp);
            db_recv.fp = fopen(dst.c_str(), "wb");
            db_recv.path = dst;
            db_recv.tid = meta->transfer_id;
            // .info 저장
            if(meta->info_data[0]){
                std::string info_dst = dst + ".info";
                FILE* fi = fopen(info_dst.c_str(), "w");
                if(fi){ fwrite(meta->info_data, 1, strnlen(meta->info_data, 511), fi); fclose(fi); }
            }
        } else if(data && len >= sizeof(PktDbSaveData)){
            // DATA: 파일에 쓰기
            auto* d = reinterpret_cast<const PktDbSaveData*>(data);
            if(db_recv.fp && d->chunk_bytes > 0){
                fwrite(data + sizeof(PktDbSaveData), 1, d->chunk_bytes, db_recv.fp);
            }
            if(d->is_last && db_recv.fp){
                fclose(db_recv.fp);
                bewe_log_push(0,"[DB] Save complete: %s\n", db_recv.path.c_str());
                db_recv.fp = nullptr;
                db_recv.path.clear();
            }
        }
    };

    srv->cb.on_db_delete = [&](const char* who, const char* filename, const char* operator_name){
        // Central 연결 시 → Central로 포워드
        if(srv->cb.on_relay_broadcast){
            PktDbDeleteReq req{};
            strncpy(req.filename, filename, 127);
            strncpy(req.operator_name, operator_name, 31);
            auto pkt = make_packet(PacketType::DB_DELETE_REQ, &req, sizeof(req));
            srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
            bewe_log_push(0,"[CMD:%s] DB_DELETE '%s' by '%s' → Central\n", who, filename, operator_name);
        } else {
            // Central 없음 → 로컬 삭제 (flat — operator_name 무시)
            std::string fpath = BEWEPaths::database_dir() + "/" + filename;
            int r = remove(fpath.c_str());
            remove((fpath + ".info").c_str());
            bewe_log_push(0,"[CMD:%s] DB_DELETE '%s': %s\n", who, filename,
                          r==0 ? "OK" : strerror(errno));
        }
    };

    srv->cb.on_db_download_req = [&](uint8_t op_idx, const char* who, const char* filename, const char* operator_name){
        // Central 연결 시 → Central로 포워드
        if(srv->cb.on_relay_broadcast){
            PktDbDownloadReq req{};
            strncpy(req.filename, filename, 127);
            strncpy(req.operator_name, operator_name, 31);
            auto pkt = make_packet(PacketType::DB_DOWNLOAD_REQ, &req, sizeof(req));
            srv->cb.on_relay_broadcast(pkt.data(), pkt.size(), true);
            bewe_log_push(0,"[CMD:%s] DB_DOWNLOAD '%s' by '%s' → Central\n", who, filename, operator_name);
        } else {
            bewe_log_push(0,"[CMD:%s] DB_DOWNLOAD '%s': no Central, not supported\n", who, filename);
        }
    };

    // ── Start server ─────────────────────────────────────────────────────
    if(!srv->start(0)){
        bewe_log_push(0,"[BEWE CLI] Server start failed\n");
        delete srv; srv=nullptr;
        g_shutdown.store(true);
    } else {
        host_port = srv->listen_port();
        v.net_srv = srv;
        srv->set_host_info(login_get_id(), (uint8_t)login_get_tier());
        bewe_log_push(0,"[BEWE CLI] Server started on port %d\n", host_port);

        // Central MUX adapter
        if(central_host[0] != '\0'){
            std::string sid = v.station_name + "_" + std::string(login_get_id());
            int rfd = central_cli.open_room(
                central_host, central_port, sid, v.station_name,
                v.station_lat, v.station_lon,
                (uint8_t)login_get_tier());
            if(rfd >= 0){
                // Relay CHANNEL_SYNC callback
                central_cli.set_on_central_ch_sync([&v](const uint8_t* pkt, size_t len){
                    size_t entry_sz = sizeof(ChSyncEntry); // 80 bytes
                    if(len < 9 + entry_sz*10) return;
                    const uint8_t* payload = pkt + 9;
                    for(int i=0; i<MAX_CHANNELS && i<10; i++){
                        uint32_t mask;
                        memcpy(&mask, payload + i*entry_sz + 12, sizeof(mask));
                        v.channels[i].audio_mask.store(mask);
                    }
                });
                // Central DB 목록 수신
                extern std::vector<DbFileEntry> g_db_list;
                extern std::mutex g_db_list_mtx;
                central_cli.set_on_central_db_list([&](const uint8_t* pkt, size_t len){
                    extern std::vector<DbFileEntry> g_db_list;
                    extern std::mutex g_db_list_mtx;
                    if(len < 9 + sizeof(PktDbList)) return;
                    const uint8_t* payload = pkt + 9;
                    auto* hdr2 = reinterpret_cast<const PktDbList*>(payload);
                    uint16_t cnt2 = hdr2->count;
                    size_t expected = sizeof(PktDbList) + cnt2 * sizeof(DbFileEntry);
                    if(len - 9 < expected) return;
                    const DbFileEntry* ent = reinterpret_cast<const DbFileEntry*>(payload + sizeof(PktDbList));
                    std::vector<DbFileEntry> entries(ent, ent + cnt2);
                    { std::lock_guard<std::mutex> lk(g_db_list_mtx);
                      g_db_list = entries; }
                    // 직접 접속 JOIN에도 DB_LIST 전달
                    if(srv) srv->broadcast_db_list(entries);
                    bewe_log_push(0,"[Central] DB_LIST: %u files\n", cnt2);
                });

                // DB 다운로드 .info 수신 (Central → HOST) — .wav 보다 먼저 도착
                central_cli.set_on_central_db_dl_info([](const uint8_t* pkt, size_t len){
                    if(len < 9 + sizeof(PktDbDownloadInfo)) return;
                    const auto* di = reinterpret_cast<const PktDbDownloadInfo*>(pkt + 9);
                    char fn[129]={}; strncpy(fn, di->filename, 128);
                    bool is_iq = (strncmp(fn,"IQ_",3)==0||strncmp(fn,"sa_",3)==0);
                    std::string dir = is_iq ? BEWEPaths::record_iq_dir() : BEWEPaths::record_audio_dir();
                    mkdir(dir.c_str(), 0755);
                    std::string ipath = dir + "/" + fn + ".info";
                    FILE* fi = fopen(ipath.c_str(), "w");
                    if(fi){
                        size_t n = strnlen(di->info_data, sizeof(di->info_data));
                        if(n > 0) fwrite(di->info_data, 1, n, fi);
                        fclose(fi);
                        bewe_log_push(0,"[DB] Download .info saved: %s\n", ipath.c_str());
                    }
                });

                // DB 다운로드 데이터 수신 (Central → HOST)
                static FILE* host_db_dl_fp = nullptr;
                static std::string host_db_dl_path;
                central_cli.set_on_central_db_dl_data([&v](const uint8_t* pkt, size_t len){
                    if(len < 9 + sizeof(PktDbDownloadData)) return;
                    const auto* d = reinterpret_cast<const PktDbDownloadData*>(pkt + 9);
                    const uint8_t* data = pkt + 9 + sizeof(PktDbDownloadData);
                    uint32_t data_len = d->chunk_bytes;
                    if(d->is_first){
                        bool is_iq = (strncmp(d->filename,"IQ_",3)==0||strncmp(d->filename,"sa_",3)==0);
                        std::string dir = is_iq ? BEWEPaths::record_iq_dir() : BEWEPaths::record_audio_dir();
                        mkdir(dir.c_str(), 0755);
                        host_db_dl_path = dir + "/" + d->filename;
                        if(host_db_dl_fp) fclose(host_db_dl_fp);
                        host_db_dl_fp = fopen(host_db_dl_path.c_str(), "wb");
                        bewe_log_push(0,"[DB] Download start: %s (%.1fMB)\n", d->filename, d->total_bytes/1048576.0);
                    }
                    if(host_db_dl_fp && data_len > 0)
                        fwrite(data, 1, data_len, host_db_dl_fp);
                    if(d->is_last && host_db_dl_fp){
                        fclose(host_db_dl_fp);
                        host_db_dl_fp = nullptr;
                        bewe_log_push(0,"[DB] Download done: %s\n", host_db_dl_path.c_str());
                        host_db_dl_path.clear();
                    }
                });

                // Central Report 목록 수신
                extern std::vector<ReportFileEntry> g_report_list;
                extern std::mutex g_report_list_mtx;
                // Central에 저장된 예약 리스트를 HOST가 받아 v.sched_entries 복원
                central_cli.set_on_central_sched_sync([&v](const uint8_t* pkt, size_t len){
                    if(len < 9 + sizeof(PktSchedSync)) return;
                    auto* ss = reinterpret_cast<const PktSchedSync*>(pkt + 9);
                    int n = std::min<int>(ss->count, MAX_SCHED_ENTRIES);
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    // 현재 활성(ARM/REC) 엔트리가 있으면 그 식별자를 기억
                    int64_t active_st   = -1;
                    float   active_freq = 0.f;
                    if(v.sched_active_idx >= 0 && v.sched_active_idx < (int)v.sched_entries.size()){
                        active_st   = (int64_t)v.sched_entries[v.sched_active_idx].start_time;
                        active_freq = v.sched_entries[v.sched_active_idx].freq_mhz;
                    }
                    std::vector<FFTViewer::SchedEntry> next;
                    next.reserve(n);
                    int new_active = -1;
                    for(int i=0; i<n; i++){
                        const auto& se = ss->entries[i];
                        if(!se.valid) continue;
                        FFTViewer::SchedEntry ne;
                        ne.start_time   = (time_t)se.start_time;
                        ne.duration_sec = se.duration_sec;
                        ne.freq_mhz     = se.freq_mhz;
                        ne.bw_khz       = se.bw_khz;
                        ne.op_index     = se.op_index;
                        strncpy(ne.operator_name, se.operator_name, sizeof(ne.operator_name)-1);
                        strncpy(ne.target,        se.target,        sizeof(ne.target)-1);
                        // 활성 엔트리는 로컬 상태/타임스탬프/채널 유지
                        if(active_st == se.start_time && fabsf(active_freq - se.freq_mhz) < 1e-4f){
                            ne.status      = v.sched_entries[v.sched_active_idx].status;
                            ne.temp_ch_idx = v.sched_entries[v.sched_active_idx].temp_ch_idx;
                            ne.rec_started = v.sched_entries[v.sched_active_idx].rec_started;
                            new_active     = (int)next.size();
                        } else {
                            ne.status = (FFTViewer::SchedEntry::Status)se.status;
                        }
                        next.push_back(ne);
                    }
                    v.sched_entries = std::move(next);
                    v.sched_active_idx = new_active;
                    bewe_log_push(0, "[Central] restored %d scheduled entries\n", (int)v.sched_entries.size());
                });

                // ── Host-owned band plan (~/BE_WE/band_plan.json) ────────
                // Load from disk on host start, mirror into v.band_segments,
                // accept JOIN/HOST edits, persist + rebroadcast.
                HostBandPlan::load_from_file();
                HostBandPlan::rebuild_cache();
                auto mirror_into_v = [&v](){
                    PktBandPlan bp{};
                    HostBandPlan::snapshot_pkt(bp);
                    std::lock_guard<std::mutex> lk(v.band_mtx);
                    v.band_segments.clear();
                    int n = std::min<int>((int)bp.count, MAX_BAND_SEGMENTS);
                    for(int i=0;i<n;i++){
                        const auto& be = bp.entries[i];
                        if(!be.valid) continue;
                        FFTViewer::BandSegment s;
                        s.freq_lo_mhz = be.freq_lo_mhz;
                        s.freq_hi_mhz = be.freq_hi_mhz;
                        s.category    = be.category;
                        strncpy(s.label,       be.label,       sizeof(s.label)-1);
                        strncpy(s.description, be.description, sizeof(s.description)-1);
                        v.band_segments.push_back(s);
                    }
                };
                mirror_into_v();
                auto rebroadcast_band_plan = [&v, mirror_into_v](){
                    HostBandPlan::save_to_file();
                    HostBandPlan::rebuild_cache();
                    PktBandPlan bp{};
                    HostBandPlan::snapshot_pkt(bp);
                    if(v.net_srv) v.net_srv->broadcast_band_plan(bp);
                    mirror_into_v();
                };
                srv->cb.on_band_add = [rebroadcast_band_plan](const PktBandEntry& e){
                    if(HostBandPlan::apply_add(e)) rebroadcast_band_plan();
                };
                srv->cb.on_band_update = [rebroadcast_band_plan](const PktBandEntry& e){
                    if(HostBandPlan::apply_update(e)) rebroadcast_band_plan();
                };
                srv->cb.on_band_remove = [rebroadcast_band_plan](const PktBandRemove& r){
                    if(HostBandPlan::apply_remove(r)) rebroadcast_band_plan();
                };

                // ── Host-owned band categories (~/BE_WE/band_categories.json) ─
                HostBandCategories::load_from_file();
                HostBandCategories::rebuild_cache();
                auto rebroadcast_band_cat = [&v](){
                    HostBandCategories::save_to_file();
                    HostBandCategories::rebuild_cache();
                    PktBandCatSync cs{};
                    HostBandCategories::snapshot_pkt(cs);
                    if(v.net_srv) v.net_srv->broadcast_band_categories(cs);
                };
                srv->cb.on_band_cat_upsert = [rebroadcast_band_cat](const PktBandCategory& c){
                    if(HostBandCategories::apply_upsert(c)) rebroadcast_band_cat();
                };
                srv->cb.on_band_cat_delete = [rebroadcast_band_cat](uint8_t id){
                    if(HostBandCategories::apply_delete(id)) rebroadcast_band_cat();
                };

                // ── Long Waterfall: serve list + file download to JOINs ─
                srv->cb.on_lwf_list_req = [&v](int op_index, const char* who){
                    PktLwfList list{};
                    LongWaterfall::scan_dir_into_list(list);
                    if(v.net_srv) v.net_srv->send_lwf_list_to_op(op_index, list);
                    bewe_log_push(0, "[LWF] LIST_REQ from op=%d '%s' → %u files\n",
                                  op_index, who?who:"?", (unsigned)list.count);
                };
                srv->cb.on_lwf_dl_req = [&v](int op_index, const char* who, const char* fn){
                    if(!fn || !fn[0]) return;
                    if(strchr(fn, '/')) return;
                    std::string full = BEWEPaths::hist_host_dir() + "/" + fn;
                    std::string who_s = who ? who : "?";
                    static std::atomic<uint8_t> tid_ctr{1};
                    uint8_t tid = tid_ctr.fetch_add(1);
                    if(tid == 0) tid = tid_ctr.fetch_add(1);  // 0 reserved for non-HIST
                    bewe_log_push(0, "[LWF] DL_REQ from op=%d '%s' file=%s tid=%u\n",
                                  op_index, who_s.c_str(), fn, (unsigned)tid);
                    std::thread([&v, op_index, full, tid](){
                        if(v.net_srv) v.net_srv->send_file_to(op_index, full.c_str(), tid);
                    }).detach();
                };
                // STREAM opt-in: JOIN이 LWF_LIVE_REQ 보낼 때만 그 op에 한해 LIVE_START unicast.
                srv->cb.on_lwf_live_req = [&v](int op_index, const char* who){
                    PktLwfLiveStart ls{};
                    if(!LongWaterfall::snapshot_live_start(ls)){
                        bewe_log_push(1, "[LWF] LIVE_REQ op=%d '%s' but no LIVE file open\n",
                                      op_index, who?who:"?");
                        return;
                    }
                    if(v.net_srv) v.net_srv->send_lwf_live_start_to_op(op_index, ls);
                    bewe_log_push(0, "[LWF] LIVE_REQ from op=%d '%s' → LIVE_START unicast\n",
                                  op_index, who?who:"?");
                };

                // 새 JOIN이 Central을 통해 들어오면 cached band plan + category 즉시 푸시
                central_cli.set_on_central_conn_open([&v, &central_cli](uint16_t cid){
                    std::vector<uint8_t> bp_pkt;
                    {
                        std::lock_guard<std::mutex> lk(HostBandPlan::g_mtx);
                        bp_pkt = HostBandPlan::g_cached_pkt;
                    }
                    std::vector<uint8_t> bc_pkt;
                    {
                        std::lock_guard<std::mutex> lk(HostBandCategories::g_mtx);
                        bc_pkt = HostBandCategories::g_cached_pkt;
                    }
                    bewe_log_push(0, "[HostBand] CONN_OPEN cid=%u → push plan %zu + cats %zu bytes\n",
                                  cid, bp_pkt.size(), bc_pkt.size());
                    if(!bc_pkt.empty())
                        central_cli.enqueue_relay_broadcast(bc_pkt.data(), bc_pkt.size(), true);
                    if(!bp_pkt.empty())
                        central_cli.enqueue_relay_broadcast(bp_pkt.data(), bp_pkt.size(), true);
                    // LIVE_START는 JOIN이 STREAM 버튼으로 명시 요청(LWF_LIVE_REQ)할 때만 unicast.
                    (void)v;
                });

                // Worker → NetServer LIVE broadcast 연결.
                LongWaterfall::LiveCallbacks lcb;
                lcb.on_start = [&v](const PktLwfLiveStart& s){
                    if(v.net_srv) v.net_srv->broadcast_lwf_live_start(s);
                };
                lcb.on_row   = [&v](const PktLwfLiveRowHdr& hdr,
                                    const uint8_t* row, uint32_t row_bytes){
                    if(v.net_srv) v.net_srv->broadcast_lwf_live_row(hdr, row, row_bytes);
                };
                lcb.on_stop  = [&v](const PktLwfLiveStop& s){
                    if(v.net_srv) v.net_srv->broadcast_lwf_live_stop(s);
                };
                LongWaterfall::set_live_callbacks(lcb);

                central_cli.set_on_central_report_list([](const uint8_t* pkt, size_t len){
                    extern std::vector<ReportFileEntry> g_report_list;
                    extern std::mutex g_report_list_mtx;
                    if(len < 9 + sizeof(PktReportList)) return;
                    const uint8_t* payload = pkt + 9;
                    auto* hdr = reinterpret_cast<const PktReportList*>(payload);
                    uint16_t cnt = hdr->count;
                    size_t expected = sizeof(PktReportList) + cnt * sizeof(ReportFileEntry);
                    if(len - 9 < expected) return;
                    const ReportFileEntry* ent = reinterpret_cast<const ReportFileEntry*>(payload + sizeof(PktReportList));
                    { std::lock_guard<std::mutex> lk(g_report_list_mtx);
                      g_report_list.assign(ent, ent + cnt); }
                    bewe_log_push(0,"[Central] REPORT_LIST: %u reports\n", cnt);
                });

                srv->cb.on_relay_broadcast = [&central_cli](const uint8_t* pkt, size_t len, bool no_drop){
                    central_cli.enqueue_relay_broadcast(pkt, len, no_drop);
                };
                // Auto-reconnect function
                auto reconnect_fn = std::make_shared<std::function<void()>>();
                *reconnect_fn = [&v, &central_cli,
                                 rh = std::string(central_host), rp = central_port,
                                 reconnect_fn](){
                    std::thread([&v, &central_cli, rh, rp, reconnect_fn](){
                        for(int attempt=0; attempt<5; attempt++){
                            // 3초 대기를 0.1초 단위로 쪼개어 shutdown 즉시 반응
                            for(int i=0;i<30;i++){
                                if(g_shutdown.load()) return;
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            }
                            if(g_shutdown.load()) return;
                            if(!v.net_srv) return;
                            bewe_log_push(0,"[CLI] Central auto-reconnect attempt %d/5\n", attempt+1);
                            std::string sid2 = v.station_name + "_" + std::string(login_get_id());
                            int rfd2 = central_cli.open_room(
                                rh, rp, sid2, v.station_name,
                                v.station_lat, v.station_lon,
                                (uint8_t)login_get_tier());
                            if(rfd2 >= 0){
                                central_cli.start_mux_adapter(rfd2,
                                    [&v](int fd2){ if(v.net_srv) v.net_srv->inject_fd(fd2); },
                                    [&v](){ return v.net_srv ? (uint8_t)v.net_srv->client_count() : (uint8_t)0; },
                                    *reconnect_fn);
                                bewe_log_push(0,"[CLI] Central auto-reconnected\n");
                                return;
                            }
                        }
                        bewe_log_push(0,"[CLI] Central auto-reconnect failed after 5 attempts\n");
                    }).detach();
                };
                central_cli.start_mux_adapter(rfd,
                    [&v](int local_fd){ if(v.net_srv) v.net_srv->inject_fd(local_fd); },
                    [&v](){ return v.net_srv ? (uint8_t)v.net_srv->client_count() : (uint8_t)0; },
                    *reconnect_fn);
                bewe_log_push(0,"[BEWE CLI] Central relay connected\n");
            } else {
                bewe_log_push(0,"[BEWE CLI] Central relay unavailable\n");
            }
        }

        // Broadcast thread
        v.net_bcast_stop.store(false);
        v.net_bcast_thr = std::thread(&FFTViewer::net_bcast_worker, &v);
    }

    // TM IQ
    if(!v.sdr_stream_error.load()){
        bewe_log_push(0,"[CLI] tm_iq_open: sr=%u\n", v.header.sample_rate);
        v.tm_iq_open();
        if(v.tm_iq_file_ready){
            v.tm_iq_on.store(true);
            bewe_log_push(0,"[BEWE CLI] IQ rolling enabled\n");
        }
    }

    // ── System monitor state ─────────────────────────────────────────────
    long long cpu_last_idle=0, cpu_last_total=0, io_last_ms=0;
    read_cpu(cpu_last_idle, cpu_last_total);
    io_last_ms = read_io_ms();

    using clk = std::chrono::steady_clock;
    auto sysmon_last   = clk::now();
    auto sq_sync_last  = clk::now();
    auto status_last   = clk::now();
    auto heartbeat_last= clk::now();
    auto status_print_last = clk::now();
    auto loop_last     = clk::now();

    // SDR reconnect state
    bool     bg_join_started = false;
    std::atomic<bool> cap_joined{false};
    bool     usb_reset_done = false;
    std::atomic<bool> usb_reset_in_progress{false};
    float    sdr_retry_timer = 0.f;
    float    chassis_unpause_timer = -1.f;

    bewe_log_push(0,"[BEWE CLI] Ready. Type /help for commands.\n");
    fflush(stdout);

    // ══════════════════════════════════════════════════════════════════════
    //  Main loop
    // ══════════════════════════════════════════════════════════════════════
    while(!g_shutdown.load()){
        // ~50Hz loop (20ms sleep) — squelch 캘리브레이션이 1.2초 내 완료되도록
        // 다른 주기적 작업들은 자체 interval check 있어서 부하 무관
        auto now = clk::now();
        float dt = std::chrono::duration<float>(now - loop_last).count();
        loop_last = now;
        int sleep_ms = 20 - (int)(dt * 1000);
        if(sleep_ms > 0){
            struct pollfd pfd{STDIN_FILENO, POLLIN, 0};
            poll(&pfd, 1, sleep_ms);
        }

        // ── System monitor (1s) ──────────────────────────────────────────
        {
            float el = std::chrono::duration<float>(clk::now()-sysmon_last).count();
            if(el >= 1.0f){
                sysmon_last = clk::now();
                long long idle,total; read_cpu(idle,total);
                long long d_idle=idle-cpu_last_idle, d_total=total-cpu_last_total;
                v.sysmon_cpu=(d_total>0)?(1.0f-(float)d_idle/d_total)*100.0f:0.0f;
                cpu_last_idle=idle; cpu_last_total=total;
                v.sysmon_ghz=read_ghz();
                v.sysmon_ram=read_ram();
                v.sysmon_cpu_temp_c.store(read_cpu_temp_c());
                long long io_now=read_io_ms();
                v.sysmon_io=std::min(100.0f,(float)(io_now-io_last_ms)/10.0f);
                io_last_ms=io_now;
            }
        }

        // ── Periodic status print (30s) ──────────────────────────────────
        {
            float el = std::chrono::duration<float>(clk::now()-status_print_last).count();
            if(el >= 30.0f){
                status_print_last = clk::now();
                time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
                char ts[16]; strftime(ts,sizeof(ts),"%H:%M:%S",&tm2);
                int clients = v.net_srv ? v.net_srv->client_count() : 0;
                uint64_t net_tx=0, net_drops=0;
                if(v.net_srv){ auto ns=v.net_srv->collect_stats(); net_tx=ns.tx_bytes; net_drops=ns.drops; }
                bewe_log_push(0,"[%s] CF=%.1fMHz SR=%.2fM Clients=%d CPU=%.0f%% RAM=%.0f%% SDR=%s IQ=%s TX=%.1fMB Drops=%llu\n",
                       ts,
                       v.header.center_frequency/1e6,
                       v.header.sample_rate/1e6,
                       clients,
                       v.sysmon_cpu, v.sysmon_ram,
                       v.sdr_stream_error.load()?"ERR":(v.rx_stopped.load()?"STOP":"OK"),
                       v.tm_iq_on.load()?"ON":"OFF",
                       (double)net_tx/(1024*1024),
                       (unsigned long long)net_drops);
                fflush(stdout);
            }
        }

        // ── Squelch update (20ms, GUI 프레임레이트와 유사) + sync broadcast (100ms 유지) ─
        // 빠른 update는 스퀄치 캘리브레이션(60 샘플)이 ~1.2초 내 완료되도록 함
        if(v.net_srv){
            static auto sq_update_last = clk::now();
            float el_up = std::chrono::duration<float>(clk::now()-sq_update_last).count();
            if(el_up >= 0.02f){
                sq_update_last = clk::now();
                v.update_channel_squelch();
            }
        }
        if(v.net_srv && v.net_srv->client_count()>0){
            float el = std::chrono::duration<float>(clk::now()-sq_sync_last).count();
            if(el >= 0.1f){
                sq_sync_last = clk::now();
                v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            }
        }

        // ── Scheduled recording tick (1초마다) ───────────────────────────
        {
            static auto sched_last = clk::now();
            float el = std::chrono::duration<float>(clk::now()-sched_last).count();
            if(el >= 1.0f){
                sched_last = clk::now();
                // 상태 변화 감지용 스냅샷
                uint32_t before_hash = 0;
                {
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    for(auto& e : v.sched_entries)
                        before_hash = before_hash*131 + (uint32_t)e.status;
                }
                v.sched_tick();
                uint32_t after_hash = 0;
                {
                    std::lock_guard<std::mutex> lk(v.sched_mtx);
                    for(auto& e : v.sched_entries)
                        after_hash = after_hash*131 + (uint32_t)e.status;
                }
                // 상태 전이 발생 시 JOIN들에게 즉시 통지
                if(before_hash != after_hash)
                    v.broadcast_sched_list();
            }
        }

        // ── Time tag + wf_event broadcast (5초마다) ─────────────────────
        {
            static int cli_last_tagged_sec = -1;
            time_t now_tt = time(nullptr);
            struct tm* tt = localtime(&now_tt);
            int cur5 = tt->tm_hour*720 + tt->tm_min*12 + tt->tm_sec/5;
            if(cur5 != cli_last_tagged_sec){
                cli_last_tagged_sec = cur5;
                v.tm_add_time_tag(v.current_fft_idx);
                if(v.net_srv){
                    char lbl[32]; strftime(lbl, sizeof(lbl), "%H:%M:%S", tt);
                    v.net_srv->broadcast_wf_event(0, (int64_t)now_tt, 0, lbl);
                }
            }
        }

        // ── STATUS broadcast (1s) ────────────────────────────────────────
        if(v.net_srv && v.net_srv->client_count()>0){
            float el = std::chrono::duration<float>(clk::now()-status_last).count();
            if(el >= 1.0f){
                status_last = clk::now();
                uint8_t hwt = (v.hw.type==HWType::RTLSDR) ? 1 :
                              (v.hw.type==HWType::PLUTO)  ? 2 : 0;
                v.net_srv->broadcast_status(
                    (float)(v.header.center_frequency/1e6),
                    v.gain_db, v.header.sample_rate, hwt);
            }
        }

        // ── Heartbeat (1s) ───────────────────────────────────────────────
        if(v.net_srv){
            float el = std::chrono::duration<float>(clk::now()-heartbeat_last).count();
            bool cur_sdr_err = v.sdr_stream_error.load();
            if(el >= 1.0f){
                heartbeat_last = clk::now();
                uint8_t sdr_t_hb = 0;
                if(v.dev_blade){
                    float _t = 0.f;
                    if(bladerf_get_rfic_temperature(v.dev_blade, &_t) == 0)
                        sdr_t_hb = (uint8_t)std::min(255.f, std::max(0.f, _t));
                } else if(v.pluto_ctx){
                    float _t = v.pluto_get_temp_c();
                    if(_t > 0.f) sdr_t_hb = (uint8_t)std::min(255.f, _t);
                }
                uint8_t hst = v.spectrum_pause.load() ? 2 : 0;
                uint8_t sdr_st = (cur_sdr_err || v.rx_stopped.load()) ? 1 : 0;
                uint8_t iq_st = v.tm_iq_on.load() ? 1 : 0;
                uint8_t cpu_pct  = (uint8_t)std::min(255.f, std::max(0.f, v.sysmon_cpu));
                uint8_t ram_pct  = (uint8_t)std::min(255.f, std::max(0.f, v.sysmon_ram));
                uint8_t cpu_temp = (uint8_t)std::min(255, std::max(0, v.sysmon_cpu_temp_c.load()));
                v.net_srv->broadcast_heartbeat(hst, sdr_t_hb, sdr_st, iq_st,
                                               cpu_pct, ram_pct, cpu_temp, v.host_antenna);
            }
        }

        // ── SDR 런타임 교체 ──────────────────────────────────────────────
        if(v.pending_sdr_switch.load()){
            v.pending_sdr_switch.store(false);
            std::string new_sdr;
            { std::lock_guard<std::mutex> lk(v.pending_sdr_mtx); new_sdr = v.pending_sdr_name; }
            bewe_log_push(0, "[CLI][SDR] switching to %s ...\n", new_sdr.c_str());
            float cur_cf = (float)(v.header.center_frequency / 1e6);
            for(int ci=0; ci<MAX_CHANNELS; ci++){ v.stop_digi(ci); v.stop_dem(ci); }
            v.is_running = false;
            v.sdr_stream_error.store(true);
            if(cap.joinable()) cap.join();
            v.dev_blade = nullptr; v.dev_rtl = nullptr;
            v.pluto_ctx=nullptr; v.pluto_phy_dev=nullptr; v.pluto_rx_dev=nullptr;
            v.pluto_rx_i_ch=nullptr; v.pluto_rx_q_ch=nullptr; v.pluto_rx_buf=nullptr;
            g_sdr_force = new_sdr;
            v.is_running = true;
            if(v.initialize(cur_cf, 0.f)){
                v.set_gain(v.gain_db);
                v.sdr_stream_error.store(false);
                if(v.hw.type == HWType::BLADERF)
                    cap = std::thread(&FFTViewer::capture_and_process, &v);
                else if(v.hw.type == HWType::PLUTO)
                    cap = std::thread(&FFTViewer::capture_and_process_pluto, &v);
                else
                    cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
                bewe_log_push(0,"[CLI][SDR] switched to %s\n", new_sdr.c_str());
            } else {
                bewe_log_push(2,"[CLI][SDR] switch to %s FAILED\n", new_sdr.c_str());
            }
        }

        // ── Chassis/RX reset processing ──────────────────────────────────
        if(v.net_srv && pending_chassis1_reset.load()){
            pending_chassis1_reset.store(false);
            bewe_log_push(0,"[CLI] Chassis 1 reset ...\n");
            if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "Chassis 1 reset ...");
            if(v.net_srv) v.net_srv->broadcast_heartbeat(1);
            v.is_running = false;
            v.sdr_stream_error.store(true);
            v.tm_iq_on.store(false);
            v.spectrum_pause.store(true);
            usb_reset_pending = true;
        }
        if(v.net_srv && pending_chassis2_reset.load()){
            pending_chassis2_reset.store(false);
            bewe_log_push(0,"[CLI] Chassis 2 reset ...\n");
            if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "Chassis 2 reset ...");
            if(v.net_srv) v.net_srv->broadcast_heartbeat(2);
            v.net_bcast_pause.store(true, std::memory_order_relaxed);
            v.net_srv->pause_broadcast();
            v.net_srv->flush_clients();
            if(central_cli.is_central_connected())
                central_cli.send_net_reset(0);
            NetServer* srv_ptr = v.net_srv;
            std::atomic<bool>* bcast_pause_ptr = &v.net_bcast_pause;
            CentralClient* central_ptr = &central_cli;
            FFTViewer* vp = &v;
            std::string rh = central_host;
            int rp = central_port;
            std::thread([srv_ptr, bcast_pause_ptr, central_ptr, vp, rh, rp](){
                std::this_thread::sleep_for(std::chrono::seconds(1));
                srv_ptr->flush_clients();
                srv_ptr->resume_broadcast();
                bcast_pause_ptr->store(false, std::memory_order_relaxed);
                srv_ptr->broadcast_heartbeat(0);
                srv_ptr->broadcast_chat("SYSTEM", "Chassis 2 stable ...");
                if(!central_ptr->is_central_connected() && !rh.empty()){
                    central_ptr->stop_mux_adapter();
                    std::string sid = vp->station_name + "_" + std::string(login_get_id());
                    int rfd = central_ptr->open_room(
                        rh, rp, sid, vp->station_name,
                        vp->station_lat, vp->station_lon,
                        (uint8_t)login_get_tier());
                    if(rfd >= 0){
                        central_ptr->start_mux_adapter(rfd,
                            [vp](int fd2){ if(vp->net_srv) vp->net_srv->inject_fd(fd2); },
                            [vp](){ return vp->net_srv ? (uint8_t)vp->net_srv->client_count() : (uint8_t)0; });
                        bewe_log_push(0,"[CLI] Central reconnected after chassis 2 reset\n");
                    }
                } else if(central_ptr->is_central_connected()){
                    central_ptr->send_net_reset(1);
                }
                bewe_log_push(0,"[CLI] Chassis 2 stable\n");
            }).detach();
        }

        // RX stop/start from network
        if(v.net_srv && pending_rx_stop.load()){
            pending_rx_stop.store(false);
            if(!v.rx_stopped.load() && (v.is_running || cap.joinable())){
                bewe_log_push(0,"[CLI] RX stop (remote)\n");
                v.net_srv->broadcast_chat("SYSTEM", "RX stop");
                if(v.rec_on.load()) v.stop_rec();
                if(v.tm_iq_on.load()){ v.tm_iq_on.store(false); v.tm_iq_close(); }
                v.stop_all_dem();
                v.is_running = false;
                if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
                v.mix_stop.store(true);
                if(v.mix_thr.joinable()) v.mix_thr.join();
                if(cap.joinable()) cap.join();
                LongWaterfall::stop_worker();
                if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                if(v.dev_blade){
                    bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
                    bladerf_close(v.dev_blade); v.dev_blade=nullptr;
                }
                if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
                v.rx_stopped.store(true);
                v.sdr_stream_error.store(false);
                v.spectrum_pause.store(false);
                bewe_log_push(0,"[CLI] RX stopped\n");
            }
        }
        if(v.net_srv && pending_rx_start.load()){
            pending_rx_start.store(false);
            if(v.rx_stopped.load()){
                bewe_log_push(0,"[CLI] RX start (remote)\n");
                v.rx_stopped.store(false);
                float cur_cf = (float)(v.header.center_frequency / 1e6);
                if(cur_cf < 0.1f) cur_cf = 100.f;
                float cur_sr = v.header.sample_rate / 1e6f;
                if(cur_sr < 0.1f) cur_sr = 61.44f;
                v.is_running = true;
                if(v.initialize(cur_cf, cur_sr)){
                    v.set_gain(v.gain_db);
                    if(v.hw.type == HWType::BLADERF)
                        cap = std::thread(&FFTViewer::capture_and_process, &v);
                    else if(v.hw.type == HWType::PLUTO)
                        cap = std::thread(&FFTViewer::capture_and_process_pluto, &v);
                    else
                        cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
                    v.mix_stop.store(false);
                    v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);
                    v.net_srv->broadcast_chat("SYSTEM", "RX start");
                    bewe_log_push(0,"[CLI] RX started\n");
                } else {
                    v.is_running = false;
                    v.rx_stopped.store(true);
                    bewe_log_push(0,"[CLI] RX start failed - SDR not found\n");
                }
            }
        }

        // ── Chassis 1 unpause timer ──────────────────────────────────────
        if(chassis_unpause_timer > 0.f){
            chassis_unpause_timer -= dt;
            if(chassis_unpause_timer <= 0.f){
                chassis_unpause_timer = -1.f;
                v.spectrum_pause.store(false);
                if(v.net_srv) v.net_srv->broadcast_heartbeat(0, 0, 0);
                bewe_log_push(0,"[CLI] chassis 1 reset: spectrum_pause released\n");
            }
        }

        // ── SR 변경 후 demod 재시작 (BladeRF/RTL-SDR/Pluto 공통) ────────
        if(v.dem_restart_needed.load()){
            v.dem_restart_needed.store(false);
            for(int di=0; di<MAX_CHANNELS; di++){
                if(v.channels[di].dem_run.load()){
                    auto dm = v.channels[di].mode;
                    v.stop_dem(di);
                    v.start_dem(di, dm);
                }
            }
        }

        // ── SDR reconnect logic ──────────────────────────────────────────
        if(!v.remote_mode && v.sdr_stream_error.load() && !v.rx_stopped.load()){
            if(!bg_join_started && v.hw.type == HWType::BLADERF)
                usb_reset_pending = true;
            if(!bg_join_started && cap.joinable()){
                bg_join_started = true;
                cap_joined.store(false);
                usb_reset_done = false;
                std::thread([&cap, &cap_joined](){
                    if(cap.joinable()) cap.join();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                    cap_joined.store(true);
                }).detach();
            } else if(!cap.joinable()){
                cap_joined.store(true);
            }

            if(cap_joined.load() && usb_reset_pending && !usb_reset_done){
                usb_reset_done = true;
                usb_reset_pending = false;
                if(v.hw.type == HWType::BLADERF){
                    usb_reset_in_progress.store(true);
                    if(v.dev_blade){ bladerf_close(v.dev_blade); v.dev_blade=nullptr; }
                    std::thread([&usb_reset_in_progress](){
                        bewe_log_push(0,"[CLI] chassis 1 reset: USB reset BladeRF...\n");
                        bladerf_usb_reset();
                        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                        usb_reset_in_progress.store(false);
                    }).detach();
                }
                // Pluto/RTL-SDR: USB reset 불필요, 재시도 타이머로 즉시 진행
            }

            sdr_retry_timer -= dt;
            if(usb_reset_in_progress.load()) sdr_retry_timer = 1.f;
            if(sdr_retry_timer <= 0.f && cap_joined.load() && !usb_reset_in_progress.load()){
                sdr_retry_timer = 2.f;
                if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                float cur_cf = (float)(v.header.center_frequency / 1e6);
                if(cur_cf < 0.1f) cur_cf = 100.f;
                float cur_sr2 = v.header.sample_rate / 1e6f;
                if(cur_sr2 < 0.1f) cur_sr2 = 61.44f;
                v.is_running = true;
                if(v.initialize(cur_cf, cur_sr2)){
                    bewe_log_push(0,"[CLI] SDR reconnected - resuming at %.2f MHz\n", cur_cf);
                    v.sdr_stream_error.store(false);
                    bg_join_started = false;
                    cap_joined.store(false);
                    usb_reset_in_progress.store(false);
                    v.set_gain(v.gain_db);
                    if(v.hw.type == HWType::BLADERF)
                        cap = std::thread(&FFTViewer::capture_and_process, &v);
                    else if(v.hw.type == HWType::PLUTO)
                        cap = std::thread(&FFTViewer::capture_and_process_pluto, &v);
                    else
                        cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
                    if(v.spectrum_pause.load())
                        chassis_unpause_timer = 1.f;
                    if(v.net_srv){
                        uint8_t hst = v.spectrum_pause.load() ? 2 : 0;
                        v.net_srv->broadcast_heartbeat(hst, 0, 0);
                    }
                } else {
                    v.is_running = false;
                }
            }
        }

        // ── stdin command processing ─────────────────────────────────────
        std::string line;
        while(read_line_nb(line)){
            if(line == "/shutdown"){
                g_shutdown.store(true);
            } else if(line == "/status"){
                int clients = v.net_srv ? v.net_srv->client_count() : 0;
                bewe_log_push(0,"  CF=%.3f MHz  SR=%.2f MSPS  Gain=%.1f dB\n",
                       v.header.center_frequency/1e6,
                       v.header.sample_rate/1e6,
                       v.gain_db);
                bewe_log_push(0,"  Clients=%d  SDR=%s  IQ=%s\n",
                       clients,
                       v.sdr_stream_error.load()?"ERROR":(v.rx_stopped.load()?"STOPPED":"OK"),
                       v.tm_iq_on.load()?"ON":"OFF");
                bewe_log_push(0,"  CPU=%.0f%%  RAM=%.0f%%  IO=%.0f%%  GHz=%.2f\n",
                       v.sysmon_cpu, v.sysmon_ram, v.sysmon_io, v.sysmon_ghz);
                if(v.net_srv){
                    auto ns = v.net_srv->collect_stats();
                    auto fb = [](uint64_t b) -> std::string {
                        char buf[32];
                        if(b < 1024)               snprintf(buf,sizeof(buf),"%llu B",(unsigned long long)b);
                        else if(b < 1024*1024)     snprintf(buf,sizeof(buf),"%.1f KB",(double)b/1024);
                        else if(b < 1024ULL*1024*1024) snprintf(buf,sizeof(buf),"%.1f MB",(double)b/(1024*1024));
                        else                       snprintf(buf,sizeof(buf),"%.2f GB",(double)b/(1024ULL*1024*1024));
                        return buf;
                    };
                    bewe_log_push(0,"  NET: TX=%s  RX=%s  Drops=%llu  Q(fft=%zu audio=%zu)\n",
                           fb(ns.tx_bytes).c_str(), fb(ns.rx_bytes).c_str(),
                           (unsigned long long)ns.drops, ns.q_fft, ns.q_audio);
                }
                fflush(stdout);
            } else if(line == "/clients"){
                if(v.net_srv){
                    auto ops = v.net_srv->get_operators();
                    bewe_log_push(0,"  Connected operators (%d):\n", (int)ops.size());
                    for(auto& op : ops)
                        bewe_log_push(0,"    [%d] %s (tier %d)\n", op.index, op.name, op.tier);
                } else {
                    bewe_log_push(0,"  No server running.\n");
                }
                fflush(stdout);
            } else if(line == "/chassis 1 reset"){
                bewe_log_push(0,"[CMD:CLI] /chassis 1 reset\n");
                if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "Chassis 1 reset ...");
                if(v.net_srv) v.net_srv->broadcast_heartbeat(1);
                if(v.is_running || cap.joinable()){
                    v.is_running = false;
                    v.sdr_stream_error.store(true);
                    v.tm_iq_on.store(false);
                    v.spectrum_pause.store(true);
                    usb_reset_pending = true;
                } else {
                    bewe_log_push(0,"[CLI] No SDR connected - skip HW reset\n");
                }
            } else if(line == "/chassis 2 reset"){
                bewe_log_push(0,"[CMD:CLI] /chassis 2 reset\n");
                pending_chassis2_reset.store(true);
            } else if(line == "/rx stop"){
                bewe_log_push(0,"[CMD:CLI] /rx stop\n");
                if(v.rx_stopped.load()){
                    bewe_log_push(0,"[CLI] RX already stopped.\n");
                } else if(!v.is_running && !cap.joinable()){
                    bewe_log_push(0,"[CLI] No SDR running.\n");
                } else {
                    bewe_log_push(0,"[CLI] RX stop\n");
                    if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "RX stop");
                    if(v.rec_on.load()) v.stop_rec();
                    if(v.tm_iq_on.load()){ v.tm_iq_on.store(false); v.tm_iq_close(); }
                    v.stop_all_dem();
                    v.is_running = false;
                    if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
                    v.mix_stop.store(true);
                    if(v.mix_thr.joinable()) v.mix_thr.join();
                    if(cap.joinable()) cap.join();
                    if(v.fft_plan){ fftwf_destroy_plan(v.fft_plan); v.fft_plan=nullptr; }
                    if(v.fft_in)  { fftwf_free(v.fft_in);   v.fft_in=nullptr; }
                    if(v.fft_out) { fftwf_free(v.fft_out);  v.fft_out=nullptr; }
                    if(v.dev_blade){
                        bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
                        bladerf_close(v.dev_blade); v.dev_blade=nullptr;
                    }
                    if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
                    v.rx_stopped.store(true);
                    v.sdr_stream_error.store(false);
                    v.spectrum_pause.store(false);
                    bewe_log_push(0,"[CLI] RX stopped.\n");
                }
            } else if(line == "/rx start"){
                bewe_log_push(0,"[CMD:CLI] /rx start\n");
                if(!v.rx_stopped.load()){
                    bewe_log_push(0,"[CLI] RX already running.\n");
                } else {
                    bewe_log_push(0,"[CLI] RX start - initializing SDR ...\n");
                    v.rx_stopped.store(false);
                    float cur_cf = (float)(v.header.center_frequency / 1e6);
                    if(cur_cf < 0.1f) cur_cf = cf;
                    float cur_sr3 = v.header.sample_rate / 1e6f;
                    if(cur_sr3 < 0.1f) cur_sr3 = 61.44f;
                    v.is_running = true;
                    if(v.initialize(cur_cf, cur_sr3)){
                        v.set_gain(v.gain_db);
                        if(v.hw.type == HWType::BLADERF)
                            cap = std::thread(&FFTViewer::capture_and_process, &v);
                        else
                            cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
                        v.mix_stop.store(false);
                        v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);
                        if(v.net_srv) v.net_srv->broadcast_chat("SYSTEM", "RX start");
                        bewe_log_push(0,"[CLI] RX started. SDR online.\n");
                    } else {
                        v.is_running = false;
                        v.rx_stopped.store(true);
                        bewe_log_push(0,"[CLI] RX start failed - SDR not found.\n");
                    }
                }
            } else if(line == "/help"){
                bewe_log_push(0,"Commands:\n");
                bewe_log_push(0,"  /status          - Show system status\n");
                bewe_log_push(0,"  /clients         - List connected operators\n");
                bewe_log_push(0,"  /chassis 1 reset - USB SDR hardware reset\n");
                bewe_log_push(0,"  /chassis 2 reset - Network broadcast reset\n");
                bewe_log_push(0,"  /rx stop         - Stop SDR capture\n");
                bewe_log_push(0,"  /rx start        - Restart SDR capture\n");
                bewe_log_push(0,"  /shutdown        - Clean exit\n");
                bewe_log_push(0,"  /help            - Show this help\n");
                bewe_log_push(0,"  <text>           - Broadcast as chat message\n");
                fflush(stdout);
            } else if(!line.empty()){
                // Chat message
                if(v.net_srv)
                    v.net_srv->broadcast_chat(login_get_id(), line.c_str());
                bewe_log_push(0,"[CHAT] %s: %s\n", login_get_id(), line.c_str());
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    //  Cleanup
    // ══════════════════════════════════════════════════════════════════════
    bewe_log_push(0,"[BEWE CLI] Shutting down...\n");

    // 1) Central 쪽을 먼저 완전히 끊어서 auto-reconnect 스레드가 더 이상
    //    mux_adapter를 살리지 못하게 함 (g_shutdown 체크로 reconnect도 자가 종료)
    central_cli.stop_mux_adapter();
    central_cli.stop_polling();

    v.is_running = false;
    if(v.dev_rtl) rtlsdr_cancel_async(v.dev_rtl);
    v.stop_all_dem();
    if(v.rec_on.load()) v.stop_rec();
    if(v.tm_iq_file_ready){
        v.tm_iq_on.store(false);
        v.tm_iq_close();
    }
    v.mix_stop.store(true); if(v.mix_thr.joinable()) v.mix_thr.join();
    v.net_bcast_stop.store(true);
    v.net_bcast_cv.notify_all();
    if(v.net_bcast_thr.joinable()) v.net_bcast_thr.join();
    if(v.net_srv){ v.net_srv->stop(); delete v.net_srv; v.net_srv=nullptr; }
    if(!v.remote_mode && cap.joinable()) cap.join();
    if(v.dev_blade){
        bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(v.dev_blade); v.dev_blade=nullptr;
    }
    if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
    v.sa_cleanup();
    v.eid_cleanup();
    v.ais_pipe_stop();

    // record/ > private/ 이동: .wav + 동명의 .info 동반 이동
    auto move_dir = [](const std::string& src_dir, const std::string& dst_dir){
        DIR* d = opendir(src_dir.c_str());
        if(!d) return;
        struct dirent* ent;
        while((ent=readdir(d))!=nullptr){
            const char* n = ent->d_name;
            size_t nl = strlen(n);
            if(nl>4 && strcmp(n+nl-4,".wav")==0){
                std::string src = src_dir+"/"+n;
                std::string dst = dst_dir+"/"+n;
                rename(src.c_str(), dst.c_str());
                // .info 동반 이동 (있을 때만)
                std::string isrc = src + ".info";
                std::string idst = dst + ".info";
                if(access(isrc.c_str(), F_OK)==0)
                    rename(isrc.c_str(), idst.c_str());
            }
        }
        closedir(d);
    };
    move_dir(BEWEPaths::record_iq_dir(),    BEWEPaths::private_iq_dir());
    move_dir(BEWEPaths::record_audio_dir(), BEWEPaths::private_audio_dir());

    bewe_log_push(0,"[BEWE CLI] Stopped.\n");
}
