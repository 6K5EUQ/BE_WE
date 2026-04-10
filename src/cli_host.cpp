// ── BE_WE CLI Headless HOST Mode ─────────────────────────────────────────
// 라즈베리파이5 등 디스플레이 없는 환경에서 HOST 모드 전용 실행
// GLFW/OpenGL/ImGui 의존성 없음

#include "fft_viewer.hpp"
#include "login.hpp"
#include "bewe_paths.hpp"
#include "central_client.hpp"
#include "net_protocol.hpp"

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
                std::nth_element(tmp, tmp + 12, tmp + 60);
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
    if(!fgets(buf,sizeof(buf),stdin)) return def ? def : "";
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
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // ── Interactive prompts ──────────────────────────────────────────────
    bewe_log_push(0,"\n=== BE_WE Headless HOST ===\n\n");

    std::string id_str = prompt_input("ID");
    if(id_str.empty()){ bewe_log_push(0,"Aborted.\n"); return; }

    // 패스워드 에코 숨기기
    bewe_log_push(0,"Password: "); fflush(stdout);
    struct termios old_t, new_t;
    tcgetattr(STDIN_FILENO,&old_t); new_t=old_t;
    new_t.c_lflag &= ~(ECHO);
    tcsetattr(STDIN_FILENO,TCSANOW,&new_t);
    char pw_buf[64]={};
    if(!fgets(pw_buf,sizeof(pw_buf),stdin)){ bewe_log_push(0,"\nAborted.\n"); tcsetattr(STDIN_FILENO,TCSANOW,&old_t); return; }
    tcsetattr(STDIN_FILENO,TCSANOW,&old_t);
    pw_buf[strcspn(pw_buf,"\r\n")]=0;
    bewe_log_push(0,"\n");

    int tier = atoi(prompt_input("Tier (1/2)", "1").c_str());
    if(tier<1||tier>2){ bewe_log_push(0,"CLI HOST requires tier 1 or 2.\n"); return; }

    std::string server_str = prompt_input("Central server", "144.24.86.137");
    std::string station_str = prompt_input("Station name");
    if(station_str.empty()){ bewe_log_push(0,"Aborted.\n"); return; }

    float lat = atof(prompt_input("Latitude",  "0.0").c_str());
    float lon = atof(prompt_input("Longitude", "0.0").c_str());
    float cf  = atof(prompt_input("Center freq MHz", "450.0").c_str());

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
               v.hw.type==HWType::BLADERF ? "BladeRF" : "RTL-SDR");
        if(v.hw.type == HWType::BLADERF)
            cap = std::thread(&FFTViewer::capture_and_process, &v);
        else
            cap = std::thread(&FFTViewer::capture_and_process_rtl, &v);
    }
    v.mix_stop.store(false);
    v.mix_thr = std::thread(&FFTViewer::mix_worker, &v);

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
        v.stop_dem(idx); v.stop_digi(idx);
        v.channels[idx].reset_slot();
        v.channels[idx].s=s; v.channels[idx].e=e;
        v.channels[idx].filter_active=true;
        strncpy(v.channels[idx].owner, creator?creator:"", 31);
        v.channels[idx].audio_mask.store(0xFFFFFFFFu & ~0x1u);
        v.local_ch_out[idx] = 3;
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

    // Region IQ request from JOIN
    srv->cb.on_request_region = [&](uint8_t op_idx, const char* op_name,
                                     int32_t fft_top, int32_t fft_bot,
                                     float freq_lo, float freq_hi,
                                     int64_t time_start_ms, int64_t time_end_ms){
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
        std::thread([&v,srv,fl,fh,time_start_ms,time_end_ms,oidx,fname,sid,&central_cli,req_id_val](){
            uint32_t req_id = req_id_val;
            for(int w=0;w<200&&v.rec_busy_flag.load();w++)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            v.region.fft_top=0; v.region.fft_bot=0; // 사용 안 함 (time 기반)
            v.region.freq_lo=fl; v.region.freq_hi=fh;
            v.region.time_start_ms=time_start_ms;
            v.region.time_end_ms=time_end_ms;
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
            // META: 파일 열기
            std::string db_dir = BEWEPaths::database_dir() + "/" + std::string(meta->operator_name);
            mkdir(BEWEPaths::database_dir().c_str(), 0755);
            mkdir(db_dir.c_str(), 0755);
            std::string dst = db_dir + "/" + std::string(meta->filename);
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
                    if(len < 9 + 60*10) return;
                    const uint8_t* payload = pkt + 9;
                    for(int i=0; i<MAX_CHANNELS && i<10; i++){
                        uint32_t mask;
                        memcpy(&mask, payload + i*60 + 12, sizeof(mask));
                        v.channels[i].audio_mask.store(mask);
                    }
                });
                // Central DB 목록 수신
                extern std::vector<DbFileEntry> g_db_list;
                extern std::mutex g_db_list_mtx;
                central_cli.set_on_central_db_list([](const uint8_t* pkt, size_t len){
                    extern std::vector<DbFileEntry> g_db_list;
                    extern std::mutex g_db_list_mtx;
                    if(len < 9 + sizeof(PktDbList)) return;
                    const uint8_t* payload = pkt + 9;
                    auto* hdr2 = reinterpret_cast<const PktDbList*>(payload);
                    uint16_t cnt2 = hdr2->count;
                    size_t expected = sizeof(PktDbList) + cnt2 * sizeof(DbFileEntry);
                    if(len - 9 < expected) return;
                    const DbFileEntry* ent = reinterpret_cast<const DbFileEntry*>(payload + sizeof(PktDbList));
                    { std::lock_guard<std::mutex> lk(g_db_list_mtx);
                      g_db_list.assign(ent, ent + cnt2); }
                    bewe_log_push(0,"[Central] DB_LIST: %u files\n", cnt2);
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
                            std::this_thread::sleep_for(std::chrono::seconds(3));
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
        // ~10Hz loop (100ms sleep)
        auto now = clk::now();
        float dt = std::chrono::duration<float>(now - loop_last).count();
        loop_last = now;
        int sleep_ms = 100 - (int)(dt * 1000);
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

        // ── Channel sync broadcast (100ms) ───────────────────────────────
        if(v.net_srv && v.net_srv->client_count()>0){
            float el = std::chrono::duration<float>(clk::now()-sq_sync_last).count();
            if(el >= 0.1f){
                sq_sync_last = clk::now();
                v.update_channel_squelch();
                v.net_srv->broadcast_channel_sync(v.channels, MAX_CHANNELS);
            }
        }

        // ── STATUS broadcast (1s) ────────────────────────────────────────
        if(v.net_srv && v.net_srv->client_count()>0){
            float el = std::chrono::duration<float>(clk::now()-status_last).count();
            if(el >= 1.0f){
                status_last = clk::now();
                v.net_srv->broadcast_status(
                    (float)(v.header.center_frequency/1e6),
                    v.gain_db, v.header.sample_rate,
                    (v.hw.type==HWType::RTLSDR)?1:0);
            }
        }

        // ── Heartbeat (3s) ───────────────────────────────────────────────
        if(v.net_srv){
            float el = std::chrono::duration<float>(clk::now()-heartbeat_last).count();
            bool cur_sdr_err = v.sdr_stream_error.load();
            if(el >= 3.0f){
                heartbeat_last = clk::now();
                uint8_t sdr_t_hb = 0;
                if(v.dev_blade){
                    float _t = 0.f;
                    if(bladerf_get_rfic_temperature(v.dev_blade, &_t) == 0)
                        sdr_t_hb = (uint8_t)std::min(255.f, std::max(0.f, _t));
                }
                uint8_t hst = v.spectrum_pause.load() ? 2 : 0;
                uint8_t sdr_st = (cur_sdr_err || v.rx_stopped.load()) ? 1 : 0;
                uint8_t iq_st = v.tm_iq_on.load() ? 1 : 0;
                v.net_srv->broadcast_heartbeat(hst, sdr_t_hb, sdr_st, iq_st);
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
                usb_reset_in_progress.store(true);
                if(v.dev_blade){ bladerf_close(v.dev_blade); v.dev_blade=nullptr; }
                std::thread([&usb_reset_in_progress](){
                    bewe_log_push(0,"[CLI] chassis 1 reset: USB reset BladeRF...\n");
                    bladerf_usb_reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
                    usb_reset_in_progress.store(false);
                }).detach();
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
    central_cli.stop_mux_adapter();
    central_cli.stop_polling();
    if(v.net_srv){ v.net_srv->stop(); delete v.net_srv; v.net_srv=nullptr; }
    if(!v.remote_mode && cap.joinable()) cap.join();
    if(v.dev_blade){
        bladerf_enable_module(v.dev_blade, BLADERF_CHANNEL_RX(0), false);
        bladerf_close(v.dev_blade); v.dev_blade=nullptr;
    }
    if(v.dev_rtl){ rtlsdr_close(v.dev_rtl); v.dev_rtl=nullptr; }
    v.sa_cleanup();

    // record/ > private/ 이동
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
            }
        }
        closedir(d);
    };
    move_dir(BEWEPaths::record_iq_dir(),    BEWEPaths::private_iq_dir());
    move_dir(BEWEPaths::record_audio_dir(), BEWEPaths::private_audio_dir());

    bewe_log_push(0,"[BEWE CLI] Stopped.\n");
}
