#include "fft_viewer.hpp"
#include "login.hpp"
#include "long_waterfall.hpp"
#include "mission_push.hpp"
#include "kst_time.hpp"
#include "sigmf.hpp"
#include <ctime>
#include <algorithm>
#include <chrono>
#include <unistd.h>
#include <sys/stat.h>
#include <cmath>

// 모든 시간은 KST(UTC+9) 기준.
static void fmt_time_hms(char* out, size_t sz, const struct tm& tm_loc){
    strftime(out, sz, "%H:%M:%S", &tm_loc);
}

// 파일명용 스테이션 이름 정제: 영숫자·-·_·. 이외 → '_'
static std::string sanitize_station_fn(const char* sn){
    std::string s = (sn && sn[0]) ? sn : "host";
    for(auto& c : s){
        unsigned char u = (unsigned char)c;
        bool ok = (u>='0'&&u<='9') || (u>='A'&&u<='Z') || (u>='a'&&u<='z')
               || c=='-' || c=='_' || c=='.';
        if(!ok) c = '_';
    }
    return s;
}

// 새 파일명 형식: <station>_<prefix>_<mission_code>_<YYYY>_<freq>MHz_<HHMMSS>.wav
// prefix = "IQ" / "DE", time = KST. stop 시 add_end_hms_to_path() 로 -<HHMMSS> 추가.
static std::string build_iq_demod_filename(FFTViewer& v, const char* prefix,
                                           double cf_mhz, const struct tm& kst_tm){
    char mcode[16] = {};
    char mstation[64] = {};
    {
        std::lock_guard<std::mutex> lk(v.mission_mtx);
        if(v.mission_state == Mission::State::ACTIVE){
            strncpy(mcode, v.mission_code, sizeof(mcode) - 1);
            strncpy(mstation, v.mission_station_name, sizeof(mstation) - 1);
        }
    }
    // IQ 원본은 SigMF (.sigmf-data), 복조 음성(DE)은 표준 .wav
    const char* ext = (prefix && strcmp(prefix, "IQ") == 0) ? ".sigmf-data" : ".wav";
    char buf[256];
    if(mcode[0]){
        // 미션 활성: <station>_<prefix>_<code>_<YYYY>_<freq>MHz_<HHMMSS>
        std::string st = sanitize_station_fn(mstation[0] ? mstation : nullptr);
        char hms[16]; strftime(hms, sizeof(hms), "%H%M%S", &kst_tm);
        int year = 1900 + kst_tm.tm_year;
        snprintf(buf, sizeof(buf), "%s_%s_%s_%04d_%.3fMHz_%s%s",
                 st.c_str(), prefix, mcode, year, cf_mhz, hms, ext);
    } else {
        // 노미션: region IQ 와 동일한 간결 포맷 (station/코드 없음).
        //   <prefix>_<freq>MHz_<MonDD_YYYY_HHMMSS><ext>  (stop 시 -<endHMS> 추가됨)
        char dts[32]; strftime(dts, sizeof(dts), "%b%d_%Y_%H%M%S", &kst_tm);
        snprintf(buf, sizeof(buf), "%s_%.3fMHz_%s%s",
                 prefix, cf_mhz, dts, ext);
    }
    return buf;
}

// stop 시점에 path 의 확장자 직전에 -<end_HHMMSS> 삽입한 새 path 반환.
// (.wav / .sigmf-data 모두 처리)
static std::string add_end_hms_to_path(const std::string& path, const struct tm& kst_end){
    char hms[16]; strftime(hms, sizeof(hms), "%H%M%S", &kst_end);
    auto dot = path.find_last_of('.');
    if(dot == std::string::npos) return path;
    return path.substr(0, dot) + "-" + hms + path.substr(dot);
}

// stop 시 src → dst rename + sidecar(.info 또는 .sigmf-meta) 도 함께 이동.
static std::string rename_with_end_hms(const std::string& old_path){
    time_t te = time(nullptr);
    struct tm te_tm; KST::to_tm(te, te_tm);
    std::string new_path = add_end_hms_to_path(old_path, te_tm);
    if(new_path != old_path){
        if(rename(old_path.c_str(), new_path.c_str()) == 0){
            if(SigMF::is_sigmf_data(old_path))
                rename(SigMF::meta_path(old_path).c_str(), SigMF::meta_path(new_path).c_str());
            else
                rename((old_path + ".info").c_str(), (new_path + ".info").c_str());
            return new_path;
        }
    }
    return old_path;
}

// ── 녹음 .info 자동 생성 ─────────────────────────────────────────────────
// source_type 파라미터는 Recorder 필드(장비 이름)로 사용됨
// utc_offset_hours = INT_MIN이면 시스템 TZ 사용
void write_default_info_file(const std::string& wav_path,
                             const char* source_type,
                             double freq_mhz,
                             double bw_khz,
                             double duration_sec,
                             const char* modulation,
                             const char* operator_name,
                             const char* station_name,
                             time_t start_wall_time,
                             int utc_offset_hours,
                             uint32_t sample_rate)
{
    if(start_wall_time <= 0) start_wall_time = time(nullptr);

    // IQ 원본(.sigmf-data): bewe 청크 + .info 대신 SigMF 메타(.sigmf-meta) 작성
    if(SigMF::is_sigmf_data(wav_path)){
        SigMF::Meta m;
        m.sample_rate    = sample_rate;
        m.center_freq_hz = (freq_mhz     > 0) ? (uint64_t)(freq_mhz * 1e6 + 0.5) : 0;
        m.start_unix     = (int64_t)start_wall_time;
        m.bandwidth_hz   = (bw_khz       > 0) ? bw_khz * 1000.0 : 0;
        m.duration_s     = (duration_sec > 0) ? duration_sec    : 0;
        m.modulation     = modulation    ? modulation    : "";
        m.op             = operator_name ? operator_name : "";
        m.station        = station_name  ? station_name  : "";
        m.recorder       = source_type   ? source_type   : "";
        SigMF::write_meta(wav_path, m);
        return;
    }

    std::string info_path = wav_path + ".info";
    if(access(info_path.c_str(), F_OK) == 0) return; // 이미 있으면 보존
    FILE* f = fopen(info_path.c_str(), "w");
    if(!f) return;
    struct tm tm2; KST::to_tm(start_wall_time, tm2);
    (void)utc_offset_hours;  // 호환 위해 남기지만 KST 강제
    char day_buf[64], up_buf[64];
    strftime(day_buf, sizeof(day_buf), "%b %d, %Y", &tm2);
    fmt_time_hms(up_buf, sizeof(up_buf), tm2);

    // 파일명 (stem, 확장자 제외)
    size_t slash = wav_path.find_last_of('/');
    std::string base = (slash==std::string::npos) ? wav_path : wav_path.substr(slash+1);
    size_t dot = base.find_last_of('.');
    std::string stem = (dot==std::string::npos) ? base : base.substr(0, dot);

    fprintf(f, "File Name: %s\n", stem.c_str());
    fprintf(f, "Day: %s\n", day_buf);
    fprintf(f, "Up Time: %s\n", up_buf);
    // Down Time
    if(duration_sec > 0){
        time_t end_wt = start_wall_time + (time_t)(duration_sec + 0.5);
        struct tm tm_end; KST::to_tm(end_wt, tm_end);
        char down_buf[64]; fmt_time_hms(down_buf, sizeof(down_buf), tm_end);
        fprintf(f, "Down Time: %s\n", down_buf);
        fprintf(f, "Duration: %.1f s\n", duration_sec);
    } else {
        fprintf(f, "Down Time: \n");
        fprintf(f, "Duration: \n");
    }
    if(freq_mhz > 0) fprintf(f, "Frequency: %.4f MHz\n", freq_mhz);
    else             fprintf(f, "Frequency: \n");
    fprintf(f, "Target: \n");
    fprintf(f, "Location: %s\n", station_name ? station_name : "");
    fprintf(f, "Modulation: %s\n", modulation ? modulation : "");
    if(bw_khz > 0) fprintf(f, "Bandwidth: %.1f kHz\n", bw_khz);
    else           fprintf(f, "Bandwidth: \n");
    fprintf(f, "Signal Strength: \n");
    fprintf(f, "Protocol: \n");
    fprintf(f, "Recorder: %s\n", source_type ? source_type : "");
    fprintf(f, "Notes: \n");
    fprintf(f, "Tags: \n");
    fprintf(f, "Priority: \n");
    fprintf(f, "Operator: %s\n", operator_name ? operator_name : "");

    fclose(f);
}

// .info 파일의 Duration 및 Down Time 갱신 (다른 필드 보존)
// 녹음 종료 시 호출 > Up Time은 이미 생성 시점에 기록됨
static void update_info_file_duration(const std::string& wav_path, double duration_sec){
    if(SigMF::is_sigmf_data(wav_path)){ SigMF::update_duration(wav_path, duration_sec); return; }
    std::string info_path = wav_path + ".info";
    FILE* f = fopen(info_path.c_str(), "r");
    if(!f) return;
    std::vector<std::string> lines;
    char buf[1024];
    while(fgets(buf, sizeof(buf), f)) lines.emplace_back(buf);
    fclose(f);

    // Up Time 파싱 > Down Time 계산에 사용
    int up_hh=-1, up_mm=-1, up_ss=-1;
    for(auto& l : lines){
        if(l.rfind("Up Time:", 0) == 0){
            sscanf(l.c_str(), "Up Time: %d:%d:%d", &up_hh, &up_mm, &up_ss);
            break;
        }
    }

    bool dur_updated = false, down_updated = false;
    for(auto& l : lines){
        if(!dur_updated && l.rfind("Duration:", 0) == 0){
            std::string body = l.substr(9);
            size_t lead = body.find_first_not_of(" \t");
            std::string trimmed = (lead == std::string::npos) ? "" : body.substr(lead);
            while(!trimmed.empty() && (trimmed.back()=='\n' || trimmed.back()=='\r'))
                trimmed.pop_back();
            if(trimmed.empty()){
                char nl[64];
                snprintf(nl, sizeof(nl), "Duration: %.1f s\n", duration_sec);
                l = nl;
                dur_updated = true;
            }
        } else if(!down_updated && l.rfind("Down Time:", 0) == 0){
            std::string body = l.substr(10);
            size_t lead = body.find_first_not_of(" \t");
            std::string trimmed = (lead == std::string::npos) ? "" : body.substr(lead);
            while(!trimmed.empty() && (trimmed.back()=='\n' || trimmed.back()=='\r'))
                trimmed.pop_back();
            if(trimmed.empty() && up_hh >= 0){
                long total_sec = (long)up_hh*3600 + (long)up_mm*60 + (long)up_ss
                                 + (long)(duration_sec + 0.5);
                int dh = (int)((total_sec / 3600) % 24);
                int dm = (int)((total_sec / 60) % 60);
                int ds = (int)(total_sec % 60);
                char nl[64];
                snprintf(nl, sizeof(nl), "Down Time: %02d:%02d:%02d\n",
                         dh, dm, ds);
                l = nl;
                down_updated = true;
            }
        }
    }
    if(!dur_updated && !down_updated) return;
    f = fopen(info_path.c_str(), "w");
    if(!f) return;
    for(auto& l : lines) fputs(l.c_str(), f);
    fclose(f);
}

// 채널 모드 enum → 문자열
static const char* dem_mode_name(Channel::DemodMode m){
    switch(m){
        case Channel::DM_AM:    return "AM";
        case Channel::DM_FM:    return "FM";
        default:                return "";
    }
}

// ── IQ 녹음 워커 ─────────────────────────────────────────────────────────
void FFTViewer::rec_worker(){
    uint32_t msr=header.sample_rate;
    float off=(rec_cf_mhz-(float)(header.center_frequency/1e6f))*1e6f;
    uint32_t safe_sr=std::max(1u,rec_sr);
    uint32_t decim=std::max(1u,msr/safe_sr), actual_sr=msr/decim;
    WAVWriter wav;
    if(!wav.open(rec_filename,actual_sr)){ rec_on.store(false); return; }
    bewe_log("REC IQ: %.4f MHz  off=%.0fHz  decim=%u  SR=%u\n",rec_cf_mhz,off,decim,actual_sr);

    Oscillator osc; osc.set_freq((double)off,(double)msr);
    double ai=0,aq=0; int cnt=0;
    auto c16=[](float v)->int16_t{
        return (int16_t)(std::max(-1.0f,std::min(1.0f,v))*32767.0f);
    };

    while(!rec_stop.load(std::memory_order_relaxed) && !sdr_stream_error.load()){
        size_t wp=ring_wp.load(std::memory_order_acquire);
        size_t rp=rec_rp.load(std::memory_order_relaxed);
        if(rp==wp){ std::this_thread::sleep_for(std::chrono::microseconds(100)); continue; }
        size_t avail=std::min((wp-rp)&IQ_RING_MASK,(size_t)65536);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/hw.iq_scale, sq=ring[pos*2+1]/hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            ai+=mi; aq+=mq; cnt++;
            if(cnt>=(int)decim){
                wav.push(c16((float)(ai/cnt)),c16((float)(aq/cnt)));
                rec_frames.fetch_add(1);
                ai=aq=0; cnt=0;
            }
        }
        rec_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    wav.close();
    bewe_log("REC IQ done: %llu frames → %s\n",(unsigned long long)rec_frames.load(),rec_filename.c_str());

    // .info Duration 갱신
    if(actual_sr > 0)
        update_info_file_duration(rec_filename, (double)rec_frames.load() / (double)actual_sr);

    // stop 시점 KST HHMMSS 를 파일명 끝에 -<HHMMSS> 로 추가 (rename).
    std::string new_path = rename_with_end_hms(rec_filename);

    // RecEntry 완료 표시 (rename 후 경로)
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        for(auto& e : rec_entries)
            if(e.path == rec_filename){
                e.path = new_path;
                std::string s = new_path; auto pos = s.rfind('/');
                e.filename = (pos==std::string::npos) ? s : s.substr(pos+1);
                e.finished = true;
                break;
            }
    }
    rec_filename = new_path;

    // Central archive 로 push (활성 미션 + Central 연결 시에만 실제 전송 — worker 가 검증)
    if(rec_frames.load() > 0)
        MissionPush::enqueue(rec_filename, MFS_IQ);
}

void FFTViewer::start_rec(){
    if(rec_on.load()) return;
    int fi=selected_ch;
    if(fi<0||!channels[fi].filter_active){
        bewe_log("REC: no active channel selected\n"); return;
    }
    Channel& ch=channels[fi];
    float ss=std::min(ch.s,ch.e), se=std::max(ch.s,ch.e);
    rec_cf_mhz=(ss+se)/2.0f;
    float bw_hz=(se-ss)*1e6f;
    rec_sr=optimal_iq_sr(header.sample_rate,bw_hz);

    time_t t=time(nullptr); struct tm tm2; KST::to_tm(t,tm2);
    char fn[512];
    std::string rec_dir=active_iq_dir();
    if(rec_dir.empty()){
        bewe_log_push(1, "[REC] blocked - no active mission (Start a mission first)\n");
        MissionView::show_toast("No active mission - Start a mission first (M key)");
        return;
    }
    std::string base = build_iq_demod_filename(*this, "IQ", rec_cf_mhz, tm2);
    snprintf(fn, sizeof(fn), "%s/%s", rec_dir.c_str(), base.c_str());
    rec_filename=fn;
    rec_frames.store(0);
    rec_rp.store(ring_wp.load());
    rec_ch=fi;
    rec_stop.store(false); rec_on.store(true);
    rec_t0=std::chrono::steady_clock::now();

    // RecEntry 추가
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        RecEntry e;
        e.path=fn;
        std::string s(fn);
        auto pos=s.rfind('/');
        e.filename = (pos==std::string::npos)?s:s.substr(pos+1);
        e.finished=false; e.is_audio=false; e.is_region=false;
        e.t_start=std::chrono::steady_clock::now();
        rec_entries.push_back(e);
    }

    rec_thr=std::thread(&FFTViewer::rec_worker,this);
    bewe_log("REC start ch%d → %s  SR=%u\n",fi,fn,rec_sr);

    write_default_info_file(rec_filename, recorder_name(),
                            (double)rec_cf_mhz, (se-ss)*1000.0, 0.0,
                            "", login_get_id(), station_name.c_str(),
                            time(nullptr), utc_offset_hours(), rec_sr);
}

void FFTViewer::stop_rec(){
    if(!rec_on.load()) return;
    rec_stop.store(true);
    if(rec_thr.joinable()) rec_thr.join();
    rec_on.store(false);
}

// ── Audio 녹음 (복조 음성) ────────────────────────────────────────────────
void FFTViewer::start_audio_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(!ch.filter_active||!ch.dem_run.load()){
        bewe_log("Audio REC: ch%d not running demod\n",ch_idx); return;
    }
    if(ch.audio_rec_on.load()) return;

    // 실제 오디오 SR 계산
    float bw_hz=fabsf(ch.e-ch.s)*1e6f;
    uint32_t inter_sr,audio_decim,cap_decim;
    demod_rates(header.sample_rate,bw_hz,inter_sr,audio_decim,cap_decim);
    uint32_t asr=inter_sr/std::max(1u,audio_decim);
    ch.audio_rec_sr=asr;

    time_t t=time(nullptr); struct tm tm2; KST::to_tm(t,tm2);
    char fn[512];
    std::string rec_dir=active_audio_dir();
    if(rec_dir.empty()){
        // 노미션: 비-미션 로컬 폴더(record/audio)에 저장 → 미션창 LOCAL 탭 DEMOD 에 표시.
        rec_dir = BEWEPaths::record_audio_dir();
        mkdir(BEWEPaths::record_dir().c_str(), 0755);
        mkdir(rec_dir.c_str(), 0755);
    }
    float cf_mhz=(ch.s+ch.e)/2.0f;
    {
        std::string base = build_iq_demod_filename(*this, "DE", cf_mhz, tm2);
        snprintf(fn, sizeof(fn), "%s/%s", rec_dir.c_str(), base.c_str());
    }

    FILE* fp=fopen(fn,"wb");
    if(!fp){ bewe_log("Audio REC: cannot open %s\n",fn); return; }
    ch.audio_rec_frames=0;
    ch.audio_rec_write_wav_hdr(fp,asr,0);
    ch.audio_rec_fp=fp;
    ch.audio_rec_path=fn;
    ch.sqr_state = Channel::SQR_IDLE;
    ch.sqr_tail_remain = 0;
    ch.audio_rec_on.store(true,std::memory_order_release);

    // RecEntry 추가
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        RecEntry e;
        e.path=fn;
        std::string s(fn);
        auto pos=s.rfind('/');
        e.filename=(pos==std::string::npos)?s:s.substr(pos+1);
        e.finished=false; e.is_audio=true; e.is_region=false;
        e.ch_idx=ch_idx;
        e.t_start=std::chrono::steady_clock::now();
        rec_entries.push_back(e);
    }

    bewe_log("Audio REC start ch%d → %s  SR=%u\n",ch_idx,fn,asr);

    float bw_khz = fabsf(ch.e - ch.s) * 1000.f;
    write_default_info_file(fn, recorder_name(),
                            (double)cf_mhz, (double)bw_khz, 0.0,
                            dem_mode_name(ch.mode), login_get_id(),
                            station_name.c_str(), time(nullptr),
                            utc_offset_hours());
}

void FFTViewer::stop_audio_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(!ch.audio_rec_on.load()) return;

    ch.audio_rec_on.store(false,std::memory_order_release);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    FILE* fp=ch.audio_rec_fp;
    ch.audio_rec_fp=nullptr;
    if(fp){
        fseek(fp,0,SEEK_SET);
        ch.audio_rec_write_wav_hdr(fp,ch.audio_rec_sr,ch.audio_rec_frames);
        fclose(fp);
    }

    // 실제 녹음 출력이 없으면 파일 삭제 + RecEntry 제거
    if(ch.audio_rec_frames==0){
        remove(ch.audio_rec_path.c_str());
        remove((ch.audio_rec_path + ".info").c_str());
        bewe_log("Audio REC empty, deleted: %s\n", ch.audio_rec_path.c_str());
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        rec_entries.erase(std::remove_if(rec_entries.begin(),rec_entries.end(),
            [&](const RecEntry& e){return e.path==ch.audio_rec_path;}),rec_entries.end());
        ch.audio_rec_path.clear();
        return;
    }

    bewe_log("Audio REC done: %llu frames → %s\n",
             (unsigned long long)ch.audio_rec_frames, ch.audio_rec_path.c_str());

    if(ch.audio_rec_sr > 0)
        update_info_file_duration(ch.audio_rec_path,
                                  (double)ch.audio_rec_frames / (double)ch.audio_rec_sr);

    // stop 시점 KST HHMMSS rename
    std::string new_path = rename_with_end_hms(ch.audio_rec_path);
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        for(auto& e : rec_entries)
            if(e.path == ch.audio_rec_path){
                e.path = new_path;
                std::string s = new_path; auto pos = s.rfind('/');
                e.filename = (pos==std::string::npos) ? s : s.substr(pos+1);
                e.finished = true;
                break;
            }
    }
    ch.audio_rec_path = new_path;
    MissionPush::enqueue(ch.audio_rec_path, MFS_AUDIO);
    ch.audio_rec_path.clear();
}

// ── IQ-only worker (demod 우회 — 채널 BW에 정확히 맞춰 mixer + LPF cascade + decim) ─
// ch.dem_run이 켜져 있으면 demod path가 maybe_rec_iq를 호출하므로 이 worker는 시작 안 함.
// 채널 만들고 demod 없이 바로 IQ 녹음 시작 시 이 worker가 IQ ring을 직접 소비.
void FFTViewer::iq_only_worker(int ch_idx){
    Channel& ch = channels[ch_idx];
    uint32_t msr = header.sample_rate;
    uint64_t init_cf = live_cf_hz.load(std::memory_order_acquire);
    float ch_cf_mhz = (ch.s + ch.e) * 0.5f;
    float bw_hz = fabsf(ch.e - ch.s) * 1e6f;
    float off_hz = (ch_cf_mhz - (float)(init_cf / 1e6f)) * 1e6f;

    // 적극 decim: target sr ≈ BW × 1.25 (Nyquist + 25% margin), ceil로 BW에 가깝게
    float target_sr = bw_hz * 1.25f;
    if(target_sr < 8000.f) target_sr = 8000.f;
    uint32_t decim = (uint32_t)ceilf((float)msr / target_sr);
    if(decim < 1) decim = 1;
    uint32_t actual_sr = msr / decim;
    ch.iq_rec_sr = actual_sr;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;

    // BW LPF cascade (4-stage IIR1) — pre-decim 단계에서 anti-alias
    float cn = (bw_hz * 0.5f) / (float)msr;
    if(cn > 0.45f) cn = 0.45f;
    IIR1 lpi[4], lpq[4];
    for(int k=0; k<4; k++){ lpi[k].set(cn); lpq[k].set(cn); }

    double acc_i=0, acc_q=0; int acc_cnt=0;
    const size_t MAX_LAG = (size_t)(msr * 0.08);
    const size_t BATCH   = std::max((size_t)4096, (size_t)decim * 256);

    while(!ch.iq_only_stop_req.load(std::memory_order_relaxed) && !sdr_stream_error.load()){
        // CF 변경 감지
        uint64_t cur_cf = live_cf_hz.load(std::memory_order_acquire);
        if(cur_cf != prev_cf){
            off_hz = (ch_cf_mhz - (float)(cur_cf / 1e6f)) * 1e6f;
            osc.set_freq((double)off_hz, (double)msr);
            prev_cf = cur_cf;
        }
        size_t wp = ring_wp.load(std::memory_order_acquire);
        size_t rp = ch.iq_only_rp.load(std::memory_order_relaxed);
        size_t lag = (wp - rp) & IQ_RING_MASK;
        if(lag > MAX_LAG){
            size_t keep = (size_t)(msr * 0.02);
            rp = (wp - keep) & IQ_RING_MASK;
            ch.iq_only_rp.store(rp, std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            acc_i=acc_q=0; acc_cnt=0;
            lag = (wp - rp) & IQ_RING_MASK;
        }
        if(lag == 0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail = std::min(lag, BATCH);
        for(size_t s=0; s<avail; s++){
            size_t pos = (rp + s) & IQ_RING_MASK;
            float si = ring[pos*2]   / hw.iq_scale;
            float sq = ring[pos*2+1] / hw.iq_scale;
            float mi, mq; osc.mix(si, sq, mi, mq);
            // 4-stage LPF cascade
            mi = lpi[0].p(mi); mi = lpi[1].p(mi); mi = lpi[2].p(mi); mi = lpi[3].p(mi);
            mq = lpq[0].p(mq); mq = lpq[1].p(mq); mq = lpq[2].p(mq); mq = lpq[3].p(mq);
            acc_i += mi; acc_q += mq; acc_cnt++;
            if(acc_cnt < (int)decim) continue;
            float fi = (float)(acc_i / acc_cnt);
            float fq = (float)(acc_q / acc_cnt);
            acc_i = acc_q = 0; acc_cnt = 0;
            bool gate_open = ch.sq_gate.load(std::memory_order_relaxed);
            ch.maybe_rec_iq(fi, fq, gate_open);
        }
        ch.iq_only_rp.store((rp + avail) & IQ_RING_MASK, std::memory_order_release);
    }
    ch.iq_only_run.store(false, std::memory_order_release);
}

// ── Per-channel IQ recording (squelch-gated, decimated baseband) ──────────
void FFTViewer::start_iq_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(!ch.filter_active){
        bewe_log("IQ REC: ch%d not active\n",ch_idx); return;
    }
    if(ch.iq_rec_on.load()) return;

    // demod 켜져 있으면 demod path가 maybe_rec_iq 호출 (호환). 없으면 iq_only_worker 시작.
    bool use_iq_only = !ch.dem_run.load();

    float bw_hz=fabsf(ch.e-ch.s)*1e6f;
    uint32_t actual_inter;
    if(use_iq_only){
        // iq_only_worker가 자체적으로 actual_sr 계산해서 ch.iq_rec_sr 설정.
        // wav 헤더 작성을 위해 동일 식으로 미리 계산.
        float target_sr = bw_hz * 1.25f;
        if(target_sr < 8000.f) target_sr = 8000.f;
        uint32_t decim = (uint32_t)ceilf((float)header.sample_rate / target_sr);
        if(decim < 1) decim = 1;
        actual_inter = header.sample_rate / decim;
    } else {
        uint32_t inter_sr,audio_decim,cap_decim;
        demod_rates(header.sample_rate,bw_hz,inter_sr,audio_decim,cap_decim);
        actual_inter=header.sample_rate/cap_decim;
    }
    ch.iq_rec_sr=actual_inter;

    time_t t=time(nullptr); struct tm tm2; KST::to_tm(t,tm2);
    char fn[512];
    std::string rec_dir=active_iq_dir();
    if(rec_dir.empty()){
        bewe_log_push(1, "[IQ REC ch%d] blocked - no active mission (Start a mission first)\n", ch_idx);
        MissionView::show_toast("No active mission - Start a mission first (M key)");
        return;
    }
    float cf_mhz=(ch.s+ch.e)/2.0f;
    if(pending_sched_meta.active){
        // SCHED 형식: SCHED_IQ_<station>_<MissCode><DD>_<MonDD>.<YYYY>_<HHMMSS>-<HHMMSS>_<F.F>MHz.wav
        // "IQ" 포함시켜 분류 헬퍼(is_iq_filename)가 IQ로 인식하도록.
        // 시간은 모두 KST 기준. date code는 HIST와 동일 ('A'=Jan…'L'=Dec).
        // station 이름은 파일시스템 안전 문자만 남김 (공백/괄호/슬래시 → '_').
        std::string st = station_name;
        for(auto& c : st){
            unsigned char u = (unsigned char)c;
            bool ok = (u>='0'&&u<='9') || (u>='A'&&u<='Z') || (u>='a'&&u<='z')
                   || c=='-' || c=='_' || c=='.';
            if(!ok) c = '_';
        }
        if(st.empty()) st = "host";
        struct tm su; KST::to_tm(pending_sched_meta.start_utc, su);
        struct tm eu; KST::to_tm(pending_sched_meta.end_utc,   eu);
        snprintf(fn,sizeof(fn),
                 "%s/SCHED_IQ_%s_%c%02d_%s%d.%04d_%02d%02d%02d-%02d%02d%02d_%.1fMHz.sigmf-data",
                 rec_dir.c_str(), st.c_str(),
                 LongWaterfall::mission_letter(su.tm_mon), su.tm_mday,
                 LongWaterfall::month_abbr3(su.tm_mon),    su.tm_mday, 1900+su.tm_year,
                 su.tm_hour, su.tm_min, su.tm_sec,
                 eu.tm_hour, eu.tm_min, eu.tm_sec,
                 cf_mhz);
        pending_sched_meta.active = false;
    } else {
        std::string base = build_iq_demod_filename(*this, "IQ", cf_mhz, tm2);
        snprintf(fn, sizeof(fn), "%s/%s", rec_dir.c_str(), base.c_str());
    }

    FILE* fp=fopen(fn,"wb");
    if(!fp){ bewe_log("IQ REC: cannot open %s\n",fn); return; }
    ch.iq_rec_frames=0;
    ch.iq_rec_cf_hz=(uint64_t)(cf_mhz*1e6);
    ch.iq_rec_start_time=(int64_t)t;
    // raw IQ(.sigmf-data): 헤더 없이 데이터부터 기록. 메타는 아래 .sigmf-meta.

    ch.iq_rec_fp=fp;
    ch.iq_rec_path=fn;
    ch.iq_sqr_state=Channel::SQR_IDLE;
    ch.iq_sqr_tail_remain=0;
    ch.iq_rec_on.store(true,std::memory_order_release);

    // demod path 없으면 IQ-only worker 시작
    if(use_iq_only){
        if(ch.iq_only_thr.joinable()) ch.iq_only_thr.join();
        ch.iq_only_stop_req.store(false, std::memory_order_release);
        ch.iq_only_run.store(true, std::memory_order_release);
        ch.iq_only_rp.store(ring_wp.load(std::memory_order_acquire), std::memory_order_release);
        ch.iq_only_thr = std::thread(&FFTViewer::iq_only_worker, this, ch_idx);
    }

    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        RecEntry e;
        e.path=fn;
        std::string s(fn); auto pos=s.rfind('/');
        e.filename=(pos==std::string::npos)?s:s.substr(pos+1);
        e.finished=false; e.is_audio=false; e.is_region=false;
        e.ch_idx=ch_idx;
        e.t_start=std::chrono::steady_clock::now();
        rec_entries.push_back(e);
    }
    bewe_log("IQ REC start ch%d > %s  SR=%u  (mode=%s)\n",
             ch_idx,fn,actual_inter, use_iq_only ? "IQ-only" : "demod-piggyback");

    float bw_khz_iq = fabsf(ch.e - ch.s) * 1000.f;
    write_default_info_file(fn, recorder_name(),
                            (double)cf_mhz, (double)bw_khz_iq, 0.0,
                            dem_mode_name(ch.mode), login_get_id(),
                            station_name.c_str(), time(nullptr),
                            utc_offset_hours(), actual_inter);
}

void FFTViewer::stop_iq_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(!ch.iq_rec_on.load()) return;

    ch.iq_rec_on.store(false,std::memory_order_release);
    // IQ-only worker stop & join (demod path 사용 중이면 no-op)
    if(ch.iq_only_run.load()){
        ch.iq_only_stop_req.store(true, std::memory_order_release);
        if(ch.iq_only_thr.joinable()) ch.iq_only_thr.join();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    FILE* fp=ch.iq_rec_fp;
    ch.iq_rec_fp=nullptr;
    if(fp) fclose(fp);   // raw .sigmf-data — 헤더 재작성 불필요

    if(ch.iq_rec_frames==0){
        remove(ch.iq_rec_path.c_str());
        remove(SigMF::meta_path(ch.iq_rec_path).c_str());
        bewe_log("IQ REC empty, deleted: %s\n",ch.iq_rec_path.c_str());
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        rec_entries.erase(std::remove_if(rec_entries.begin(),rec_entries.end(),
            [&](const RecEntry& e){return e.path==ch.iq_rec_path;}),rec_entries.end());
        ch.iq_rec_path.clear();
        return;
    }

    bewe_log("IQ REC done: %llu frames > %s\n",
             (unsigned long long)ch.iq_rec_frames, ch.iq_rec_path.c_str());

    if(ch.iq_rec_sr > 0)
        update_info_file_duration(ch.iq_rec_path,
                                  (double)ch.iq_rec_frames / (double)ch.iq_rec_sr);
    // stop 시점 KST HHMMSS rename
    std::string new_path = rename_with_end_hms(ch.iq_rec_path);
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        for(auto& e : rec_entries)
            if(e.path == ch.iq_rec_path){
                e.path = new_path;
                std::string s = new_path; auto pos = s.rfind('/');
                e.filename = (pos==std::string::npos) ? s : s.substr(pos+1);
                e.finished = true;
                break;
            }
    }
    ch.iq_rec_path = new_path;
    MissionPush::enqueue(ch.iq_rec_path, MFS_IQ);
    ch.iq_rec_path.clear();
}

// ── JOIN 모드 로컬 오디오 녹음 (mix_worker에서 채널 오디오를 WAV에 씀) ──────
void FFTViewer::start_join_audio_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(ch.audio_rec_on.load()) return;

    uint32_t asr = AUDIO_SR;
    ch.audio_rec_sr = asr;

    time_t t=time(nullptr); struct tm tm2; KST::to_tm(t,tm2);
    char fn[512];
    std::string rec_dir=active_audio_dir();
    if(rec_dir.empty()){
        // 노미션: 비-미션 로컬 폴더(record/audio)에 저장 → 미션창 LOCAL 탭 DEMOD 에 표시.
        rec_dir = BEWEPaths::record_audio_dir();
        mkdir(BEWEPaths::record_dir().c_str(), 0755);
        mkdir(rec_dir.c_str(), 0755);
    }
    float cf_mhz=(ch.s+ch.e)/2.0f;
    {
        std::string base = build_iq_demod_filename(*this, "DE", cf_mhz, tm2);
        snprintf(fn, sizeof(fn), "%s/%s", rec_dir.c_str(), base.c_str());
    }

    FILE* fp=fopen(fn,"wb");
    if(!fp){ bewe_log("JOIN Audio REC: cannot open %s\n",fn); return; }
    ch.audio_rec_frames=0;
    ch.audio_rec_write_wav_hdr(fp,asr,0);
    ch.audio_rec_fp=fp;
    ch.audio_rec_path=fn;
    ch.sqr_state = Channel::SQR_IDLE;
    ch.sqr_tail_remain = 0;
    ch.audio_rec_on.store(true,std::memory_order_release);

    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        RecEntry e;
        e.path=fn;
        std::string s(fn); auto pos=s.rfind('/');
        e.filename=(pos==std::string::npos)?s:s.substr(pos+1);
        e.finished=false; e.is_audio=true; e.is_region=false;
        e.ch_idx=ch_idx;
        e.t_start=std::chrono::steady_clock::now();
        rec_entries.push_back(e);
    }
    bewe_log("JOIN Audio REC start ch%d → %s  SR=%u\n",ch_idx,fn,asr);

    float bw_khz_jaud = fabsf(ch.e - ch.s) * 1000.f;
    write_default_info_file(fn, recorder_name(),
                            (double)cf_mhz, (double)bw_khz_jaud, 0.0,
                            dem_mode_name(ch.mode), login_get_id(),
                            station_name.c_str(), time(nullptr),
                            utc_offset_hours());
}

void FFTViewer::stop_join_audio_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(!ch.audio_rec_on.load()) return;

    ch.audio_rec_on.store(false,std::memory_order_release);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    FILE* fp=ch.audio_rec_fp;
    ch.audio_rec_fp=nullptr;
    if(fp){
        fseek(fp,0,SEEK_SET);
        ch.audio_rec_write_wav_hdr(fp,ch.audio_rec_sr,ch.audio_rec_frames);
        fclose(fp);
    }

    // 실제 녹음 출력이 없으면 파일 삭제 + RecEntry 제거
    if(ch.audio_rec_frames==0){
        remove(ch.audio_rec_path.c_str());
        remove((ch.audio_rec_path + ".info").c_str());
        bewe_log("JOIN Audio REC empty, deleted: %s\n", ch.audio_rec_path.c_str());
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        rec_entries.erase(std::remove_if(rec_entries.begin(),rec_entries.end(),
            [&](const RecEntry& e){return e.path==ch.audio_rec_path;}),rec_entries.end());
        ch.audio_rec_path.clear();
        return;
    }

    bewe_log("JOIN Audio REC done: %llu frames → %s\n",
             (unsigned long long)ch.audio_rec_frames, ch.audio_rec_path.c_str());

    if(ch.audio_rec_sr > 0)
        update_info_file_duration(ch.audio_rec_path,
                                  (double)ch.audio_rec_frames / (double)ch.audio_rec_sr);
    // stop 시점 KST HHMMSS rename — LOCAL DEMOD 에 보일 최종 파일명
    std::string new_path = rename_with_end_hms(ch.audio_rec_path);
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        for(auto& e : rec_entries)
            if(e.path == ch.audio_rec_path){
                e.path = new_path;
                std::string s = new_path; auto pos = s.rfind('/');
                e.filename = (pos==std::string::npos) ? s : s.substr(pos+1);
                e.finished = true;
                break;
            }
    }
    ch.audio_rec_path = new_path;
    ch.audio_rec_path.clear();
}
