#include "fft_viewer.hpp"
#include <ctime>
#include <algorithm>
#include <chrono>

// ── IQ 녹음 워커 ─────────────────────────────────────────────────────────
void FFTViewer::rec_worker(){
    uint32_t msr=header.sample_rate;
    float off=(rec_cf_mhz-(float)(header.center_frequency/1e6f))*1e6f;
    uint32_t decim=std::max(1u,msr/rec_sr), actual_sr=msr/decim;
    WAVWriter wav;
    if(!wav.open(rec_filename,actual_sr)){ rec_on.store(false); return; }
    bewe_log("REC IQ: %.4f MHz  off=%.0fHz  decim=%u  SR=%u\n",rec_cf_mhz,off,decim,actual_sr);

    Oscillator osc; osc.set_freq((double)off,(double)msr);
    double ai=0,aq=0; int cnt=0;
    auto c16=[](float v)->int16_t{
        return (int16_t)(std::max(-1.0f,std::min(1.0f,v))*32767.0f);
    };

    while(!rec_stop.load(std::memory_order_relaxed)){
        size_t wp=ring_wp.load(std::memory_order_acquire);
        size_t rp=rec_rp.load(std::memory_order_relaxed);
        if(rp==wp){ std::this_thread::sleep_for(std::chrono::microseconds(100)); continue; }
        size_t avail=std::min((wp-rp)&IQ_RING_MASK,(size_t)65536);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/2048.0f, sq=ring[pos*2+1]/2048.0f;
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

    // RecEntry 완료 표시
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        for(auto& e : rec_entries)
            if(e.path==rec_filename){ e.finished=true; break; }
    }
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

    time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
    char fn[256];
    std::string rec_dir=BEWEPaths::recordings_dir();
    char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
    snprintf(fn,256,"%s/IQ_%.3fMHz_%s.wav",rec_dir.c_str(),
             rec_cf_mhz, dts);
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

    time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
    char fn[256];
    std::string rec_dir=BEWEPaths::recordings_dir();
    float cf_mhz=(ch.s+ch.e)/2.0f;
    float bw_khz=bw_hz/1000.0f;
    char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
    snprintf(fn,256,"%s/Audio_%.3fMHz_%s.wav",
             rec_dir.c_str(), cf_mhz, dts);

    FILE* fp=fopen(fn,"wb");
    if(!fp){ bewe_log("Audio REC: cannot open %s\n",fn); return; }
    ch.audio_rec_frames=0;
    ch.audio_rec_write_wav_hdr(fp,asr,0); // 임시 헤더
    ch.audio_rec_fp=fp;
    ch.audio_rec_path=fn;
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
        e.t_start=std::chrono::steady_clock::now();
        rec_entries.push_back(e);
    }

    bewe_log("Audio REC start ch%d → %s  SR=%u\n",ch_idx,fn,asr);
}

void FFTViewer::stop_audio_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(!ch.audio_rec_on.load()) return;

    ch.audio_rec_on.store(false,std::memory_order_release);
    // demod 스레드가 maybe_rec_audio 완료 대기 (짧게)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    FILE* fp=ch.audio_rec_fp;
    ch.audio_rec_fp=nullptr;
    if(fp){
        // 헤더 재작성
        fseek(fp,0,SEEK_SET);
        ch.audio_rec_write_wav_hdr(fp,ch.audio_rec_sr,ch.audio_rec_frames);
        fclose(fp);
    }
    bewe_log("Audio REC done: %llu frames → %s\n",
             (unsigned long long)ch.audio_rec_frames, ch.audio_rec_path.c_str());

    // RecEntry 완료 표시
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        for(auto& e : rec_entries)
            if(e.path==ch.audio_rec_path){ e.finished=true; break; }
    }
    ch.audio_rec_path.clear();
}

// ── JOIN 모드 로컬 오디오 녹음 (mix_worker에서 채널 오디오를 WAV에 씀) ──────
void FFTViewer::start_join_audio_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(ch.audio_rec_on.load()) return;

    uint32_t asr = AUDIO_SR;
    ch.audio_rec_sr = asr;

    time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
    char fn[256];
    std::string rec_dir=BEWEPaths::recordings_dir();
    float cf_mhz=(ch.s+ch.e)/2.0f;
    char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",&tm2);
    snprintf(fn,256,"%s/Audio_%.3fMHz_%s.wav",rec_dir.c_str(),cf_mhz,dts);

    FILE* fp=fopen(fn,"wb");
    if(!fp){ bewe_log("JOIN Audio REC: cannot open %s\n",fn); return; }
    ch.audio_rec_frames=0;
    ch.audio_rec_write_wav_hdr(fp,asr,0);
    ch.audio_rec_fp=fp;
    ch.audio_rec_path=fn;
    ch.audio_rec_on.store(true,std::memory_order_release);

    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        RecEntry e;
        e.path=fn;
        std::string s(fn); auto pos=s.rfind('/');
        e.filename=(pos==std::string::npos)?s:s.substr(pos+1);
        e.finished=false; e.is_audio=true; e.is_region=false;
        e.t_start=std::chrono::steady_clock::now();
        rec_entries.push_back(e);
    }
    bewe_log("JOIN Audio REC start ch%d → %s  SR=%u\n",ch_idx,fn,asr);
}

void FFTViewer::stop_join_audio_rec(int ch_idx){
    if(ch_idx<0||ch_idx>=MAX_CHANNELS) return;
    Channel& ch=channels[ch_idx];
    if(!ch.audio_rec_on.load()) return;

    ch.audio_rec_on.store(false,std::memory_order_release);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // mix_worker 완료 대기

    FILE* fp=ch.audio_rec_fp;
    ch.audio_rec_fp=nullptr;
    if(fp){
        fseek(fp,0,SEEK_SET);
        ch.audio_rec_write_wav_hdr(fp,ch.audio_rec_sr,ch.audio_rec_frames);
        fclose(fp);
    }
    bewe_log("JOIN Audio REC done: %llu frames → %s\n",
             (unsigned long long)ch.audio_rec_frames, ch.audio_rec_path.c_str());
    {
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        for(auto& e : rec_entries)
            if(e.path==ch.audio_rec_path){ e.finished=true; break; }
    }
    ch.audio_rec_path.clear();
}