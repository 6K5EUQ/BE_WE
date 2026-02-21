#include "fft_viewer.hpp"
#include <ctime>
#include <algorithm>
#include <chrono>

void FFTViewer::rec_worker(){
    uint32_t msr=header.sample_rate;
    float off=(rec_cf_mhz-(float)(header.center_frequency/1e6f))*1e6f;
    uint32_t decim=std::max(1u,msr/rec_sr), actual_sr=msr/decim;
    WAVWriter wav;
    if(!wav.open(rec_filename,actual_sr)){ rec_on.store(false); return; }
    printf("REC: %.4f MHz  off=%.0fHz  decim=%u  SR=%u\n",rec_cf_mhz,off,decim,actual_sr);

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
    printf("REC done: %llu frames → %s\n",(unsigned long long)rec_frames.load(),rec_filename.c_str());
}

void FFTViewer::start_rec(){
    if(rec_on.load()) return;
    int fi=selected_ch;
    if(fi<0||!channels[fi].filter_active){
        printf("REC: no active channel selected\n"); return;
    }
    Channel& ch=channels[fi];
    float ss=std::min(ch.s,ch.e), se=std::max(ch.s,ch.e);
    rec_cf_mhz=(ss+se)/2.0f;
    float bw_hz=(se-ss)*1e6f;
    rec_sr=optimal_iq_sr(header.sample_rate,bw_hz);

    time_t t=time(nullptr); struct tm tm2; localtime_r(&t,&tm2);
    char fn[256];
    snprintf(fn,256,"/home/dsa/BE_WE/recordings/iq_%.4fMHz_BW%.0fkHz_%04d%02d%02d_%02d%02d%02d.wav",
             rec_cf_mhz,bw_hz/1000.0f,
             tm2.tm_year+1900,tm2.tm_mon+1,tm2.tm_mday,
             tm2.tm_hour,tm2.tm_min,tm2.tm_sec);
    rec_filename=fn;
    rec_frames.store(0);
    rec_rp.store(ring_wp.load());
    rec_ch=fi;
    rec_stop.store(false); rec_on.store(true);
    rec_t0=std::chrono::steady_clock::now();
    rec_thr=std::thread(&FFTViewer::rec_worker,this);
    printf("REC start ch%d → %s  SR=%u\n",fi,fn,rec_sr);
}

void FFTViewer::stop_rec(){
    if(!rec_on.load()) return;
    rec_stop.store(true);
    if(rec_thr.joinable()) rec_thr.join();
    rec_on.store(false);
}
