#pragma once
#include "config.hpp"
#include <fftw3.h>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

// ── Oscillator ────────────────────────────────────────────────────────────
struct Oscillator {
    float re=1,im=0,dre=1,dim=0; int cnt=0;
    static constexpr int NORM=4096;
    void set_freq(double freq_hz,double sr){
        double w=-2.0*M_PI*freq_hz/sr;
        dre=(float)cos(w); dim=(float)sin(w); re=1; im=0; cnt=0;
    }
    inline void mix(float si,float sq,float& mi,float& mq){
        mi=si*re-sq*im; mq=si*im+sq*re;
        float nr=re*dre-im*dim, ni=re*dim+im*dre; re=nr; im=ni;
        if(++cnt>=NORM){ float m=1.0f/sqrtf(re*re+im*im+1e-30f); re*=m; im*=m; cnt=0; }
    }
};

// ── 1st-order IIR low-pass filter ─────────────────────────────────────────
struct IIR1 {
    float a=0,b=1,s=0;
    void set(double cn){ a=(float)exp(-2.0*M_PI*cn); b=1-a; }
    inline float p(float x){ s=a*s+b*x; return s; }
};

// ── WAV file writer (stereo int16, for IQ recording) ──────────────────────
struct WAVWriter {
    FILE*    fp=nullptr;
    uint32_t sample_rate=0;
    uint64_t num_samples=0;
    std::vector<int16_t> buf;
    static constexpr size_t BUF_FRAMES=65536;

    bool open(const std::string& fn, uint32_t sr){
        fp=fopen(fn.c_str(),"wb"); if(!fp) return false;
        sample_rate=sr; num_samples=0; buf.reserve(BUF_FRAMES*2); write_hdr(); return true;
    }
    void push(int16_t i,int16_t q){
        buf.push_back(i); buf.push_back(q); ++num_samples;
        if(buf.size()>=BUF_FRAMES*2) flush();
    }
    void flush(){ if(!fp||buf.empty()) return; fwrite(buf.data(),2,buf.size(),fp); buf.clear(); }
    void close(){ flush(); if(!fp) return; fseek(fp,0,SEEK_SET); write_hdr(); fclose(fp); fp=nullptr; }
private:
    void write_hdr(){
        auto w32=[&](uint32_t v){ fwrite(&v,4,1,fp); };
        auto w16=[&](uint16_t v){ fwrite(&v,2,1,fp); };
        uint32_t db=(uint32_t)(num_samples*4);
        fwrite("RIFF",1,4,fp); w32(36+db); fwrite("WAVE",1,4,fp);
        fwrite("fmt ",1,4,fp); w32(16); w16(1); w16(2);
        w32(sample_rate); w32(sample_rate*4); w16(4); w16(16);
        fwrite("data",1,4,fp); w32(db);
    }
};

// ── Per-channel state ─────────────────────────────────────────────────────
struct Channel {
    // Filter geometry (absolute MHz)
    float s=0, e=0;
    bool  filter_active=false;
    bool  selected=false;

    // Demodulation mode
    enum DemodMode{ DM_NONE=0, DM_AM, DM_FM, DM_MAGIC, DM_DMR } mode=DM_NONE;

    // Magic mode: detected modulation (0=analyzing, 1=AM, 2=FM, 3=DSB, 4=SSB, 5=CW)
    std::atomic<int> magic_det{0};
    int   pan=0;   // -1=L  0=both  1=R
    // audio_mask: bit0=host local, bit_i=operator_i gets audio
    std::atomic<uint32_t> audio_mask{0x1};  // default: host only

    // Demod thread
    std::atomic<bool>   dem_run{false};
    std::atomic<bool>   dem_stop_req{false};
    std::thread         dem_thr;
    std::atomic<size_t> dem_rp{0};
    bool                dem_paused=false;         // 주파수 범위 벗어나 자동 pause됨
    DemodMode           dem_paused_mode=DM_NONE;  // pause 직전 mode 보존

    // Per-channel audio ring (float mono)
    static constexpr size_t AR_SZ   = 16384;
    static constexpr size_t AR_MASK = AR_SZ-1;
    float ar[AR_SZ]={};
    std::atomic<size_t> ar_wp{0};
    std::atomic<size_t> ar_rp{0};

    void push_audio(float v){
        size_t w=ar_wp.load(std::memory_order_relaxed);
        ar[w&AR_MASK]=v;
        ar_wp.store(w+1,std::memory_order_release);
    }
    bool pop_audio(float& v){
        size_t r=ar_rp.load(std::memory_order_relaxed);
        size_t w=ar_wp.load(std::memory_order_acquire);
        if(r==w) return false;
        v=ar[r&AR_MASK];
        ar_rp.store(r+1,std::memory_order_release);
        return true;
    }
    size_t audio_avail(){
        return ar_wp.load(std::memory_order_acquire)-ar_rp.load(std::memory_order_relaxed);
    }

    // ── Audio recording (demod 스레드 내에서만 접근) ──────────────────────
    std::atomic<bool> audio_rec_on{false};
    FILE*             audio_rec_fp      = nullptr;
    uint64_t          audio_rec_frames  = 0;
    uint32_t          audio_rec_sr      = 0;
    std::string       audio_rec_path;

    // mono int16 WAV 헤더 기록 (open / close 시 호출)
    void audio_rec_write_wav_hdr(FILE* fp, uint32_t sr, uint64_t frames){
        auto w32=[&](uint32_t v){ fwrite(&v,4,1,fp); };
        auto w16=[&](uint16_t v){ fwrite(&v,2,1,fp); };
        uint32_t db=(uint32_t)(frames*2);
        fwrite("RIFF",1,4,fp); w32(36+db); fwrite("WAVE",1,4,fp);
        fwrite("fmt ",1,4,fp); w32(16); w16(1); w16(1); // PCM, mono
        w32(sr); w32(sr*2); w16(2); w16(16);
        fwrite("data",1,4,fp); w32(db);
    }

    // demod worker에서 호출: out 샘플을 녹음 파일에 기록
    inline void maybe_rec_audio(float out){
        if(!audio_rec_on.load(std::memory_order_relaxed)) return;
        if(!audio_rec_fp) return;
        int16_t s16=(int16_t)(out<-1.f?-32767:out>1.f?32767:(int)(out*32767.f));
        fwrite(&s16,2,1,audio_rec_fp);
        audio_rec_frames++;
    }

    // Squelch
    std::atomic<float> sq_threshold{-50.0f};
    std::atomic<float> sq_sig{-120.0f}, sq_nf{0.0f};
    std::atomic<bool>  sq_gate{false};
    std::atomic<bool>  sq_calibrated{false};

    // Filter move-drag state
    bool  move_drag=false;
    float move_anchor=0;
    float move_s0=0, move_e0=0;

    // Filter resize-drag state
    bool  resize_drag=false;
    int   resize_side=0;  // -1=left edge, +1=right edge

    Channel()=default;
    Channel(const Channel&)=delete;
    Channel& operator=(const Channel&)=delete;
};

// ── DSP helpers ───────────────────────────────────────────────────────────
static inline void apply_hann(fftwf_complex* in, int n){
    for(int i=0;i<n;i++){
        float w=0.5f*(1-cosf(2*M_PI*i/(n-1)));
        in[i][0]*=w; in[i][1]*=w;
    }
}

static inline uint32_t optimal_iq_sr(uint32_t main_sr, float bw_hz){
    float target=bw_hz*2.8f; if(target<10000) target=10000;
    uint32_t decim=(uint32_t)(main_sr/target); if(decim<1) decim=1;
    return main_sr/decim;
}

static inline void demod_rates(uint32_t main_sr, float bw_hz,
                                uint32_t& inter_sr, uint32_t& audio_decim, uint32_t& cap_decim){
    float min_inter=bw_hz*3.0f;
    if(min_inter<(float)AUDIO_SR) min_inter=(float)AUDIO_SR;
    uint32_t ad=(uint32_t)ceilf(min_inter/AUDIO_SR); if(ad<1) ad=1;
    uint32_t isr=AUDIO_SR*ad;
    uint32_t cd=main_sr/isr; if(cd<1) cd=1;
    inter_sr=isr; audio_decim=ad; cap_decim=cd;
}