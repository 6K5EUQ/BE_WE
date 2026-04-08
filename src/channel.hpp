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
    void flush(){
        if(!fp||buf.empty()) return;
        fwrite(buf.data(),2,buf.size(),fp);
        buf.clear();
        // 헤더 갱신 (녹음 중에도 파일을 읽을 수 있도록)
        long pos=ftell(fp);
        fseek(fp,0,SEEK_SET); write_hdr();
        fseek(fp,pos,SEEK_SET);
        fflush(fp);
    }
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
    char  owner[32]={};   // creator ID (empty = unknown)

    // Demodulation mode
    enum DemodMode{ DM_NONE=0, DM_AM, DM_FM, DM_MAGIC } mode=DM_NONE;

    // Digital decode mode (D키로 토글, 음성 복조와 독립)
    enum DigitalMode{ DIGI_NONE=0, DIGI_AIS, DIGI_ADSB, DIGI_DEMOD } digital_mode=DIGI_NONE;
    int    digi_demod_type = 0;   // 0=ASK, 1=FSK, 2=BPSK
    float  digi_baud_rate = 1200; // user-specified baud rate

    // Magic mode: detected modulation (0=analyzing, 1=AM, 2=FM, 3=DSB, 4=SSB, 5=CW)
    std::atomic<int> magic_det{0};
    int   pan=0;   // -1=L  0=both  1=R
    // audio_mask: bit0=host local, bit_i=operator_i gets audio
    std::atomic<uint32_t> audio_mask{0x1};  // default: host only

    // Demod thread (음성)
    std::atomic<bool>   dem_run{false};
    std::atomic<bool>   dem_stop_req{false};
    std::thread         dem_thr;
    std::atomic<size_t> dem_rp{0};
    bool                dem_paused=false;         // 주파수 범위 벗어나 자동 pause됨
    DemodMode           dem_paused_mode=DM_NONE;  // pause 직전 mode 보존

    // Digital decode thread (AIS 등)
    std::atomic<bool>   digi_run{false};
    std::atomic<bool>   digi_stop_req{false};
    std::thread         digi_thr;
    std::atomic<size_t> digi_rp{0};

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

    // 스컬치 기반 녹음 상태머신 (worker 스레드 전용)
    enum SqRecState : int { SQR_IDLE=0, SQR_RECORDING=1, SQR_TAIL=2 };
    int      sqr_state       = SQR_IDLE;
    uint32_t sqr_tail_remain = 0;

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

    // demod worker에서 호출: 스컬치 기반 녹음 상태머신
    inline void maybe_rec_audio(float out, bool gate_open){
        if(!audio_rec_on.load(std::memory_order_relaxed)) return;
        if(!audio_rec_fp) return;

        uint32_t tail_samples = audio_rec_sr; // 1초

        switch(sqr_state){
        case SQR_IDLE:
            if(!gate_open) return;
            sqr_state = SQR_RECORDING;
            break;
        case SQR_RECORDING:
            if(!gate_open){
                sqr_state = SQR_TAIL;
                sqr_tail_remain = tail_samples;
            }
            break;
        case SQR_TAIL:
            if(gate_open){
                sqr_state = SQR_RECORDING;
            } else if(sqr_tail_remain == 0){
                sqr_state = SQR_IDLE;
                return;
            } else {
                sqr_tail_remain--;
            }
            break;
        }

        int16_t s16=(int16_t)(out<-1.f?-32767:out>1.f?32767:(int)(out*32767.f));
        fwrite(&s16,2,1,audio_rec_fp);
        audio_rec_frames++;
    }

    // ── Per-channel IQ recording (demod 스레드 내에서만 접근) ────────────
    std::atomic<bool> iq_rec_on{false};
    FILE*             iq_rec_fp      = nullptr;
    uint64_t          iq_rec_frames  = 0;
    uint32_t          iq_rec_sr      = 0;
    std::string       iq_rec_path;
    int      iq_sqr_state       = SQR_IDLE;
    uint32_t iq_sqr_tail_remain = 0;

    uint64_t iq_rec_cf_hz = 0;   // center freq for bewe chunk

    // ── HOST→JOIN 동기화 녹음 시간 (CH_SYNC 경유) ────────────────────────
    uint32_t synced_iq_rec_secs    = 0;
    uint32_t synced_audio_rec_secs = 0;
    int64_t  iq_rec_start_time = 0;

    // WAV 헤더 + bewe 메타데이터 청크 (총 72바이트)
    void iq_rec_write_wav_hdr(FILE* fp, uint32_t sr, uint64_t frames){
        auto w32=[&](uint32_t v){ fwrite(&v,4,1,fp); };
        auto w16=[&](uint16_t v){ fwrite(&v,2,1,fp); };
        uint32_t db=(uint32_t)(frames*4);
        uint32_t bewe_chunk=28; // "bewe"(4)+size(4)+payload(20)
        fwrite("RIFF",1,4,fp); w32(36+db+bewe_chunk); fwrite("WAVE",1,4,fp);
        fwrite("fmt ",1,4,fp); w32(16); w16(1); w16(2);
        w32(sr); w32(sr*4); w16(4); w16(16);
        // bewe 청크 (data 앞에 배치)
        fwrite("bewe",1,4,fp); w32(20);
        fwrite(&iq_rec_cf_hz,8,1,fp);
        fwrite(&iq_rec_start_time,8,1,fp);
        fwrite(&sr,4,1,fp);
        // data 청크
        fwrite("data",1,4,fp); w32(db);
    }

    inline void maybe_rec_iq(float fi, float fq, bool gate_open){
        if(!iq_rec_on.load(std::memory_order_relaxed)) return;
        if(!iq_rec_fp) return;
        uint32_t tail_samples = iq_rec_sr; // 1 second tail
        switch(iq_sqr_state){
        case SQR_IDLE:
            if(!gate_open) return;
            iq_sqr_state = SQR_RECORDING;
            break;
        case SQR_RECORDING:
            if(!gate_open){ iq_sqr_state = SQR_TAIL; iq_sqr_tail_remain = tail_samples; }
            break;
        case SQR_TAIL:
            if(gate_open) iq_sqr_state = SQR_RECORDING;
            else if(iq_sqr_tail_remain == 0){ iq_sqr_state = SQR_IDLE; return; }
            else iq_sqr_tail_remain--;
            break;
        }
        auto c16=[](float v)->int16_t{ return (int16_t)(v<-1.f?-32767:v>1.f?32767:(int)(v*32767.f)); };
        int16_t si=c16(fi), sq=c16(fq);
        fwrite(&si,2,1,iq_rec_fp);
        fwrite(&sq,2,1,iq_rec_fp);
        iq_rec_frames++;
        // 매 65536 샘플마다 헤더 갱신 (녹음 중 실시간 분석 가능)
        if((iq_rec_frames & 0xFFFF) == 0){
            long pos=ftell(iq_rec_fp);
            fseek(iq_rec_fp,0,SEEK_SET);
            iq_rec_write_wav_hdr(iq_rec_fp,iq_rec_sr,iq_rec_frames);
            fseek(iq_rec_fp,pos,SEEK_SET);
            fflush(iq_rec_fp);
        }
    }

    // Squelch (UI 스레드에서 FFT 기반으로 중앙 관리)
    std::atomic<float> sq_threshold{-50.0f};
    std::atomic<float> sq_sig{-120.0f}, sq_nf{0.0f};
    std::atomic<bool>  sq_gate{false};
    std::atomic<bool>  sq_calibrated{false};
    // 캘리브레이션 (UI 스레드 전용)
    int   sq_calib_cnt = 0;
    float sq_calib_buf[60] = {};  // ~1초 @ 60fps
    int   sq_gate_hold = 0;      // gate hold 카운터 (프레임 단위)
    float sq_last_close_t = -10.f; // 게이트 마지막 닫힌 시점 (ImGui::GetTime)
    bool  sq_gate_prev = false;  // 이전 프레임 게이트 상태
    float sq_active_time = 0.0f; // 스컬치 열린 누적 시간(초)
    float sq_total_time  = 0.0f; // 전체 경과 시간 (프레임 기반 누적)

    // Filter move-drag state
    bool  move_drag=false;
    float move_anchor=0;
    float move_s0=0, move_e0=0;

    // Filter resize-drag state
    bool  resize_drag=false;
    int   resize_side=0;  // -1=left edge, +1=right edge

    // 채널 슬롯 재사용 시 모든 비-스레드 상태를 초기값으로 복원
    void reset_slot(){
        s=0; e=0;
        filter_active=false;
        selected=false;
        memset(owner, 0, sizeof(owner));
        mode=DM_NONE;
        digital_mode=DIGI_NONE;
        digi_demod_type=0;
        digi_baud_rate=1200;
        magic_det.store(0);
        pan=0;
        audio_mask.store(0x1);
        // demod/digi 스레드는 호출 전에 stop_dem/stop_digi로 정리할 것
        dem_rp.store(0);
        dem_paused=false;
        dem_paused_mode=DM_NONE;
        digi_rp.store(0);
        // audio ring
        ar_wp.store(0);
        ar_rp.store(0);
        // audio recording
        sqr_state=SQR_IDLE;
        sqr_tail_remain=0;
        // iq recording
        iq_sqr_state=SQR_IDLE;
        iq_sqr_tail_remain=0;
        // squelch
        sq_threshold.store(-50.0f);
        sq_sig.store(-120.0f);
        sq_nf.store(0.0f);
        sq_gate.store(false);
        sq_calibrated.store(false);
        sq_calib_cnt=0;
        memset(sq_calib_buf, 0, sizeof(sq_calib_buf));
        sq_gate_hold=0;
        sq_active_time=0; sq_total_time=0;
        // drag state
        move_drag=false;
        move_anchor=0;
        move_s0=0; move_e0=0;
        resize_drag=false;
        resize_side=0;
    }

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

// Nuttall window: -93dB sidelobe suppression (vs Hann -31dB)
static inline void apply_nuttall(fftwf_complex* in, int n){
    const double a0=0.355768, a1=0.487396, a2=0.144232, a3=0.012604;
    for(int i=0;i<n;i++){
        double x=2.0*M_PI*i/(n-1);
        float w=(float)(a0 - a1*cos(x) + a2*cos(2*x) - a3*cos(3*x));
        in[i][0]*=w; in[i][1]*=w;
    }
}

// Pre-compute Nuttall window into float buffer (for VOLK multiply)
static inline void fill_nuttall_window(float* buf, int n){
    const double a0=0.355768, a1=0.487396, a2=0.144232, a3=0.012604;
    for(int i=0;i<n;i++){
        double x=2.0*M_PI*i/(n-1);
        buf[i]=(float)(a0 - a1*cos(x) + a2*cos(2*x) - a3*cos(3*x));
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