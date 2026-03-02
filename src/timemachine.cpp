#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <algorithm>

static char s_iq_path[256]={};
static constexpr off_t WAV_HDR_SIZE = 44; // WAV 헤더 크기

// WAV 헤더 작성 (stereo int16: L=I, R=Q)
static void write_rolling_wav_header(int fd, uint32_t sample_rate, uint32_t n_frames){
    uint32_t data_bytes  = n_frames * 4;
    uint32_t chunk_size  = 36 + data_bytes;
    uint16_t audio_fmt   = 1, channels = 2;
    uint32_t byte_rate   = sample_rate * 4;
    uint16_t block_align = 4, bits = 16;
    uint8_t hdr[44]={};
    memcpy(hdr+0,"RIFF",4); memcpy(hdr+4,&chunk_size,4);
    memcpy(hdr+8,"WAVE",4); memcpy(hdr+12,"fmt ",4);
    uint32_t sc1=16; memcpy(hdr+16,&sc1,4);
    memcpy(hdr+20,&audio_fmt,2); memcpy(hdr+22,&channels,2);
    memcpy(hdr+24,&sample_rate,4); memcpy(hdr+28,&byte_rate,4);
    memcpy(hdr+32,&block_align,2); memcpy(hdr+34,&bits,2);
    memcpy(hdr+36,"data",4); memcpy(hdr+40,&data_bytes,4);
    pwrite(fd, hdr, 44, 0);
}

void FFTViewer::tm_iq_open(){
    if(tm_iq_file_ready) return;
    struct stat st{};
    std::string tm_dir=BEWEPaths::time_temp_dir();
    const char* TM_IQ_DIR=tm_dir.c_str();
    if(stat(TM_IQ_DIR,&st)!=0) mkdir(TM_IQ_DIR,0755);
    uint32_t sr=header.sample_rate;
    if(sr==0){ fprintf(stderr,"TM: sample_rate 0\n"); return; }
    tm_iq_total_samples=(int64_t)sr*(int64_t)TM_IQ_SECS;
    snprintf(s_iq_path,sizeof(s_iq_path),"%s/iq_rolling_%uMSPS.wav",TM_IQ_DIR,sr/1000000);
    // 기존 파일 항상 삭제 후 새로 생성
    if(access(s_iq_path,F_OK)==0){ remove(s_iq_path); printf("TM: removed old %s\n",s_iq_path); }
    tm_iq_fd=open(s_iq_path, O_RDWR|O_CREAT|O_TRUNC, 0644);
    if(tm_iq_fd<0){ fprintf(stderr,"TM: open failed: %s\n",strerror(errno)); return; }
    // WAV 헤더 placeholder (n_frames=0, Stop 시 갱신)
    write_rolling_wav_header(tm_iq_fd, sr, 0);
    tm_iq_write_sample=0; tm_iq_chunk_write=0; tm_iq_chunk_sample_start=0;
    memset(tm_iq_chunk_time,0,sizeof(tm_iq_chunk_time));
    tm_iq_batch_buf.assign(TM_IQ_BATCH*2, 0);
    tm_iq_batch_cnt=0;
    tm_iq_file_ready=true;
    printf("TM IQ rolling: ready (wav)  max %.1f GB\n",
           (double)(tm_iq_total_samples*2*sizeof(int16_t))/1e9);
}

void FFTViewer::tm_iq_close(){
    if(tm_iq_fd>=0 && tm_iq_batch_cnt>0) tm_iq_flush_batch();
    if(tm_iq_fd>=0){
        // Stop: WAV 헤더를 실제 샘플 수로 갱신
        uint32_t actual = (uint32_t)std::min(tm_iq_write_sample, tm_iq_total_samples);
        write_rolling_wav_header(tm_iq_fd, header.sample_rate, actual);
        close(tm_iq_fd); tm_iq_fd=-1;
        printf("TM IQ rolling: closed  %.2f sec\n",(double)actual/header.sample_rate);
    }
    tm_iq_file_ready=false; tm_iq_write_sample=0; tm_iq_batch_cnt=0;
    memset(tm_iq_chunk_time,0,sizeof(tm_iq_chunk_time));
}

// 배치 버퍼 → 파일 플러시 (내부용)
void FFTViewer::tm_iq_flush_batch(){
    if(tm_iq_fd<0||tm_iq_batch_cnt<=0) return;
    int n=tm_iq_batch_cnt;
    int written=0;
    int16_t* buf=tm_iq_batch_buf.data();
    while(written<n){
        int64_t max_total=tm_iq_total_samples;
        int64_t pos=(tm_iq_write_sample<max_total)
                    ? tm_iq_write_sample
                    : tm_iq_write_sample % max_total;
        int64_t avail=max_total-pos;
        int chunk=(int)std::min((int64_t)(n-written),(int64_t)avail);
        off_t offset = WAV_HDR_SIZE + pos*2*(off_t)sizeof(int16_t);
        ssize_t bytes=(ssize_t)chunk*2*(ssize_t)sizeof(int16_t);
        pwrite(tm_iq_fd, buf+written*2, (size_t)bytes, offset);
        written+=chunk; tm_iq_write_sample+=chunk;
        int64_t cur_sec=tm_iq_write_sample/(int64_t)header.sample_rate;
        int ci=(int)(cur_sec%(int64_t)TM_IQ_SECS);
        if(ci!=tm_iq_chunk_write){ tm_iq_chunk_write=ci; tm_iq_chunk_time[ci]=time(nullptr); }
    }
    tm_iq_batch_cnt=0;
}

void FFTViewer::tm_iq_write(const int16_t* buf, int n_pairs){
    if(!tm_iq_file_ready||tm_iq_fd<0) return;
    int src=0;
    while(src<n_pairs){
        int space=TM_IQ_BATCH-tm_iq_batch_cnt;
        int copy=std::min(n_pairs-src, space);
        // SC16_Q11 → ×16 스케일링: ±2048 → ±32768 (URH 풀스케일 정규화)
        const int16_t* src_ptr = buf + src*2;
        int16_t* dst_ptr = tm_iq_batch_buf.data() + tm_iq_batch_cnt*2;
        for(int i=0; i<copy*2; i++){
            int32_t v = (int32_t)src_ptr[i] * 16;
            dst_ptr[i] = (int16_t)std::max(-32768, std::min(32767, v));
        }
        tm_iq_batch_cnt+=copy; src+=copy;
        if(tm_iq_batch_cnt>=TM_IQ_BATCH) tm_iq_flush_batch();
    }
}

void FFTViewer::tm_mark_rows(int fi){
    if(!tm_iq_file_ready) return;
    iq_row_avail[fi%MAX_FFTS_MEMORY]=true;
}

void FFTViewer::tm_add_time_tag(int fft_idx){
    time_t now=time(nullptr);
    struct tm* t=localtime(&now);
    int cur5=t->tm_hour*720+t->tm_min*12+t->tm_sec/5; // 5초 단위 카운터
    if(cur5==last_tagged_sec&&last_tagged_sec!=-1) return;
    last_tagged_sec=cur5;
    WfEvent ev; ev.fft_idx=fft_idx; ev.wall_time=now; ev.type=0;
    strftime(ev.label,sizeof(ev.label),"%M:%S",t);
    std::lock_guard<std::mutex> lk(wf_events_mtx);
    wf_events.push_back(ev);
    int cutoff=fft_idx-MAX_FFTS_MEMORY;
    wf_events.erase(std::remove_if(wf_events.begin(),wf_events.end(),
        [&](const WfEvent& e){ return e.fft_idx<cutoff; }),wf_events.end());
}

void FFTViewer::tm_add_event_tag(int type){
    time_t now=time(nullptr); struct tm* t=localtime(&now);
    WfEvent ev; ev.fft_idx=current_fft_idx; ev.wall_time=now; ev.type=type;
    snprintf(ev.label,sizeof(ev.label),"%s  %02d:%02d:%02d",
             type==1?"IQ Start":"IQ Stop",t->tm_hour,t->tm_min,t->tm_sec);
    std::lock_guard<std::mutex> lk(wf_events_mtx);
    wf_events.push_back(ev);
}

time_t FFTViewer::fft_idx_to_wall_time(int fft_idx) const {
    std::lock_guard<std::mutex> lk(wf_events_mtx);
    if(wf_events.empty()) return 0;

    // fft_idx 기준으로 가장 가까운 두 이벤트 찾기 (앞뒤)
    // wf_events는 fft_idx 오름차순이라고 가정
    const WfEvent* prev = nullptr;
    const WfEvent* next = nullptr;
    for(const auto& ev : wf_events){
        if(ev.fft_idx <= fft_idx) prev = &ev;
        if(ev.fft_idx >= fft_idx && !next) next = &ev;
    }

    if(prev && next && prev != next){
        // 두 이벤트 사이 보간: 실제 wall_time 차이로 rps 추정
        int64_t fi_diff = next->fft_idx - prev->fft_idx;
        int64_t wt_diff = (int64_t)next->wall_time - (int64_t)prev->wall_time;
        if(fi_diff > 0 && wt_diff > 0){
            int64_t offset = fft_idx - prev->fft_idx;
            return (time_t)(prev->wall_time + offset * wt_diff / fi_diff);
        }
    }
    if(prev){
        // prev만 있으면 rps로 외삽
        float rps = (float)header.sample_rate / (float)fft_size / (float)time_average;
        if(rps <= 0) rps = 37.5f;
        int64_t offset = fft_idx - prev->fft_idx;
        return (time_t)(prev->wall_time + (int64_t)(offset / rps));
    }
    if(next){
        float rps = (float)header.sample_rate / (float)fft_size / (float)time_average;
        if(rps <= 0) rps = 37.5f;
        int64_t offset = fft_idx - next->fft_idx; // 음수
        return (time_t)(next->wall_time + (int64_t)(offset / rps));
    }
    return 0;
}

void FFTViewer::tm_update_display(){
    // wf_events 기반 실제 rps 계산 (JOIN/HOST 모두 정확)
    float rps = 0.0f;
    {
        std::lock_guard<std::mutex> lk(wf_events_mtx);
        // 가장 멀리 떨어진 두 이벤트로 실제 rps 추정
        if(wf_events.size() >= 2){
            const WfEvent& first = wf_events.front();
            const WfEvent& last  = wf_events.back();
            int64_t fi_diff = last.fft_idx - first.fft_idx;
            int64_t wt_diff = (int64_t)last.wall_time - (int64_t)first.wall_time;
            if(fi_diff > 0 && wt_diff > 0)
                rps = (float)fi_diff / (float)wt_diff;
        }
    }
    if(rps <= 0){
        rps = (float)header.sample_rate / (float)fft_size / (float)time_average;
        if(rps <= 0) rps = 37.5f;
    }

    // 최대 오프셋 = freeze 시점 기준 버퍼 용량 (최대 MAX_FFTS_MEMORY-1 행)
    int max_rows=std::min(tm_freeze_idx, MAX_FFTS_MEMORY-1);
    tm_max_sec=(float)max_rows/rps;

    tm_offset=std::max(0.0f,std::min(tm_offset,tm_max_sec));
    int row_offset=(int)(tm_offset*rps);
    tm_display_fft_idx=tm_freeze_idx - row_offset;
    if(tm_display_fft_idx<0) tm_display_fft_idx=0;
}

bool FFTViewer::tm_rec_start(){
    if(!tm_iq_file_ready||tm_iq_fd<0){ rec_na_timer=3.0f; return false; }
    int disp_row=tm_display_fft_idx%MAX_FFTS_MEMORY;
    if(!iq_row_avail[disp_row]){ rec_na_timer=3.0f; return false; }
    int fi=selected_ch;
    if(fi<0||!channels[fi].filter_active){ rec_na_timer=3.0f; return false; }
    int64_t samp_offset=(int64_t)((double)header.sample_rate*tm_offset);
    int64_t read_pos=tm_iq_write_sample-samp_offset;
    if(read_pos<0) read_pos=tm_iq_total_samples+read_pos;
    read_pos=read_pos%tm_iq_total_samples;
    tm_rec_read_pos=read_pos; tm_rec_active=true;
    start_rec(); return true;
}