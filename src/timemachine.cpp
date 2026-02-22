#include "fft_viewer.hpp"
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <algorithm>

static char s_iq_path[256]={};

void FFTViewer::tm_iq_open(){
    if(tm_iq_file_ready) return;
    struct stat st{};
    if(stat(TM_IQ_DIR,&st)!=0) mkdir(TM_IQ_DIR,0755);
    uint32_t sr=header.sample_rate;
    if(sr==0){ fprintf(stderr,"TM: sample_rate 0\n"); return; }
    tm_iq_total_samples=(int64_t)sr*(int64_t)TM_IQ_SECS;
    snprintf(s_iq_path,sizeof(s_iq_path),"%s/iq_rolling_%uMSPS.raw",TM_IQ_DIR,sr/1000000);
    if(access(s_iq_path,F_OK)==0){ remove(s_iq_path); printf("TM: removed old %s\n",s_iq_path); }
    // O_WRONLY|O_CREAT|O_RDWR, unbuffered
    tm_iq_fd=open(s_iq_path, O_RDWR|O_CREAT|O_TRUNC, 0644);
    if(tm_iq_fd<0){ fprintf(stderr,"TM: open failed: %s\n",strerror(errno)); return; }
    tm_iq_write_sample=0; tm_iq_chunk_write=0; tm_iq_chunk_sample_start=0;
    memset(tm_iq_chunk_time,0,sizeof(tm_iq_chunk_time));
    tm_iq_file_ready=true;
    printf("TM IQ rolling: ready (unbuffered fd)  max %.1f GB\n",
           (double)(tm_iq_total_samples*2*sizeof(int16_t))/1e9);
}

void FFTViewer::tm_iq_close(){
    if(tm_iq_fd>=0){ close(tm_iq_fd); tm_iq_fd=-1; }
    tm_iq_file_ready=false; tm_iq_write_sample=0;
    memset(tm_iq_chunk_time,0,sizeof(tm_iq_chunk_time));
    printf("TM IQ rolling: closed\n");
}

void FFTViewer::tm_iq_write(const int16_t* buf, int n_pairs){
    if(!tm_iq_file_ready||tm_iq_fd<0) return;
    int64_t max_total=tm_iq_total_samples;
    int written=0;
    while(written<n_pairs){
        int64_t pos=(tm_iq_write_sample<max_total)
                    ? tm_iq_write_sample
                    : tm_iq_write_sample % max_total;
        int64_t avail=max_total-pos;
        int chunk=(int)std::min((int64_t)(n_pairs-written),avail);
        off_t offset=pos*2*(off_t)sizeof(int16_t);
        ssize_t bytes=(ssize_t)chunk*2*(ssize_t)sizeof(int16_t);
        pwrite(tm_iq_fd, buf+written*2, (size_t)bytes, offset); // pwrite: 위치+write 원자적
        written+=chunk; tm_iq_write_sample+=chunk;
        int64_t cur_sec=tm_iq_write_sample/(int64_t)header.sample_rate;
        int ci=(int)(cur_sec%(int64_t)TM_IQ_SECS);
        if(ci!=tm_iq_chunk_write){ tm_iq_chunk_write=ci; tm_iq_chunk_time[ci]=time(nullptr); }
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
             type==1?"TM_IQ Start":"TM_IQ Stop",t->tm_hour,t->tm_min,t->tm_sec);
    std::lock_guard<std::mutex> lk(wf_events_mtx);
    wf_events.push_back(ev);
}

void FFTViewer::tm_update_display(){
    float rps=(float)header.sample_rate/(float)fft_size/(float)time_average;
    if(rps<=0) rps=37.5f;
    int max_rows=std::min(total_ffts,MAX_FFTS_MEMORY)-1;
    tm_max_sec=(float)max_rows/rps;
    tm_offset=std::max(0.0f,std::min(tm_offset,tm_max_sec));
    int row_offset=(int)(tm_offset*rps);
    tm_display_fft_idx=current_fft_idx-row_offset;
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