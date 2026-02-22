#include "fft_viewer.hpp"
#include <sys/stat.h>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
// IQ 롤링 파일 관리
// 파일 구조: int16 interleaved IQ, 61.44MSPS
// 전체 크기 = sample_rate * TM_IQ_SECS * 2 * sizeof(int16_t)
// ─────────────────────────────────────────────────────────────────────────────

void FFTViewer::tm_iq_open(){
    if(tm_iq_file_ready) return;

    // 디렉터리 생성
    struct stat st{};
    if(stat(TM_IQ_DIR,&st)!=0)
        mkdir(TM_IQ_DIR,0755);

    uint32_t sr=header.sample_rate;
    if(sr==0){ fprintf(stderr,"TM: sample_rate 0, skip\n"); return; }

    tm_iq_total_samples=(int64_t)sr * (int64_t)TM_IQ_SECS; // IQ 쌍 수
    int64_t file_bytes=tm_iq_total_samples * 2 * (int64_t)sizeof(int16_t);

    char path[256];
    snprintf(path,sizeof(path),"%s/iq_rolling_%uMSPS.raw",TM_IQ_DIR,sr/1000000);

    tm_iq_file=fopen(path,"r+b");
    if(!tm_iq_file){
        // 파일 없으면 새로 생성 + 미리 할당
        tm_iq_file=fopen(path,"w+b");
        if(!tm_iq_file){ fprintf(stderr,"TM: fopen failed: %s\n",strerror(errno)); return; }
        printf("TM: pre-allocating %.1f GB → %s\n",(double)file_bytes/1e9,path);
        if(fseeko(tm_iq_file,(off_t)file_bytes-1,SEEK_SET)!=0||
           fwrite("\0",1,1,tm_iq_file)!=1){
            fprintf(stderr,"TM: prealloc failed\n");
            fclose(tm_iq_file); tm_iq_file=nullptr; return;
        }
        fflush(tm_iq_file);
    }
    tm_iq_write_sample=0;
    tm_iq_chunk_write=0;
    tm_iq_chunk_sample_start=0;
    memset(tm_iq_chunk_time,0,sizeof(tm_iq_chunk_time));
    tm_iq_file_ready=true;
    printf("TM IQ rolling: ready  %lld samples / %.1f GB\n",
           (long long)tm_iq_total_samples,(double)file_bytes/1e9);
}

void FFTViewer::tm_iq_close(){
    if(tm_iq_file){ fclose(tm_iq_file); tm_iq_file=nullptr; }
    tm_iq_file_ready=false;
    tm_iq_write_sample=0;
    memset(tm_iq_chunk_time,0,sizeof(tm_iq_chunk_time));
    printf("TM IQ rolling: closed\n");
}

// n_samples = IQ 쌍 수 (samples * 2 int16 = 1 IQ pair)
void FFTViewer::tm_iq_write(const int16_t* buf, int n_pairs){
    if(!tm_iq_file_ready||!tm_iq_file) return;
    uint32_t sr=header.sample_rate;
    int64_t samples_per_sec=(int64_t)sr;
    int64_t total=tm_iq_total_samples; // IQ 쌍 수

    int written=0;
    while(written<n_pairs){
        int64_t pos=tm_iq_write_sample % total;
        int64_t avail=total-pos;
        int chunk=std::min((int64_t)(n_pairs-written),avail);
        off_t offset=pos*2*(off_t)sizeof(int16_t);
        fseeko(tm_iq_file,offset,SEEK_SET);
        fwrite(buf+written*2, sizeof(int16_t)*2, chunk, tm_iq_file);
        written+=chunk;
        tm_iq_write_sample+=chunk;

        // 초 경계마다 청크 타임스탬프 업데이트
        int64_t cur_sec=tm_iq_write_sample/samples_per_sec;
        int chunk_idx=(int)(cur_sec % (int64_t)TM_IQ_SECS);
        if(chunk_idx!=tm_iq_chunk_write){
            tm_iq_chunk_write=chunk_idx;
            tm_iq_chunk_time[chunk_idx]=time(nullptr);
        }
    }
}

// FFT 행 fi 에 IQ 가용 플래그 세팅
void FFTViewer::tm_mark_rows(int fi){
    if(!tm_iq_file_ready) return;
    iq_row_avail[fi % MAX_FFTS_MEMORY]=true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 타임머신 뷰: 현재 오프셋 기준 display_fft_idx 계산
// ui.cpp에서 매 프레임 호출
// ─────────────────────────────────────────────────────────────────────────────
void FFTViewer::tm_update_display(){
    // rows_per_sec = sample_rate / fft_size / time_average
    float rps=(float)header.sample_rate/(float)fft_size/(float)time_average;
    if(rps<=0) rps=37.5f;

    // 최대 오프셋: 버퍼에 있는 행 수 기준
    int max_rows=std::min(total_ffts, MAX_FFTS_MEMORY)-1;
    tm_max_sec=(float)max_rows/rps;

    tm_offset=std::max(0.0f,std::min(tm_offset,tm_max_sec));
    int row_offset=(int)(tm_offset*rps);
    tm_display_fft_idx=current_fft_idx - row_offset;
    if(tm_display_fft_idx<0) tm_display_fft_idx=0;
}

// ─────────────────────────────────────────────────────────────────────────────
// 타임머신에서 IQ 데이터 읽어서 기존 rec_worker와 동일한 파일로 저장
// 현재 tm_display_fft_idx 시점의 IQ를 선택 채널 대역으로 저장
// ─────────────────────────────────────────────────────────────────────────────
bool FFTViewer::tm_rec_start(){
    // IQ 파일 준비 확인
    if(!tm_iq_file_ready||!tm_iq_file){
        rec_na_timer=3.0f; return false;
    }
    // 현재 뷰 시점에 IQ 데이터 있는지 확인
    int disp_row=tm_display_fft_idx % MAX_FFTS_MEMORY;
    if(!iq_row_avail[disp_row]){
        rec_na_timer=3.0f; return false;
    }
    // 채널 선택 확인
    int fi=selected_ch;
    if(fi<0||!channels[fi].filter_active){
        rec_na_timer=3.0f; return false;
    }
    // 기존 start_rec 호출 (파일명/경로는 그대로)
    // tm_mode에서는 rec_worker가 IQ 파일을 직접 읽어야 하지만
    // 지금은 구조 단순화: SSD IQ 파일에서 오프셋 읽기를 
    // rec_worker에 tm_read_offset으로 전달
    float rps=(float)header.sample_rate/(float)fft_size/(float)time_average;
    int row_offset=(int)(tm_offset*rps);
    int64_t samp_offset=(int64_t)header.sample_rate * tm_offset;
    int64_t read_pos=(tm_iq_write_sample - samp_offset);
    if(read_pos<0) read_pos=tm_iq_total_samples + read_pos;
    read_pos = read_pos % tm_iq_total_samples;
    tm_rec_read_pos=read_pos;
    tm_rec_active=true;
    start_rec(); // 기존 rec 함수 호출 (채널 필터 기준)
    return true;
}