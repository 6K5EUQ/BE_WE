#include "fft_viewer.hpp"
#include <unistd.h>
#include <sys/stat.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>
#include <algorithm>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
// WAV 헤더 작성 (stereo int16: L=I, R=Q)
// ─────────────────────────────────────────────────────────────────────────────
static void write_wav_header(FILE* f, uint32_t sample_rate, uint32_t n_frames){
    uint32_t data_bytes = n_frames * 2 * 2; // frames * channels * bytes
    uint32_t chunk_size = 36 + data_bytes;
    uint16_t audio_fmt  = 1;   // PCM
    uint16_t channels   = 2;
    uint32_t byte_rate  = sample_rate * 4;
    uint16_t block_align= 4;
    uint16_t bits       = 16;
    uint32_t subchunk2  = data_bytes;

    fwrite("RIFF",1,4,f); fwrite(&chunk_size,4,1,f);
    fwrite("WAVE",1,4,f);
    fwrite("fmt ",1,4,f);
    uint32_t sc1=16; fwrite(&sc1,4,1,f);
    fwrite(&audio_fmt,2,1,f); fwrite(&channels,2,1,f);
    fwrite(&sample_rate,4,1,f); fwrite(&byte_rate,4,1,f);
    fwrite(&block_align,2,1,f); fwrite(&bits,2,1,f);
    fwrite("data",1,4,f); fwrite(&subchunk2,4,1,f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 파일명 생성
// iq_91.7010MHz_BW393kHz_20260222_172219-172240.wav
// ─────────────────────────────────────────────────────────────────────────────
static void make_filename(char* out, size_t sz,
                          float cf_mhz, float bw_khz,
                          time_t t_start, time_t t_end)
{
    struct tm *ts=localtime(&t_start);
    char date[16], s_start[8], s_end[8];
    strftime(date,   sizeof(date),  "%Y%m%d",  ts);
    strftime(s_start,sizeof(s_start),"%H%M%S", ts);
    struct tm *te=localtime(&t_end);
    strftime(s_end,  sizeof(s_end), "%H%M%S",  te);

    snprintf(out, sz,
             "/home/dsa/BE_WE/recordings/"
             "iq_%.4fMHz_BW%.0fkHz_%s_%s-%s.wav",
             (double)cf_mhz, (double)bw_khz, date, s_start, s_end);
}

// ─────────────────────────────────────────────────────────────────────────────
// 영역 IQ 추출 및 WAV 저장
//
// 알고리즘:
//   1. fft 행 인덱스 → 롤링 파일 샘플 오프셋 변환
//   2. 롤링 파일에서 원시 IQ (61.44 MSPS int16) 읽기
//   3. 주파수 mix-down: e^(-j2π*offset*n/sr) 곱셈
//   4. 박스필터 + 데시메이션 → 출력 샘플레이트 ≈ bw_hz
//   5. WAV 저장
// ─────────────────────────────────────────────────────────────────────────────
void FFTViewer::region_save(){
    if(!region.active){ return; }
    if(rec_busy_flag.load()){ return; } // 이미 저장 중
    if(!tm_iq_file_ready||tm_iq_fd<0){
        region.active=false; return;
    }

    // 저장에 필요한 모든 값을 캡처 (스레드 안전)
    rec_busy_flag.store(true);
    rec_state=REC_BUSY;
    rec_anim_timer=0.0f;
    region.active=false;

    // 백그라운드 스레드에서 실행
    std::thread([this](){
        do_region_save_work();
        if(!sa_mode){  // SA 모드면 rec_state는 do_region_save_work 내에서 처리
            rec_state=REC_SUCCESS;
            rec_success_timer=3.0f;
        }
        rec_busy_flag.store(false);
    }).detach();
}

void FFTViewer::do_region_save_work(){
    uint32_t sr=header.sample_rate;          // 61440000
    int64_t  max_total=tm_iq_total_samples;

    // ── 주파수 계산 ───────────────────────────────────────────────────────
    float cf_abs_mhz=(region.freq_lo+region.freq_hi)*0.5f;
    float bw_mhz    = region.freq_hi - region.freq_lo;
    float bw_khz    = bw_mhz * 1000.0f;
    float tune_mhz  = (float)(header.center_frequency/1e6);
    float offset_hz = (cf_abs_mhz - tune_mhz) * 1e6f; // mix-down 오프셋

    // 데시메이션 비율: sr / bw_hz (정수)
    uint32_t bw_hz = (uint32_t)(bw_mhz * 1e6f);
    if(bw_hz < 1000) bw_hz = 1000;
    int decim = std::max(1, (int)((float)sr / (float)bw_hz));
    uint32_t out_sr = sr / decim;

    // ── fft 행 → 샘플 오프셋 변환 (row_write_pos 기준) ───────────────────
    // row_write_pos[fi % MAX] = 해당 행 IQ 데이터가 끝나는 파일 위치
    // 행 fi 의 IQ 구간: [row_write_pos[fi-1], row_write_pos[fi])
    // fft_top = 최신(작은 y), fft_bot = 오래됨(큰 y)

    int ri_top = region.fft_top % MAX_FFTS_MEMORY;
    int ri_bot = region.fft_bot % MAX_FFTS_MEMORY;
    // 이전 행의 write_pos = 시작 위치
    int ri_bot_prev = (region.fft_bot - 1 + MAX_FFTS_MEMORY*4) % MAX_FFTS_MEMORY;

    int64_t samp_end   = row_write_pos[ri_top];  // fft_top 행 끝
    int64_t samp_start = row_write_pos[ri_bot_prev]; // fft_bot 행 시작 (= fft_bot-1 행 끝)

    if(samp_end == 0 || samp_start == 0){
        fprintf(stderr,"region_save: row_write_pos not populated\n");
        return;
    }

    // IQ 데이터 없음
    {
        bool any_avail=false;
        int lo=region.fft_bot, hi=region.fft_top;
        if(lo>hi) std::swap(lo,hi);
        int cnt=std::min(hi-lo+1, MAX_FFTS_MEMORY);
        for(int i=0;i<cnt;i++){
            if(iq_row_avail[(lo+i)%MAX_FFTS_MEMORY]){ any_avail=true; break; }
        }
        if(!any_avail){
            fprintf(stderr,"region_save: no IQ data (STOP 이후 영역)\n");
            return;
        }
    }

    if(samp_start < 0) samp_start = 0;
    if(samp_end   < 0) samp_end   = 0;
    if(samp_start > samp_end) std::swap(samp_start, samp_end);

    // 롤링 파일 범위 클램프
    // 파일이 아직 max_total 미만이면 [0, tm_iq_write_sample)
    int64_t valid_start = (tm_iq_write_sample >= max_total)
                          ? tm_iq_write_sample - max_total : 0;
    samp_start = std::max(samp_start, valid_start);
    samp_end   = std::min(samp_end,   tm_iq_write_sample);

    if(samp_end <= samp_start){
        fprintf(stderr,"region_save: no valid IQ data in range\n");
        return;
    }

    int64_t n_in = samp_end - samp_start;
    int64_t n_out = n_in / decim;
    if(n_out < 1){ return; }

    // ── 출력 파일 열기 ─────────────────────────────────────────────────────
    char outpath[512];
    if(sa_mode){
        // SA 전용 임시 경로
        const char* sa_dir = "/home/dsa/BE_WE/recordings/SA_Temp";
        struct stat sd{}; if(stat(sa_dir,&sd)!=0) mkdir(sa_dir,0755);
        struct tm *ts=localtime(&region.time_start);
        char date[16],s_start[8];
        strftime(date,sizeof(date),"%Y%m%d",ts);
        strftime(s_start,sizeof(s_start),"%H%M%S",ts);
        snprintf(outpath,sizeof(outpath),"%s/sa_%.4fMHz_BW%.0fkHz_%s_%s.wav",
                 sa_dir,(double)cf_abs_mhz,(double)bw_khz,date,s_start);
    } else {
        make_filename(outpath, sizeof(outpath),
                      cf_abs_mhz, bw_khz,
                      region.time_start, region.time_end);
    }
    FILE* wf = fopen(outpath, "wb");
    if(!wf){
        fprintf(stderr,"region_save: fopen failed: %s\n", outpath);
        return;
    }
    // WAV 헤더 자리 확보 (나중에 덮어쓸 것)
    write_wav_header(wf, out_sr, (uint32_t)n_out);

    // ── 청크 단위 읽기 + mix-down + decimate + 저장 ───────────────────────
    const int CHUNK = 65536; // 샘플 단위
    std::vector<int16_t> in_buf(CHUNK * 2);
    std::vector<int16_t> out_buf;
    out_buf.reserve(CHUNK / decim * 2 + 4);

    double phase = 0.0;
    double phase_inc = -2.0 * M_PI * (double)offset_hz / (double)sr;

    int64_t actual_out = 0;
    int64_t pos = samp_start;

    // 박스필터 누산기
    double box_i = 0, box_q = 0;
    int    box_cnt = 0;

    while(pos < samp_end){
        int64_t file_pos = pos % max_total;
        int     to_read  = (int)std::min((int64_t)CHUNK, samp_end - pos);
        // 파일 끝 넘어가면 두 번 읽기
        int64_t avail = max_total - file_pos;
        int     r1 = (int)std::min((int64_t)to_read, avail);
        int     r2 = to_read - r1;

        static constexpr off_t WAV_HDR_SIZE = 44;
        off_t off1 = WAV_HDR_SIZE + file_pos*2*(off_t)sizeof(int16_t);
        pread(tm_iq_fd, in_buf.data(),        (size_t)r1*2*sizeof(int16_t), off1);
        if(r2 > 0)
            pread(tm_iq_fd, in_buf.data()+r1*2, (size_t)r2*2*sizeof(int16_t), WAV_HDR_SIZE);

        out_buf.clear();
        for(int i=0; i<to_read; i++){
            float si = in_buf[i*2  ] / 32768.0f;
            float sq = in_buf[i*2+1] / 32768.0f;

            // Mix-down
            float cp = (float)cos(phase);
            float sp = (float)sin(phase);
            float mi = si*cp - sq*sp;
            float mq = si*sp + sq*cp;
            phase += phase_inc;
            if(phase >  M_PI) phase -= 2.0*M_PI;
            if(phase < -M_PI) phase += 2.0*M_PI;

            // 박스필터 누산
            box_i += mi; box_q += mq; box_cnt++;
            if(box_cnt >= decim){
                float oi = (float)(box_i / decim);
                float oq = (float)(box_q / decim);
                box_i = 0; box_q = 0; box_cnt = 0;

                int16_t wi = (int16_t)(std::max(-1.0f,std::min(1.0f,oi))*32767.0f);
                int16_t wq = (int16_t)(std::max(-1.0f,std::min(1.0f,oq))*32767.0f);
                out_buf.push_back(wi);
                out_buf.push_back(wq);
                actual_out++;
            }
        }
        if(!out_buf.empty())
            fwrite(out_buf.data(), sizeof(int16_t), out_buf.size(), wf);

        pos += to_read;
    }

    // WAV 헤더 실제 샘플 수로 갱신
    rewind(wf);
    write_wav_header(wf, out_sr, (uint32_t)actual_out);
    fclose(wf);

    printf("Region IQ saved: %s  (%.1f sec  %.0f kHz SR)\n",
           outpath, (double)actual_out/out_sr, (double)out_sr/1000.0);

    // SA 모드: 저장 완료 후 SA 워터폴 계산 시작
    if(sa_mode){
        sa_mode = false;
        sa_temp_path = outpath;
        sa_start(outpath);
    } else {
        region.active = false;
    }
}