#include "fft_viewer.hpp"
#include <fftw3.h>
#include <dirent.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>

// ── Jet colormap (sa 전용) ─────────────────────────────────────────────────
static uint32_t sa_jet(float t){
    t = t<0?0:t>1?1:t;
    float r = 1.5f-fabsf(4*t-3);
    float g = 1.5f-fabsf(4*t-2);
    float b = 1.5f-fabsf(4*t-1);
    auto c=[](float v)->uint8_t{ v=v<0?0:v>1?1:v; return (uint8_t)(v*255); };
    return IM_COL32(c(r),c(g),c(b),255);
}

// Hann 윈도우
static void sa_hann(float* buf, int n){
    for(int i=0;i<n;i++){
        float w = 0.5f*(1.0f-cosf(2*M_PIf*i/(n-1)));
        buf[i*2  ] *= w;
        buf[i*2+1] *= w;
    }
}

void FFTViewer::sa_cleanup(){
    // 스레드 먼저 종료 대기 (this 캡처 스레드가 살아있으면 크래시)
    if(sa_thread.joinable()) sa_thread.join();

    // 현재 추적 중인 파일 삭제
    if(!sa_temp_path.empty()){
        remove(sa_temp_path.c_str());
        sa_temp_path.clear();
    }
    // SA_Temp 폴더 내 모든 파일 삭제
    {
        const char* sa_dir="/home/dsa/BE_WE/recordings/SA_Temp";
        DIR* d=opendir(sa_dir);
        if(d){
            struct dirent* e;
            char path[512];
            while((e=readdir(d))!=nullptr){
                if(e->d_name[0]=='.') continue;
                snprintf(path,sizeof(path),"%s/%s",sa_dir,e->d_name);
                remove(path);
            }
            closedir(d);
        }
    }
    if(sa_texture){ glDeleteTextures(1,&sa_texture); sa_texture=0; }
    sa_tex_w=0; sa_tex_h=0;
    sa_pixel_ready.store(false);
}

void FFTViewer::sa_upload_texture(){
    std::lock_guard<std::mutex> lk(sa_pixel_mtx);
    if(sa_pixel_buf.empty()) return;
    if(sa_texture) glDeleteTextures(1,&sa_texture);
    glGenTextures(1,&sa_texture);
    glBindTexture(GL_TEXTURE_2D,sa_texture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,
                 sa_tex_w,sa_tex_h,0,
                 GL_RGBA,GL_UNSIGNED_BYTE,sa_pixel_buf.data());
    glBindTexture(GL_TEXTURE_2D,0);
    sa_pixel_ready.store(false);
}

void FFTViewer::sa_start(const std::string& wav_path){
    // 이전 스레드 정리
    if(sa_thread.joinable()) sa_thread.join();
    sa_computing.store(true);
    sa_pixel_ready.store(false);

    int fft_n = sa_fft_size;

    sa_thread = std::thread([this, wav_path, fft_n](){
        // ── WAV 읽기 ──────────────────────────────────────────────────────
        FILE* f = fopen(wav_path.c_str(),"rb");
        if(!f){ sa_computing.store(false); return; }

        // 헤더 44바이트 skip
        uint8_t hdr[44]; fread(hdr,1,44,f);

        // data_bytes를 실제 파일 크기로 계산 (헤더 불일치 대비)
        fseek(f, 0, SEEK_END);
        long file_sz = ftell(f);
        fseek(f, 44, SEEK_SET);
        long data_bytes_actual = file_sz - 44;
        if(data_bytes_actual <= 0){ fclose(f); sa_computing.store(false); return; }
        int64_t n_samples = data_bytes_actual / (int64_t)(2*sizeof(int16_t));

        // FFT size 자동 축소: n_samples에 맞는 가장 큰 2의 거듭제곱
        int actual_fft_n = fft_n;
        while(actual_fft_n > 64 && n_samples < (int64_t)actual_fft_n)
            actual_fft_n >>= 1;
        if(n_samples < (int64_t)actual_fft_n){ fclose(f); sa_computing.store(false); return; }

        // 전체 IQ 로드
        std::vector<int16_t> raw(n_samples*2);
        fread(raw.data(), sizeof(int16_t), (size_t)(n_samples*2), f);
        fclose(f);

        // ── FFTW 플랜 ─────────────────────────────────────────────────────
        std::vector<float> in_f(actual_fft_n*2);
        fftwf_complex* out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*actual_fft_n);
        fftwf_plan plan = fftwf_plan_dft_1d(actual_fft_n,
                              (fftwf_complex*)in_f.data(), out,
                              FFTW_FORWARD, FFTW_ESTIMATE);

        int64_t hop  = actual_fft_n;
        int64_t rows = n_samples / hop;
        // rows 부족 시 50% 오버랩으로 보완
        if(rows < 8 && actual_fft_n > 1){
            hop  = std::max((int64_t)1, hop / 2);
            rows = (n_samples - actual_fft_n) / hop + 1;
        }
        if(rows < 1){ fftwf_destroy_plan(plan); fftwf_free(out); sa_computing.store(false); return; }

        // ── power 누산 → normalize 범위 파악 ─────────────────────────────
        // 첫 패스: min/max dB 파악
        std::vector<float> all_db(rows * actual_fft_n);
        const float scale = 1.0f / (32768.0f * actual_fft_n);

        for(int64_t r=0; r<rows; r++){
            for(int i=0;i<actual_fft_n;i++){
                in_f[i*2  ] = raw[(r*hop+i)*2  ] * scale;
                in_f[i*2+1] = raw[(r*hop+i)*2+1] * scale;
            }
            sa_hann(in_f.data(), actual_fft_n);
            fftwf_execute(plan);

            int half = actual_fft_n/2;
            // FFT shift: 음수 주파수 → 양수
            for(int i=0;i<actual_fft_n;i++){
                int bin = (i + half) % actual_fft_n;
                float re=out[bin][0], im=out[bin][1];
                float ms = re*re+im*im+1e-12f;
                all_db[r*actual_fft_n+i] = 10.0f*log10f(ms);
            }
            // DC bin 억제
            all_db[r*actual_fft_n + half] = all_db[r*actual_fft_n + half - 1];
        }

        fftwf_destroy_plan(plan); fftwf_free(out);

        // ── 히스토그램 이퀄라이제이션으로 대비 극대화 ───────────────────
        // 1) dB 범위 파악 (1st~99th percentile)
        std::vector<float> sorted_db(all_db);
        std::sort(sorted_db.begin(), sorted_db.end());
        float db_lo = sorted_db[(size_t)(sorted_db.size()*0.01f)];
        float db_hi = sorted_db[(size_t)(sorted_db.size()*0.99f)];
        if(db_hi - db_lo < 1.0f) db_hi = db_lo + 1.0f;

        // 2) 256-bin 히스토그램 누적분포함수(CDF) 계산
        const int BINS = 256;
        std::vector<int> hist(BINS, 0);
        float db_rng_inv = 1.0f / (db_hi - db_lo);
        for(float v : all_db){
            int b = (int)((v - db_lo) * db_rng_inv * (BINS-1));
            b = b<0?0:b>=BINS?BINS-1:b;
            hist[b]++;
        }
        // CDF 정규화 → 룩업테이블
        std::vector<float> cdf(BINS);
        cdf[0] = hist[0];
        for(int i=1;i<BINS;i++) cdf[i] = cdf[i-1] + hist[i];
        float cdf_min = *std::min_element(cdf.begin(), cdf.end());
        float cdf_rng = cdf[BINS-1] - cdf_min;
        if(cdf_rng < 1.0f) cdf_rng = 1.0f;
        std::vector<float> lut(BINS);
        for(int i=0;i<BINS;i++) lut[i] = (cdf[i]-cdf_min)/cdf_rng;

        // ── 픽셀 버퍼 생성 (히스토그램 이퀄라이제이션 LUT 적용) ──────────
        // OpenGL 최대 텍스처 크기 쿼리 (일반적으로 16384)
        GLint max_tex = 16384;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_tex);

        // rows가 최대 텍스처 높이 초과 시 행 병합 (시간축 다운샘플)
        int64_t out_rows = rows;
        int merge = 1;
        while(out_rows > (int64_t)max_tex){
            merge++;
            out_rows = rows / merge;
        }
        if(out_rows < 1) out_rows = 1;

        std::vector<uint32_t> pixels(out_rows * actual_fft_n);
        for(int64_t r=0; r<out_rows; r++){
            for(int i=0;i<actual_fft_n;i++){
                // merge행 평균 dB → LUT
                float db_avg = 0.0f;
                int cnt = 0;
                for(int m=0; m<merge; m++){
                    int64_t src_r = r*merge + m;
                    if(src_r >= rows) break;
                    db_avg += all_db[src_r*actual_fft_n+i];
                    cnt++;
                }
                if(cnt > 0) db_avg /= cnt;
                int b = (int)((db_avg - db_lo) * db_rng_inv * (BINS-1));
                b = b<0?0:b>=BINS?BINS-1:b;
                pixels[r*actual_fft_n+i] = sa_jet(lut[b]);
            }
        }

        // 메인 스레드로 전달
        {
            std::lock_guard<std::mutex> lk(sa_pixel_mtx);
            sa_pixel_buf = std::move(pixels);
            sa_tex_w = actual_fft_n;
            sa_tex_h = (int)out_rows;
        }
        sa_pixel_ready.store(true);
        sa_computing.store(false);
    });
}