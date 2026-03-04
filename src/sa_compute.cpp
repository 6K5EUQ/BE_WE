#include "fft_viewer.hpp"
#include "audio.hpp"
#include "bewe_paths.hpp"
#include <fftw3.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <thread>

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
    sa_playing.store(false);  // 재생 중이면 중단
    if(sa_play_thread.joinable()) sa_play_thread.join();
    if(sa_thread.joinable()) sa_thread.join();

    sa_temp_path.clear();
    if(sa_texture){ glDeleteTextures(1,&sa_texture); sa_texture=0; }
    sa_tex_w=0; sa_tex_h=0;
    sa_pixel_ready.store(false);
    {
        std::lock_guard<std::mutex> lk(sa_pixel_mtx);
        sa_pixel_buf.clear();
    }
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
        // ── WAV 읽기 + bewe 청크 파싱 ────────────────────────────────────
        FILE* f = fopen(wav_path.c_str(),"rb");
        if(!f){ sa_computing.store(false); return; }

        // RIFF 헤더 파싱: 청크를 순회하여 data 오프셋과 bewe 메타데이터 추출
        uint64_t meta_cf_hz  = 0;
        int64_t  meta_time   = 0;
        uint32_t meta_sr     = 0;
        long     data_offset = 44; // 기본값 (표준 44바이트)
        long     data_size   = 0;

        // fmt chunk의 sample_rate 읽기 (offset 24, uint32)
        fseek(f, 24, SEEK_SET);
        uint32_t wav_sr = 0; fread(&wav_sr, 4, 1, f);

        // 청크 순회: "RIFF"(4) + size(4) + "WAVE"(4) = 12바이트 이후부터
        fseek(f, 12, SEEK_SET);
        while(true){
            char id[5]={};
            uint32_t csz = 0;
            if(fread(id, 1, 4, f) != 4) break;
            if(fread(&csz, 4, 1, f) != 1) break;
            long chunk_data_pos = ftell(f);
            if(strncmp(id, "data", 4) == 0){
                data_offset = chunk_data_pos;
                data_size   = (long)csz;
            } else if(strncmp(id, "bewe", 4) == 0 && csz >= 20){
                fread(&meta_cf_hz, 8, 1, f);
                fread(&meta_time,  8, 1, f);
                fread(&meta_sr,    4, 1, f);
            }
            // 다음 청크로 이동 (짝수 정렬)
            long next = chunk_data_pos + (long)csz + ((long)csz & 1);
            if(fseek(f, next, SEEK_SET) != 0) break;
        }
        if(data_size <= 0){
            // 파싱 실패 시 파일 크기로 추정
            fseek(f, 0, SEEK_END);
            long file_sz = ftell(f);
            data_offset = 44;
            data_size   = file_sz - 44;
        }
        if(meta_sr == 0) meta_sr = wav_sr;

        // SA 메타데이터 저장
        sa_center_freq_hz = meta_cf_hz;
        sa_start_time     = meta_time;
        sa_sample_rate    = meta_sr > 0 ? meta_sr : wav_sr;

        fseek(f, data_offset, SEEK_SET);
        long data_bytes_actual = data_size;
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
            sa_pixel_buf   = std::move(pixels);
            sa_tex_w       = actual_fft_n;
            sa_tex_h       = (int)out_rows;
        }
        // SA 좌표 메타데이터 저장 (뷰 계산에 사용)
        sa_total_rows   = rows;    // 실제 FFT 행 수 (시간축)
        sa_actual_fft_n = actual_fft_n;
        // 뷰 리셋 (새 SA 로드 시)
        sa_view_x0 = 0.0f; sa_view_x1 = 1.0f;
        sa_view_y0 = 0.0f; sa_view_y1 = 1.0f;
        sa_sel_active = false;
        sa_pixel_ready.store(true);
        sa_computing.store(false);
    });
}

// ── SA 선택 영역 복조 재생 ─────────────────────────────────────────────────────
// 선택된 UV 범위 → 시간축 샘플 추출 → 주파수 mix-down + 복조 → ALSA 재생
void FFTViewer::sa_play_demod(){
    sa_playing.store(true);

    // 필요한 파라미터 스냅샷 (스레드 안전)
    std::string wav_path = sa_temp_path;
    uint64_t cf_hz       = sa_center_freq_hz;
    uint32_t wav_sr      = sa_sample_rate;
    int      demod_mode  = sa_demod_mode;
    float    sel_y0      = sa_sel_y0;
    float    sel_y1      = sa_sel_y1;
    float    sel_x0      = sa_sel_x0;
    float    sel_x1      = sa_sel_x1;
    int64_t  total_rows  = sa_total_rows;
    int      fft_n       = sa_actual_fft_n;

    if(wav_path.empty() || wav_sr == 0 || total_rows == 0 || fft_n == 0){
        sa_playing.store(false); return;
    }

    // WAV 파일 열기 + data 오프셋 파싱
    FILE* f = fopen(wav_path.c_str(), "rb");
    if(!f){ sa_playing.store(false); return; }

    long data_offset = 44;
    fseek(f, 12, SEEK_SET);
    while(true){
        char id[5]={};
        uint32_t csz=0;
        if(fread(id,1,4,f)!=4) break;
        if(fread(&csz,4,1,f)!=1) break;
        long cdp = ftell(f);
        if(strncmp(id,"data",4)==0){ data_offset=cdp; break; }
        fseek(f, cdp+(long)csz+((long)csz&1), SEEK_SET);
    }

    // 선택된 시간 범위 → 샘플 범위
    // total_rows 행 × fft_n = 전체 샘플 (50% 오버랩 없이, 근사치)
    int64_t total_samp = (int64_t)total_rows * (int64_t)fft_n;
    int64_t samp_start = (int64_t)(sel_y0 * total_samp);
    int64_t samp_end   = (int64_t)(sel_y1 * total_samp);
    if(samp_start >= samp_end){ fclose(f); sa_playing.store(false); return; }

    // 선택된 주파수 범위 → mix-down 중심 주파수
    // sel_x0..x1은 텍스처 UV: 0 = cf - sr/2, 1 = cf + sr/2
    double bw_hz = (double)wav_sr;
    double sel_cf_hz  = (double)cf_hz - bw_hz * 0.5
                      + ((sel_x0 + sel_x1) * 0.5) * bw_hz;
    double offset_hz  = sel_cf_hz - (double)cf_hz;
    // 선택 대역폭 (데시메이션용)
    double sel_bw_hz  = (sel_x1 - sel_x0) * bw_hz;
    if(sel_bw_hz < 1000.0) sel_bw_hz = 1000.0;

    int decim = std::max(1, (int)(bw_hz / sel_bw_hz));
    uint32_t out_sr = wav_sr / decim;
    if(out_sr < 4000) { out_sr = 4000; decim = (int)(wav_sr / out_sr); }

    // 읽기: 선택 범위 IQ 샘플
    int64_t n_read = samp_end - samp_start;
    fseek(f, data_offset + (long)(samp_start * 4), SEEK_SET); // 4 = 2ch * int16
    std::vector<int16_t> raw(n_read * 2);
    int64_t n_got = (int64_t)fread(raw.data(), sizeof(int16_t), (size_t)(n_read*2), f) / 2;
    fclose(f);
    if(n_got < 1){ sa_playing.store(false); return; }

    // Mix-down + 박스필터 데시메이션 + 복조
    const float SCL = 1.0f / 32768.0f;
    float phase = 0.0f;
    float dphi  = (float)(2.0 * M_PI * (-offset_hz) / (double)wav_sr); // mix to baseband
    float prev_phase_fm = 0.0f; // FM 이전 위상 (미분용)

    int64_t n_out = n_got / decim;
    std::vector<int16_t> out_buf(n_out * 2); // stereo
    int64_t oi = 0;
    for(int64_t i = 0; i < n_got && oi < n_out; i += decim){
        // box-filter average over decim samples
        float sum_i = 0.f, sum_q = 0.f;
        int cnt = 0;
        for(int d = 0; d < decim && (i+d) < n_got; d++){
            int64_t idx = i + d;
            float si = raw[idx*2  ] * SCL;
            float sq = raw[idx*2+1] * SCL;
            // mix-down: multiply by e^(-j*phase)
            float cp = cosf(phase + d * dphi);
            float sp = sinf(phase + d * dphi);
            sum_i += si * cp + sq * sp;
            sum_q += -si * sp + sq * cp;
            cnt++;
        }
        phase += decim * dphi;
        // wrap phase to [-π,π]
        while(phase >  (float)M_PI) phase -= 2.f*(float)M_PI;
        while(phase < -(float)M_PI) phase += 2.f*(float)M_PI;
        if(cnt == 0) continue;
        float mi = sum_i / cnt, mq = sum_q / cnt;

        float audio = 0.f;
        if(demod_mode == 1){ // AM: envelope
            audio = sqrtf(mi*mi + mq*mq);
            audio = audio * 2.f - 1.f; // 기준선 제거 (대략)
        } else { // FM: phase diff
            float cur_phase = atan2f(mq, mi);
            float diff = cur_phase - prev_phase_fm;
            if(diff >  (float)M_PI) diff -= 2.f*(float)M_PI;
            if(diff < -(float)M_PI) diff += 2.f*(float)M_PI;
            audio = diff * (float)(wav_sr / decim) / (float)(2.0 * M_PI * 75000.0);
            prev_phase_fm = cur_phase;
        }
        audio = audio > 1.f ? 1.f : audio < -1.f ? -1.f : audio;
        int16_t s = (int16_t)(audio * 28000.f);
        out_buf[oi*2  ] = s;
        out_buf[oi*2+1] = s;
        oi++;
    }

    // ALSA 재생
    AlsaOut alsa;
    if(alsa.open(out_sr)){
        const int BLOCK = 512;
        for(int64_t b = 0; b < oi && sa_playing.load(); b += BLOCK){
            int frames = (int)std::min((int64_t)BLOCK, oi - b);
            alsa.write(out_buf.data() + b*2, frames);
        }
        alsa.close();
    }
    sa_playing.store(false);
}