#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include <unistd.h>
#include <sys/stat.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>

// ─────────────────────────────────────────────────────────────────────────────
// Kaiser FIR LPF 생성
// cutoff_norm: 정규화 컷오프 (0~0.5, 0.5=Nyquist)
// beta: Kaiser 윈도우 파라미터 (6=~44dB, 8=~58dB, 10=~74dB stopband)
// ─────────────────────────────────────────────────────────────────────────────
static std::vector<float> make_kaiser_lpf(int ntaps, double cutoff_norm, double beta)
{
    // Modified Bessel function I0
    auto I0 = [](double x) -> double {
        double sum = 1.0, term = 1.0;
        for(int k = 1; k <= 30; k++){
            term *= (x/2.0/k) * (x/2.0/k);
            sum += term;
            if(term < 1e-12 * sum) break;
        }
        return sum;
    };

    int M = ntaps - 1;
    double I0b = I0(beta);
    std::vector<float> h(ntaps);
    double sum = 0.0;
    for(int n = 0; n < ntaps; n++){
        double nd = n - M/2.0;
        // Sinc (LPF ideal)
        double sinc = (nd == 0.0) ? 2.0*cutoff_norm
                                  : sin(2.0*M_PI*cutoff_norm*nd) / (M_PI*nd);
        // Kaiser window
        double r = 2.0*n/M - 1.0;
        double w = I0(beta * sqrt(std::max(0.0, 1.0 - r*r))) / I0b;
        h[n] = (float)(sinc * w);
        sum += h[n];
    }
    // 정규화
    for(auto& v : h) v /= (float)sum;
    return h;
}

// ─────────────────────────────────────────────────────────────────────────────
// WAV 헤더 작성 (stereo int16: L=I, R=Q)
// bewe 커스텀 청크: center_freq_hz(uint64) + start_time(int64) + sample_rate(uint32) = 20바이트
// ─────────────────────────────────────────────────────────────────────────────
static void write_wav_header(FILE* f, uint32_t sample_rate, uint32_t n_frames,
                              uint64_t center_freq_hz=0, int64_t start_time=0){
    uint32_t data_bytes  = n_frames * 2 * 2; // frames * channels * bytes
    // bewe 청크: "bewe"(4) + size(4) + payload(20) = 28
    const uint32_t BEWE_SIZE = 20;
    uint32_t chunk_size  = 36 + data_bytes + 4 + 4 + BEWE_SIZE;
    uint16_t audio_fmt   = 1;   // PCM
    uint16_t channels    = 2;
    uint32_t byte_rate   = sample_rate * 4;
    uint16_t block_align = 4;
    uint16_t bits        = 16;
    uint32_t subchunk2   = data_bytes;

    fwrite("RIFF",1,4,f); fwrite(&chunk_size,4,1,f);
    fwrite("WAVE",1,4,f);
    fwrite("fmt ",1,4,f);
    uint32_t sc1=16; fwrite(&sc1,4,1,f);
    fwrite(&audio_fmt,2,1,f); fwrite(&channels,2,1,f);
    fwrite(&sample_rate,4,1,f); fwrite(&byte_rate,4,1,f);
    fwrite(&block_align,2,1,f); fwrite(&bits,2,1,f);
    // bewe 청크 (data 앞에 배치 > 파서 안정성 향상)
    fwrite("bewe",1,4,f);
    uint32_t bewe_sz = BEWE_SIZE; fwrite(&bewe_sz,4,1,f);
    fwrite(&center_freq_hz,8,1,f);
    fwrite(&start_time,    8,1,f);
    fwrite(&sample_rate,   4,1,f);
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
    if(t_start <= 0) t_start = time(nullptr);
    if(t_end   <= 0) t_end   = time(nullptr);
    struct tm *ts=localtime(&t_start);
    char dts[32]; strftime(dts,sizeof(dts),"%b%d_%Y_%H%M%S",ts);
    struct tm *te=localtime(&t_end);
    char s_end[8]; strftime(s_end,sizeof(s_end),"%H%M%S",te);
    (void)bw_khz;

    snprintf(out, sz,
             "%s/IQ_%.3fMHz_%s-%s.wav",
             BEWEPaths::record_iq_dir().c_str(),
             (double)cf_mhz, dts, s_end);
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

std::string FFTViewer::do_region_save_work(){
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

    // Kaiser FIR LPF (데시메이션 비율에 따라 탭 수·beta 결정)
    // 컷오프: 0.45/decim (Nyquist의 90% → alias 억압 충분)
    // 탭 수: 최소 31, 데시메이션이 클수록 더 많은 탭 필요
    std::vector<float> fir_taps;
    std::vector<float> fir_state; // 지연 라인
    if(decim > 1){
        int ntaps = std::max(31, decim * 8 + 1);
        if(ntaps % 2 == 0) ntaps++;              // 홀수 탭 유지
        ntaps = std::min(ntaps, 1023);           // 상한 제한
        double cutoff_norm = 0.45 / decim;
        double beta = 8.0;                       // ~58dB stopband
        fir_taps  = make_kaiser_lpf(ntaps, cutoff_norm, beta);
        fir_state.assign((ntaps - 1) * 2, 0.0f); // 지연 라인 초기화 (I,Q 쌍)
    }

    // ── fft 행 인덱스 → 롤링 파일 샘플 위치 직접 참조 ──────────────────
    // row_write_pos[fi % MAX_FFTS_MEMORY] = 그 행이 끝나는 IQ 샘플 위치
    // time_t 기반 변환(초단위 오차)을 제거하고 row_write_pos 직접 사용
    if(tm_iq_write_sample <= 0){
        bewe_log_push(0,"[region_save] FAIL: tm_iq not started (write_sample=0)\n");
        return "";
    }

    // fft_top = 영역 위쪽(더 최근), fft_bot = 아래쪽(더 오래됨)
    // row_write_pos[top] = top 행 끝 IQ 샘플 위치 → samp_end 기준
    // row_write_pos[bot] = bot 행 끝 IQ 샘플 위치 → samp_start 기준
    int64_t snap_write = tm_iq_write_sample;
    time_t  snap_now   = time(nullptr);
    int64_t max_cap    = max_total;

    auto row_to_samp = [&](int fft_idx) -> int64_t {
        int slot = fft_idx % MAX_FFTS_MEMORY;
        int64_t pos = row_write_pos[slot];
        // row_write_pos가 0이면 아직 기록 안 됨 → snap_write 기반 fallback
        if(pos <= 0) return -1;
        return pos;
    };

    int64_t samp_start = -1, samp_end = -1;

    if(region.samp_start > 0 && region.samp_end > 0){
        // HOST IQ 좌표 직접 지정 (JOIN이 row_write_pos 기반으로 계산) → 지연 0
        samp_start = region.samp_start;
        samp_end   = region.samp_end;
    } else if(region.time_start_ms > 0 && region.time_end_ms > 0){
        // 절대 wall_time_ms → 샘플 위치 (JOIN 요청 시 가장 정확, ms 정밀도)
        int64_t snap_now_ms = (int64_t)snap_now * 1000LL;
        samp_start = snap_write - (snap_now_ms - region.time_start_ms) * (int64_t)sr / 1000LL;
        samp_end   = snap_write - (snap_now_ms - region.time_end_ms)   * (int64_t)sr / 1000LL;
    } else {
        // row_write_pos 기반 (HOST 자체 요청)
        samp_end   = row_to_samp(region.fft_top);
        samp_start = row_to_samp(region.fft_bot);
        if(samp_end < 0 || samp_start < 0){
            // row_wall_ms 사용 (ms 정밀도 fallback)
            auto ts2samp_ms = [&](int64_t ts_ms) -> int64_t {
                int64_t snap_now_ms = (int64_t)snap_now * 1000LL;
                return snap_write - (snap_now_ms - ts_ms) * (int64_t)sr / 1000LL;
            };
            if(samp_start < 0) samp_start = (region.time_start_ms > 0) ? ts2samp_ms(region.time_start_ms) : -1;
            if(samp_end   < 0) samp_end   = (region.time_end_ms > 0) ? ts2samp_ms(region.time_end_ms) : -1;
        }
    }

    if(samp_start > samp_end) std::swap(samp_start, samp_end);

    // 롤링 파일 유효 범위 클램프
    int64_t valid_start = (snap_write >= max_cap) ? snap_write - max_cap : 0;
    samp_start = std::max(samp_start, valid_start);
    samp_end   = std::min(samp_end,   snap_write);

    bewe_log_push(0,"[region_save] samp_start=%lld samp_end=%lld snap_write=%lld valid_start=%lld max_total=%lld sr=%u decim=%d\n",
                   (long long)samp_start,(long long)samp_end,
                   (long long)snap_write,(long long)valid_start,
                   (long long)max_total, sr, decim);
    bewe_log_push(0,"[region_save] n_in=%lld n_out=%lld sec=%.1f fft_top=%d fft_bot=%d\n",
                   (long long)(samp_end-samp_start), (long long)((samp_end-samp_start)/decim),
                   (double)(samp_end-samp_start)/(double)sr,
                   region.fft_top, region.fft_bot);
    if(samp_end <= samp_start){
        bewe_log_push(0,"[region_save] FAIL: no valid IQ data in range\n");
        return "";
    }

    int64_t n_in = samp_end - samp_start;
    int64_t n_out = n_in / decim;
    if(n_out < 1){ return ""; }

    // ── 출력 파일 열기 ─────────────────────────────────────────────────────
    char outpath[512];
    make_filename(outpath, sizeof(outpath),
                  cf_abs_mhz, bw_khz,
                  region.time_start_ms / 1000, region.time_end_ms / 1000);
    FILE* wf = fopen(outpath, "wb");
    if(!wf){
        bewe_log_push(0,"[region_save] FAIL: fopen failed: %s\n", outpath);
        return "";
    }
    // WAV 헤더 작성 (bewe 청크 포함: 중심주파수, 시작시각)
    uint64_t cf_hz_meta = (uint64_t)(cf_abs_mhz * 1e6 + 0.5);
    int64_t  t_start_meta = region.time_start_ms / 1000LL;  // 메타데이터는 초 단위 유지
    write_wav_header(wf, out_sr, (uint32_t)n_out, cf_hz_meta, t_start_meta);

    // ── 청크 단위 읽기 + mix-down + decimate + 저장 ───────────────────────
    const int CHUNK = 65536; // 샘플 단위
    std::vector<int16_t> in_buf(CHUNK * 2);
    std::vector<int16_t> out_buf;
    out_buf.reserve(CHUNK / decim * 2 + 4);

    double phase = 0.0;
    double phase_inc = -2.0 * M_PI * (double)offset_hz / (double)sr;

    int64_t actual_out = 0;
    int64_t pos = samp_start;

    // FIR 지연 라인 인덱스 및 입력 카운터 (데시메이션용)
    int fir_ntaps   = fir_taps.empty() ? 0 : (int)fir_taps.size();
    int fir_dly_pos = 0;  // 순환 버퍼 인덱스
    int decim_cnt   = 0;  // 데시메이션 카운터

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

            if(decim <= 1){
                // 데시메이션 없음 → 그대로 출력
                int16_t wi = (int16_t)(std::max(-1.0f,std::min(1.0f,mi))*32767.0f);
                int16_t wq = (int16_t)(std::max(-1.0f,std::min(1.0f,mq))*32767.0f);
                out_buf.push_back(wi);
                out_buf.push_back(wq);
                actual_out++;
            } else {
                // Kaiser FIR LPF → 데시메이션
                // 지연 라인에 새 복소수 샘플 삽입 (I와 Q 각각 별도 처리, 같은 지연 라인 공유 불가)
                // → 지연 라인을 복소수(I,Q 쌍)로 관리
                // fir_state: [I0,Q0, I1,Q1, ...] 크기 = (ntaps-1)*2
                int dly_idx = fir_dly_pos * 2;
                fir_state[dly_idx    ] = mi;
                fir_state[dly_idx + 1] = mq;
                fir_dly_pos = (fir_dly_pos + 1) % (fir_ntaps - 1);

                decim_cnt++;
                if(decim_cnt >= decim){
                    decim_cnt = 0;
                    // FIR 합성: 현재 샘플 포함 ntaps개
                    float oi = mi * fir_taps[0];
                    float oq = mq * fir_taps[0];
                    int dly_sz = fir_ntaps - 1;
                    for(int t = 1; t < fir_ntaps; t++){
                        int idx = ((fir_dly_pos - 1 - t + dly_sz * 2) % dly_sz) * 2;
                        oi += fir_state[idx    ] * fir_taps[t];
                        oq += fir_state[idx + 1] * fir_taps[t];
                    }
                    int16_t wi = (int16_t)(std::max(-1.0f,std::min(1.0f,oi))*32767.0f);
                    int16_t wq = (int16_t)(std::max(-1.0f,std::min(1.0f,oq))*32767.0f);
                    out_buf.push_back(wi);
                    out_buf.push_back(wq);
                    actual_out++;
                }
            }
        }
        if(!out_buf.empty())
            fwrite(out_buf.data(), sizeof(int16_t), out_buf.size(), wf);

        pos += to_read;
    }

    // WAV 헤더 실제 샘플 수로 갱신 (bewe 메타데이터 유지)
    rewind(wf);
    write_wav_header(wf, out_sr, (uint32_t)actual_out, cf_hz_meta, t_start_meta);
    fclose(wf);

    bewe_log_push(0,"Region IQ saved: %s  (%.1f sec  %.0f kHz SR)\n",
           outpath, (double)actual_out/out_sr, (double)out_sr/1000.0);

    // file_xfers에 추가
    {
        std::lock_guard<std::mutex> lk(file_xfer_mtx);
        bool updated=false;
        for(auto& xf : file_xfers){
            if(!xf.finished){
                xf.filename  = (strrchr(outpath,'/')?strrchr(outpath,'/')+1:outpath);
                xf.local_path= outpath;
                xf.finished  = true;
                xf.is_sa     = true;
                updated=true; break;
            }
        }
        if(!updated){
            FileXfer xf{};
            xf.filename   = (strrchr(outpath,'/')?strrchr(outpath,'/')+1:outpath);
            xf.local_path = outpath;
            xf.finished   = true;
            xf.is_sa      = true;
            file_xfers.push_back(xf);
        }
    }
    // rec_entries에 완료 항목 추가 (SA 모드 아닌 경우만)
    if(!sa_mode){
        std::lock_guard<std::mutex> lk(rec_entries_mtx);
        const char* bn = strrchr(outpath,'/');
        RecEntry e{};
        e.path     = outpath;
        e.filename = bn ? bn+1 : outpath;
        e.finished = true;
        e.is_audio = false;
        e.is_region= false;
        e.t_start  = std::chrono::steady_clock::now();
        rec_entries.push_back(e);
    }

    // SA 모드: 저장 완료 후 SA 워터폴 계산 시작
    if(sa_mode){
        sa_mode = false;
        sa_temp_path = outpath;
        sa_start(outpath);
    } else {
        region.active = false;
    }
    return outpath;
}