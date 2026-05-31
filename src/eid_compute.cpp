#include "fft_viewer.hpp"
#include "sigmf.hpp"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <sys/stat.h>

// ── EID envelope 추출 (비동기 스레드) ────────────────────────────────────────
void FFTViewer::eid_start(const std::string& wav_path){
    if(eid_thread.joinable()) eid_thread.join();
    eid_computing.store(true);
    eid_data_ready.store(false);
    sa_temp_path = wav_path;  // FFT 콤보에서 재계산할 때 사용

    eid_thread = std::thread([this, wav_path](){
        // ── IQ/audio 소스 열기 (SigMF .sigmf-data 또는 legacy .wav + bewe) ──
        SigMF::Source src;
        if(!SigMF::open_source(wav_path, src)){ eid_computing.store(false); return; }
        FILE* f = src.f;   // open_source가 데이터 시작 위치로 seek 완료
        uint32_t meta_sr    = src.sample_rate;
        uint32_t wav_sr     = src.sample_rate;   // alias (derived 계산 fallback)
        uint64_t meta_cf_hz = src.center_freq_hz;
        int64_t  meta_time  = src.start_unix;
        long     data_size  = src.data_size;

        const int nch = src.nch;  // 1=mono(audio), 2=stereo(IQ)
        int64_t n_samples = data_size / (int64_t)(nch * (int)sizeof(int16_t));
        if(n_samples < 1){ fclose(f); eid_computing.store(false); return; }

        // ── WAV 로드 + envelope/I/Q/phase/freq 추출 ─────────────────────────
        // mono: 오디오로 취급 (I=sample, Q=0, env=|sample|, phase/freq는 의미 없음)
        // stereo: IQ로 취급 (기존 로직)
        const int64_t BLOCK = 65536;
        std::vector<float> env(n_samples), ch_i(n_samples), ch_q(n_samples);
        std::vector<float> phase(n_samples), inst_freq(n_samples);
        std::vector<int16_t> raw(BLOCK * nch);
        int64_t done = 0;
        const float SCL = 1.0f / 32768.0f;
        float prev_phase = 0.f;
        const float TWO_PI = 6.283185307f;

        while(done < n_samples){
            int64_t todo = std::min(BLOCK, n_samples - done);
            int64_t got = (int64_t)fread(raw.data(), sizeof(int16_t), (size_t)(todo * nch), f) / nch;
            if(got <= 0) break;
            for(int64_t i = 0; i < got; i++){
                int64_t idx = done + i;
                if(nch == 2){
                    float fi = raw[i * 2    ] * SCL;
                    float fq = raw[i * 2 + 1] * SCL;
                    env[idx]  = sqrtf(fi * fi + fq * fq);
                    ch_i[idx] = fi;
                    ch_q[idx] = fq;
                    float ph  = atan2f(fq, fi);
                    phase[idx] = ph;
                    float dp = ph - prev_phase;
                    if(dp >  3.14159265f) dp -= TWO_PI;
                    if(dp < -3.14159265f) dp += TWO_PI;
                    inst_freq[idx] = (idx == 0) ? 0.f : dp;
                    prev_phase = ph;
                } else {
                    float s = raw[i] * SCL;
                    env[idx]       = fabsf(s);
                    ch_i[idx]      = s;
                    ch_q[idx]      = 0.f;
                    phase[idx]     = 0.f;
                    inst_freq[idx] = 0.f;
                }
            }
            done += got;
        }
        fclose(f);
        if(done < 1){ eid_computing.store(false); return; }
        env.resize(done); ch_i.resize(done); ch_q.resize(done);
        phase.resize(done); inst_freq.resize(done);

        // inst_freq를 Hz 단위로 변환 (sr / 2π * dp)
        {
            uint32_t sr_val = meta_sr > 0 ? meta_sr : wav_sr;
            float scale = (float)sr_val / TWO_PI;
            for(int64_t i = 0; i < done; i++) inst_freq[i] *= scale;
        }

        // ── 자동 스케일: 1st~99th percentile ────────────────────────────────
        std::vector<float> sorted_env(env);
        std::sort(sorted_env.begin(), sorted_env.end());
        float amp_lo = sorted_env[(size_t)(sorted_env.size() * 0.01f)];
        float amp_hi = sorted_env.back();  // 실제 최대값 (클리핑 방지)
        if(amp_hi - amp_lo < 0.001f) amp_hi = amp_lo + 0.001f;
        float margin_lo = (amp_hi - amp_lo) * 0.05f;
        float margin_hi = amp_hi * 0.20f;
        amp_lo -= margin_lo;
        amp_hi += margin_hi;
        if(amp_lo < 0.f) amp_lo = 0.f;

        // 노이즈 레벨: 5th percentile
        float noise_lvl = sorted_env[(size_t)(sorted_env.size() * 0.05f)];

        // inst_freq Y범위 초기화
        float freq_lo = inst_freq[0], freq_hi = freq_lo;
        { int64_t step = std::max((int64_t)1, done/4000);
          for(int64_t i = 0; i < done; i += step){
              if(inst_freq[i] < freq_lo) freq_lo = inst_freq[i];
              if(inst_freq[i] > freq_hi) freq_hi = inst_freq[i];
          }
          float fm = (freq_hi - freq_lo) * 0.05f;
          freq_lo -= fm; freq_hi += fm;
        }

        // ── 데이터 전달 ─────────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(eid_data_mtx);
            eid_envelope      = std::move(env);
            eid_ch_i          = std::move(ch_i);
            eid_ch_q          = std::move(ch_q);
            eid_phase         = std::move(phase);
            eid_inst_freq     = std::move(inst_freq);
            eid_total_samples = done;
            eid_sample_rate   = meta_sr > 0 ? meta_sr : wav_sr;
            eid_is_iq         = (nch == 2);   // stereo = IQ → Audio 탭에서 AM/FM 복조 가능
        }
        eid_view_t0        = 0.0;
        eid_view_t1        = (double)done;
        eid_amp_min        = amp_lo;
        eid_amp_max        = amp_hi;
        eid_y_min[0]       = amp_lo;
        eid_y_max[0]       = amp_hi;
        eid_y_min[3]       = freq_lo;
        eid_y_max[3]       = freq_hi;
        eid_noise_level    = noise_lvl;
        eid_center_freq_hz = meta_cf_hz;
        eid_start_time_meta = meta_time;
        eid_view_mode      = 0; // reset to Signal on new load
        eid_phase_detrend_hz = 0.0f;
        eid_data_ready.store(true);
        eid_computing.store(false);
        bewe_log("EID: loaded %lld samples, sr=%u, cf=%llu\n",
                 (long long)done, eid_sample_rate, (unsigned long long)meta_cf_hz);
    });
}

// ── 태그 내 펄스 자동 분석 (envelope rising edge 기반) ──────────────────────
void FFTViewer::eid_auto_analyze_tag(EidTag& tag){
    tag.auto_pri_us = 0; tag.auto_prf_hz = 0; tag.auto_pulse_count = 0;
    int64_t i0 = std::max((int64_t)0, (int64_t)tag.s0);
    int64_t i1 = std::min(eid_total_samples, (int64_t)ceil(tag.s1));
    if(i1 - i0 < 4 || eid_envelope.empty()) return;

    // threshold: 구간 내 median * 비율
    std::vector<float> seg(eid_envelope.begin()+i0, eid_envelope.begin()+i1);
    std::sort(seg.begin(), seg.end());
    float median = seg[seg.size()/2];
    float thr = median * 1.5f;
    if(thr < eid_noise_level * 1.2f) thr = eid_noise_level * 1.2f;

    // rising edge 검출
    std::vector<int64_t> edges;
    bool above = false;
    for(int64_t i = i0; i < i1; i++){
        if(!above && eid_envelope[i] >= thr){ above = true; edges.push_back(i); }
        else if(above && eid_envelope[i] < thr * 0.7f) above = false;
    }

    tag.auto_pulse_count = (int)edges.size();
    if(edges.size() < 2) return;

    // 인접 edge 간격의 중앙값
    std::vector<float> intervals;
    for(size_t j = 1; j < edges.size(); j++)
        intervals.push_back((float)(edges[j] - edges[j-1]));
    std::sort(intervals.begin(), intervals.end());
    float median_interval = intervals[intervals.size()/2];

    uint32_t sr = eid_sample_rate > 0 ? eid_sample_rate : 1;
    tag.auto_pri_us = median_interval / sr * 1e6f;
    tag.auto_prf_hz = (float)sr / median_interval;
}

// ── ch_i/ch_q 로부터 envelope/phase/inst_freq 재계산 ─────────────────────────
void FFTViewer::eid_recompute_derived(){
    eid_edit_gen++;   // IQ 수정됨 → Audio 탭 복조 캐시 무효화
    std::lock_guard<std::mutex> lk(eid_data_mtx);
    int64_t n = eid_total_samples;
    if(n <= 0 || (int64_t)eid_ch_i.size() < n) return;
    eid_envelope.resize(n);
    eid_phase.resize(n);
    eid_inst_freq.resize(n);
    float prev_ph = 0.f;
    const float TWO_PI = 6.283185307f;
    for(int64_t i = 0; i < n; i++){
        float fi = eid_ch_i[i], fq = eid_ch_q[i];
        eid_envelope[i] = sqrtf(fi*fi + fq*fq);
        float ph = atan2f(fq, fi);
        eid_phase[i] = ph;
        float dp = ph - prev_ph;
        if(dp >  3.14159265f) dp -= TWO_PI;
        if(dp < -3.14159265f) dp += TWO_PI;
        eid_inst_freq[i] = (i == 0) ? 0.f : dp;
        prev_ph = ph;
    }
    float scale = (float)eid_sample_rate / TWO_PI;
    for(int64_t i = 0; i < n; i++) eid_inst_freq[i] *= scale;

    // auto-scale 재계산
    std::vector<float> sorted_env(eid_envelope);
    std::sort(sorted_env.begin(), sorted_env.end());
    float lo = sorted_env[(size_t)(sorted_env.size()*0.01f)];
    float hi = sorted_env.back();
    if(hi-lo<0.001f) hi=lo+0.001f;
    float margin_lo = (hi-lo)*0.05f;
    float margin_hi = hi*0.20f;
    eid_amp_min = std::max(0.f, lo-margin_lo);
    eid_amp_max = hi+margin_hi;
    eid_y_min[0] = eid_amp_min;
    eid_y_max[0] = eid_amp_max;
    eid_noise_level = sorted_env[(size_t)(sorted_env.size()*0.05f)];
}

// ── FFT 기반 BPF (brick-wall, 블록 처리) ─────────────────────────────────────
// uv_lo/uv_hi: 스펙트로그램 주파수축 UV [0,1] (0=-sr/2, 0.5=DC, 1=+sr/2)
void FFTViewer::eid_apply_bpf(float uv_lo, float uv_hi){
    // 원본 백업 (첫 적용 시에만)
    if(!eid_bpf_active){
        eid_orig_ch_i = eid_ch_i;
        eid_orig_ch_q = eid_ch_q;
    }

    // 항상 원본에서 시작 (중첩 BPF 방지)
    std::vector<float> work_i = eid_orig_ch_i;
    std::vector<float> work_q = eid_orig_ch_q;
    int64_t n = (int64_t)work_i.size();
    if(n < 1) return;

    int fft_n = 65536;
    while(fft_n > n) fft_n >>= 1;
    if(fft_n < 64) fft_n = 64;

    fftwf_complex* in  = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*fft_n);
    fftwf_complex* out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*fft_n);
    fftwf_plan fwd = fftwf_plan_dft_1d(fft_n, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    fftwf_plan inv = fftwf_plan_dft_1d(fft_n, out, in, FFTW_BACKWARD, FFTW_ESTIMATE);

    eid_bpf_center_uv = (uv_lo + uv_hi) * 0.5f;  // 복조 시 이 중심으로 재중심(mix-down)
    double sr = (double)eid_sample_rate;
    // UV → baseband Hz: UV 0 = -sr/2, UV 0.5 = DC, UV 1 = +sr/2
    double bpf_lo = ((double)uv_lo - 0.5) * sr;
    double bpf_hi = ((double)uv_hi - 0.5) * sr;

    for(int64_t offset = 0; offset < n; offset += fft_n){
        int64_t block_n = std::min((int64_t)fft_n, n - offset);

        for(int64_t i = 0; i < block_n; i++){
            in[i][0] = work_i[offset+i];
            in[i][1] = work_q[offset+i];
        }
        for(int64_t i = block_n; i < fft_n; i++){
            in[i][0] = 0; in[i][1] = 0;
        }

        fftwf_execute(fwd);

        // 대역 외 bin 제로화
        for(int k = 0; k < fft_n; k++){
            double freq_hz = (k <= fft_n/2) ? (double)k*sr/fft_n : (double)(k-fft_n)*sr/fft_n;
            if(freq_hz < bpf_lo || freq_hz > bpf_hi){
                out[k][0] = 0; out[k][1] = 0;
            }
        }

        fftwf_execute(inv);

        float inv_n = 1.0f / fft_n;
        for(int64_t i = 0; i < block_n; i++){
            work_i[offset+i] = in[i][0] * inv_n;
            work_q[offset+i] = in[i][1] * inv_n;
        }
    }

    fftwf_destroy_plan(fwd); fftwf_destroy_plan(inv);
    fftwf_free(in); fftwf_free(out);

    {
        std::lock_guard<std::mutex> lk(eid_data_mtx);
        eid_ch_i = std::move(work_i);
        eid_ch_q = std::move(work_q);
    }
    eid_bpf_active = true;

    eid_recompute_derived();
    sa_recompute_from_iq();
}

void FFTViewer::eid_undo_bpf(){
    if(!eid_bpf_active) return;
    {
        std::lock_guard<std::mutex> lk(eid_data_mtx);
        eid_ch_i = eid_orig_ch_i;
        eid_ch_q = eid_orig_ch_q;
    }
    eid_bpf_active = false;
    eid_bpf_center_uv = 0.5f;   // BPF 해제 → 재중심 없음(DC)
    eid_recompute_derived();
    sa_recompute_from_iq();
}

// ─────────────────────────────────────────────────────────────────────────────
// Save File: 현재 EID 상태(필터·샘플 수정 반영)를 원본 폴더에 새 WAV로 저장
// stereo int16 (L=I, R=Q), bewe 메타 청크 보존, 원본 .info 있으면 복사
// ─────────────────────────────────────────────────────────────────────────────
namespace {
inline void eid_write_wav_header(FILE* f, uint32_t sample_rate, uint32_t n_frames,
                                  uint64_t center_freq_hz, int64_t start_time){
    uint32_t data_bytes  = n_frames * 2 * 2; // frames * channels(2) * bytes(2)
    const uint32_t BEWE_SIZE = 20;
    uint32_t chunk_size  = 36 + data_bytes + 4 + 4 + BEWE_SIZE;
    uint16_t audio_fmt   = 1;
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
    fwrite("bewe",1,4,f);
    uint32_t bewe_sz = BEWE_SIZE; fwrite(&bewe_sz,4,1,f);
    fwrite(&center_freq_hz,8,1,f);
    fwrite(&start_time,    8,1,f);
    fwrite(&sample_rate,   4,1,f);
    fwrite("data",1,4,f); fwrite(&subchunk2,4,1,f);
}
inline bool eid_path_exists(const std::string& p){
    struct stat st; return ::stat(p.c_str(), &st) == 0;
}
}

// 기본 저장 경로 계산: 원본 경로 기반 "IQ_Filtered_..." / "Audio_Filtered_..." 규칙
std::string FFTViewer::eid_default_filtered_path(){
    std::string src = sa_temp_path;
    if(src.empty()) return "";

    size_t slash = src.find_last_of('/');
    std::string dir  = (slash == std::string::npos) ? "." : src.substr(0, slash);
    std::string base = (slash == std::string::npos) ? src : src.substr(slash+1);

    size_t dot = base.find_last_of('.');
    std::string stem = (dot == std::string::npos) ? base : base.substr(0, dot);
    std::string ext  = (dot == std::string::npos) ? ""   : base.substr(dot);

    auto starts_with = [](const std::string& s, const char* p){
        size_t n = strlen(p); return s.size() >= n && s.compare(0,n,p) == 0;
    };
    std::string new_stem;
    if(starts_with(stem, "IQ_Filtered_") ||
       starts_with(stem, "Audio_Filtered_") ||
       starts_with(stem, "Filtered_"))           new_stem = stem;
    else if(starts_with(stem, "IQ_"))            new_stem = "IQ_Filtered_"    + stem.substr(3);
    else if(starts_with(stem, "Audio_"))         new_stem = "Audio_Filtered_" + stem.substr(6);
    else                                         new_stem = "Filtered_"       + stem;

    std::string out_path = dir + "/" + new_stem + ext;
    for(int n=2; eid_path_exists(out_path); n++){
        out_path = dir + "/" + new_stem + "_" + std::to_string(n) + ext;
        if(n > 9999) return "";
    }
    return out_path;
}

// 지정 경로에 WAV 저장 (.info는 호출자 책임)
std::string FFTViewer::eid_save_filtered_to(const std::string& out_path){
    if(out_path.empty()) return "";
    FILE* f = fopen(out_path.c_str(), "wb");
    if(!f){
        bewe_log_push(0, "[EID] Save File: fopen failed: %s\n", out_path.c_str());
        return "";
    }
    bool sig = SigMF::is_sigmf_data(out_path);
    uint32_t n_frames = 0;
    {
        std::lock_guard<std::mutex> lk(eid_data_mtx);
        n_frames = (uint32_t)std::min(eid_ch_i.size(), eid_ch_q.size());
        // .sigmf-data: raw(헤더 없음) / .wav: 기존 RIFF(stereo) 헤더
        if(!sig)
            eid_write_wav_header(f, eid_sample_rate, n_frames,
                                 eid_center_freq_hz, eid_start_time_meta);
        constexpr size_t CHUNK = 4096;
        std::vector<int16_t> buf(CHUNK * 2);
        for(size_t i = 0; i < n_frames; i += CHUNK){
            size_t n = std::min(CHUNK, (size_t)n_frames - i);
            for(size_t j = 0; j < n; j++){
                float fi = std::max(-1.0f, std::min(1.0f, eid_ch_i[i+j]));
                float fq = std::max(-1.0f, std::min(1.0f, eid_ch_q[i+j]));
                buf[j*2  ] = (int16_t)(fi * 32767.0f);
                buf[j*2+1] = (int16_t)(fq * 32767.0f);
            }
            fwrite(buf.data(), sizeof(int16_t), n*2, f);
        }
    }
    fclose(f);
    // SigMF: 원본 메타(.sigmf-meta) 필드 승계 + sr/cf/시각/duration 갱신
    if(sig){
        SigMF::Meta m;
        SigMF::read_meta(sa_temp_path, m);
        m.sample_rate    = eid_sample_rate;
        m.center_freq_hz = eid_center_freq_hz;
        m.start_unix     = eid_start_time_meta;
        m.duration_s     = (eid_sample_rate > 0) ? (double)n_frames / (double)eid_sample_rate : 0;
        SigMF::write_meta(out_path, m);
    }
    bewe_log_push(0, "[EID] Save File: %s\n", out_path.c_str());
    return out_path;
}

// 원본 폴더에 기본 파일명으로 저장 + 원본 .info 있으면 복사 (하위 호환)
std::string FFTViewer::eid_save_filtered(){
    std::string out_path = eid_default_filtered_path();
    if(out_path.empty()){
        bewe_log_push(0, "[EID] Save File: no source path\n");
        return "";
    }
    if(eid_save_filtered_to(out_path).empty()) return "";

    // 원본 .info 복사 (legacy .wav 만 — SigMF 는 eid_save_filtered_to 가 메타 작성)
    if(!SigMF::is_sigmf_data(out_path)){
        std::string src_info = sa_temp_path + ".info";
        if(eid_path_exists(src_info)){
            std::string dst_info = out_path + ".info";
            FILE* fi = fopen(src_info.c_str(), "rb");
            FILE* fo = fopen(dst_info.c_str(), "wb");
            if(fi && fo){
                char cbuf[4096]; size_t r;
                while((r = fread(cbuf, 1, sizeof(cbuf), fi)) > 0)
                    fwrite(cbuf, 1, r, fo);
            }
            if(fi) fclose(fi);
            if(fo) fclose(fo);
        }
    }
    return out_path;
}

void FFTViewer::eid_remove_samples(double s0, double s1){
    int64_t i0 = std::max((int64_t)0, (int64_t)s0);
    int64_t i1 = std::min(eid_total_samples, (int64_t)ceil(s1));
    if(i1 <= i0) return;
    eid_edit_gen++;   // IQ 수정 → 복조 캐시 무효화
    int64_t count = i1 - i0;

    {
        std::lock_guard<std::mutex> lk(eid_data_mtx);
        auto erase = [&](std::vector<float>& vec){
            if(i0 < (int64_t)vec.size() && i1 <= (int64_t)vec.size())
                vec.erase(vec.begin()+i0, vec.begin()+i1);
        };
        erase(eid_envelope);
        erase(eid_ch_i);
        erase(eid_ch_q);
        erase(eid_phase);
        erase(eid_inst_freq);
        eid_total_samples -= count;
    }

    // 태그 조정
    for(auto it = eid_tags.begin(); it != eid_tags.end(); ){
        if(it->s0 >= s0 && it->s1 <= s1){
            it = eid_tags.erase(it);
        } else {
            if(it->s0 > s1){ it->s0 -= count; it->s1 -= count; }
            else if(it->s1 > s0){ it->s1 = std::min(it->s1, s0); }
            ++it;
        }
    }

    // 뷰 상태 조정
    if(eid_view_t0 > s1) eid_view_t0 -= count;
    else if(eid_view_t0 > s0) eid_view_t0 = s0;
    if(eid_view_t1 > s1) eid_view_t1 -= count;
    else if(eid_view_t1 > s0) eid_view_t1 = s0;
    eid_view_t0 = std::max(0.0, eid_view_t0);
    eid_view_t1 = std::min((double)eid_total_samples, eid_view_t1);
    if(eid_view_t1 <= eid_view_t0){ eid_view_t0=0; eid_view_t1=(double)eid_total_samples; }
    eid_view_stack.clear();
    sa_view_history.clear();

    // 스펙트로그램 재계산
    sa_recompute_from_iq();
}

void FFTViewer::eid_select_samples(double s0, double s1){
    int64_t i0 = std::max((int64_t)0, (int64_t)s0);
    int64_t i1 = std::min(eid_total_samples, (int64_t)ceil(s1));
    if(i1 <= i0) return;
    if(i0 == 0 && i1 == eid_total_samples) return;

    {
        std::lock_guard<std::mutex> lk(eid_data_mtx);
        auto crop = [&](std::vector<float>& vec){
            if((int64_t)vec.size() < i1) return;
            vec = std::vector<float>(vec.begin()+i0, vec.begin()+i1);
        };
        crop(eid_envelope);
        crop(eid_ch_i);
        crop(eid_ch_q);
        crop(eid_phase);
        crop(eid_inst_freq);
        eid_total_samples = i1 - i0;
    }

    // BPF 백업 무효화 (길이 불일치 방지)
    eid_bpf_active = false;
    eid_orig_ch_i.clear(); eid_orig_ch_q.clear();

    // 태그 조정: 범위 밖 삭제, 범위 안은 좌표 이동
    double new_len = (double)(i1 - i0);
    for(auto it = eid_tags.begin(); it != eid_tags.end(); ){
        if(it->s1 <= (double)i0 || it->s0 >= (double)i1){
            it = eid_tags.erase(it); continue;
        }
        it->s0 = std::max(0.0, it->s0 - (double)i0);
        it->s1 = std::min(new_len, it->s1 - (double)i0);
        if(it->s1 <= it->s0){ it = eid_tags.erase(it); continue; }
        ++it;
    }

    // 뷰 전체로 리셋
    eid_view_t0 = 0.0;
    eid_view_t1 = new_len;
    eid_view_stack.clear();
    sa_view_history.clear();

    // 스펙트로그램 재계산 (전체 보기로 리셋)
    sa_recompute_from_iq(true);
}

void FFTViewer::eid_cleanup(){
    if(eid_thread.joinable()) eid_thread.join();
    {
        std::lock_guard<std::mutex> lk(eid_data_mtx);
        eid_envelope.clear();  eid_envelope.shrink_to_fit();
        eid_ch_i.clear();      eid_ch_i.shrink_to_fit();
        eid_ch_q.clear();      eid_ch_q.shrink_to_fit();
        eid_phase.clear();     eid_phase.shrink_to_fit();
        eid_inst_freq.clear(); eid_inst_freq.shrink_to_fit();
    }
    eid_data_ready.store(false);
    eid_computing.store(false);
    eid_total_samples    = 0;
    eid_sample_rate      = 0;
    eid_noise_level      = 0.f;
    eid_center_freq_hz   = 0;
    eid_view_mode        = 0;
    eid_phase_detrend_hz = 0.0f;
    eid_bpf_active       = false;
    eid_orig_ch_i.clear(); eid_orig_ch_i.shrink_to_fit();
    eid_orig_ch_q.clear(); eid_orig_ch_q.shrink_to_fit();
    eid_tags.clear();
    eid_view_stack.clear();
    sa_view_history.clear();
    eid_baud_mode=false; eid_baud_s0=-1; eid_baud_s1=-1; eid_baud_click=0; eid_baud_drag=-1;
    eid_undo_stack.clear();
    eid_redo_stack.clear();
}

// ── Undo/Redo 시스템 ──────────────────────────────────────────────────────
FFTViewer::EidUndoEntry FFTViewer::eid_snapshot() const {
    EidUndoEntry e;
    e.envelope   = eid_envelope;
    e.ch_i       = eid_ch_i;
    e.ch_q       = eid_ch_q;
    e.phase      = eid_phase;
    e.inst_freq  = eid_inst_freq;
    e.orig_ch_i  = eid_orig_ch_i;
    e.orig_ch_q  = eid_orig_ch_q;
    e.tags       = eid_tags;
    e.view_stack = eid_view_stack;
    e.sa_history = sa_view_history;
    e.total_samples = eid_total_samples;
    e.view_t0 = eid_view_t0;  e.view_t1 = eid_view_t1;
    e.sa_vx0 = sa_view_x0;  e.sa_vx1 = sa_view_x1;
    e.sa_vy0 = sa_view_y0;  e.sa_vy1 = sa_view_y1;
    e.bpf_active = eid_bpf_active;
    e.baud_mode = eid_baud_mode;
    e.baud_s0 = eid_baud_s0;  e.baud_s1 = eid_baud_s1;
    e.baud_click = eid_baud_click;
    e.baseline_active = eid_baseline_active;
    e.baseline_val = eid_baseline_val;
    e.baseline_imode = eid_baseline_imode;
    e.pending_active = eid_pending_active;
    e.pending_s0 = eid_pending_s0;  e.pending_s1 = eid_pending_s1;
    memcpy(e.y_min, eid_y_min, sizeof(e.y_min));
    memcpy(e.y_max, eid_y_max, sizeof(e.y_max));
    e.amp_min = eid_amp_min;  e.amp_max = eid_amp_max;
    e.noise_level = eid_noise_level;
    return e;
}

void FFTViewer::eid_restore(const EidUndoEntry& e){
    eid_edit_gen++;   // IQ 복원(undo/redo) → 복조 캐시 무효화
    bool data_changed = (e.total_samples != eid_total_samples ||
                         e.ch_i.size() != eid_ch_i.size() ||
                         e.bpf_active != eid_bpf_active);
    {
        std::lock_guard<std::mutex> lk(eid_data_mtx);
        eid_envelope  = e.envelope;
        eid_ch_i      = e.ch_i;
        eid_ch_q      = e.ch_q;
        eid_phase     = e.phase;
        eid_inst_freq = e.inst_freq;
        eid_total_samples = e.total_samples;
    }
    eid_orig_ch_i = e.orig_ch_i;
    eid_orig_ch_q = e.orig_ch_q;
    eid_tags      = e.tags;
    eid_view_stack = e.view_stack;
    sa_view_history = e.sa_history;
    eid_view_t0 = e.view_t0;  eid_view_t1 = e.view_t1;
    sa_view_x0 = e.sa_vx0;  sa_view_x1 = e.sa_vx1;
    sa_view_y0 = e.sa_vy0;  sa_view_y1 = e.sa_vy1;
    eid_bpf_active = e.bpf_active;
    eid_baud_mode = e.baud_mode;
    eid_baud_s0 = e.baud_s0;  eid_baud_s1 = e.baud_s1;
    eid_baud_click = e.baud_click;
    eid_baseline_active = e.baseline_active;
    eid_baseline_val = e.baseline_val;
    eid_baseline_imode = e.baseline_imode;
    eid_pending_active = e.pending_active;
    eid_pending_s0 = e.pending_s0;  eid_pending_s1 = e.pending_s1;
    memcpy(eid_y_min, e.y_min, sizeof(eid_y_min));
    memcpy(eid_y_max, e.y_max, sizeof(eid_y_max));
    eid_amp_min = e.amp_min;  eid_amp_max = e.amp_max;
    eid_noise_level = e.noise_level;
    if(data_changed) sa_recompute_from_iq();
}

void FFTViewer::eid_push_undo(){
    eid_undo_stack.push_back(eid_snapshot());
    if((int)eid_undo_stack.size() > EID_UNDO_MAX)
        eid_undo_stack.pop_front();
    eid_redo_stack.clear();
}

void FFTViewer::eid_do_undo(){
    if(eid_undo_stack.empty()) return;
    eid_redo_stack.push_back(eid_snapshot());
    if((int)eid_redo_stack.size() > EID_UNDO_MAX)
        eid_redo_stack.pop_front();
    EidUndoEntry e = std::move(eid_undo_stack.back());
    eid_undo_stack.pop_back();
    eid_restore(e);
}

void FFTViewer::eid_do_redo(){
    if(eid_redo_stack.empty()) return;
    eid_undo_stack.push_back(eid_snapshot());
    if((int)eid_undo_stack.size() > EID_UNDO_MAX)
        eid_undo_stack.pop_front();
    EidUndoEntry e = std::move(eid_redo_stack.back());
    eid_redo_stack.pop_back();
    eid_restore(e);
}

// ── IQ(eid_ch_i/q) → AM/FM 복조 → 임시 mono WAV(≈AUDIO_SR) ──────────────────
// dem_worker 와 동일한 복조 수학. 녹음 IQ 는 이미 채널 baseband 이므로 mixing 없음.
std::string FFTViewer::eid_iq_demod_tempwav(int am_fm){
    size_t N = std::min(eid_ch_i.size(), eid_ch_q.size());
    uint32_t sr_in = eid_sample_rate;
    if(N < 2 || sr_in == 0) return "";
    uint32_t decim = std::max(1u, (uint32_t)llround((double)sr_in / (double)AUDIO_SR));
    uint32_t asr   = std::max(1u, sr_in / decim);   // 출력 SR (≈48k)

    float am_dc=0.f, am_dc_alpha = 1.0f - expf(-2.0f*(float)M_PI*30.0f/(float)sr_in);
    float alf_cut = std::min(12000.0f, (float)sr_in*0.45f);
    IIR1 alf;    alf.set(alf_cut/(float)sr_in);
    IIR1 deemph; deemph.set(std::min(3183.0f,(float)sr_in*0.45f)/(float)sr_in);
    float agc_rms=0.01f; const float AGC_TARGET=1.0f, AGC_ATTACK=0.001f, AGC_RELEASE=0.0001f;
    double aac=0; int acnt=0;

    // BPF 활성 시: 선택 대역 중심을 DC 로 mix-down(재중심). FM 판별기는 신호가 DC 에
    // 있어야 동작 — 오프셋이 크면 위상증분이 wrap 되어 소리가 안 난다. (AM 은 무해)
    float f_off = eid_bpf_active ? (eid_bpf_center_uv - 0.5f) * (float)sr_in : 0.f;
    Oscillator osc; osc.set_freq((double)f_off, (double)sr_in);
    float prev_i=0.f, prev_q=0.f;

    std::vector<int16_t> out; out.reserve(N/decim + 16);
    for(size_t n=0; n<N; n++){
        float fi, fq; osc.mix(eid_ch_i[n], eid_ch_q[n], fi, fq);
        float samp;
        if(am_fm==0){   // AM: envelope + DC제거 + AGC
            float env = sqrtf(fi*fi+fq*fq);
            am_dc += am_dc_alpha*(env-am_dc);
            float audio = alf.p(env-am_dc);
            float rms_in = audio*audio;
            if(rms_in>agc_rms) agc_rms += (rms_in-agc_rms)*AGC_ATTACK;
            else               agc_rms += (rms_in-agc_rms)*AGC_RELEASE;
            float gain = (agc_rms>1e-9f)?(AGC_TARGET/sqrtf(agc_rms)):100.0f;
            gain = std::min(gain,1000.0f);
            samp = std::max(-1.0f,std::min(1.0f,audio*gain));
        } else {        // FM: phase discriminator + 적응형 LPF + 50us de-emphasis
            float cross=fi*prev_q - fq*prev_i, dot=fi*prev_i + fq*prev_q;
            float d=atan2f(cross, dot+1e-12f); prev_i=fi; prev_q=fq;
            samp = deemph.p(alf.p(d)) * 4.0f;
        }
        aac += samp; acnt++;
        if(acnt >= (int)decim){
            float o = std::max(-1.0f,std::min(1.0f,(float)(aac/acnt)));
            out.push_back((int16_t)lrintf(o*32767.0f));
            aac=0; acnt=0;
        }
    }
    if(out.empty()) return "";

    std::string tmp = "/tmp/bewe_iqdemod.wav";
    FILE* f = fopen(tmp.c_str(),"wb");
    if(!f) return "";
    uint32_t nf=(uint32_t)out.size();
    uint32_t data_bytes=nf*2, byte_rate=asr*2, chunk=36+data_bytes; uint32_t sc1=16;
    uint16_t fmt=1, ch=1, balign=2, bits=16;
    fwrite("RIFF",1,4,f); fwrite(&chunk,4,1,f); fwrite("WAVE",1,4,f);
    fwrite("fmt ",1,4,f); fwrite(&sc1,4,1,f);
    fwrite(&fmt,2,1,f); fwrite(&ch,2,1,f); fwrite(&asr,4,1,f);
    fwrite(&byte_rate,4,1,f); fwrite(&balign,2,1,f); fwrite(&bits,2,1,f);
    fwrite("data",1,4,f); fwrite(&data_bytes,4,1,f);
    fwrite(out.data(),2,nf,f);
    fclose(f);
    bewe_log_push(0,"[EID] IQ %s demod -> %u Hz mono, %u frames\n", am_fm?"FM":"AM", asr, nf);
    return tmp;
}

// IQ면 AM/FM 복조 wav 재생, 아니면 원본(이미 audio) 재생. off_sec 위치부터.
void FFTViewer::eid_audio_play(double off_sec){
    if(sa_temp_path.empty()) return;
    if(eid_is_iq){
        // 캐시: 같은 소스+모드+편집세대(BPF 등 반영)면 재복조 생략
        if(eid_iq_tmp_src != sa_temp_path || eid_iq_tmp_mode != eid_audio_demod
           || eid_iq_tmp_gen != eid_edit_gen || eid_iq_tmp_path.empty()){
            std::string tmp = eid_iq_demod_tempwav(eid_audio_demod);
            if(tmp.empty()) return;
            eid_iq_tmp_path = tmp; eid_iq_tmp_src = sa_temp_path;
            eid_iq_tmp_mode = eid_audio_demod; eid_iq_tmp_gen = eid_edit_gen;
        }
        audio_play_start(eid_iq_tmp_path, off_sec);
    } else {
        audio_play_start(sa_temp_path, off_sec);
    }
}
