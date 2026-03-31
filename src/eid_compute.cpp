#include "fft_viewer.hpp"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>

// ── EID envelope 추출 (비동기 스레드) ────────────────────────────────────────
void FFTViewer::eid_start(const std::string& wav_path){
    if(eid_thread.joinable()) eid_thread.join();
    eid_computing.store(true);
    eid_data_ready.store(false);
    sa_temp_path = wav_path;  // FFT 콤보에서 재계산할 때 사용

    eid_thread = std::thread([this, wav_path](){
        // ── WAV 파싱 (sa_start 패턴 동일) ───────────────────────────────────
        FILE* f = fopen(wav_path.c_str(), "rb");
        if(!f){ eid_computing.store(false); return; }

        uint64_t meta_cf_hz = 0;
        int64_t  meta_time  = 0;
        uint32_t meta_sr    = 0;
        long     data_offset = 44;
        long     data_size   = 0;

        // fmt chunk sample_rate
        fseek(f, 24, SEEK_SET);
        uint32_t wav_sr = 0; fread(&wav_sr, 4, 1, f);

        // 청크 순회
        fseek(f, 12, SEEK_SET);
        while(true){
            char id[5] = {};
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
            long next = chunk_data_pos + (long)csz + ((long)csz & 1);
            if(fseek(f, next, SEEK_SET) != 0) break;
        }
        if(data_size <= 0){
            fseek(f, 0, SEEK_END);
            long file_sz = ftell(f);
            data_offset = 44;
            data_size   = file_sz - 44;
        }
        if(meta_sr == 0) meta_sr = wav_sr;

        fseek(f, data_offset, SEEK_SET);
        if(data_size <= 0){ fclose(f); eid_computing.store(false); return; }
        int64_t n_samples = data_size / (int64_t)(2 * sizeof(int16_t));
        if(n_samples < 1){ fclose(f); eid_computing.store(false); return; }

        // ── IQ 로드 + envelope/I/Q/phase/freq 추출 ──────────────────────────
        const int64_t BLOCK = 65536;
        std::vector<float> env(n_samples), ch_i(n_samples), ch_q(n_samples);
        std::vector<float> phase(n_samples), inst_freq(n_samples);
        std::vector<int16_t> raw(BLOCK * 2);
        int64_t done = 0;
        const float SCL = 1.0f / 32768.0f;
        float prev_phase = 0.f;
        const float TWO_PI = 6.283185307f;

        while(done < n_samples){
            int64_t todo = std::min(BLOCK, n_samples - done);
            int64_t got = (int64_t)fread(raw.data(), sizeof(int16_t), (size_t)(todo * 2), f) / 2;
            if(got <= 0) break;
            for(int64_t i = 0; i < got; i++){
                float fi = raw[i * 2    ] * SCL;
                float fq = raw[i * 2 + 1] * SCL;
                int64_t idx = done + i;
                env[idx]  = sqrtf(fi * fi + fq * fq);
                ch_i[idx] = fi;
                ch_q[idx] = fq;
                float ph  = atan2f(fq, fi);
                phase[idx] = ph;
                // unwrapped phase diff → instantaneous frequency
                float dp = ph - prev_phase;
                if(dp >  3.14159265f) dp -= TWO_PI;
                if(dp < -3.14159265f) dp += TWO_PI;
                inst_freq[idx] = (idx == 0) ? 0.f : dp;
                prev_phase = ph;
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
        float amp_hi = sorted_env[(size_t)(sorted_env.size() * 0.99f)];
        if(amp_hi - amp_lo < 0.001f) amp_hi = amp_lo + 0.001f;
        // 여유 5%
        float margin = (amp_hi - amp_lo) * 0.05f;
        amp_lo -= margin;
        amp_hi += margin;
        if(amp_lo < 0.f) amp_lo = 0.f;

        // 노이즈 레벨: 5th percentile
        float noise_lvl = sorted_env[(size_t)(sorted_env.size() * 0.05f)];

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
        }
        eid_view_t0        = 0.0;
        eid_view_t1        = (double)done;
        eid_amp_min        = amp_lo;
        eid_amp_max        = amp_hi;
        eid_noise_level    = noise_lvl;
        eid_center_freq_hz = meta_cf_hz;
        eid_view_mode      = 0; // reset to Signal on new load
        eid_phase_detrend_hz = 0.0f;
        eid_data_ready.store(true);
        eid_computing.store(false);
        bewe_log("EID: loaded %lld samples, sr=%u, cf=%llu\n",
                 (long long)done, eid_sample_rate, (unsigned long long)meta_cf_hz);
    });
}

// ── ch_i/ch_q 로부터 envelope/phase/inst_freq 재계산 ─────────────────────────
void FFTViewer::eid_recompute_derived(){
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
    float hi = sorted_env[(size_t)(sorted_env.size()*0.99f)];
    if(hi-lo<0.001f) hi=lo+0.001f;
    float margin = (hi-lo)*0.05f;
    eid_amp_min = std::max(0.f, lo-margin);
    eid_amp_max = hi+margin;
    eid_noise_level = sorted_env[(size_t)(sorted_env.size()*0.05f)];
}

// ── FFT 기반 BPF (brick-wall, 블록 처리) ─────────────────────────────────────
void FFTViewer::eid_apply_bpf(double freq_lo_hz, double freq_hi_hz){
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

    double sr = (double)eid_sample_rate;
    double cf = (double)eid_center_freq_hz;
    double bpf_lo = freq_lo_hz - cf;  // baseband offset
    double bpf_hi = freq_hi_hz - cf;

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
    eid_recompute_derived();
    sa_recompute_from_iq();
}

void FFTViewer::eid_remove_samples(double s0, double s1){
    int64_t i0 = std::max((int64_t)0, (int64_t)s0);
    int64_t i1 = std::min(eid_total_samples, (int64_t)ceil(s1));
    if(i1 <= i0) return;
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
    sa_freq_view_stack.clear();

    // 스펙트로그램 재계산
    sa_recompute_from_iq();
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
}
