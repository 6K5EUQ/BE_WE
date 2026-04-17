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

        // fmt chunk: num_channels (offset 22), sample_rate (offset 24)
        fseek(f, 22, SEEK_SET);
        uint16_t wav_ch = 0; fread(&wav_ch, 2, 1, f);
        uint32_t wav_sr = 0; fread(&wav_sr, 4, 1, f);
        if(wav_ch == 0) wav_ch = 2; // 헤더 이상 → 기존 IQ 가정

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
        // 기존 WAV: data 청크 안에 bewe 청크가 끼어있을 수 있음 → 스킵
        {
            char peek[4]={};
            if(fread(peek,1,4,f)==4 && strncmp(peek,"bewe",4)==0){
                uint32_t bsz=0; fread(&bsz,4,1,f);
                fseek(f, ftell(f)+(long)bsz+((long)bsz&1), SEEK_SET);
                long skipped = ftell(f) - data_offset;
                data_size -= skipped;
                data_offset = ftell(f);
            } else {
                fseek(f, data_offset, SEEK_SET);
            }
        }
        if(data_size <= 0){ fclose(f); eid_computing.store(false); return; }
        const int nch = (wav_ch >= 2) ? 2 : 1;  // 1=mono(audio), 2=stereo(IQ)
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
