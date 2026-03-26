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

        // ── IQ 로드 + envelope 추출 ─────────────────────────────────────────
        // 블록 단위로 읽어서 메모리 절약 (raw IQ 전체를 동시에 올리지 않음)
        const int64_t BLOCK = 65536;
        std::vector<float> env(n_samples);
        std::vector<int16_t> raw(BLOCK * 2);
        int64_t done = 0;
        const float SCL = 1.0f / 32768.0f;

        while(done < n_samples){
            int64_t todo = std::min(BLOCK, n_samples - done);
            int64_t got = (int64_t)fread(raw.data(), sizeof(int16_t), (size_t)(todo * 2), f) / 2;
            if(got <= 0) break;
            for(int64_t i = 0; i < got; i++){
                float fi = raw[i * 2    ] * SCL;
                float fq = raw[i * 2 + 1] * SCL;
                env[done + i] = sqrtf(fi * fi + fq * fq);
            }
            done += got;
        }
        fclose(f);
        if(done < 1){ eid_computing.store(false); return; }
        env.resize(done);

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

        // ── 데이터 전달 ─────────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(eid_data_mtx);
            eid_envelope      = std::move(env);
            eid_total_samples = done;
            eid_sample_rate   = meta_sr > 0 ? meta_sr : wav_sr;
        }
        eid_view_t0  = 0.0;
        eid_view_t1  = (double)done;
        eid_amp_min  = amp_lo;
        eid_amp_max  = amp_hi;
        eid_data_ready.store(true);
        eid_computing.store(false);
        bewe_log("EID: loaded %lld samples, sr=%u\n", (long long)done, eid_sample_rate);
    });
}

void FFTViewer::eid_cleanup(){
    if(eid_thread.joinable()) eid_thread.join();
    {
        std::lock_guard<std::mutex> lk(eid_data_mtx);
        eid_envelope.clear();
        eid_envelope.shrink_to_fit();
    }
    eid_data_ready.store(false);
    eid_computing.store(false);
    eid_total_samples = 0;
    eid_sample_rate   = 0;
}
