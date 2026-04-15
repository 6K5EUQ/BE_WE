#include "lora_demod.hpp"
#include <fftw3.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>

namespace {

struct CF { float r, i; };

// Build a complex downchirp of length N at bandwidth BW sampled at exactly BW
// (one chirp spans N samples covering a full [-BW/2, +BW/2] sweep).
//   phase(n) = -2π · ( -0.5·n + 0.5·n²/N )     (normalized freq units, fs=1)
// This matches the reference LoRa convention: upchirp sweeps -BW/2 → +BW/2,
// so multiplying by the conjugate downchirp dechirps to a single tone.
// dir=+1: standard downchirp (conjugate of upchirp: freq sweeps +BW/2 → -BW/2)
// dir=-1: reversed (= upchirp reference; used when input signal is downchirps)
static std::vector<CF> make_dechirp_ref(int N, int dir)
{
    std::vector<CF> out(N);
    const double twopi = 6.283185307179586;
    double s = (dir >= 0) ? 1.0 : -1.0;
    for(int n = 0; n < N; n++){
        double phase = -twopi * s * (-0.5 * n + 0.5 * (double)n * n / (double)N);
        out[n].r = (float)std::cos(phase);
        out[n].i = (float)std::sin(phase);
    }
    return out;
}

static int argmax_mag(fftwf_complex* spec, int N, float* out_peak = nullptr,
                      float* out_floor_rms = nullptr)
{
    int best = 0;
    float bestM = -1.f;
    double sum_m2 = 0.0;
    for(int i = 0; i < N; i++){
        float re = spec[i][0], im = spec[i][1];
        float m = re*re + im*im;
        sum_m2 += m;
        if(m > bestM){ bestM = m; best = i; }
    }
    if(out_peak)       *out_peak       = std::sqrt(bestM);
    // Noise floor = RMS excluding the peak bin
    if(out_floor_rms){
        double floor_m2 = sum_m2 - bestM;
        if(N > 1) floor_m2 /= (double)(N - 1);
        *out_floor_rms = (float)std::sqrt(std::max(0.0, floor_m2));
    }
    return best;
}

// Dechirp+FFT one symbol worth of input samples (decimated by D so BW==fs).
// base: index into I/Q arrays. N input samples spaced by D are consumed.
// Returns FFT argmax bin index [0..N). Fills peak/floor if requested.
static int demod_symbol(const float* I, const float* Q,
                        int64_t base, int N, int D,
                        const std::vector<CF>& dchirp,
                        fftwf_complex* in, fftwf_complex* out,
                        fftwf_plan plan,
                        float* out_peak = nullptr,
                        float* out_floor = nullptr)
{
    for(int i = 0; i < N; i++){
        int64_t idx = base + (int64_t)i * D;
        float si = I[idx], sq = Q[idx];
        float dr = dchirp[i].r, di = dchirp[i].i;
        // (si + j·sq) * (dr + j·di)
        in[i][0] = si*dr - sq*di;
        in[i][1] = si*di + sq*dr;
    }
    fftwf_execute(plan);
    return argmax_mag(out, N, out_peak, out_floor);
}

// LoRa sync word encoding: 2 symbols carry upper/lower 4 bits, shifted to bit3.
// ref: Semtech app note. sync0 == ((sync>>4) & 0xF) << 3, sync1 == (sync & 0xF) << 3.
// Decode by inverse: sync = ((k0 >> 3) << 4) | (k1 >> 3). (k shifted right 3.)
static uint8_t decode_sync_word(int k0, int k1, int N)
{
    // Values are modulo N; take the lowest 7 bits for SF≥7 before shift-right.
    int n0 = k0 & (N - 1);
    int n1 = k1 & (N - 1);
    return (uint8_t)((((n0 >> 3) & 0xF) << 4) | ((n1 >> 3) & 0xF));
}

} // namespace

bool lora_demod_auto(const float* I, const float* Q,
                     int64_t n_samples, uint32_t sr,
                     std::vector<uint8_t>& out_bits,
                     LoraDemodResult& info)
{
    out_bits.clear();
    info = {};

    if(!I || !Q || n_samples < 4096 || sr == 0) {
        info.detail = "입력 버퍼가 너무 짧거나 sample_rate=0";
        return false;
    }

    struct Cand { int sf; uint32_t bw; int dir; int64_t align; uint8_t sync; float score; int k0; };
    std::vector<Cand> cands;

    // BW 후보: LoRa 표준 125 / 250 / 500 kHz
    const uint32_t BW_CANDS[] = { 125000u, 250000u, 500000u };

    for(uint32_t BW : BW_CANDS){
        int D = std::max(1, (int)std::lround((double)sr / (double)BW));
        if(D < 1) continue;

        for(int sf = 7; sf <= 12; sf++){
            const int N = 1 << sf;
            const int64_t need = (int64_t)N * D * 11;
            if(n_samples < need) continue;

            for(int dir = +1; dir >= -1; dir -= 2){
                auto dchirp = make_dechirp_ref(N, dir);
                fftwf_complex* in  = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * N);
                fftwf_complex* out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * N);
                fftwf_plan plan = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

                int64_t step = (int64_t)std::max(1, N / 4) * D;
                int64_t last_off = n_samples - need;
                for(int64_t off = 0; off <= last_off; off += step){
                    int ks[8]; float peaks[8], floors[8];
                    for(int s = 0; s < 8; s++){
                        int64_t base = off + (int64_t)s * N * D;
                        ks[s] = demod_symbol(I, Q, base, N, D, dchirp, in, out, plan,
                                             &peaks[s], &floors[s]);
                    }
                    // 8개 중 최빈값(=mode) 기준으로 6개 이상 일치하면 후보
                    int mode_k = ks[0], mode_cnt = 1;
                    for(int i = 0; i < 8; i++){
                        int c = 0;
                        for(int j = 0; j < 8; j++) if(ks[j] == ks[i]) c++;
                        if(c > mode_cnt){ mode_cnt = c; mode_k = ks[i]; }
                    }
                    if(mode_cnt < 6) continue;

                    float mean_peak = 0.f, mean_floor = 0.f;
                    for(int s = 0; s < 8; s++){ mean_peak += peaks[s]; mean_floor += floors[s]; }
                    mean_peak /= 8.f; mean_floor = std::max(1e-6f, mean_floor / 8.f);
                    float score = (mean_peak / mean_floor) * ((float)mode_cnt / 8.f);
                    if(score < 2.0f) continue;

                    // sync word (preamble 직후 2심볼)
                    int k_sync0 = demod_symbol(I, Q, off + (int64_t)8 * N * D, N, D,
                                               dchirp, in, out, plan);
                    int k_sync1 = demod_symbol(I, Q, off + (int64_t)9 * N * D, N, D,
                                               dchirp, in, out, plan);
                    uint8_t sync = decode_sync_word(k_sync0, k_sync1, N);

                    cands.push_back({sf, BW, dir, off, sync, score, mode_k});
                }

                fftwf_destroy_plan(plan);
                fftwf_free(in);
                fftwf_free(out);
            }
        }
    }

    if(cands.empty()){
        info.detail = "preamble 찾지 못함 (SF 7~12, BW 125/250/500 kHz, 양방향 chirp 시도)";
        return false;
    }

    auto best = std::max_element(cands.begin(), cands.end(),
        [](const Cand& a, const Cand& b){ return a.score < b.score; });

    info.sf       = best->sf;
    info.bw       = best->bw;
    info.sync     = best->sync;
    info.align    = best->align;
    info.score    = best->score;

    const int N = 1 << best->sf;
    const int D = std::max(1, (int)std::lround((double)sr / (double)best->bw));
    const int64_t data_off = best->align + (int64_t)std::llround(12.25 * (double)N * (double)D);

    auto dchirp = make_dechirp_ref(N, best->dir);
    fftwf_complex* in  = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * N);
    fftwf_complex* out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * N);
    fftwf_plan plan = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    int n_syms = 0;
    for(int64_t pos = data_off; pos + (int64_t)N * D <= n_samples; pos += (int64_t)N * D){
        int k = demod_symbol(I, Q, pos, N, D, dchirp, in, out, plan);
        for(int b = best->sf - 1; b >= 0; b--)
            out_bits.push_back((uint8_t)((k >> b) & 1));
        n_syms++;
    }

    fftwf_destroy_plan(plan);
    fftwf_free(in);
    fftwf_free(out);

    info.n_symbols = n_syms;

    char buf[256];
    snprintf(buf, sizeof(buf),
             "LoRa CSS  SF=%d  BW=%u Hz  dir=%s  sync=0x%02X  align=%lld  syms=%d  score=%.2f  preamble_k=%d",
             info.sf, info.bw, (best->dir > 0 ? "up" : "down"),
             info.sync, (long long)info.align,
             info.n_symbols, info.score, best->k0);
    info.detail = buf;
    return n_syms > 0;
}
