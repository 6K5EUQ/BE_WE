#include "fft_viewer.hpp"
#include "proto_decode.hpp"
#include <fftw3.h>
#include <cmath>
#include <cstring>
#include <chrono>
#include <ctime>
#include <mutex>
#include <numeric>

// FFTW thread safety: plan creation must be serialized
static std::mutex g_fftw_mtx;

// ─────────────────────────────────────────────────────────────────────────
// Higher-order cumulant computation
// ─────────────────────────────────────────────────────────────────────────
CumulantResult compute_cumulants(const float* I, const float* Q, int N)
{
    // Normalize so E[|z|^2] = 1
    double power = 0;
    for (int i = 0; i < N; i++)
        power += (double)I[i]*I[i] + (double)Q[i]*Q[i];
    power /= N;
    float norm = 1.0f / sqrtf((float)power + 1e-30f);

    // Accumulate moments
    std::complex<double> M20{0,0}, M40{0,0};
    double M42 = 0, M63 = 0;

    for (int i = 0; i < N; i++) {
        double zi = I[i] * norm, zq = Q[i] * norm;
        std::complex<double> z(zi, zq);
        double zz = zi*zi + zq*zq; // |z|^2

        std::complex<double> z2 = z * z;
        std::complex<double> z4 = z2 * z2;

        M20 += z2;
        M40 += z4;
        M42 += zz * zz;   // |z|^4
        M63 += zz * zz * zz; // |z|^6
    }
    M20 /= N; M40 /= N; M42 /= N; M63 /= N;

    CumulantResult r;
    r.C20 = std::complex<float>(M20);
    r.C40 = std::complex<float>(M40 - 3.0 * M20 * M20);
    r.M42 = (float)M42;
    float C42_real = (float)(M42 - std::norm(M20) - 2.0);
    r.abs_C40 = std::abs(r.C40);
    r.abs_C42 = fabsf(C42_real);
    r.C63_real = (float)(M63 - 9.0 * M42 + 12.0);
    return r;
}

// ─────────────────────────────────────────────────────────────────────────
// Feature extraction for AMC
// ─────────────────────────────────────────────────────────────────────────
AmcFeatures extract_features(const float* I, const float* Q, int N, float work_sr)
{
    AmcFeatures f{};
    if (N < 100) return f;

    // Envelope statistics
    std::vector<float> env(N);
    double env_sum = 0, env_sum2 = 0;
    for (int i = 0; i < N; i++) {
        env[i] = sqrtf(I[i]*I[i] + Q[i]*Q[i]);
        env_sum += env[i];
        env_sum2 += (double)env[i] * env[i];
    }
    f.env_mean = (float)(env_sum / N);
    float env_var = (float)(env_sum2 / N - (double)f.env_mean * f.env_mean);
    f.env_std = sqrtf(env_var > 0 ? env_var : 0);
    f.env_cv = (f.env_mean > 1e-10f) ? f.env_std / f.env_mean : 0;

    // Envelope gamma (95th percentile / mean)
    std::vector<float> env_sorted(env);
    std::sort(env_sorted.begin(), env_sorted.end());
    float p95 = env_sorted[(int)(N * 0.95f)];
    f.env_gamma = (f.env_mean > 1e-10f) ? p95 / f.env_mean : 0;

    // Instantaneous frequency
    double freq_sum = 0, freq_sum2 = 0;
    std::vector<float> inst_freq(N);
    for (int i = 1; i < N; i++) {
        float cross = I[i]*Q[i-1] - Q[i]*I[i-1];
        float dot   = I[i]*I[i-1] + Q[i]*Q[i-1];
        inst_freq[i] = atan2f(cross, dot + 1e-30f);
        freq_sum += inst_freq[i];
        freq_sum2 += (double)inst_freq[i] * inst_freq[i];
    }
    float freq_mean = (float)(freq_sum / (N-1));
    float freq_var = (float)(freq_sum2 / (N-1) - (double)freq_mean * freq_mean);
    f.freq_std = sqrtf(freq_var > 0 ? freq_var : 0);

    // Inst. freq histogram -> count peaks (for FSK level detection)
    {
        const int NBINS = 64;
        int hist[NBINS] = {};
        for (int i = 1; i < N; i++) {
            int bin = (int)((inst_freq[i] + M_PIf) / (2.0f * M_PIf) * NBINS);
            if (bin < 0) bin = 0; if (bin >= NBINS) bin = NBINS - 1;
            hist[bin]++;
        }
        int max_h = *std::max_element(hist, hist + NBINS);
        int thresh = std::max(1, max_h / 3);
        // Count connected regions above threshold
        int peaks = 0;
        bool in_peak = false;
        for (int b = 0; b < NBINS; b++) {
            if (hist[b] >= thresh && !in_peak) { peaks++; in_peak = true; }
            if (hist[b] < thresh) in_peak = false;
        }
        f.freq_n_levels = peaks;
    }

    // Spectral asymmetry (requires small FFT)
    f.spec_asymmetry = 0;
    {
        int fft_n = 512;
        while (fft_n < N && fft_n < 4096) fft_n *= 2;
        if (fft_n > N) fft_n = N;
        // Simple power estimation: upper vs lower half
        double pwr_lo = 0, pwr_hi = 0;
        for (int i = 0; i < fft_n; i++) {
            float pwr = I[i]*I[i] + Q[i]*Q[i];
            if (i < fft_n/2) pwr_lo += pwr;
            else pwr_hi += pwr;
        }
        float total = (float)(pwr_lo + pwr_hi);
        if (total > 0)
            f.spec_asymmetry = fabsf((float)(pwr_hi - pwr_lo)) / total;
    }

    // Cumulants
    f.cum = compute_cumulants(I, Q, N);

    return f;
}

// ─────────────────────────────────────────────────────────────────────────
// AMC classification (decision tree)
// ─────────────────────────────────────────────────────────────────────────
AmcResult classify_modulation(const float* I, const float* Q, int N, float work_sr)
{
    AmcResult r;
    if (N < 100) return r;

    AmcFeatures f = extract_features(I, Q, N, work_sr);

    // SNR estimation (rough: signal power / noise floor from low-envelope samples)
    float sig_power = f.env_mean * f.env_mean;
    // Use 5th percentile as noise estimate
    std::vector<float> env(N);
    for (int i = 0; i < N; i++) env[i] = sqrtf(I[i]*I[i] + Q[i]*Q[i]);
    std::sort(env.begin(), env.end());
    float noise_est = env[(int)(N * 0.05f)];
    float noise_power = noise_est * noise_est;
    r.snr_est = (noise_power > 1e-20f) ? 10.0f * log10f(sig_power / noise_power) : 40.0f;

    // Step 1: Analog vs Digital
    if (f.env_cv < 0.15f && f.freq_std > 0.05f) {
        r.mod = ModType::FM; r.confidence = 0.7f;
        r.is_digital = false;
        return r;
    }
    if (f.spec_asymmetry > 0.28f) {
        r.mod = ModType::SSB; r.confidence = 0.6f;
        r.is_digital = false;
        return r;
    }
    if (f.env_cv > 0.2f && f.env_cv < 0.5f && f.freq_std < 0.03f) {
        r.mod = ModType::AM; r.confidence = 0.6f;
        r.is_digital = false;
        return r;
    }

    // Digital classification
    r.is_digital = true;

    // Step 2: Constant envelope test
    bool const_env = (f.env_gamma < 1.3f && f.env_cv < 0.15f);

    if (const_env) {
        // Step 3A: FSK vs PSK
        if (f.freq_n_levels >= 4) {
            r.mod = ModType::FSK4; r.confidence = 0.7f;
        } else if (f.freq_n_levels >= 2) {
            // Could be 2FSK, GFSK, or MSK
            r.mod = ModType::FSK2; r.confidence = 0.7f;
        } else {
            // PSK: use cumulants
            if (f.cum.abs_C40 > 1.5f) {
                r.mod = ModType::BPSK; r.confidence = 0.8f;
            } else if (f.cum.abs_C40 > 0.5f) {
                r.mod = ModType::QPSK; r.confidence = 0.7f;
            } else {
                r.mod = ModType::PSK8; r.confidence = 0.6f;
            }
        }
    } else {
        // Step 3B: Non-constant envelope
        if (f.cum.abs_C42 > 1.5f) {
            // BPSK or ASK
            if (f.freq_std < 0.1f) {
                // Low freq variation = ASK/OOK
                // Check if OOK (min envelope near zero)
                float min_env = env[(int)(N * 0.05f)];
                if (min_env / f.env_mean < 0.15f)
                    r.mod = ModType::OOK;
                else
                    r.mod = ModType::ASK2;
                r.confidence = 0.7f;
            } else {
                r.mod = ModType::BPSK; r.confidence = 0.6f;
            }
        } else if (f.cum.abs_C42 > 0.3f && f.cum.abs_C42 < 0.9f &&
                   f.cum.abs_C40 < 0.9f) {
            r.mod = ModType::QAM16; r.confidence = 0.5f;
        } else {
            // Ambiguous - could be FSK with envelope variation
            if (f.freq_n_levels >= 2) {
                r.mod = ModType::FSK2; r.confidence = 0.5f;
            } else {
                r.mod = ModType::UNKNOWN; r.confidence = 0.3f;
            }
        }
    }

    return r;
}

// ─────────────────────────────────────────────────────────────────────────
// Baud rate estimation
// ─────────────────────────────────────────────────────────────────────────
float estimate_baud_rate(const float* I, const float* Q, int N,
                         float work_sr, ModType mod_hint)
{
    const int BAUD_FFT_N = 8192;
    if (N < BAUD_FFT_N / 2) return 0;

    fftwf_complex* in  = nullptr;
    fftwf_complex* out = nullptr;
    fftwf_plan plan = nullptr;
    {
        std::lock_guard<std::mutex> lk(g_fftw_mtx);
        in  = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * BAUD_FFT_N);
        out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * BAUD_FFT_N);
        plan = fftwf_plan_dft_1d(BAUD_FFT_N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    }

    memset(in, 0, sizeof(fftwf_complex) * BAUD_FFT_N);
    int use_N = std::min(N, BAUD_FFT_N);

    // Prepare input based on modulation type
    if (mod_hint == ModType::FSK2 || mod_hint == ModType::FSK4 ||
        mod_hint == ModType::GFSK || mod_hint == ModType::MSK) {
        // Phase derivative squared
        float prev_phase = atan2f(Q[0], I[0]);
        for (int i = 1; i < use_N; i++) {
            float phase = atan2f(Q[i], I[i]);
            float dp = phase - prev_phase;
            if (dp >  M_PIf) dp -= 2.0f * M_PIf;
            if (dp < -M_PIf) dp += 2.0f * M_PIf;
            prev_phase = phase;
            in[i-1][0] = dp * dp;
            in[i-1][1] = 0;
        }
    } else if (mod_hint == ModType::BPSK) {
        // Squared signal (removes 180-degree modulation)
        for (int i = 0; i < use_N; i++) {
            float r = I[i]*I[i] - Q[i]*Q[i];
            float j = 2.0f * I[i] * Q[i];
            float env = sqrtf(r*r + j*j);
            in[i][0] = env * env;
            in[i][1] = 0;
        }
    } else if (mod_hint == ModType::QPSK) {
        // Fourth power
        for (int i = 0; i < use_N; i++) {
            float r2 = I[i]*I[i] - Q[i]*Q[i];
            float j2 = 2.0f * I[i] * Q[i];
            float r4 = r2*r2 - j2*j2;
            float j4 = 2.0f * r2 * j2;
            float env = sqrtf(r4*r4 + j4*j4);
            in[i][0] = env;
            in[i][1] = 0;
        }
    } else {
        // Default: squared envelope (works for ASK/OOK/QAM/unknown)
        for (int i = 0; i < use_N; i++) {
            in[i][0] = I[i]*I[i] + Q[i]*Q[i];
            in[i][1] = 0;
        }
    }

    // Remove DC
    double dc = 0;
    for (int i = 0; i < BAUD_FFT_N; i++) dc += in[i][0];
    dc /= BAUD_FFT_N;
    for (int i = 0; i < BAUD_FFT_N; i++) in[i][0] -= (float)dc;

    // Apply Blackman-Harris window
    for (int i = 0; i < BAUD_FFT_N; i++) {
        double x = 2.0 * M_PI * i / (BAUD_FFT_N - 1);
        float w = (float)(0.35875 - 0.48829*cos(x) + 0.14128*cos(2*x) - 0.01168*cos(3*x));
        in[i][0] *= w;
    }

    fftwf_execute(plan);

    // Find peak (skip DC bin, search reasonable baud range: 50 Hz to work_sr/4)
    int bin_min = std::max(1, (int)(50.0f * BAUD_FFT_N / work_sr));
    int bin_max = std::min(BAUD_FFT_N / 2, (int)(work_sr / 4.0f * BAUD_FFT_N / work_sr));

    float peak_mag = 0;
    int peak_bin = bin_min;
    for (int i = bin_min; i < bin_max; i++) {
        float mag = out[i][0]*out[i][0] + out[i][1]*out[i][1];
        if (mag > peak_mag) { peak_mag = mag; peak_bin = i; }
    }

    float raw_baud = (float)peak_bin * work_sr / (float)BAUD_FFT_N;

    // Parabolic interpolation for sub-bin precision
    if (peak_bin > bin_min && peak_bin < bin_max - 1) {
        float y0 = out[peak_bin-1][0]*out[peak_bin-1][0] + out[peak_bin-1][1]*out[peak_bin-1][1];
        float y1 = peak_mag;
        float y2 = out[peak_bin+1][0]*out[peak_bin+1][0] + out[peak_bin+1][1]*out[peak_bin+1][1];
        float delta = 0.5f * (y0 - y2) / (y0 - 2*y1 + y2 + 1e-30f);
        raw_baud = ((float)peak_bin + delta) * work_sr / (float)BAUD_FFT_N;
    }

    {
        std::lock_guard<std::mutex> lk(g_fftw_mtx);
        fftwf_destroy_plan(plan);
        fftwf_free(in);
        fftwf_free(out);
    }

    return snap_baud(raw_baud);
}

// ─────────────────────────────────────────────────────────────────────────
// Signal Database
// ─────────────────────────────────────────────────────────────────────────
const SigProfile SIG_DB[] = {
    {"AIS", "Automatic Identification System",
     156.025f, 162.025f,
     {ModType::GFSK, ModType::MSK, ModType::FSK2}, 3,
     {9600.0f}, 1, 0.05f,
     0x7E7E, 16, 25.0f, 0},

    {"POCSAG-512", "Pager 512 baud",
     0, 500,
     {ModType::FSK2}, 1,
     {512.0f}, 1, 0.05f,
     0x7CD215D8ULL, 32, 25.0f, 1},

    {"POCSAG-1200", "Pager 1200 baud",
     0, 500,
     {ModType::FSK2}, 1,
     {1200.0f}, 1, 0.05f,
     0x7CD215D8ULL, 32, 25.0f, 1},

    {"POCSAG-2400", "Pager 2400 baud",
     0, 500,
     {ModType::FSK2}, 1,
     {2400.0f}, 1, 0.05f,
     0x7CD215D8ULL, 32, 25.0f, 1},

    {"FLEX", "Motorola FLEX Pager",
     0, 500,
     {ModType::FSK2, ModType::FSK4}, 2,
     {1600.0f, 3200.0f, 6400.0f}, 3, 0.05f,
     0xA6C6AAAAULL, 32, 25.0f, -1},

    {"ACARS", "Aircraft Comm Addressing",
     129.0f, 137.0f,
     {ModType::ASK2, ModType::OOK, ModType::MSK}, 3,
     {2400.0f}, 1, 0.05f,
     0x2B2AULL, 16, 25.0f, 2},

    {"APRS", "Amateur Packet Radio",
     144.0f, 145.0f,
     {ModType::FSK2, ModType::GFSK}, 2,
     {1200.0f}, 1, 0.05f,
     0x7EULL, 8, 25.0f, -1},

    {"DMR", "Digital Mobile Radio",
     136.0f, 470.0f,
     {ModType::FSK4}, 1,
     {4800.0f}, 1, 0.05f,
     0x755FD7DF75F7ULL, 48, 12.5f, -1},

    {"P25", "APCO Project 25",
     136.0f, 870.0f,
     {ModType::FSK4}, 1,
     {4800.0f}, 1, 0.05f,
     0x5575F5FF77FFULL, 48, 12.5f, -1},

    {"NXDN", "NXDN Digital",
     136.0f, 470.0f,
     {ModType::FSK4}, 1,
     {4800.0f, 2400.0f}, 2, 0.05f,
     0, 0, 6.25f, -1},

    {"TETRA", "TETRA Trunked Radio",
     380.0f, 400.0f,
     {ModType::QPSK}, 1,
     {18000.0f}, 1, 0.05f,
     0, 0, 25.0f, -1},

    {"Cospas-Sarsat", "Emergency Beacon 406MHz",
     406.0f, 406.1f,
     {ModType::BPSK}, 1,
     {400.0f}, 1, 0.10f,
     0x2CF2ULL, 15, 3.0f, -1},

    {"NOAA APT", "Weather Satellite APT",
     137.1f, 137.9125f,
     {ModType::AM, ModType::FM}, 2,
     {4160.0f}, 1, 0.05f,
     0, 0, 34.0f, -1},
};

const int SIG_DB_COUNT = sizeof(SIG_DB) / sizeof(SIG_DB[0]);

std::vector<SigMatch> match_signal(float center_freq_mhz, ModType detected_mod,
                                   float detected_baud, float channel_bw_khz)
{
    std::vector<SigMatch> matches;

    for (int i = 0; i < SIG_DB_COUNT; i++) {
        const SigProfile& p = SIG_DB[i];
        float score = 0;

        // Frequency match (0.3 weight)
        if (p.freq_mhz_min > 0 || p.freq_mhz_max > 0) {
            if (center_freq_mhz >= p.freq_mhz_min &&
                center_freq_mhz <= p.freq_mhz_max)
                score += 0.3f;
        }

        // Modulation match (0.35 weight)
        for (int m = 0; m < p.mod_count; m++) {
            if (p.mod_types[m] == detected_mod) {
                score += 0.35f; break;
            }
            // Partial credit for related types
            if ((p.mod_types[m] == ModType::GFSK && detected_mod == ModType::FSK2) ||
                (p.mod_types[m] == ModType::MSK  && detected_mod == ModType::FSK2) ||
                (p.mod_types[m] == ModType::FSK2 && detected_mod == ModType::GFSK)) {
                score += 0.25f; break;
            }
        }

        // Baud rate match (0.35 weight)
        if (p.baud_count > 0 && detected_baud > 0) {
            for (int b = 0; b < p.baud_count; b++) {
                float err = fabsf(detected_baud - p.baud_rates[b]) / p.baud_rates[b];
                if (err < p.baud_tolerance) {
                    score += 0.35f * (1.0f - err / p.baud_tolerance);
                    break;
                }
            }
        }

        if (score > 0.3f)
            matches.push_back({i, score});
    }

    std::sort(matches.begin(), matches.end(),
              [](const SigMatch& a, const SigMatch& b){ return a.score > b.score; });
    return matches;
}

// ─────────────────────────────────────────────────────────────────────────
// Auto-ID worker thread (state machine)
// ─────────────────────────────────────────────────────────────────────────
void FFTViewer::auto_id_worker(int ch_idx)
{
    Channel& ch = channels[ch_idx];
    uint32_t msr = header.sample_rate;

    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(header.center_frequency/1e6f)) * 1e6f;
    float bw_hz  = fabsf(ch.e - ch.s) * 1e6f;
    float center_freq_mhz = (ch.s + ch.e) / 2.0f;

    // Decimation target: work_sr = max(50kHz, bw*2.5)
    uint32_t target_work = std::max(50000u, (uint32_t)(bw_hz * 2.5f));
    uint32_t total_decim = std::max(1u, msr / target_work);
    uint32_t work_sr = msr / total_decim;

    bewe_log_push(0, "[AUTO_ID] CH%d start cf=%.4fMHz bw=%.1fkHz work_sr=%u\n",
                  ch_idx, center_freq_mhz, bw_hz/1000, work_sr);

    // DSP state
    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    double cap_i = 0, cap_q = 0; int cap_cnt = 0;

    float lpf_cn = (bw_hz * 0.45f) / (float)work_sr;
    if (lpf_cn > 0.45f) lpf_cn = 0.45f;
    IIR1 lpi1, lpq1, lpi2, lpq2;
    lpi1.set(lpf_cn); lpq1.set(lpf_cn);
    lpi2.set(lpf_cn); lpq2.set(lpf_cn);

    // Analysis buffers (1.5 seconds)
    const int ANALYSIS_SAMP = (int)(work_sr * 1.5f);
    std::vector<float> ana_i, ana_q;
    ana_i.reserve(ANALYSIS_SAMP);
    ana_q.reserve(ANALYSIS_SAMP);

    // State machine
    AutoIdState state = AutoIdState::COLLECTING;
    ch.auto_id.state.store((int)state);
    snprintf(ch.auto_id.protocol_name, sizeof(ch.auto_id.protocol_name), "---");

    // Demod state
    float prev_i = 0, prev_q = 0, disc_dc = 0, agc_peak = 0.001f;
    float sym_phase = 0, sps = 0;
    bool was_gate_open = false;

    // Decoder
    std::unique_ptr<ProtoDecoder> decoder;

    // Re-analysis timer
    int decode_sample_count = 0;
    const int REANALYZE_INTERVAL = work_sr * 30;

    const size_t BATCH = std::max((size_t)1, (size_t)(total_decim * work_sr / 50));

    while (!ch.digi_stop_req.load(std::memory_order_relaxed) && !sdr_stream_error.load()) {
        size_t wp = ring_wp.load(std::memory_order_acquire);
        size_t rp = ch.digi_rp.load(std::memory_order_relaxed);
        if (rp == wp) {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }
        size_t lag = (wp - rp) & IQ_RING_MASK;
        if (lag > IQ_RING_CAPACITY / 2) {
            // Too far behind: skip to recent
            rp = (wp - (size_t)(msr * 0.02)) & IQ_RING_MASK;
            ch.digi_rp.store(rp, std::memory_order_release);
            continue;
        }
        size_t avail = std::min(lag, BATCH);

        for (size_t s = 0; s < avail; s++) {
            size_t pos = (rp + s) & IQ_RING_MASK;
            float si = ring[pos*2] / hw.iq_scale;
            float sq = ring[pos*2+1] / hw.iq_scale;
            float mi, mq; osc.mix(si, sq, mi, mq);
            cap_i += mi; cap_q += mq; cap_cnt++;
            if (cap_cnt < (int)total_decim) continue;

            float fi = (float)(cap_i / cap_cnt);
            float fq = (float)(cap_q / cap_cnt);
            cap_i = cap_q = 0; cap_cnt = 0;
            fi = lpi1.p(fi); fq = lpq1.p(fq);
            fi = lpi2.p(fi); fq = lpq2.p(fq);

            bool gate_open = ch.sq_gate.load(std::memory_order_relaxed);

            switch (state) {
            case AutoIdState::COLLECTING: {
                if (!gate_open) { was_gate_open = false; continue; }
                ana_i.push_back(fi);
                ana_q.push_back(fq);
                if ((int)ana_i.size() >= ANALYSIS_SAMP) {
                    state = AutoIdState::ANALYZING;
                    ch.auto_id.state.store((int)state);
                }
                break;
            }

            case AutoIdState::ANALYZING: {
                // Run AMC
                AmcResult amc = classify_modulation(
                    ana_i.data(), ana_q.data(), (int)ana_i.size(), (float)work_sr);
                ch.auto_id.mod_type.store((int)amc.mod);
                ch.auto_id.snr_est.store(amc.snr_est);

                // Baud estimation
                float baud = estimate_baud_rate(
                    ana_i.data(), ana_q.data(), (int)ana_i.size(),
                    (float)work_sr, amc.mod);
                ch.auto_id.baud_rate.store(baud);

                bewe_log_push(0, "[AUTO_ID] CH%d: %s %.0f bd SNR=%.1fdB conf=%.0f%%\n",
                              ch_idx, mod_type_name(amc.mod), baud, amc.snr_est,
                              amc.confidence * 100);

                // Signal DB matching
                state = AutoIdState::MATCHING;
                ch.auto_id.state.store((int)state);

                auto matches = match_signal(center_freq_mhz, amc.mod, baud, bw_hz/1000);

                if (!matches.empty() && matches[0].score > 0.5f) {
                    const SigProfile& sig = SIG_DB[matches[0].db_idx];
                    snprintf(ch.auto_id.protocol_name, sizeof(ch.auto_id.protocol_name),
                             "%s", sig.name);
                    ch.auto_id.confidence.store(matches[0].score);
                    bewe_log_push(0, "[AUTO_ID] CH%d: matched %s (%.0f%%)\n",
                                  ch_idx, sig.name, matches[0].score * 100);
                    decoder = create_decoder(sig.decoder_id);
                } else {
                    snprintf(ch.auto_id.protocol_name, sizeof(ch.auto_id.protocol_name),
                             "Unknown");
                    ch.auto_id.confidence.store(amc.confidence);
                    decoder = create_decoder(-1); // generic hex
                }

                // Setup demod
                sps = (baud > 0) ? (float)work_sr / baud : 16.0f;
                sym_phase = 0; prev_i = prev_q = 0; disc_dc = 0; agc_peak = 0.001f;
                decode_sample_count = 0;
                was_gate_open = false;

                ana_i.clear(); ana_q.clear();
                ana_i.shrink_to_fit(); ana_q.shrink_to_fit();

                state = AutoIdState::DECODING;
                ch.auto_id.state.store((int)state);

                // Output initial identification to DEMOD log
                {
                    time_t now = time(nullptr);
                    struct tm tm2; localtime_r(&now, &tm2);
                    char ts[12]; strftime(ts, sizeof(ts), "%H:%M:%S", &tm2);
                    digi_log_push(3, "[%s] CH%d AUTO: %s %s %.0fbd (%.0f%%)",
                                 ts, ch_idx, ch.auto_id.protocol_name,
                                 mod_type_name(amc.mod), baud, ch.auto_id.confidence.load()*100);
                }
                break;
            }

            case AutoIdState::DECODING: {
                if (!gate_open) {
                    if (was_gate_open) {
                        sym_phase = 0;
                        if (decoder) decoder->reset();
                    }
                    was_gate_open = false;
                    prev_i = fi; prev_q = fq;
                    continue;
                }
                was_gate_open = true;

                // Demodulate based on detected modulation
                ModType mod = (ModType)ch.auto_id.mod_type.load();
                float demod_val = 0;

                switch (mod) {
                case ModType::OOK: case ModType::ASK2: case ModType::ASK4: {
                    float env = sqrtf(fi*fi + fq*fq);
                    disc_dc += 0.001f * (env - disc_dc);
                    demod_val = env - disc_dc;
                    break;
                }
                case ModType::FSK2: case ModType::FSK4:
                case ModType::GFSK: case ModType::MSK: {
                    float cross = fi*prev_q - fq*prev_i;
                    float dot   = fi*prev_i + fq*prev_q;
                    float disc  = atan2f(cross, dot + 1e-30f);
                    disc_dc += 0.001f * (disc - disc_dc);
                    demod_val = disc - disc_dc;
                    break;
                }
                case ModType::BPSK: {
                    float dot = fi*prev_i + fq*prev_q;
                    demod_val = dot;
                    break;
                }
                case ModType::QPSK: {
                    float dot = fi*prev_i + fq*prev_q;
                    demod_val = dot;
                    break;
                }
                default:
                    demod_val = sqrtf(fi*fi + fq*fq);
                    break;
                }
                prev_i = fi; prev_q = fq;

                // AGC
                float aabs = fabsf(demod_val);
                if (aabs > agc_peak)
                    agc_peak = 0.05f * aabs + 0.95f * agc_peak;
                else
                    agc_peak = 0.0005f * aabs + 0.9995f * agc_peak;
                if (agc_peak < 0.001f) agc_peak = 0.001f;
                demod_val /= agc_peak;

                // Symbol timing
                sym_phase += 1.0f;
                if (sym_phase >= sps) {
                    sym_phase -= sps;
                    uint8_t bit = (demod_val > 0) ? 1 : 0;

                    if (decoder && decoder->feed_bit(bit)) {
                        DecoderOutput out = decoder->get_output();
                        if (out.valid && !out.summary.empty()) {
                            time_t now = time(nullptr);
                            struct tm tm2; localtime_r(&now, &tm2);
                            char ts[12]; strftime(ts, sizeof(ts), "%H:%M:%S", &tm2);
                            digi_log_push(3, "[%s] CH%d %s | %s",
                                         ts, ch_idx,
                                         ch.auto_id.protocol_name,
                                         out.summary.c_str());
                        }
                    }
                }

                // Re-analysis timer
                decode_sample_count++;
                if (decode_sample_count >= REANALYZE_INTERVAL) {
                    state = AutoIdState::COLLECTING;
                    ch.auto_id.state.store((int)state);
                    ana_i.clear(); ana_q.clear();
                    ana_i.reserve(ANALYSIS_SAMP);
                    ana_q.reserve(ANALYSIS_SAMP);
                    decode_sample_count = 0;
                }
                break;
            }

            default: break;
            }
        }
        ch.digi_rp.store((rp + avail) & IQ_RING_MASK, std::memory_order_release);
    }

    ch.digi_run.store(false);
    bewe_log_push(0, "[AUTO_ID] CH%d worker exited\n", ch_idx);
}
