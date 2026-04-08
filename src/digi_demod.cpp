#include "fft_viewer.hpp"
#include <cmath>
#include <cstring>
#include <chrono>
#include <ctime>

// Generic digital demodulator: ASK / FSK / BPSK
// Runs as separate thread per channel (same pattern as ais_worker)
void FFTViewer::digi_demod_worker(int ch_idx){
    Channel& ch = channels[ch_idx];
    uint32_t msr = header.sample_rate;
    float baud = ch.digi_baud_rate;
    int demod_type = ch.digi_demod_type; // 0=ASK 1=FSK 2=BPSK

    if(baud < 10.f) baud = 10.f;
    if(baud > 100000.f) baud = 100000.f;

    static const char* type_names[] = {"ASK","FSK","BPSK"};
    const char* tname = type_names[demod_type < 3 ? demod_type : 0];

    // Decimation: target 16x oversampling
    uint32_t target_work = (uint32_t)(baud * 16);
    if(target_work < 1000) target_work = 1000;
    uint32_t total_decim = std::max(1u, (msr + target_work/2) / target_work);
    uint32_t work_sr = msr / total_decim;
    float sps = (float)work_sr / baud; // samples per symbol

    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(header.center_frequency/1e6f)) * 1e6f;
    float bw_hz = fabsf(ch.e - ch.s) * 1e6f;

    bewe_log_push(0, "[DEMOD] CH%d %s %0.fbd  work_sr=%u sps=%.1f\n",
                  ch_idx, tname, baud, work_sr, sps);

    // DSP state
    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    double cap_i=0, cap_q=0; int cap_cnt=0;

    // Two-stage IIR LPF
    float lpf_cn = (baud * 0.8f) / (float)work_sr;
    if(lpf_cn > 0.45f) lpf_cn = 0.45f;
    IIR1 lpi1, lpq1, lpi2, lpq2;
    lpi1.set(lpf_cn); lpq1.set(lpf_cn);
    lpi2.set(lpf_cn); lpq2.set(lpf_cn);

    // Demod state
    float prev_i = 0, prev_q = 0;
    float disc_dc = 0;
    float agc_peak = 0.001f;
    const float DC_ALPHA = 1.0f - expf(-2.0f * M_PIf * 200.0f / (float)work_sr);
    const float AGC_ATTACK = 0.05f;
    const float AGC_DECAY  = 0.0005f;

    // Symbol timing
    float sym_phase = 0;
    uint8_t byte_val = 0;
    int bit_count = 0;
    int total_bits = 0;
    bool was_gate_open = false;

    // Hex output buffer
    char hex_line[256];
    int hex_pos = 0;

    auto flush_hex = [&](){
        if(hex_pos > 0){
            hex_line[hex_pos] = '\0';
            time_t now = time(nullptr);
            struct tm tm2; localtime_r(&now, &tm2);
            char ts[12]; strftime(ts, sizeof(ts), "%H:%M:%S", &tm2);
            digi_log_push(3, "[%s] CH%d %s %0.fbd | %s",
                         ts, ch_idx, tname, baud, hex_line);
            hex_pos = 0;
        }
    };

    const size_t BATCH = std::max((size_t)1, (size_t)(total_decim * work_sr / 50));

    while(!ch.digi_stop_req.load(std::memory_order_relaxed) && !sdr_stream_error.load()){
        size_t wp = ring_wp.load(std::memory_order_acquire);
        size_t rp = ch.digi_rp.load(std::memory_order_relaxed);
        if(rp == wp){ std::this_thread::sleep_for(std::chrono::microseconds(200)); continue; }
        size_t avail = std::min((wp - rp) & IQ_RING_MASK, BATCH);

        for(size_t s = 0; s < avail; s++){
            size_t pos = (rp + s) & IQ_RING_MASK;
            float si = ring[pos*2] / hw.iq_scale;
            float sq = ring[pos*2+1] / hw.iq_scale;
            float mi, mq; osc.mix(si, sq, mi, mq);
            cap_i += mi; cap_q += mq; cap_cnt++;
            if(cap_cnt < (int)total_decim) continue;

            float fi = (float)(cap_i / cap_cnt);
            float fq = (float)(cap_q / cap_cnt);
            cap_i = cap_q = 0; cap_cnt = 0;

            // Two-stage LPF
            fi = lpi1.p(fi); fq = lpq1.p(fq);
            fi = lpi2.p(fi); fq = lpq2.p(fq);

            // Squelch gate
            bool gate_open = ch.sq_gate.load(std::memory_order_relaxed);
            if(!gate_open){
                if(was_gate_open){
                    flush_hex();
                    sym_phase = 0; bit_count = 0; byte_val = 0;
                    total_bits = 0;
                }
                was_gate_open = false;
                prev_i = fi; prev_q = fq;
                continue;
            }
            was_gate_open = true;

            // Demodulate
            float demod_val = 0;
            switch(demod_type){
            case 0: { // ASK
                float env = sqrtf(fi*fi + fq*fq);
                disc_dc += DC_ALPHA * (env - disc_dc);
                demod_val = env - disc_dc;
                break;
            }
            case 1: { // FSK
                float cross = fi*prev_q - fq*prev_i;
                float dot   = fi*prev_i + fq*prev_q;
                float disc  = atan2f(cross, dot + 1e-30f);
                disc_dc += DC_ALPHA * (disc - disc_dc);
                demod_val = disc - disc_dc;
                break;
            }
            case 2: { // BPSK (differential)
                float dot = fi*prev_i + fq*prev_q;
                demod_val = dot;
                break;
            }
            }
            prev_i = fi; prev_q = fq;

            // AGC
            float aabs = fabsf(demod_val);
            if(aabs > agc_peak)
                agc_peak = AGC_ATTACK * aabs + (1.0f - AGC_ATTACK) * agc_peak;
            else
                agc_peak = AGC_DECAY * aabs + (1.0f - AGC_DECAY) * agc_peak;
            if(agc_peak < 0.001f) agc_peak = 0.001f;
            demod_val /= agc_peak;

            // Symbol timing (fixed-rate)
            sym_phase += 1.0f;
            if(sym_phase >= sps){
                sym_phase -= sps;

                uint8_t bit = (demod_val > 0) ? 1 : 0;
                byte_val = (byte_val << 1) | bit;
                bit_count++;
                total_bits++;

                if(bit_count >= 8){
                    if(hex_pos < 240){
                        hex_pos += snprintf(hex_line + hex_pos,
                                           sizeof(hex_line) - hex_pos,
                                           "%02X ", byte_val);
                    }
                    bit_count = 0;
                    byte_val = 0;

                    // Flush every 16 bytes (one line)
                    if(total_bits >= 128){
                        flush_hex();
                        total_bits = 0;
                    }
                }
            }
        }
        ch.digi_rp.store(rp + avail, std::memory_order_release);
    }

    flush_hex();
    ch.digi_run.store(false);
    bewe_log_push(0, "[DEMOD] CH%d worker exited\n", ch_idx);
}
