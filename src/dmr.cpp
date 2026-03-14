extern "C" {
#include <mbelib.h>
}
#include "fft_viewer.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <chrono>

// ── DMR 상수 ──────────────────────────────────────────────────────────────
static constexpr int    DMR_BAUD        = 4800;
static constexpr int    DMR_SLOT_BITS   = 264;
static constexpr int    MBE_FRAME_SAMPS = 160;

// DMR Voice Sync (48비트)
static const uint8_t DMR_BS_VOICE_SYNC[48] = {
    0,1,1,1,0,1,0,1, 0,1,0,1,1,1,1,1,
    1,1,0,1,0,1,1,1, 1,1,0,1,1,1,1,1,
    0,1,1,1,0,1,0,1, 1,1,1,1,0,1,1,1
};
static const uint8_t DMR_MS_VOICE_SYNC[48] = {
    1,1,1,1,1,1,0,0, 1,1,1,0,0,1,0,0,
    0,0,1,0,0,1,1,0, 0,0,1,0,0,1,1,0,
    1,1,1,1,1,1,0,0, 1,1,0,0,1,0,0,0
};
static const uint8_t DMR_BS_DATA_SYNC[48] = {
    1,1,0,1,1,1,0,0, 1,1,0,1,0,0,0,1,
    0,1,0,0,1,1,1,0, 0,1,0,0,1,1,1,0,
    1,1,0,1,1,1,0,0, 0,1,1,0,0,0,1,1
};

static inline int sym_to_dibit(float sym){
    if(sym >  2.0f) return 0x01;  // +3 → 01
    if(sym >= 0.0f) return 0x00;  // +1 → 00
    if(sym > -2.0f) return 0x02;  // -1 → 10
    return 0x03;                   // -3 → 11
}

static int sync_match(const uint8_t* bits, const uint8_t* pattern, int len){
    int errs = 0;
    for(int i=0;i<len;i++) if(bits[i] != pattern[i]) errs++;
    return errs;
}

static int find_sync(const uint8_t* bits, int bit_cnt, int max_err,
                     int& out_err, bool& out_voice, bool& out_bs){
    for(int start=0; start <= bit_cnt - DMR_SLOT_BITS; start++){
        const uint8_t* sp = bits + start + 98;
        int ev = sync_match(sp, DMR_BS_VOICE_SYNC, 48);
        int em = sync_match(sp, DMR_MS_VOICE_SYNC, 48);
        int ed = sync_match(sp, DMR_BS_DATA_SYNC,  48);
        int best = std::min({ev, em, ed});
        if(best <= max_err){
            out_err = best;
            out_voice = (best == ev || best == em);
            out_bs = (best == ev || best == ed);
            return start;
        }
    }
    return -1;
}

// ── DMR 워커 ──────────────────────────────────────────────────────────────
void FFTViewer::dmr_worker(int ch_idx){
    Channel& ch = channels[ch_idx];
    uint32_t msr = header.sample_rate;

    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(header.center_frequency/1e6f)) * 1e6f;
    float bw_hz  = fabsf(ch.e - ch.s) * 1e6f;
    if(bw_hz < 6250.0f) bw_hz = 12500.0f;

    // sps=10 목표
    uint32_t target_sr = DMR_BAUD * 10;
    uint32_t cap_decim = std::max(1u, msr / target_sr);
    uint32_t inter_sr  = msr / cap_decim;
    float sps = (float)inter_sr / (float)DMR_BAUD;

    printf("[DMR ch%d] start off=%.0fHz inter_sr=%u sps=%.2f decim=%u msr=%u\n",
           ch_idx, off_hz, inter_sr, sps, cap_decim, msr);
    fflush(stdout);

    // DSP
    Oscillator osc;
    osc.set_freq((double)off_hz, (double)msr);
    double cap_i=0, cap_q=0;
    int cap_cnt=0;

    // ── Pre-decimation LPF: msr에서 3단 IIR1 (±5kHz) ─────────────────────
    // 각 단 6dB/oct → 3단 = 18dB/oct → aliasing 대폭 억제
    IIR1 pre_lpi[3], pre_lpq[3];
    { float cn = 5000.0f / (float)msr;
      if(cn > 0.45f) cn = 0.45f;
      for(int i=0;i<3;i++){ pre_lpi[i].set(cn); pre_lpq[i].set(cn); } }

    // ── Post-decimation LPF: inter_sr에서 2단 IIR1 (±4kHz) ───────────────
    IIR1 post_lpi[2], post_lpq[2];
    { float cn = 4000.0f / (float)inter_sr;
      if(cn > 0.45f) cn = 0.45f;
      for(int i=0;i<2;i++){ post_lpi[i].set(cn); post_lpq[i].set(cn); } }

    // Squelch
    const float SQL_ALPHA     = 0.05f;
    const int   SQL_HOLD_SAMP = (int)(inter_sr * 0.3f);
    const int   CALIB_SAMP    = (int)(inter_sr * 0.5f);
    float sql_avg = -120.0f;
    bool  calibrated = ch.sq_calibrated.load(std::memory_order_relaxed);
    bool  gate_open  = false;
    int   gate_hold  = 0;
    int   sq_ui_tick = 0;
    std::vector<float> calib_buf;
    if(!calibrated) calib_buf.reserve(CALIB_SAMP);

    // FM discriminator
    float prev_i=0, prev_q=0;

    // disc를 주파수 편차 단위로 정규화: disc × inter_sr/(2π) = Hz
    // DMR: outer=±1944Hz, inner=±648Hz → ±3/±1 레벨 목표
    // scale factor: 3.0 / (2π × 1944 / inter_sr)
    float disc_to_sym = (float)inter_sr / (2.0f * (float)M_PI * 648.0f);
    // 이렇게 하면: inner(±648Hz) → disc=±0.084 → ×disc_to_sym = ±1.0
    //             outer(±1944Hz) → disc=±0.253 → ×disc_to_sym = ±3.0

    // DC remover (시상수 ~50ms — 충분히 길게)
    float disc_dc = 0.f;
    const float DC_ALPHA = 1.f / (0.05f * (float)inter_sr);

    // M&M 타이밍
    float sym_phase = 0.f, mu = 0.f, p_sym = 0.f;

    // 비트 버퍼
    static constexpr int BUF_BITS = DMR_SLOT_BITS * 8;
    uint8_t bits[BUF_BITS] = {};
    int     bit_cnt = 0;

    // mbelib
    char ambe_fr[4][24];
    char ambe_d[49];
    mbe_parms cur_mp, prev_mp, enh_mp;
    mbe_initMbeParms(&cur_mp, &prev_mp, &enh_mp);

    const int up_int = 6; // 8kHz → 48kHz

    const size_t MAX_LAG = (size_t)(msr * 0.08);
    const size_t BATCH   = (size_t)(msr / 50);

    int  voice_burst_cnt = 0;
    auto last_info_time = std::chrono::steady_clock::now();
    auto last_diag_time = std::chrono::steady_clock::now();
    int  diag_sym_cnt = 0;
    bool gate_was_open = false;

    while(!ch.digi_stop_req.load(std::memory_order_relaxed)){
        size_t wp  = ring_wp.load(std::memory_order_acquire);
        size_t rp  = ch.digi_rp.load(std::memory_order_relaxed);
        size_t lag = (wp - rp) & IQ_RING_MASK;

        if(lag > MAX_LAG){
            size_t keep = (size_t)(msr * 0.02);
            rp = (wp - keep) & IQ_RING_MASK;
            ch.digi_rp.store(rp, std::memory_order_release);
            lag = (wp - rp) & IQ_RING_MASK;
            prev_i = prev_q = 0; bit_cnt = 0;
            cap_i = cap_q = 0; cap_cnt = 0;
            for(int i=0;i<3;i++){ pre_lpi[i].s=0; pre_lpq[i].s=0; }
            for(int i=0;i<2;i++){ post_lpi[i].s=0; post_lpq[i].s=0; }
            disc_dc = 0.f; sym_phase = mu = p_sym = 0.f;
        }
        if(lag == 0){
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            continue;
        }

        size_t avail = std::min(lag, BATCH);
        for(size_t s=0; s<avail; s++){
            size_t pos = (rp + s) & IQ_RING_MASK;
            float ri = ring[pos*2  ] / hw.iq_scale;
            float rq = ring[pos*2+1] / hw.iq_scale;

            float mi, mq;
            osc.mix(ri, rq, mi, mq);

            // ── Pre-decimation LPF (3단, msr에서) ───────────────────────
            for(int i=0;i<3;i++){ mi = pre_lpi[i].p(mi); mq = pre_lpq[i].p(mq); }

            // decimation (box-car)
            cap_i += mi; cap_q += mq; cap_cnt++;
            if(cap_cnt < (int)cap_decim) continue;
            float di = (float)(cap_i / cap_cnt);
            float dq = (float)(cap_q / cap_cnt);
            cap_i = cap_q = 0; cap_cnt = 0;

            // ── Post-decimation LPF (2단, inter_sr에서) ─────────────────
            for(int i=0;i<2;i++){ di = post_lpi[i].p(di); dq = post_lpq[i].p(dq); }

            // Squelch
            float p_inst = di*di + dq*dq;
            float db_inst = (p_inst > 1e-12f) ? 10.0f*log10f(p_inst) : -120.0f;
            sql_avg = SQL_ALPHA*db_inst + (1.0f-SQL_ALPHA)*sql_avg;
            if(!calibrated){
                if((int)calib_buf.size() < CALIB_SAMP) calib_buf.push_back(db_inst);
                if((int)calib_buf.size() >= CALIB_SAMP){
                    std::vector<float> tmp = calib_buf;
                    size_t p20 = tmp.size()/5;
                    std::nth_element(tmp.begin(), tmp.begin()+(ptrdiff_t)p20, tmp.end());
                    ch.sq_threshold.store(tmp[p20]+10.0f, std::memory_order_relaxed);
                    calibrated=true;
                    ch.sq_calibrated.store(true, std::memory_order_relaxed);
                    calib_buf.clear(); calib_buf.shrink_to_fit();
                }
            }
            {
                float thr = ch.sq_threshold.load(std::memory_order_relaxed);
                const float HYS = 3.0f;
                if(calibrated){
                    if(!gate_open && sql_avg >= thr){ gate_open=true; gate_hold=SQL_HOLD_SAMP; }
                    if( gate_open){
                        if(sql_avg >= thr-HYS) gate_hold=SQL_HOLD_SAMP;
                        else if(--gate_hold <= 0){ gate_open=false; gate_hold=0; }
                    }
                }
                if(++sq_ui_tick >= 256){ sq_ui_tick=0;
                    ch.sq_sig .store(sql_avg,  std::memory_order_relaxed);
                    ch.sq_gate.store(gate_open, std::memory_order_relaxed);
                }
            }
            if(gate_open != gate_was_open){
                gate_was_open = gate_open;
                printf("[DMR ch%d] gate %s sql=%.1fdB thr=%.1fdB\n",
                       ch_idx, gate_open?"OPEN":"CLOSE", sql_avg,
                       ch.sq_threshold.load(std::memory_order_relaxed));
                fflush(stdout);
            }
            if(!gate_open){
                prev_i=di; prev_q=dq;
                continue;
            }

            // FM discriminator
            float cross = di*prev_q - dq*prev_i;
            float dot   = di*prev_i + dq*prev_q;
            float disc  = (prev_i!=0.f||prev_q!=0.f) ? atan2f(cross, dot) : 0.f;
            prev_i = di; prev_q = dq;

            // DC 제거
            disc_dc += DC_ALPHA * (disc - disc_dc);
            disc -= disc_dc;

            // 주파수 기반 정규화: disc(rad) → 심볼 레벨 (±1,±3)
            float norm = disc * disc_to_sym;

            // M&M 타이밍
            sym_phase += 1.0f;
            if(sym_phase >= (sps + mu)){
                sym_phase -= (sps + mu);
                float cur = norm;
                float ec = (cur>0.f?1.f:-1.f)*p_sym - (p_sym>0.f?1.f:-1.f)*cur;
                mu += 0.01f * ec;
                if(mu > 1.5f) mu = 1.5f;
                if(mu < -1.5f) mu = -1.5f;
                p_sym = cur;

                int dibit = sym_to_dibit(cur);
                if(bit_cnt < BUF_BITS){
                    bits[bit_cnt++] = (uint8_t)((dibit >> 1) & 1);
                    if(bit_cnt < BUF_BITS)
                        bits[bit_cnt++] = (uint8_t)(dibit & 1);
                }

                // 진단: 2초마다
                diag_sym_cnt++;
                auto dnow = std::chrono::steady_clock::now();
                float del = std::chrono::duration<float>(dnow - last_diag_time).count();
                if(del >= 2.0f){
                    last_diag_time = dnow;
                    printf("[DMR ch%d] D disc=%.4f dc=%.4f norm=%.2f bits=%d syms=%d\n",
                           ch_idx, disc+disc_dc, disc_dc, cur, bit_cnt, diag_sym_cnt);
                    fflush(stdout);
                    diag_sym_cnt = 0;
                }

                // ── sync 탐색 + 버스트 처리 ─────────────────────────────
                while(bit_cnt >= DMR_SLOT_BITS + 48){
                    int serr; bool is_voice, is_bs;
                    int fpos = find_sync(bits, bit_cnt, 10, serr, is_voice, is_bs);

                    if(fpos < 0){
                        if(bit_cnt >= BUF_BITS - 100){
                            int keep = DMR_SLOT_BITS * 3;
                            int drop = bit_cnt - keep;
                            memmove(bits, bits + drop, keep);
                            bit_cnt = keep;
                        }
                        break;
                    }

                    if(is_voice){
                        voice_burst_cnt++;

                        auto now_t = std::chrono::steady_clock::now();
                        float el = std::chrono::duration<float>(now_t - last_info_time).count();
                        if(voice_burst_cnt == 1 || el >= 1.0f){
                            last_info_time = now_t;
                            printf("[DMR ch%d] VOICE bursts=%d err=%d type=%s\n",
                                   ch_idx, voice_burst_cnt, serr, is_bs?"BS":"MS");
                            fflush(stdout);
                        }

                        // AMBE+2 → 음성
                        const uint8_t* burst = bits + fpos;
                        const uint8_t* pay1 = burst;
                        const uint8_t* pay2 = burst + 146;

                        for(int frame=0; frame<3; frame++){
                            memset(ambe_fr, 0, sizeof(ambe_fr));
                            const uint8_t* src = (frame < 2) ? (pay1 + frame*49) : pay2;
                            for(int b=0; b<49; b++){
                                int row = b / 24;
                                int col = b % 24;
                                if(row < 4) ambe_fr[row][col] = (char)(src[b] & 1);
                            }

                            mbe_eccAmbe3600x2450C0(ambe_fr);
                            mbe_eccAmbe3600x2450Data(ambe_fr, ambe_d);
                            mbe_decodeAmbe2450Parms(ambe_d, &cur_mp, &prev_mp);
                            mbe_spectralAmpEnhance(&cur_mp);

                            short pcm[MBE_FRAME_SAMPS];
                            int errs=0, errs2=0; char err_str[64]="";
                            mbe_processAmbe3600x2450Frame(pcm, &errs, &errs2, err_str,
                                                          ambe_fr, ambe_d,
                                                          &cur_mp, &prev_mp, &enh_mp, 3);
                            mbe_moveMbeParms(&cur_mp, &prev_mp);

                            for(int i=0; i<MBE_FRAME_SAMPS; i++){
                                float smp = pcm[i] / 32768.0f;
                                for(int u=0; u<up_int; u++)
                                    ch.push_audio(smp);
                            }
                        }
                    }

                    int remove = fpos + DMR_SLOT_BITS;
                    if(remove < bit_cnt){
                        memmove(bits, bits + remove, (bit_cnt - remove));
                        bit_cnt -= remove;
                    } else {
                        bit_cnt = 0;
                    }
                }
            }
        }
        ch.digi_rp.store((rp + avail) & IQ_RING_MASK, std::memory_order_release);
    }
    printf("[DMR ch%d] worker exit\n", ch_idx);
    fflush(stdout);
}
