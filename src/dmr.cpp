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
static constexpr int    DMR_BURST_DIBITS= 132;   // 264 bits = 132 dibits
static constexpr int    DMR_BURST_BITS  = 264;
static constexpr int    MBE_FRAME_SAMPS = 160;   // 20ms @ 8kHz

// ── DMR Sync 패턴 (48비트 → 24 dibits, burst의 dibit[54..77] 위치) ──────
// 원본 비트에서 정확히 변환: bit pair (b0,b1) → dibit value
// 4FSK: 01→+3(0x01), 00→+1(0x00), 10→-1(0x02), 11→-3(0x03)
//
// BS Voice: 0x755FD7DF75F7
// bits: 01 11 01 01 01 01 11 11  11 01 01 11 11 01 11 11  01 11 01 01 11 11 01 11
static const uint8_t DMR_BS_VOICE_SYNC[24] = {
    0x01,0x03,0x01,0x01,0x01,0x01,0x03,0x03,0x03,0x01,0x01,0x03,
    0x03,0x01,0x03,0x03,0x01,0x03,0x01,0x01,0x03,0x03,0x01,0x03
};
// MS Voice: 0xDFF57D75DF5D
// bits: 11 01 11 11 11 01 01 01  01 11 01 01 11 01 01 01  11 01 11 11 01 01 11 01  → wait
// 원본 bits: 1,1,1,1,1,1,0,0, 1,1,1,0,0,1,0,0, 0,0,1,0,0,1,1,0, 0,0,1,0,0,1,1,0, 1,1,1,1,1,1,0,0, 1,1,0,0,1,0,0,0
// pairs:     11 11 11 00 11 10 01 00 00 10 01 10 00 10 01 10 11 11 11 00 11 00 10 00
static const uint8_t DMR_MS_VOICE_SYNC[24] = {
    0x03,0x03,0x03,0x00,0x03,0x02,0x01,0x00,0x00,0x02,0x01,0x02,
    0x00,0x02,0x01,0x02,0x03,0x03,0x03,0x00,0x03,0x00,0x02,0x00
};
// BS Data: 0xDFF57D75DF5D → actually different
// 원본 bits: 1,1,0,1,1,1,0,0, 1,1,0,1,0,0,0,1, 0,1,0,0,1,1,1,0, 0,1,0,0,1,1,1,0, 1,1,0,1,1,1,0,0, 0,1,1,0,0,0,1,1
// pairs:     11 01 11 00 11 01 00 01 01 00 11 10 01 00 11 10 11 01 11 00 01 10 00 11
static const uint8_t DMR_BS_DATA_SYNC[24] = {
    0x03,0x01,0x03,0x00,0x03,0x01,0x00,0x01,0x01,0x00,0x03,0x02,
    0x01,0x00,0x03,0x02,0x03,0x01,0x03,0x00,0x01,0x02,0x00,0x03
};

// ── DSD rW/rX/rY/rZ: AMBE+2 인터리브 스케줄 ──────────────────────────────
// 각 AMBE 프레임에 대해 36 dibit를 ambe_fr[4][24]로 배치
// rW[i], rX[i] = ambe_fr 행/열 (dibit 상위비트)
// rY[i], rZ[i] = ambe_fr 행/열 (dibit 하위비트)
static const int rW[36] = {
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 2,
    0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2
};
static const int rX[36] = {
    23, 10, 22, 9, 21, 8, 20, 7, 19, 6, 18, 5,
    17,  4, 16, 3, 15, 2, 14, 1, 13, 0, 12, 10,
    11,  9, 10, 8,  9, 7,  8, 6,  7, 5,  6,  4
};
static const int rY[36] = {
    0, 2, 0, 2, 0, 2, 0, 2, 0, 3, 0, 3,
    1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3,
    1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3
};
static const int rZ[36] = {
     5, 3, 4, 2, 3, 1, 2,  0, 1, 13, 0, 12,
    22, 11, 21, 10, 20, 9, 19, 8, 18, 7, 17, 6,
    16, 5, 15, 4, 14, 3, 13, 2, 12, 1, 11, 0
};

// ── 적응형 4FSK 심볼 슬라이서 ─────────────────────────────────────────────
// 실제 심볼 레벨을 추적하여 슬라이서 경계를 자동 조정
struct AdaptiveSlicer {
    float max_pos = 2.0f;  // +3 레벨 추적
    float max_neg = -2.0f; // -3 레벨 추적
    static constexpr float ADAPT_RATE = 0.08f;  // 빠른 적응
    static constexpr float CLAMP_HI = 4.0f;     // 노이즈 피크 제한
    static constexpr float CLAMP_LO = 0.5f;     // 최소 피크

    void reset(){
        max_pos = 2.0f;
        max_neg = -2.0f;
    }

    int decide(float sym){
        // 피크 레벨 추적 (노이즈 클램핑 적용)
        if(sym > 0.3f){
            float s = (sym > CLAMP_HI) ? CLAMP_HI : sym;
            max_pos += ADAPT_RATE * (s - max_pos);
        }
        if(sym < -0.3f){
            float s = (sym < -CLAMP_HI) ? -CLAMP_HI : sym;
            max_neg += ADAPT_RATE * (s - max_neg);
        }
        // 최소값 보장
        if(max_pos < CLAMP_LO) max_pos = CLAMP_LO;
        if(max_neg > -CLAMP_LO) max_neg = -CLAMP_LO;

        // 슬라이서 경계: outer/inner 중간점 = peak * 2/3
        float thr_hi = max_pos * 0.667f;
        float thr_lo = max_neg * 0.667f;

        if(sym > thr_hi)  return 0x01;  // +3
        if(sym >= 0.0f)   return 0x00;  // +1
        if(sym > thr_lo)  return 0x02;  // -1
        return 0x03;                     // -3
    }
};

// sync를 dibit 레벨에서 매칭 (더 정확함)
static int sync_match_dibit(const uint8_t* dibits, const uint8_t* pattern, int len){
    int errs = 0;
    for(int i=0;i<len;i++) if(dibits[i] != pattern[i]) errs++;
    return errs;
}

// dibit 버퍼에서 sync 탐색. DMR burst = 132 dibits, sync는 dibit[54..77]
static int find_sync_dibit(const uint8_t* dibits, int dibit_cnt, int max_err,
                           int& out_err, bool& out_voice, bool& out_bs){
    for(int start=0; start <= dibit_cnt - DMR_BURST_DIBITS; start++){
        const uint8_t* sp = dibits + start + 54;  // sync 위치: 108bits/2 = 54 dibits
        int ev = sync_match_dibit(sp, DMR_BS_VOICE_SYNC, 24);
        int em = sync_match_dibit(sp, DMR_MS_VOICE_SYNC, 24);
        int ed = sync_match_dibit(sp, DMR_BS_DATA_SYNC,  24);
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

// ── AMBE 프레임 추출 (DSD 방식: rW/rX/rY/rZ 인터리브 스케줄) ──────────────
// burst 내 3개 AMBE 프레임의 dibit 위치:
//   Frame1: dibit[0..35]   (sync 앞 36 dibits)
//   Frame2: dibit[36..53] + dibit[78..95]  (sync 앞 18 + sync 뒤 18 = 36 dibits)
//   Frame3: dibit[96..131] (sync 뒤 36 dibits)
static void extract_ambe_frame(const uint8_t* frame_dibits, char ambe_fr[4][24]){
    memset(ambe_fr, 0, 4*24);
    for(int i=0; i<36; i++){
        int dibit = frame_dibits[i];
        ambe_fr[rW[i]][rX[i]] = (char)((dibit >> 1) & 1);
        ambe_fr[rY[i]][rZ[i]] = (char)(dibit & 1);
    }
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

    // ── Pre-decimation LPF: msr에서 2단 IIR1 (±6kHz) ─────────────────────
    IIR1 pre_lpi[2], pre_lpq[2];
    { float cn = 6000.0f / (float)msr;
      if(cn > 0.45f) cn = 0.45f;
      for(int i=0;i<2;i++){ pre_lpi[i].set(cn); pre_lpq[i].set(cn); } }

    // ── Post-decimation LPF: inter_sr에서 1단 IIR1 (±6kHz) ───────────────
    IIR1 post_lpi, post_lpq;
    { float cn = 6000.0f / (float)inter_sr;
      if(cn > 0.45f) cn = 0.45f;
      post_lpi.set(cn); post_lpq.set(cn); }

    // 스컬치는 UI 스레드에서 FFT 기반으로 중앙 관리 (sq_gate 읽기만)
    bool gate_open = false;

    // FM discriminator
    float prev_i=0, prev_q=0;

    // disc를 주파수 편차 단위로 정규화: disc × inter_sr/(2π) = Hz
    // DMR: outer=±1944Hz, inner=±648Hz → ±3/±1 레벨 목표
    float disc_to_sym = (float)inter_sr / (2.0f * (float)M_PI * 648.0f);

    // DC remover (시상수 ~20ms — 4FSK에 적합하게)
    float disc_dc = 0.f;
    const float DC_ALPHA = 1.f / (0.02f * (float)inter_sr);

    // M&M 타이밍
    float sym_phase = 0.f, mu = 0.f, p_sym = 0.f;

    // 적응형 슬라이서
    AdaptiveSlicer slicer;

    // dibit 버퍼 (비트가 아닌 dibit 단위로 저장)
    static constexpr int BUF_DIBITS = DMR_BURST_DIBITS * 8;
    uint8_t dibits[BUF_DIBITS] = {};
    int     dibit_cnt = 0;

    // mbelib
    char ambe_fr[4][24];
    char ambe_d[49];
    mbe_parms cur_mp, prev_mp, enh_mp;
    mbe_initMbeParms(&cur_mp, &prev_mp, &enh_mp);

    const int up_int = 6; // 8kHz → 48kHz

    const size_t MAX_LAG = (size_t)(msr * 0.08);
    const size_t BATCH   = (size_t)(msr / 50);

    // AMBE 에러 통계
    int ambe_err_total = 0, ambe_err_frames = 0;

    // AMBE 디코딩 헬퍼: burst의 132 dibits에서 3 AMBE 프레임 추출 → 오디오 출력
    auto decode_voice_burst = [&](const uint8_t* burst){
        // Frame1: dibit[0..35], Frame2: [36..53]+[78..95], Frame3: [96..131]
        const uint8_t* frame_srcs[3];
        uint8_t f2_combined[36];
        frame_srcs[0] = burst;
        memcpy(f2_combined,     burst+36, 18);
        memcpy(f2_combined+18,  burst+78, 18);
        frame_srcs[1] = f2_combined;
        frame_srcs[2] = burst + 96;

        for(int frame=0; frame<3; frame++){
            extract_ambe_frame(frame_srcs[frame], ambe_fr);

            // mbe_processAmbe3600x2450Frame이 내부적으로
            // ECC, decode, spectralAmpEnhance, moveMbeParms 모두 수행
            short pcm[MBE_FRAME_SAMPS];
            int errs2=0, errs3=0; char err_str[64]="";
            mbe_processAmbe3600x2450Frame(pcm, &errs2, &errs3, err_str,
                                          ambe_fr, ambe_d,
                                          &cur_mp, &prev_mp, &enh_mp, 3);

            ambe_err_total += errs2;
            ambe_err_frames++;

            // 선형 보간 업샘플링 (8kHz → 48kHz)
            for(int i=0; i<MBE_FRAME_SAMPS; i++){
                float cur_s = pcm[i] / 32768.0f;
                float nxt_s = (i+1 < MBE_FRAME_SAMPS) ? pcm[i+1]/32768.0f : cur_s;
                for(int u=0; u<up_int; u++){
                    float t = (float)u / (float)up_int;
                    ch.push_audio(cur_s + t * (nxt_s - cur_s));
                }
            }
        }
    };

    int  voice_burst_cnt = 0;
    auto last_info_time = std::chrono::steady_clock::now();
    auto last_diag_time = std::chrono::steady_clock::now();
    int  diag_sym_cnt = 0;
    bool gate_was_open = false;

    // TDMA 추적 상태
    // DMR TDMA: 같은 슬롯 반복 주기 = 288 dibits (2 × 144)
    // burst 132 dibits 처리 후 다음 같은 슬롯 burst까지 156 dibits
    static constexpr int TDMA_FRAME_DIBITS = 288;
    static constexpr int TRACK_SKIP = TDMA_FRAME_DIBITS - DMR_BURST_DIBITS; // 156
    int track_remain = 0;  // 0=search mode, 1~5=tracking (remaining bursts)

    while(!ch.digi_stop_req.load(std::memory_order_relaxed)){
        size_t wp  = ring_wp.load(std::memory_order_acquire);
        size_t rp  = ch.digi_rp.load(std::memory_order_relaxed);
        size_t lag = (wp - rp) & IQ_RING_MASK;

        if(lag > MAX_LAG){
            size_t keep = (size_t)(msr * 0.02);
            rp = (wp - keep) & IQ_RING_MASK;
            ch.digi_rp.store(rp, std::memory_order_release);
            lag = (wp - rp) & IQ_RING_MASK;
            prev_i = prev_q = 0; dibit_cnt = 0;
            cap_i = cap_q = 0; cap_cnt = 0;
            for(int i=0;i<2;i++){ pre_lpi[i].s=0; pre_lpq[i].s=0; }
            post_lpi.s=0; post_lpq.s=0;
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

            // ── Pre-decimation LPF (2단, msr에서) ───────────────────────
            for(int i=0;i<2;i++){ mi = pre_lpi[i].p(mi); mq = pre_lpq[i].p(mq); }

            // decimation (box-car)
            cap_i += mi; cap_q += mq; cap_cnt++;
            if(cap_cnt < (int)cap_decim) continue;
            float di = (float)(cap_i / cap_cnt);
            float dq = (float)(cap_q / cap_cnt);
            cap_i = cap_q = 0; cap_cnt = 0;

            // ── Post-decimation LPF ─────────────────────────────────────
            di = post_lpi.p(di); dq = post_lpq.p(dq);

            // 스컬치 게이트 읽기 (UI 스레드에서 FFT 기반으로 관리)
            gate_open = ch.sq_gate.load(std::memory_order_relaxed);
            if(gate_open != gate_was_open){
                gate_was_open = gate_open;
                if(gate_open){
                    slicer.reset();
                    mbe_initMbeParms(&cur_mp, &prev_mp, &enh_mp);
                    track_remain = 0;
                    dibit_cnt = 0;
                    disc_dc = 0.f;
                }
                printf("[DMR ch%d] gate %s\n", ch_idx, gate_open?"OPEN":"CLOSE");
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

                // dibit 저장 (적응형 슬라이서 사용)
                int dibit = slicer.decide(cur);
                if(dibit_cnt < BUF_DIBITS)
                    dibits[dibit_cnt++] = (uint8_t)dibit;

                // 진단: 2초마다
                diag_sym_cnt++;
                auto dnow = std::chrono::steady_clock::now();
                float del = std::chrono::duration<float>(dnow - last_diag_time).count();
                if(del >= 2.0f){
                    last_diag_time = dnow;
                    float avg_ambe_err = (ambe_err_frames>0) ? (float)ambe_err_total/ambe_err_frames : 0;
                    printf("[DMR ch%d] D norm=%.2f slicer=[%.2f,%.2f] ambe_err=%.1f/%d dibits=%d\n",
                           ch_idx, cur, slicer.max_neg, slicer.max_pos,
                           avg_ambe_err, ambe_err_frames, dibit_cnt);
                    ambe_err_total=0; ambe_err_frames=0;
                    fflush(stdout);
                    diag_sym_cnt = 0;
                }

                // ── sync 탐색 + TDMA 연속 추적 + 버스트 처리 ──────────
                for(;;){
                    if(track_remain > 0){
                        // ── TRACKING MODE: 같은 슬롯의 다음 burst를 고정 간격으로 추출
                        int need = TRACK_SKIP + DMR_BURST_DIBITS;
                        if(dibit_cnt < need) break;

                        const uint8_t* burst = dibits + TRACK_SKIP;
                        voice_burst_cnt++;
                        decode_voice_burst(burst);
                        track_remain--;

                        if(track_remain == 0){
                            // superframe 경계: sync 검증 → 연속 추적
                            const uint8_t* center = burst + 54;
                            int ev = sync_match_dibit(center, DMR_BS_VOICE_SYNC, 24);
                            int em = sync_match_dibit(center, DMR_MS_VOICE_SYNC, 24);
                            int best = std::min(ev, em);
                            if(best <= 8){
                                // sync 확인됨 → 다음 superframe 연속 추적
                                track_remain = 6;
                            }
                            // best > 8 → search mode로 돌아감 (track_remain=0)

                            auto now_t = std::chrono::steady_clock::now();
                            float el = std::chrono::duration<float>(now_t - last_info_time).count();
                            if(el >= 1.0f){
                                last_info_time = now_t;
                                printf("[DMR ch%d] VOICE bursts=%d sync_chk=%d %s\n",
                                       ch_idx, voice_burst_cnt, best,
                                       (best<=8)?"→ continue":"→ resync");
                                fflush(stdout);
                            }
                        }

                        // 처리된 데이터 제거
                        int remove = TRACK_SKIP + DMR_BURST_DIBITS;
                        memmove(dibits, dibits + remove, dibit_cnt - remove);
                        dibit_cnt -= remove;
                    } else {
                        // ── SEARCH MODE: sync 패턴 탐색
                        if(dibit_cnt < DMR_BURST_DIBITS + 24) break;

                        int serr; bool is_voice, is_bs;
                        int fpos = find_sync_dibit(dibits, dibit_cnt, 4, serr, is_voice, is_bs);

                        if(fpos < 0){
                            if(dibit_cnt >= BUF_DIBITS - 50){
                                int keep = DMR_BURST_DIBITS * 3;
                                int drop = dibit_cnt - keep;
                                memmove(dibits, dibits + drop, keep);
                                dibit_cnt = keep;
                            }
                            break;
                        }

                        if(is_voice){
                            voice_burst_cnt++;
                            decode_voice_burst(dibits + fpos);

                            // TDMA 추적 시작: superframe의 나머지 5 burst 추적
                            track_remain = 5;

                            printf("[DMR ch%d] SYNC err=%d type=%s bursts=%d → tracking\n",
                                   ch_idx, serr, is_bs?"BS":"MS", voice_burst_cnt);
                            fflush(stdout);
                            last_info_time = std::chrono::steady_clock::now();
                        }

                        // 처리된 버스트 제거 (voice든 data든)
                        int remove = fpos + DMR_BURST_DIBITS;
                        if(remove < dibit_cnt){
                            memmove(dibits, dibits + remove, dibit_cnt - remove);
                            dibit_cnt -= remove;
                        } else {
                            dibit_cnt = 0;
                        }
                    }
                }
            }
        }
        ch.digi_rp.store((rp + avail) & IQ_RING_MASK, std::memory_order_release);
    }
    printf("[DMR ch%d] worker exit\n", ch_idx);
    fflush(stdout);
}
