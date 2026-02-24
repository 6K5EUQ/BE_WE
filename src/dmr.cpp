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
static constexpr int    DMR_BAUD        = 4800;       // 심볼 레이트
static constexpr int    DMR_SLOT_BITS   = 264;        // 버스트당 비트 수
static constexpr int    DMR_AMBE_BITS   = 72;         // AMBE 프레임 비트 (3×24)
static constexpr float  DMR_AUDIO_SR    = 8000.0f;    // mbelib 출력 샘플레이트
static constexpr int    MBE_FRAME_SAMPS = 160;        // mbelib 출력 160샘플/프레임

// DMR Voice Sync 패턴 (48비트, BS_VOICE_SYNC)
// 0x755FD7DF75F7 → 비트열 (MSB first)
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

// 슬롯 타이밍 내 위치 (264비트 중 AMBE 위치)
// Burst: [98 payload bits][48 sync bits][98 payload bits] = 244 + CACH 20 = 264
// 실제 음성 버스트: 첫 98비트 + 마지막 98비트 에 AMBE 데이터
// AMBE 프레임: slot 내 3개 × 49비트 = 147비트 (ECC 포함은 144비트)
// 간략화: 첫 패이로드 49비트 × 3 = 147비트를 AMBE로 처리

// ── 4FSK 심볼 → 디비트 매핑 ──────────────────────────────────────────────
// +3 → 01, +1 → 00, -1 → 10, -3 → 11
static inline int sym_to_dibit(float sym){
    if(sym >  2.0f) return 0x01;
    if(sym >= 0.0f) return 0x00;
    if(sym > -2.0f) return 0x02;
    return 0x03;
}

// ── 비트 배열 동기 패턴 매칭 (허용 오차: 최대 4비트 오류) ────────────────
static int sync_match(const uint8_t* bits, const uint8_t* pattern, int len){
    int errs = 0;
    for(int i=0;i<len;i++) if(bits[i] != pattern[i]) errs++;
    return errs;
}

// ── DMR 워커 ──────────────────────────────────────────────────────────────
void FFTViewer::dmr_worker(int ch_idx){
    Channel& ch = channels[ch_idx];
    uint32_t msr = header.sample_rate;

    float off_hz  = (((ch.s+ch.e)/2.0f) - (float)(header.center_frequency/1e6f)) * 1e6f;
    float bw_hz   = fabsf(ch.e - ch.s) * 1e6f;
    if(bw_hz < 6250.0f) bw_hz = 12500.0f;

    // 채널 필터 데시메이션: ~ 4× 오버샘플 목표 (심볼당 4샘플)
    // inter_sr ≈ 4 × 4800 = 19200
    int cap_decim = std::max(1, (int)(msr / 19200.0f + 0.5f));
    uint32_t inter_sr = msr / cap_decim;
    // 심볼당 샘플 수
    float sps = (float)inter_sr / (float)DMR_BAUD;

    printf("DMR[%d]: off=%.0fHz  inter_sr=%u  sps=%.2f\n",
           ch_idx, off_hz, inter_sr, sps);

    // ── DSP 상태 ──────────────────────────────────────────────────────────
    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    double cap_i=0, cap_q=0; int cap_cnt=0;
    IIR1 lpi, lpq;
    { float cn = (bw_hz*0.5f)/(float)inter_sr; if(cn>0.45f)cn=0.45f; lpi.set(cn); lpq.set(cn); }

    // ── 스컬치 상태 ────────────────────────────────────────────────────────
    const float SQL_ALPHA     = 0.05f;
    const int   SQL_HOLD_SAMP = (int)(inter_sr * 0.2f);
    const int   CALIB_SAMP    = (int)(inter_sr * 0.5f);
    float sql_avg = -120.0f;
    bool  calibrated = ch.sq_calibrated.load(std::memory_order_relaxed);
    bool  gate_open  = false;
    int   gate_hold  = 0;
    int   sq_ui_tick = 0;
    std::vector<float> calib_buf;
    if(!calibrated) calib_buf.reserve(CALIB_SAMP);

    // FM 디스크리미네이터 상태
    float prev_i=0, prev_q=0;

    // 심볼 동기 (Gardner TED)
    float sym_buf[8] = {};      // 최근 심볼 버퍼
    float sps_int = roundf(sps);
    float mu = 0.0f;            // 타이밍 오프셋 (0~1)
    float omega = sps;          // 현재 추정 심볼 간격
    float omega_mid = sps;
    float omega_gain = 0.005f;
    float mu_gain    = 0.01f;
    int   interp_cnt = 0;

    // 4FSK 레벨 추정 (자동 스케일)
    float sym_scale = 1.0f;
    std::vector<float> sym_hist;
    sym_hist.reserve(1024);

    // 비트 버퍼 (264비트 × 2슬롯)
    static constexpr int BUF_BITS = DMR_SLOT_BITS * 2;
    uint8_t bits[BUF_BITS] = {};
    int     bit_cnt = 0;

    // AMBE 프레임 버퍼 (slot당 3프레임, 각 프레임 49비트)
    // 실제 AMBE+2 frame: ambe_fr[4][24] = 96비트, ambe_d[49]
    char ambe_fr[4][24];
    char ambe_d[49];

    // mbelib 상태
    mbe_parms cur_mp, prev_mp, enh_mp;
    mbe_initMbeParms(&cur_mp, &prev_mp, &enh_mp);

    // 오디오 리샘플러: 8000Hz → AUDIO_SR(48000Hz)
    float resamp_ratio = (float)AUDIO_SR / 48000.0f; // 8000/48000
    // 실제로는 mbelib 8kHz 출력을 48kHz로 업샘플
    float up_ratio = 48000.0f / (float)AUDIO_SR; // 6.0
    int   up_int   = (int)up_ratio; // 6

    // AGC
    float agc_rms = 0.01f;

    const size_t MAX_LAG = (size_t)(msr * 0.08);
    const size_t BATCH   = (size_t)cap_decim * 64;

    bool synced = false;
    int  sync_lost_cnt = 0;
    int  frame_bit_pos = 0;   // 현재 슬롯 내 비트 위치

    while(!ch.dem_stop_req.load(std::memory_order_relaxed)){
        size_t wp  = ring_wp.load(std::memory_order_acquire);
        size_t rp  = ch.dem_rp.load(std::memory_order_relaxed);
        size_t lag = (wp - rp) & IQ_RING_MASK;

        if(lag > MAX_LAG){
            size_t keep = (size_t)(msr * 0.02);
            rp = (wp - keep) & IQ_RING_MASK;
            ch.dem_rp.store(rp, std::memory_order_release);
            lag = (wp - rp) & IQ_RING_MASK;
            // DSP 상태 리셋
            prev_i = prev_q = 0; synced = false; bit_cnt = 0;
        }
        if(lag == 0){
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            continue;
        }

        size_t avail = std::min(lag, BATCH);
        for(size_t s=0; s<avail; s++){
            size_t pos = (rp + s) & IQ_RING_MASK;
            float ri = ring[pos*2  ] / 2048.0f;
            float rq = ring[pos*2+1] / 2048.0f;

            // Mix-down
            float mi, mq;
            osc.mix(ri, rq, mi, mq);

            // LP 필터
            float fi = lpi.p(mi);
            float fq = lpq.p(mq);

            // 채널 데시메이션
            cap_i += fi; cap_q += fq; cap_cnt++;
            if(cap_cnt < cap_decim) continue;
            float di = (float)(cap_i / cap_decim);
            float dq = (float)(cap_q / cap_decim);
            cap_i = cap_q = 0; cap_cnt = 0;

            // ── 스컬치 ────────────────────────────────────────────────────
            float p_inst = di*di + dq*dq;
            float db_inst = (p_inst > 1e-12f) ? 10.0f*log10f(p_inst) : -120.0f;
            sql_avg = SQL_ALPHA*db_inst + (1.0f-SQL_ALPHA)*sql_avg;
            if(!calibrated){
                if((int)calib_buf.size() < CALIB_SAMP) calib_buf.push_back(db_inst);
                if((int)calib_buf.size() >= CALIB_SAMP){
                    std::vector<float> tmp = calib_buf;
                    size_t p20 = tmp.size()/5;
                    std::nth_element(tmp.begin(), tmp.begin()+p20, tmp.end());
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
            if(!gate_open){
                if(synced){ synced=false; bit_cnt=0; sync_lost_cnt=0; }
                prev_i=di; prev_q=dq;
                continue;
            }

            // FM 디스크리미네이터: arg(conj(prev) * cur)
            float cross = prev_i * dq - prev_q * di;
            float dot   = prev_i * di + prev_q * dq;
            float disc  = atan2f(cross, dot) * (float)inter_sr / (float)(2*M_PI*DMR_BAUD);
            prev_i = di; prev_q = dq;

            // 심볼 타이밍 회복 (단순 카운터 기반)
            interp_cnt++;
            if(interp_cnt < (int)omega) continue;
            interp_cnt = 0;
            omega = sps; // 간략화: 고정 레이트

            // 레벨 히스토그램으로 4FSK 스케일 추정
            sym_hist.push_back(disc);
            if(sym_hist.size() >= 512){
                std::vector<float> tmp(sym_hist);
                std::sort(tmp.begin(), tmp.end());
                // ±1, ±3 레벨 → 75%/25% 지점 평균
                float p25 = tmp[(int)(tmp.size()*0.25)];
                float p75 = tmp[(int)(tmp.size()*0.75)];
                if(fabsf(p75 - p25) > 0.01f)
                    sym_scale = 3.0f / std::max(fabsf(p75), fabsf(p25));
                sym_hist.clear();
            }

            // 심볼 → 디비트 → 2비트
            float scaled = disc * sym_scale;
            int dibit = sym_to_dibit(scaled);
            if(bit_cnt < BUF_BITS){
                bits[bit_cnt++] = (dibit >> 1) & 1;
                if(bit_cnt < BUF_BITS)
                    bits[bit_cnt++] = dibit & 1;
            }

            // ── 프레임 동기 탐색 ────────────────────────────────────────
            if(!synced && bit_cnt >= DMR_SLOT_BITS){
                // sync 패턴은 버스트 내 비트 98~145 (0-index)
                for(int start=0; start <= bit_cnt - DMR_SLOT_BITS; start++){
                    int e0 = sync_match(bits + start + 98, DMR_BS_VOICE_SYNC, 48);
                    int e1 = sync_match(bits + start + 98, DMR_MS_VOICE_SYNC, 48);
                    if(e0 <= 8 || e1 <= 8){
                        // 동기 발견: 버퍼 앞부분 정렬
                        memmove(bits, bits + start, (bit_cnt - start) * sizeof(uint8_t));
                        bit_cnt -= start;
                        synced = true;
                        frame_bit_pos = 0;
                        printf("DMR[%d]: sync found (err=%d)\n", ch_idx, std::min(e0,e1));
                        break;
                    }
                }
                // 동기 못 찾으면 절반 버림
                if(!synced && bit_cnt >= BUF_BITS){
                    memmove(bits, bits + DMR_SLOT_BITS/2,
                            (bit_cnt - DMR_SLOT_BITS/2) * sizeof(uint8_t));
                    bit_cnt -= DMR_SLOT_BITS/2;
                }
                continue;
            }

            // ── 264비트 버스트 처리 ──────────────────────────────────────
            if(synced && bit_cnt >= DMR_SLOT_BITS){
                // sync 재확인
                int e0 = sync_match(bits + 98, DMR_BS_VOICE_SYNC, 48);
                int e1 = sync_match(bits + 98, DMR_MS_VOICE_SYNC, 48);
                bool voice_burst = (e0 <= 12 || e1 <= 12);

                if(!voice_burst){
                    sync_lost_cnt++;
                    if(sync_lost_cnt > 6){
                        synced = false; bit_cnt = 0; sync_lost_cnt = 0;
                        printf("DMR[%d]: sync lost\n", ch_idx);
                        continue; // memmove 건너뜀 (bit_cnt=0 이므로)
                    }
                } else {
                    sync_lost_cnt = 0;
                    // ── AMBE+2 프레임 추출 ──────────────────────────────
                    // Voice burst payload: bits[0..97] + bits[146..243]
                    // 각각 98비트, 합 196비트
                    // AMBE+2 (3600x2450): 3 frames × 49bits = 147bits
                    // 첫 49비트 × 3 = ambe_fr[4][24] 형태로 패킹
                    // Slot 1 처리 (Slot 2는 bits[264..527]이지만 단일 슬롯만 처리)
                    const uint8_t* payload = bits;  // 0~97
                    const uint8_t* payload2 = bits + 146; // 146~243

                    // 3개의 AMBE+2 프레임 (각 49비트 = ambe_d[49])
                    for(int frame=0; frame<3; frame++){
                        memset(ambe_fr, 0, sizeof(ambe_fr));
                        // 49비트 추출: frame 0 → bits[0..48], frame 1 → bits[49..97],
                        // frame 2 → payload2[0..48]
                        const uint8_t* src = (frame < 2) ? (payload + frame*49)
                                                          : (payload2);
                        // ambe_fr[4][24]: 96비트 배열에 49비트 packed
                        for(int b=0; b<49; b++){
                            int row = b / 24;
                            int col = b % 24;
                            if(row < 4) ambe_fr[row][col] = (src[b] & 1);
                        }

                        // ECC + 복조
                        mbe_eccAmbe3600x2450C0(ambe_fr);
                        mbe_eccAmbe3600x2450Data(ambe_fr, ambe_d);
                        mbe_decodeAmbe2450Parms(ambe_d, &cur_mp, &prev_mp);
                        mbe_spectralAmpEnhance(&cur_mp);

                        // 음성 합성: 8kHz, 160샘플
                        short pcm[MBE_FRAME_SAMPS];
                        int errs=0, errs2=0; char err_str[64]="";
                        mbe_processAmbe3600x2450Frame(pcm, &errs, &errs2, err_str,
                                                      ambe_fr, ambe_d,
                                                      &cur_mp, &prev_mp, &enh_mp, 3);
                        mbe_moveMbeParms(&cur_mp, &prev_mp);

                        // AGC + 48kHz 업샘플(단순 hold) → push_audio
                        for(int i=0; i<MBE_FRAME_SAMPS; i++){
                            float smp = pcm[i] / 32768.0f;
                            // AGC
                            float abs_s = fabsf(smp);
                            if(abs_s > agc_rms) agc_rms += (abs_s - agc_rms) * 0.001f;
                            else                agc_rms += (abs_s - agc_rms) * 0.0001f;
                            if(agc_rms > 0.001f) smp /= (agc_rms * 8.0f);
                            smp = smp < -1.0f ? -1.0f : smp > 1.0f ? 1.0f : smp;
                            // 8kHz → 48kHz: 6× 반복 (ZOH)
                            for(int u=0; u<up_int; u++)
                                ch.push_audio(smp);
                        }
                    }
                }

                // 처리된 264비트 제거
                memmove(bits, bits + DMR_SLOT_BITS,
                        (bit_cnt - DMR_SLOT_BITS) * sizeof(uint8_t));
                bit_cnt -= DMR_SLOT_BITS;
            }
        }
        ch.dem_rp.store((rp + avail) & IQ_RING_MASK, std::memory_order_release);
    }
    printf("DMR[%d]: worker exit\n", ch_idx);
}