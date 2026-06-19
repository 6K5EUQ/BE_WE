// ── BLE HOST 워커: IQ ring 독립 read-ptr 탭 → 채널 DDC → ~4 MHz 데시메이트 →
//    FM 판별기(GFSK) → BtleDecoder(AA동기/디화이트닝/CRC/PDU). ais_decode.cpp 미러.
//    오디오/스컬치 없음(패킷 디코더). 결과는 host_emit() → 로그+일단위 저장+JOIN 팬아웃.
#include "fft_viewer.hpp"
#include "btle_module.hpp"
#include "btle_decode.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

namespace btle_mod {

static int64_t now_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

static constexpr double BTLE_TARGET_SR = 4000000.0;   // 4 sps @ 1 Mbit/s

// 채널 중심 MHz → 광고 채널 인덱스 (37=2402 / 38=2426 / 39=2480, 최근접, 기본 37)
static int adv_chan_of(float cmhz){
    if(std::fabs(cmhz-2426.f) < 6.f) return 38;
    if(std::fabs(cmhz-2480.f) < 6.f) return 39;
    return 37;
}

void worker(FFTViewer& v, int ch_idx){
    Channel& ch = v.channels[ch_idx];
    uint32_t msr = v.header.sample_rate;
    uint64_t init_cf = v.live_cf_hz.load(std::memory_order_acquire);
    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(init_cf/1e6f)) * 1e6f;
    float bw_hz  = fabsf(ch.e-ch.s) * 1e6f;
    int   adv_chan = adv_chan_of((ch.s+ch.e)/2.0f);

    // ── DDC: ~4 MHz 정수배 데시메이트 ──
    uint32_t decim  = std::max(1u, (uint32_t)llround((double)msr / BTLE_TARGET_SR));
    double   fs_out = (double)msr / decim;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;
    float    prev_center = (ch.s+ch.e)/2.0f;
    // 데시메이션 전 anti-alias LPF: cutoff = min(채널BW/2, fs_out*0.45)
    IIR1 lpi[4], lpq[4];
    { float cut = std::min(bw_hz*0.5f, (float)fs_out*0.45f);
      float cn = cut/(float)msr; if(cn>0.45f)cn=0.45f; if(cn<0.005f)cn=0.005f;
      for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); } }
    double dec_i=0, dec_q=0; uint32_t dec_cnt=0;
    float  prev_i=0, prev_q=0;                              // FM 판별기 상태

    BtleDecoder dec; dec.reset(fs_out, ch_idx, adv_chan);
    dec.on_record = [&v,ch_idx](const BtleRecord& r){ BtleRecord m=r; m.ch=ch_idx; host_emit(v, m); };

    bewe_log_push(0,"BTLE[%d] start: %.4f MHz (ch%d)  BW=%.1f kHz  decim=%u fs_out=%.3f MHz (%.2f sps)\n",
        ch_idx,(ch.s+ch.e)/2.0f, adv_chan, bw_hz/1000.f, decim, fs_out/1e6, fs_out/1e6);

    // FM 판별 출력 묶음 (~20 ms 분량)
    std::vector<float> fm; fm.reserve((size_t)(fs_out*0.05)+64);

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =std::max<size_t>(4096, msr/50);
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());
    int64_t last_diag=now_ms();

    while(!worker_stop_req(ch_idx) && !v.sdr_stream_error.load() && ch.filter_active){
        { uint64_t cur=v.live_cf_hz.load(std::memory_order_acquire);
          float cc=(ch.s+ch.e)/2.0f;
          if(cur!=prev_cf || cc!=prev_center){
              off_hz=(cc-(float)(cur/1e6))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur; prev_center=cc;
          }
        }
        size_t wp=v.ring_wp.load(std::memory_order_acquire);
        size_t rp=my_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        if(lag>MAX_LAG){                                    // 과부하 → 경계 점프 + 상태 리셋
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK; my_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            dec_i=dec_q=0; dec_cnt=0; prev_i=prev_q=0;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        fm.clear();
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]/v.hw.iq_scale, sq=v.ring[pos*2+1]/v.hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            mi=lpi[0].p(mi); mi=lpi[1].p(mi); mi=lpi[2].p(mi); mi=lpi[3].p(mi);
            mq=lpq[0].p(mq); mq=lpq[1].p(mq); mq=lpq[2].p(mq); mq=lpq[3].p(mq);
            dec_i+=mi; dec_q+=mq;
            if(++dec_cnt < decim) continue;
            float oi=(float)(dec_i/dec_cnt), oq=(float)(dec_q/dec_cnt);
            dec_i=dec_q=0; dec_cnt=0;
            // FM 판별: arg(z·conj(prev)) = 순시주파수 (GFSK mark/space)
            float d = atan2f(oq*prev_i - oi*prev_q, oi*prev_i + oq*prev_q + 1e-20f);
            prev_i=oi; prev_q=oq;
            fm.push_back(d);
        }
        if(!fm.empty()) dec.process(fm.data(), fm.size());
        my_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);

        int64_t t=now_ms();
        if(t-last_diag>=3000){                              // ~3초 진단 (콘솔)
            bewe_log_push(0,"BTLE[%d] diag: maxlev=%.3f sync=%ld lenOK=%ld crcOK=%ld crcFAIL=%ld\n",
                ch_idx, dec.dg_maxlev, dec.dg_sync, dec.dg_lenok, dec.dg_ok, dec.dg_fail);
            dec.diag_reset(); last_diag=t;
        }
    }
    if(!worker_stop_req(ch_idx)) worker_natural_exit(v, ch_idx);
    bewe_log_push(0,"BTLE[%d] stop\n",ch_idx);
}

} // namespace btle_mod
