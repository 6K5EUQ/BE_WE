// ── ADS-B HOST 워커: IQ ring 독립 read-ptr 탭 → 1090 baseband → ~2.4 MHz magnitude ──
// demod_worker / 오디오 ring 과 완전 분리 (ACARS/AIS 워커와 동형 격리).
// 오디오레이트(48k)로 내리는 demod_rates() 와 달리, OOK 펄스(0.5 µs) 보존 위해
// ~2.4 MHz 로 자체 데시메이션 후 magnitude = sqrt(I²+Q²) 를 디코더에 투입.
#include "fft_viewer.hpp"
#include "adsb_module.hpp"
#include "adsb_decode.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

namespace adsb_mod {

// adsb_module.cpp 의 워커 슬롯 접근자
std::atomic<size_t>& worker_rp(int ch);
bool worker_stop_req(int ch);
void worker_natural_exit(FFTViewer& v, int ch);

static constexpr double ADSB_TARGET_SR = 2400000.0;   // 목표 출력 레이트 (≥2 MHz)

void worker(FFTViewer& v, int ch_idx){
    Channel& ch=v.channels[ch_idx];
    uint32_t msr=v.header.sample_rate;
    uint64_t init_cf=v.live_cf_hz.load(std::memory_order_acquire);
    // 1090 ES 고정: 채널 중심이 곧 목표 주파수. center - SDR중심 = 오프셋.
    float off_hz=(((ch.s+ch.e)/2.0f)-(float)(init_cf/1e6f))*1e6f;

    // ── 자체 데시메이션: msr → ~2.4 MHz ──
    uint32_t decim=(uint32_t)llround((double)msr/ADSB_TARGET_SR); if(decim<1) decim=1;
    double   fs_out=(double)msr/decim;

    Oscillator osc; osc.set_freq((double)off_hz,(double)msr);
    uint64_t prev_cf=init_cf;
    float    prev_center=(ch.s+ch.e)/2.0f;   // 채널필터 이동(드래그) 감지용

    // OOK 펄스(0.5 µs) 보존이 최우선 — 추가 IIR 저역통과는 펄스를 뭉개 수율을 떨어뜨린다
    // (실측: 4단 IIR 적용 시 위치고정 1/3로 감소). decim 시의 boxcar 평균이 anti-alias 역할.
    double cap_i=0, cap_q=0; int cap_cnt=0;

    AdsbDecoder dec;
    dec.on_record=[&v](const AdsbRecord& m){ host_emit(v, m); };
    dec.reset(fs_out, ch_idx);
    bewe_log_push(0,"ADSB[%d] start: %.4f MHz  fs_out=%.3f MHz (decim=%u)\n",
        ch_idx,(ch.s+ch.e)/2.0f, fs_out/1e6, decim);

    // magnitude 출력 묶음 (~20 ms 분량)
    std::vector<float> mag; mag.reserve((size_t)(fs_out*0.05)+64);

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =(size_t)(msr/50);                 // ~20 ms 입력
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());

    while(!worker_stop_req(ch_idx) && !v.sdr_stream_error.load() && ch.filter_active){
        { uint64_t cur=v.live_cf_hz.load(std::memory_order_acquire);
          float cc=(ch.s+ch.e)/2.0f;                       // SDR 중심 또는 채널 이동 시 재튜닝
          if(cur!=prev_cf || cc!=prev_center){
              off_hz=(cc-(float)(cur/1e6))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur; prev_center=cc;
          }
        }
        size_t wp=v.ring_wp.load(std::memory_order_acquire);
        size_t rp=my_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        if(lag>MAX_LAG){                                   // 밀리면 점프 + 필터 리셋
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK;
            my_rp.store(rp,std::memory_order_release);
            cap_i=cap_q=0; cap_cnt=0;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        mag.clear();
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]/v.hw.iq_scale, sq=v.ring[pos*2+1]/v.hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            cap_i+=mi; cap_q+=mq; cap_cnt++;
            if(cap_cnt<(int)decim) continue;
            float fi=(float)(cap_i/cap_cnt), fq=(float)(cap_q/cap_cnt);
            cap_i=cap_q=0; cap_cnt=0;
            mag.push_back(sqrtf(fi*fi+fq*fq));
        }
        if(!mag.empty()) dec.process(mag.data(), mag.size());
        // 주기 진단 (~3초): 신호 유무·프리앰블·CRC 통계 — "왜 안 잡히나" 추적
        if(dec.dg_samp >= (long)(fs_out*3.0)){
            bewe_log_push(0,"ADSB[%d] diag: maxmag=%.0f preamble=%ld crcOK=%ld crcFAIL=%ld\n",
                ch_idx, dec.dg_maxmag, dec.dg_pre, dec.dg_ok, dec.dg_fail);
            dec.diag_reset();
        }
        my_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    if(!worker_stop_req(ch_idx)) worker_natural_exit(v, ch_idx);
    bewe_log_push(0,"ADSB[%d] stop\n",ch_idx);
}

} // namespace adsb_mod
