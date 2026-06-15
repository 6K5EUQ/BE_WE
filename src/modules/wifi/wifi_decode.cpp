// ── WiFi HOST 워커: IQ ring 독립 read-ptr 탭 → 채널 DDC(주파수이동+LPF+데시메이트)
//    → ~20 MSPS 채널 baseband → (M1) 진단 측정 + IQ 덤프 / (M2+) 802.11 비콘 디코드.
// demod_worker / 오디오 ring 과 완전 분리. 결과는 host_emit() 으로 로그+저장+브로드캐스트.
#include "fft_viewer.hpp"
#include "wifi_module.hpp"
#include "wifi_decode.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cstdio>
#include <ctime>
#include <sys/stat.h>

namespace wifi_mod {

static int64_t now_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void worker(FFTViewer& v, int ch_idx){
    Channel& ch = v.channels[ch_idx];
    uint32_t msr = v.header.sample_rate;                 // 스테이션(광대역) SR
    uint64_t init_cf = v.live_cf_hz.load(std::memory_order_acquire);
    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(init_cf/1e6f)) * 1e6f;
    float bw_hz  = fabsf(ch.e-ch.s) * 1e6f;              // 채널필터 폭 (~20 MHz)

    // DDC: 채널 BW 유지하며 ~20 MSPS 로 데시메이트 (정수배). 광대역 캡처의 sub-band 추출.
    uint32_t decim = std::max(1u, (uint32_t)llround((double)msr / 20.0e6));
    uint32_t out_sr = msr / decim;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;
    // 반대역 anti-alias LPF (decim 전). cutoff ~ BW/2.
    IIR1 lpi[3], lpq[3];
    { float cn=(bw_hz*0.5f)/(float)msr; if(cn>0.45f)cn=0.45f; if(cn<0.01f)cn=0.01f;
      for(int k=0;k<3;k++){ lpi[k].set(cn); lpq[k].set(cn); } }
    double dec_i=0, dec_q=0; uint32_t dec_cnt=0;

    WifiDecoder dec; dec.reset(out_sr);
    dec.on_record = [&v,ch_idx](const WifiRecord& r){ /* M2+: 실제 비콘 */ (void)v;(void)ch_idx;(void)r; };

    // ── M1 IQ 덤프 (complex float32 interleaved). census/EVM 오프라인 분석용. 약 3초 cap. ──
    std::string dir = BEWEPaths::data_dir() + "/modules/wifi";
    mkdir((BEWEPaths::data_dir()+"/modules").c_str(), 0755);
    mkdir(dir.c_str(), 0755);
    char fn[320]; char ts[32];
    { struct tm tmv; KST::to_tm((time_t)(now_ms()/1000), tmv); strftime(ts,sizeof(ts),"%Y%m%d_%H%M%S",&tmv); }
    snprintf(fn, sizeof(fn), "%s/wifi_cap_%s_ch%d_%uMSPS.cf32", dir.c_str(), ts, ch_idx, out_sr/1000000);
    FILE* dump = fopen(fn, "wb");
    uint64_t dump_cap = (uint64_t)out_sr * 3;            // 3초
    uint64_t dump_n = 0;

    bewe_log_push(0,"WiFi[%d] start: %.3f MHz  filterBW=%.1f MHz  station=%u MSPS  decim=%u out=%u MSPS  dump=%s\n",
        ch_idx,(ch.s+ch.e)/2.0f, bw_hz/1e6f, msr/1000000, decim, out_sr/1000000, dump?fn:"(none)");
    if(bw_hz < 16.0e6f)
        bewe_log_push(0,"WiFi[%d] WARN: filter %.1f MHz < 20 MHz — WiFi 채널엔 ~20 MHz 필터 권장\n", ch_idx, bw_hz/1e6f);

    const size_t MAX_LAG = (size_t)(msr*0.08);
    const size_t BATCH   = std::max<size_t>(4096, msr/50);
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());
    int64_t last_emit = now_ms();
    std::vector<float> dbuf; dbuf.reserve(BATCH*2/std::max(1u,decim)+4);

    while(!worker_stop_req(ch_idx) && !v.sdr_stream_error.load() && ch.filter_active){
        { uint64_t cur=v.live_cf_hz.load(std::memory_order_acquire);
          if(cur!=prev_cf){
              off_hz=(((ch.s+ch.e)/2.0f)-(float)(cur/1e6f))*1e6f;
              osc.set_freq((double)off_hz,(double)msr); prev_cf=cur;
          }
        }
        size_t wp=v.ring_wp.load(std::memory_order_acquire);
        size_t rp=my_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;
        if(lag>MAX_LAG){                                  // 과부하 → 경계로 점프 + 상태 리셋
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK; my_rp.store(rp,std::memory_order_release);
            for(int k=0;k<3;k++){ lpi[k].s=lpq[k].s=0; }
            dec_i=dec_q=0; dec_cnt=0;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        dbuf.clear();
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]/v.hw.iq_scale, sq=v.ring[pos*2+1]/v.hw.iq_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            mi=lpi[0].p(mi); mi=lpi[1].p(mi); mi=lpi[2].p(mi);
            mq=lpq[0].p(mq); mq=lpq[1].p(mq); mq=lpq[2].p(mq);
            dec_i+=mi; dec_q+=mq;
            if(++dec_cnt < decim) continue;
            float oi=(float)(dec_i/dec_cnt), oq=(float)(dec_q/dec_cnt);
            dec_i=dec_q=0; dec_cnt=0;
            dec.feed(oi,oq);                              // 측정/(M2)디코드
            if(dump && dump_n<dump_cap){ dbuf.push_back(oi); dbuf.push_back(oq); dump_n++; }
        }
        if(dump && !dbuf.empty()) fwrite(dbuf.data(),sizeof(float),dbuf.size(),dump);
        if(dump && dump_n>=dump_cap){ fclose(dump); dump=nullptr;
            bewe_log_push(0,"WiFi[%d] IQ dump done: %s (%llu samples)\n",ch_idx,fn,(unsigned long long)dump_cap); }
        my_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);

        // 1초마다 진단 레코드 (end-to-end 파이프 검증 + 측정)
        int64_t t=now_ms();
        if(t-last_emit>=1000){
            last_emit=t;
            WifiRecord r=dec.snapshot();
            r.t_ms=t; r.ch=ch_idx; r.freq=(ch.s+ch.e)/2.0f;
            host_emit(v, r);
        }
    }
    if(dump) fclose(dump);
    if(!worker_stop_req(ch_idx)) worker_natural_exit(v, ch_idx);
    bewe_log_push(0,"WiFi[%d] stop\n",ch_idx);
}

} // namespace wifi_mod
