// ── DMR HOST 워커: IQ ring 독립 read-ptr 탭 → 채널 DDC → FM 판별기 → boxcar
//    정합필터 → DmrDecoder(심볼동기/sync/Slot Type Golay/BPTC/CSBK·LC). AIS 와 동형.
//    결과는 host_emit() → 로그+일단위 저장+전 JOIN 브로드캐스트.
#include "fft_viewer.hpp"
#include "dmr_module.hpp"
#include "dmr_decode.hpp"
#include "dmr_ambe.hpp"
#include "bewe_paths.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <string>
#include <sys/stat.h>

namespace dmr_mod {

static int64_t now_ms(){
    return (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void worker(FFTViewer& v, int ch_idx){
    Channel& ch = v.channels[ch_idx];
    uint32_t msr = v.header.sample_rate;
    const float inv_scale=1.0f/v.hw.iq_scale;  // ÷ → ×
    uint64_t init_cf = v.live_cf_hz.load(std::memory_order_acquire);
    float off_hz = (((ch.s+ch.e)/2.0f) - (float)(init_cf/1e6f)) * 1e6f;
    float bw_hz  = fabsf(ch.e-ch.s) * 1e6f;

    // ── DDC: ~48 kHz 정수배 데시메이트 (4800 sym/s → ~10 sps) ──
    uint32_t decim  = std::max(1u, (uint32_t)llround((double)msr / 48000.0));
    uint32_t out_sr = msr / decim;

    Oscillator osc; osc.set_freq((double)off_hz, (double)msr);
    uint64_t prev_cf = init_cf;
    IIR1 lpi[4], lpq[4];
    { float cut = std::min(bw_hz*0.5f, out_sr*0.45f);
      float cn = cut/(float)msr; if(cn>0.45f)cn=0.45f; if(cn<0.005f)cn=0.005f;
      for(int k=0;k<4;k++){ lpi[k].set(cn); lpq[k].set(cn); } }
    double dec_i=0, dec_q=0; uint32_t dec_cnt=0;

    // FM 판별기 상태 + boxcar 정합필터 (1심볼 = round(out_sr/4800) 샘플)
    float prev_i=0, prev_q=0;
    int   W = std::max(1, (int)llround((double)out_sr/4800.0));
    std::vector<float> box(W,0.f); int box_pos=0; double box_sum=0;

    DmrDecoder dec; dec.configure((double)out_sr);
    long frames=0;
    // ── 통화별 음성 녹음 (8kHz mono int16 WAV; rec_id = 파일명 키 = 음성레코드에 부착) ──
    FILE*    rec_fp=nullptr; uint64_t rec_frames=0, cur_rec_id=0;
    uint32_t pend_src=0, pend_dst=0; int pend_slot=0;
    dec.on_record = [&](const DmrRecord& r){
        frames++; DmrRecord m=r; m.ch=ch_idx;
        if(m.is_voice){                                   // 콜 컨텍스트 캡처 + 녹음키 부착
            if(m.src_id) pend_src=m.src_id;
            if(m.dst_id) pend_dst=m.dst_id;
            if(m.slot>0) pend_slot=m.slot;
            m.rec_id = cur_rec_id;                        // rec_open 전이면 0
        }
        host_emit(v, m);
    };

    // ── 음성: AMBE(mbelib) → PCM → 8k→out_sr 리샘플 → 채널 오디오 ring ──
    // 음성 디코드 동안만 push → 음성 없으면 무음(스컬치). FM 오디오는 ext_audio 로 억제.
    dmr::DmrAmbeDecoder ambe;
    const float VOICE_GAIN = 2.2f;
    const int up = std::max(1, (int)llround((double)out_sr/8000.0));   // 8k→out_sr 정수배
    float a_prev = 0.f;
    std::vector<float> nbuf; nbuf.reserve(256);
    // 한 샘플 출력: 로컬 ring(L/R/pan 재생) + 네트워크(구독 operator) — dem_worker 와 동형
    auto emit_audio = [&](float out){
        ch.push_audio(out);
        if(v.net_srv && (ch.audio_mask.load() & ~0x1u)){
            nbuf.push_back(out);
            if(nbuf.size()>=256){ uint32_t mask=(ch.audio_mask.load()>>1);
                v.net_srv->send_audio(mask,(uint8_t)ch_idx,(int8_t)ch.pan,nbuf.data(),(uint32_t)nbuf.size());
                nbuf.clear(); }
        }
    };
    // 녹음 종료(헤더 frames 갱신 후 닫기) / 시작(통화별 새 WAV)
    auto rec_close = [&](){
        if(rec_fp){ fseek(rec_fp,0,SEEK_SET); ch.audio_rec_write_wav_hdr(rec_fp,8000,rec_frames); fclose(rec_fp); rec_fp=nullptr; }
        rec_frames=0; cur_rec_id=0;
    };
    auto rec_open = [&](){
        rec_close();
        std::string dir = BEWEPaths::data_dir()+"/modules/dmr/rec";
        mkdir((BEWEPaths::data_dir()+"/modules").c_str(),0755);
        mkdir((BEWEPaths::data_dir()+"/modules/dmr").c_str(),0755);
        mkdir(dir.c_str(),0755);
        cur_rec_id=(uint64_t)now_ms();
        char fn[192]; snprintf(fn,sizeof(fn),"%s/dmr_%llu_%u_%u_%d.wav", dir.c_str(),
            (unsigned long long)cur_rec_id, pend_src, pend_dst, pend_slot);
        rec_fp=fopen(fn,"wb"); rec_frames=0;
        if(rec_fp) ch.audio_rec_write_wav_hdr(rec_fp,8000,0);   // 자리표시 헤더
    };
    dec.on_voice = [&](const uint8_t* fr, int nf, bool nc){
        if(nc){ ambe.reset(); rec_open(); }                    // 새 통화 → 새 WAV
        for(int f=0; f<nf; f++){
            short pcm[160]; ambe.decode(fr + f*36, pcm);
            if(rec_fp){ fwrite(pcm,sizeof(short),160,rec_fp); rec_frames+=160; }   // 8kHz mono int16 녹음
            for(int i=0;i<160;i++){
                float s = pcm[i] * (VOICE_GAIN/32768.f);
                if(s>1.f)s=1.f; else if(s<-1.f)s=-1.f;
                for(int j=1;j<=up;j++){ float al=(float)j/up;     // 선형보간 업샘플
                    emit_audio(a_prev + (s-a_prev)*al); }
                a_prev = s;
            }
        }
    };
    ch.ext_audio.store(true, std::memory_order_relaxed);   // DMR 이 채널 오디오 소유

    bewe_log_push(0,"DMR[%d] start: %.4f MHz  BW=%.1f kHz  decim=%u out=%u Hz (%.2f sps)\n",
        ch_idx,(ch.s+ch.e)/2.0f, bw_hz/1000.f, decim, out_sr, (double)out_sr/4800.0);

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =std::max<size_t>(4096, msr/50);
    std::atomic<size_t>& my_rp = worker_rp(ch_idx);
    my_rp.store(v.ring_wp.load());
    int64_t last_diag=now_ms();
    bool gate_prev=false;   // 스컬치 게이트 이전상태 (AM/FM 과 동일 sq_gate 사용)

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
        if(lag>MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK; my_rp.store(rp,std::memory_order_release);
            for(int k=0;k<4;k++){ lpi[k].s=lpq[k].s=0; }
            dec_i=dec_q=0; dec_cnt=0; prev_i=prev_q=0;
            std::fill(box.begin(),box.end(),0.f); box_sum=0; box_pos=0;
            dec.reset();
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(1000)); continue; }

        // ── 스컬치 게이트: AM/FM 과 동일한 ch.sq_gate (HOST FFT 기반) 사용.
        //    닫힘 = 신호 없음 → 복조기에 노이즈 안 넣음(가짜 voice-sync 방지).
        //    닫힘 edge 에서 예약 음성(B–F) 폐기 + AMBE 리셋 → 잔향/클릭 차단.
        bool gate = ch.sq_gate.load(std::memory_order_relaxed);
        if(gate_prev && !gate){ dec.clear_voice(); ambe.reset(); a_prev=0.f; rec_close(); }  // 통화 끝 → WAV 닫기
        gate_prev = gate;

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=v.ring[pos*2]*inv_scale, sq=v.ring[pos*2+1]*inv_scale;
            float mi,mq; osc.mix(si,sq,mi,mq);
            mi=lpi[0].p(mi); mi=lpi[1].p(mi); mi=lpi[2].p(mi); mi=lpi[3].p(mi);
            mq=lpq[0].p(mq); mq=lpq[1].p(mq); mq=lpq[2].p(mq); mq=lpq[3].p(mq);
            dec_i+=mi; dec_q+=mq;
            if(++dec_cnt < decim) continue;
            float oi=(float)(dec_i/dec_cnt), oq=(float)(dec_q/dec_cnt);
            dec_i=dec_q=0; dec_cnt=0;

            // FM 판별 (순시주파수) → boxcar 정합필터(이동평균) → 디코더
            float dft = atan2f(oq*prev_i - oi*prev_q, oi*prev_i + oq*prev_q + 1e-20f);
            prev_i=oi; prev_q=oq;
            box_sum += dft - box[box_pos]; box[box_pos]=dft;
            if(++box_pos>=W) box_pos=0;
            if(gate) dec.feed((float)(box_sum/W));   // 스컬치 열림 구간만 복조
        }
        my_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);

        int64_t t=now_ms();
        if(t-last_diag>=10000){
            bewe_log_push(0,"DMR[%d] diag: records=%ld\n", ch_idx, frames);
            last_diag=t;
        }
    }
    rec_close();                                            // 종료 시 녹음 마무리
    ch.ext_audio.store(false, std::memory_order_relaxed);   // FM 오디오 복귀
    if(!worker_stop_req(ch_idx)) worker_natural_exit(v, ch_idx);
    bewe_log_push(0,"DMR[%d] stop\n",ch_idx);
}

} // namespace dmr_mod
