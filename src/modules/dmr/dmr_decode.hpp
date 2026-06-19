#pragma once
// ── DMR 스트리밍 디코더 (ETSI TS 102 361-1, Tier II 메타/시그널링) ───────────
// 입력 = 정합필터(boxcar I&D) 적용된 FM 판별기 샘플 1개씩(feed). DDC/판별기/정합
// 필터는 워커(dmr_decode.cpp). 내부: 심볼 strobe(sync 탐지) → 버스트별 sample-space
// 정밀추출(위상+국소omega 보정) → Slot Type Golay / BPTC(196,96) → LC/CSBK → on_record.
// 오프라인 하니스(tools/dmr_offline.cpp)에서 실신호 CRC 검증된 로직을 스트리밍화.
#include "dmr_sync.hpp"
#include "dmr_fec.hpp"
#include "dmr_meta.hpp"
#include <functional>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>

class DmrDecoder {
public:
    std::function<void(const DmrRecord&)> on_record;
    // 음성버스트 → AMBE 프레임 dibit 방출 (frames = nframes*36 dibit; new_call=새 통화).
    // mbelib 비의존 — 소비자(워커/하니스)가 dmr_ambe.hpp 로 디코드.
    std::function<void(const uint8_t* frames, int nframes, bool new_call)> on_voice;

    long dbgDet=0, dbgExtract=0, dbgVoice=0, dbgData=0;   // 진단 카운터
    void configure(double out_sr){ omega_ = omegaNom_ = out_sr/4800.0; reset(); buildPats(); }
    void reset(){
        buf_.clear(); base_=0; strobe_=0; inited_=false;
        symv_.clear(); syms_.clear(); pend_.clear();
        lastSyncSmp_=-1e18; lastVoiceSmp_=-1e18;
        polVote_[0]=polVote_[1]=0; polLock_=-1;
        voicePend_.clear();
    }

    void feed(float s){
        buf_.push_back(s);
        double tot = (double)base_ + buf_.size();
        if(!inited_){ if(buf_.size() > 64){ strobe_ = (double)base_ + 2*omega_; inited_=true; } else return; }
        // 심볼 strobe (coarse, sync 탐지용)
        while(strobe_ + 2.0 < tot){
            float v = interpAbs(strobe_);
            symv_.push_back(v); syms_.push_back(strobe_);
            if(symv_.size() > 220){ symv_.pop_front(); syms_.pop_front(); }
            strobe_ += omega_;
            if(symv_.size() >= dmr::SYNC_SYMS){
                float win[dmr::SYNC_SYMS];
                int n=(int)symv_.size();
                for(int k=0;k<dmr::SYNC_SYMS;k++) win[k]=symv_[n-dmr::SYNC_SYMS+k];
                int sc=signCorr(win);
                if(sc>=22){
                    double syncStart = syms_[n-dmr::SYNC_SYMS];   // 버스트-sym 54 위치
                    if(syncStart - lastSyncSmp_ > 20*omega_){       // 근접 중복 억제
                        pend_.push_back(syncStart); lastSyncSmp_=syncStart; dbgDet++;
                    }
                }
            }
        }
        // 충분히 lookahead 된 pend 처리 (INFO2 = +77 심볼)
        while(!pend_.empty() && pend_.front() + 84*omega_ < strobe_){
            double ss = pend_.front(); pend_.pop_front();
            if(ss - 56*omega_ >= (double)base_) extractBurst(ss);
        }
        // 음성 슈퍼프레임 B–F (예약, voice-A 가 등록) — sync 없이 위치+A레벨로 추출
        while(!voicePend_.empty() && voicePend_.front().s0 + 84*omega_ < strobe_){
            VPend vp = voicePend_.front(); voicePend_.pop_front();
            if(vp.s0 - 56*omega_ >= (double)base_) extractVoiceBurst(vp);
        }
        // 버퍼 트림 (필요분 보존 + 여유)
        if(buf_.size() > 24000){
            size_t drop = buf_.size() - 8000;
            buf_.erase(buf_.begin(), buf_.begin()+drop); base_ += drop;
        }
    }

private:
    double omega_=10.0, omegaNom_=10.0, strobe_=0, lastSyncSmp_=-1e18, lastVoiceSmp_=-1e18;
    int polVote_[2]={0,0}, polLock_=-1;
    uint64_t base_=0; bool inited_=false;
    std::vector<float> buf_;
    std::deque<float> symv_; std::deque<double> syms_;
    std::deque<double> pend_;
    static constexpr double VBURST = 288.0;   // 음성버스트 간격(심볼) = 60ms × 4800
    // 음성 슈퍼프레임 B–F 예약: voice-A 만 트리거(데이터버스트는 안 함 → 과추출 없음).
    // A 의 sync-보정 레벨/극성을 B–F 에 물려줌(B–F 는 EMB 라 sync 없음).
    struct VPend{ double s0,sdc,slvl,ol; int pol; };
    std::deque<VPend> voicePend_;

    // sync 부호열 (모든 sync = ±3 → 부호상관, AGC-독립)
    int8_t patSgn_[dmr::N_SYNC][dmr::SYNC_SYMS];
    void buildPats(){
        for(int p=0;p<dmr::N_SYNC;p++){
            int8_t s[dmr::SYNC_SYMS]; dmr::sync_to_syms(dmr::SYNC_PATTERNS[p].bits48,s);
            for(int k=0;k<dmr::SYNC_SYMS;k++) patSgn_[p][k]=(s[k]<0)?-1:1;
        }
    }
    int signCorr(const float* w){     // 최대 일치수(정상/반전)
        int best=-1;
        for(int p=0;p<dmr::N_SYNC;p++){
            int m0=0,m1=0;
            for(int k=0;k<dmr::SYNC_SYMS;k++){ int s=(w[k]<0)?-1:1;
                if(s==patSgn_[p][k]) m0++; else m1++; }
            if(m0>best)best=m0; if(m1>best)best=m1;
        }
        return best;
    }
    float interpAbs(double p){
        long i=(long)floor(p) - (long)base_; double f=p-floor(p);
        if(i<0 || i+1>=(long)buf_.size()) return 0.f;
        return buf_[i]*(1.f-(float)f)+buf_[i+1]*(float)f;
    }

    // 버스트 추출: approxS0 = sync 심볼0(=버스트-sym 54) 근사 샘플위치
    void extractBurst(double approxS0){ dbgExtract++;
        // 2D 타이밍: 위상 dl × 국소 omega ol, burst-sym 0..131 Σ|d| 최대
        double S0=approxS0, ol=omega_, bsc=-1;
        for(double col=omega_-0.10; col<=omega_+0.10+1e-9; col+=0.01)
          for(double dl=-omega_*0.6; dl<=omega_*0.6+1e-9; dl+=0.3){
            double base=approxS0+dl, sc=0;
            for(int b=0;b<132;b+=2) sc+=fabs(interpAbs(base+(b-54)*col));
            if(sc>bsc){ bsc=sc; S0=base; ol=col; }
          }
        // 국소 dc/outer (sync 24심볼 = 전부 ±3)
        double sdc=0; for(int k=0;k<dmr::SYNC_SYMS;k++) sdc+=interpAbs(S0+k*ol); sdc/=dmr::SYNC_SYMS;
        double slvl=0; for(int k=0;k<dmr::SYNC_SYMS;k++) slvl+=fabs(interpAbs(S0+k*ol)-sdc); slvl/=dmr::SYNC_SYMS;
        if(slvl < 1e-4) return;
        auto dibitAt=[&](double bsym,int pol)->int{
            double v=interpAbs(S0+(bsym-54.0)*ol); v=pol?(2*sdc-v):v;
            int sym=(v>=sdc+2*slvl/3)?3:(v>=sdc)?1:(v>=sdc-2*slvl/3)?-1:-3;
            return dmr::sym_to_dibit(sym);
        };
        auto bitsAt=[&](double b0,int nsym,int pol,uint8_t* out){
            for(int s=0;s<nsym;s++){ int db=dibitAt(b0+s,pol); out[2*s]=(db>>1)&1; out[2*s+1]=db&1; }
        };
        // 극성 + sync 타입: sync 24심볼 재구성 → biterr 최소
        // 극성 + sync 타입: 24심볼의 부호(hi 비트)만 비교 — 약신호서도 robust.
        // (magnitude LSB 는 노이즈 큼 → 부호로만 BS_VOICE vs BS_DATA 명확 구분)
        int pol=0,syncErr=99; dmr::SyncType stype=dmr::SyncType::NONE;
        uint32_t patHi[dmr::N_SYNC];
        for(int p=0;p<dmr::N_SYNC;p++){ uint32_t ph=0; uint64_t pb=dmr::SYNC_PATTERNS[p].bits48;
            for(int k=0;k<24;k++) ph=(ph<<1)|(uint32_t)((pb>>(47-2*k))&1); patHi[p]=ph; }
        for(int pp=0;pp<2;pp++){
            uint32_t got=0;
            for(int k=0;k<24;k++){ double vv=interpAbs(S0+k*ol); if(pp)vv=2*sdc-vv;
                got=(got<<1)|((vv-sdc<0)?1u:0u); }
            for(int p=0;p<dmr::N_SYNC;p++){
                int e=__builtin_popcount(got^patHi[p]);
                if(e<syncErr){syncErr=e;pol=pp;stype=dmr::SYNC_PATTERNS[p].type;}
            }
        }
        // 전역 극성 고정: 확신 버스트(syncErr 작음)로 투표 → lock. 약신호 버스트의
        // 극성 뒤집힘 방지(극성 틀리면 AMBE MSB 전부 반전 → 음성 깨짐).
        if(syncErr<=3){ polVote_[pol]++;
            if(polLock_<0 && polVote_[0]+polVote_[1]>=6) polLock_=(polVote_[0]>=polVote_[1])?0:1; }
        if(polLock_>=0) pol=polLock_;
        // 고정극성서 stype 재결정: BS_VOICE 와 BS_DATA 는 부호 보수관계 → 두 극성서
        // 동점 → lock 된 극성으로만 판별해야 voice/data 정확.
        { uint32_t got=0; for(int k=0;k<24;k++){ double vv=interpAbs(S0+k*ol); if(pol)vv=2*sdc-vv;
            got=(got<<1)|((vv-sdc<0)?1u:0u); }
          syncErr=99; for(int p=0;p<dmr::N_SYNC;p++){ int e=__builtin_popcount(got^patHi[p]);
            if(e<syncErr){syncErr=e;stype=dmr::SYNC_PATTERNS[p].type;} } }
        // 심볼클럭 트래킹: 확신 버스트의 국소 omega 로 strobe omega 보정.
        // (파일 SR 반올림/TX ppm → strobe 드리프트 → sync 놓침 방지)
        if(syncErr<=2){ omega_ += 0.1*(ol-omega_);
            if(omega_<omegaNom_-0.3) omega_=omegaNom_-0.3;
            if(omega_>omegaNom_+0.3) omega_=omegaNom_+0.3; }
        // sign-corr(≥22)가 이미 진짜 sync 검증 → 약한신호로 magnitude syncErr 높아도
        // voice/data 판별은 Slot Type Golay 성공여부로 (robust). syncErr 게이트 폐기.

        DmrRecord m{};
        m.slot = dmr::sync_direct_slot(stype);      // direct 만; repeater=0(CACH 미구현)
        bool voiceSync = dmr::sync_is_voice(stype); // 부호기반 타입 → voice/data 분류

        // Slot Type: burst-sym 49..53 + 78..82 → Golay(20,8). 성공 = DATA, 실패 = VOICE.
        uint8_t ab[10],bb[10]; bitsAt(49,5,pol,ab); bitsAt(78,5,pol,bb);
        uint32_t cw=0; for(int i=0;i<10;i++)cw=(cw<<1)|ab[i]; for(int i=0;i<10;i++)cw=(cw<<1)|bb[i];
        uint8_t d8; int gerr=dmr::golay20_8_decode(cw,d8);

        if(!voiceSync && gerr<=2){ dbgData++;         // ── DATA 버스트 (data-sync) ──
            m.color_code=(d8>>4)&0xF; m.data_type=d8&0xF;
            bool emit=false;
            if(m.data_type==1||m.data_type==2||m.data_type==3){     // LC/Term/CSBK
                uint8_t info[196]; bitsAt(0,49,pol,info); bitsAt(83,49,pol,info+98);
                uint8_t p96[96]; dmr::bptc196_96(info,p96);
                if(m.data_type==3){                  // CSBK
                    m.csbko=(int)dmr::bits_to_uint(p96+2,6);
                    uint16_t comp=dmr::crc16_ccitt(p96,80), stored=(uint16_t)dmr::bits_to_uint(p96+80,16);
                    m.crc_ok=((comp^stored)==0x5A5A);
                    m.dst_id=dmr::bits_to_uint(p96+16,24); m.src_id=dmr::bits_to_uint(p96+40,24);
                    emit=m.crc_ok;
                } else {                             // Voice LC Hdr / Term LC
                    m.flco=(int)dmr::bits_to_uint(p96+2,6); m.call_type=dmr_flco_calltype(m.flco);
                    m.dst_id=dmr::bits_to_uint(p96+24,24); m.src_id=dmr::bits_to_uint(p96+48,24);
                    m.crc_ok=false; m.is_voice=true; emit=(m.flco==0x00||m.flco==0x03);
                }
            }
            if(emit && on_record) on_record(m);
        } else if(voiceSync && on_voice){ dbgVoice++;  // ── VOICE 버스트 (voice-sync) → 3 AMBE ──
            // 음성버스트는 60ms(=288심볼)마다 sync 보유 → omega 트래킹으로 전부 검출.
            // 프레임0=sym0..35, 프레임1=36..53++78..95(SYNC 건너뜀), 프레임2=96..131.
            bool new_call = (S0 - lastVoiceSmp_) > 4.0*VBURST*ol;
            uint8_t fr[3*36];
            for(int k=0;k<36;k++) fr[k]       = (uint8_t)dibitAt(k,pol);
            for(int k=0;k<18;k++) fr[36+k]    = (uint8_t)dibitAt(36+k,pol);
            for(int k=0;k<18;k++) fr[36+18+k] = (uint8_t)dibitAt(78+k,pol);
            for(int k=0;k<36;k++) fr[72+k]    = (uint8_t)dibitAt(96+k,pol);
            on_voice(fr, 3, new_call);
            lastVoiceSmp_ = S0;
            // 슈퍼프레임 B–F 예약 (A 의 레벨/극성 물려줌; B–F 는 EMB 라 sync 없음)
            for(int n=1;n<=5;n++) voicePend_.push_back({S0+n*VBURST*ol, sdc, slvl, ol, pol});
        }
    }

    // 음성버스트(A 포함): sync 없이 위치로 추출 (극성 고정, 타이밍/레벨 자체추정)
    void extractVoiceBurst(const VPend& vp){
        // 타이밍만 국소 보정(레벨/극성은 voice-A 에서 물려받음 → B–F 도 깨끗)
        double S0=vp.s0, bsc=-1;
        for(double dl=-omega_*0.5; dl<=omega_*0.5+1e-9; dl+=0.25){
            double base=vp.s0+dl, sc=0;
            for(int b=0;b<132;b+=2) sc+=fabs(interpAbs(base+(b-54)*vp.ol));
            if(sc>bsc){ bsc=sc; S0=base; }
        }
        double sdc=vp.sdc, slvl=vp.slvl, ol=vp.ol; int pol=vp.pol;
        if(slvl<1e-4 || !on_voice) return;
        lastVoiceSmp_ = S0;        // B–F 도 갱신 → A 의 new_call 오판(360ms 공백) 방지
        auto dibitAt=[&](double bsym)->int{
            double v=interpAbs(S0+(bsym-54.0)*ol); v=pol?(2*sdc-v):v;
            int sym=(v>=sdc+2*slvl/3)?3:(v>=sdc)?1:(v>=sdc-2*slvl/3)?-1:-3;
            return dmr::sym_to_dibit(sym);
        };
        uint8_t fr[3*36];
        for(int k=0;k<36;k++) fr[k]       = (uint8_t)dibitAt(k);
        for(int k=0;k<18;k++) fr[36+k]    = (uint8_t)dibitAt(36+k);
        for(int k=0;k<18;k++) fr[36+18+k] = (uint8_t)dibitAt(78+k);
        for(int k=0;k<36;k++) fr[72+k]    = (uint8_t)dibitAt(96+k);
        on_voice(fr, 3, false);
    }
};
