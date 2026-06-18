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
#include <cmath>

class DmrDecoder {
public:
    std::function<void(const DmrRecord&)> on_record;

    void configure(double out_sr){ omega_ = out_sr/4800.0; reset(); buildPats(); }
    void reset(){
        buf_.clear(); base_=0; strobe_=0; inited_=false;
        symv_.clear(); syms_.clear(); pend_.clear(); lastSyncSmp_=-1e18;
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
                        pend_.push_back(syncStart); lastSyncSmp_=syncStart;
                    }
                }
            }
        }
        // 충분히 lookahead 된 pend 처리 (INFO2 = +77 심볼)
        while(!pend_.empty() && pend_.front() + 84*omega_ < strobe_){
            double ss = pend_.front(); pend_.pop_front();
            if(ss - 56*omega_ >= (double)base_) extractBurst(ss);
        }
        // 버퍼 트림 (필요분 보존 + 여유)
        if(buf_.size() > 16384){
            size_t drop = buf_.size() - 6000;
            buf_.erase(buf_.begin(), buf_.begin()+drop); base_ += drop;
        }
    }

private:
    double omega_=10.0, strobe_=0, lastSyncSmp_=-1e18;
    uint64_t base_=0; bool inited_=false;
    std::vector<float> buf_;
    std::deque<float> symv_; std::deque<double> syms_;
    std::deque<double> pend_;

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
    void extractBurst(double approxS0){
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
        int pol=0,syncErr=99; dmr::SyncType stype=dmr::SyncType::NONE;
        for(int pp=0;pp<2;pp++){
            uint8_t sb[48]; bitsAt(54,24,pp,sb);
            uint64_t got=0; for(int i=0;i<48;i++) got=(got<<1)|sb[i];
            for(int p=0;p<dmr::N_SYNC;p++){
                int e=__builtin_popcountll(got^dmr::SYNC_PATTERNS[p].bits48);
                if(e<syncErr){syncErr=e;pol=pp;stype=dmr::SYNC_PATTERNS[p].type;}
            }
        }
        if(syncErr>4) return;                       // 진짜 sync 아님

        DmrRecord m{};
        m.is_voice = dmr::sync_is_voice(stype);
        m.slot = dmr::sync_direct_slot(stype);      // direct 만; repeater=0(CACH 미구현)

        // Slot Type: burst-sym 49..53 + 78..82 → Golay(20,8) → CC|DataType
        uint8_t ab[10],bb[10]; bitsAt(49,5,pol,ab); bitsAt(78,5,pol,bb);
        uint32_t cw=0; for(int i=0;i<10;i++)cw=(cw<<1)|ab[i]; for(int i=0;i<10;i++)cw=(cw<<1)|bb[i];
        uint8_t d8; int gerr=dmr::golay20_8_decode(cw,d8);
        if(gerr<=3){
            m.color_code = (d8>>4)&0xF;
            m.data_type  = d8&0xF;
        }
        bool emit=false;
        if(gerr<=3 && (m.data_type==1||m.data_type==2||m.data_type==3)){   // LC/Term/CSBK
            uint8_t info[196]; bitsAt(0,49,pol,info); bitsAt(83,49,pol,info+98);
            uint8_t p96[96]; dmr::bptc196_96(info,p96);
            if(m.data_type==3){                        // CSBK
                m.csbko = (int)dmr::bits_to_uint(p96+2,6);
                uint16_t comp=dmr::crc16_ccitt(p96,80), stored=(uint16_t)dmr::bits_to_uint(p96+80,16);
                m.crc_ok = ((comp^stored)==0x5A5A);
                m.dst_id = dmr::bits_to_uint(p96+16,24);
                m.src_id = dmr::bits_to_uint(p96+40,24);
                emit = m.crc_ok;                       // 검증된 것만
            } else {                                   // Voice LC Hdr / Term LC
                m.flco = (int)dmr::bits_to_uint(p96+2,6);
                m.call_type = dmr_flco_calltype(m.flco);
                m.dst_id = dmr::bits_to_uint(p96+24,24);
                m.src_id = dmr::bits_to_uint(p96+48,24);
                // RS(12,9) 검증 미구현 → 알려진 voice FLCO 만 신뢰
                m.crc_ok = false;
                emit = (m.flco==0x00 || m.flco==0x03);
                m.is_voice = true;
            }
        }
        if(emit && on_record) on_record(m);
    }
};
