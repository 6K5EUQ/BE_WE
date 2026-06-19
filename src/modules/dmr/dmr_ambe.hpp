#pragma once
// ── DMR AMBE+2 음성 → PCM (mbelib 래퍼) ─────────────────────────────────────
// DMR 음성프레임(72b)을 mbelib ambe_fr[4][24] 로 디인터리브 후 디코드 → 160샘플@8kHz.
// 디인터리브 테이블/절차는 DSD(szechyjs/dsd)·dsd-fme 소스에서 검증(byte-identical):
//   dmr_const.h rW/rX/rY/rZ, dmr_voice.c fill loop. mbelib 가 디스크램블+ECC 내부수행 —
//   생바이트만 넣고 호출. mbe_initMbeParms 는 통화당 1회(예측기 상태 유지).
// AMBE+2 = DVSI 독점 코덱; mbelib 는 리버스엔지니어링(ISC). 특허 회색지대 인지.
//
// 이 헤더는 mbelib 링크 필요 → 워커(dmr_decode.cpp)·음성 하니스에서만 include.
// DmrDecoder 본체는 mbelib 비의존(음성프레임 dibit 만 콜백 방출).
#include <cstdint>
extern "C" {
#include "mbelib.h"
}

namespace dmr {

// DMR AMBE 디인터리브 스케줄 (DSD/dsd-fme dmr_const.h, byte-identical 검증)
static const int AMBE_rW[36] = {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,2,0,2,0,2,0,2,0,2,0,2,0,2};
static const int AMBE_rX[36] = {23,10,22,9,21,8,20,7,19,6,18,5,17,4,16,3,15,2,14,1,13,0,12,10,11,9,10,8,9,7,8,6,7,5,6,4};
static const int AMBE_rY[36] = {0,2,0,2,0,2,0,2,0,3,0,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3,1,3};
static const int AMBE_rZ[36] = {5,3,4,2,3,1,2,0,1,13,0,12,22,11,21,10,20,9,19,8,18,7,17,6,16,5,15,4,14,3,13,2,12,1,11,0};

// 36 dibit (각 0..3, MSB=먼저전송비트) → ambe_fr[4][24]
inline void ambe_deinterleave(const uint8_t dibits36[36], char ambe_fr[4][24]){
    for(int i=0;i<36;i++){
        int d = dibits36[i];
        ambe_fr[AMBE_rW[i]][AMBE_rX[i]] = (char)(1 & (d>>1));   // MSB
        ambe_fr[AMBE_rY[i]][AMBE_rZ[i]] = (char)(1 & d);        // LSB
    }
}

class DmrAmbeDecoder {
public:
    DmrAmbeDecoder(){ reset(); }
    void reset(){ mbe_initMbeParms(&cur_, &prev_, &prev_enh_); }   // 통화 시작마다

    // 음성프레임 1개(36 dibit) → 160 int16 @ 8kHz. 반환=AMBE ECC 오류수.
    int decode(const uint8_t dibits36[36], short out160[160]){
        char ambe_fr[4][24]; char ambe_d[49];
        int errs=0, errs2=0; char es[64];
        ambe_deinterleave(dibits36, ambe_fr);          // 생바이트 — 디스크램블 금지
        mbe_processAmbe3600x2450Frame(out160, &errs, &errs2, es,
                                      ambe_fr, ambe_d, &cur_, &prev_, &prev_enh_, 3);
        return errs2;
    }
private:
    mbe_parms cur_, prev_, prev_enh_;
};

} // namespace dmr
