#pragma once
// ── DMR 동기 패턴 (ETSI TS 102 361-1 §9.1.1, Table 9.2) ─────────────────────
// 48비트 SYNC = 24 심볼(4FSK 디비트). 모든 SYNC 는 외곽 편이(±3)만 사용한다.
// 디비트→심볼 매핑(ETSI): 01→+3, 00→+1, 10→-1, 11→-3  (심볼당 2비트 MSB-first).
//   hi 비트 = 부호(0:+,1:-), lo 비트 = 크기(0:1,1:3).
// 알고리즘/상수는 ETSI 규격 및 dsd(GPLv2)/dsdcc(GPLv3)/go-dmr 에서 교차검증한
// 값만 클린룸 반영 — 소스 verbatim 복사 없음(AIS GNU-AIS 차용 자세와 동일).
#include <cstdint>
#include <cstddef>

namespace dmr {

// 48비트 동기 워드 (상위 48비트만 유효). BS/MS 는 go-dmr 대비 bit-exact 검증.
// direct-mode 4종(⚠)은 단일 출처 — 실신호/2차 소스로 확정 전까지 주의.
enum class SyncType : uint8_t {
    NONE=0,
    BS_VOICE, BS_DATA,
    MS_VOICE, MS_DATA, MS_RC,
    DIRECT_VOICE_TS1, DIRECT_DATA_TS1,
    DIRECT_VOICE_TS2, DIRECT_DATA_TS2,
};

struct SyncPattern { SyncType type; uint64_t bits48; const char* name; };

static constexpr SyncPattern SYNC_PATTERNS[] = {
    { SyncType::BS_VOICE,         0x755FD7DF75F7ULL, "BS_VOICE" },
    { SyncType::BS_DATA,          0xDFF57D75DF5DULL, "BS_DATA"  },
    { SyncType::MS_VOICE,         0x7F7D5DD57DFDULL, "MS_VOICE" },
    { SyncType::MS_DATA,          0xD5D7F77FD757ULL, "MS_DATA"  },
    { SyncType::MS_RC,            0x77D55F7DFD77ULL, "MS_RC"    },
    { SyncType::DIRECT_VOICE_TS1, 0x5D577F7757FFULL, "DIR_V_TS1"},
    { SyncType::DIRECT_DATA_TS1,  0xF7FDD5DDFD55ULL, "DIR_D_TS1"},
    { SyncType::DIRECT_VOICE_TS2, 0x7DFFD5F55D5FULL, "DIR_V_TS2"},
    { SyncType::DIRECT_DATA_TS2,  0xD7557F5FF7F5ULL, "DIR_D_TS2"},
};
static constexpr int N_SYNC = (int)(sizeof(SYNC_PATTERNS)/sizeof(SYNC_PATTERNS[0]));
static constexpr int SYNC_SYMS = 24;   // 48비트 / 2

// 디비트(hi,lo) → 심볼(+3/+1/-1/-3)
inline int dibit_to_sym(int hi, int lo){
    int sign = hi ? -1 : +1;
    int mag  = lo ?  3 :  1;
    return sign*mag;
}
// 심볼(+3/+1/-1/-3) → 디비트 2비트 (hi<<1|lo)
inline int sym_to_dibit(int sym){
    int hi = (sym<0) ? 1 : 0;
    int lo = (sym==3 || sym==-3) ? 1 : 0;
    return (hi<<1)|lo;
}

// 48비트 SYNC 를 24 심볼로 전개 (MSB-first). out[0] = 최상위 디비트.
inline void sync_to_syms(uint64_t bits48, int8_t out[SYNC_SYMS]){
    for(int k=0;k<SYNC_SYMS;k++){
        int hi = (int)((bits48 >> (47 - 2*k    )) & 1);
        int lo = (int)((bits48 >> (47 - 2*k - 1)) & 1);
        out[k] = (int8_t)dibit_to_sym(hi, lo);
    }
}

// 심볼 24개 윈도우(하드 ±3/±1) 를 모든 SYNC 패턴과 정상/반전 양쪽 비교.
// 반환: 최소 불일치 심볼 수. best 에 매칭 타입, inverted 에 극성반전 여부.
// win[i] 는 슬라이스된 정수 심볼(+3/+1/-1/-3).
inline int sync_correlate(const int8_t* win, SyncType& best, bool& inverted){
    int best_dist = SYNC_SYMS+1; best = SyncType::NONE; inverted=false;
    int8_t ref[SYNC_SYMS];
    for(int p=0;p<N_SYNC;p++){
        sync_to_syms(SYNC_PATTERNS[p].bits48, ref);
        int d0=0, d1=0;
        for(int k=0;k<SYNC_SYMS;k++){
            if(win[k]!=ref[k])  d0++;     // 정상
            if(win[k]!=-ref[k]) d1++;     // 극성반전
        }
        if(d0<best_dist){ best_dist=d0; best=SYNC_PATTERNS[p].type; inverted=false; }
        if(d1<best_dist){ best_dist=d1; best=SYNC_PATTERNS[p].type; inverted=true; }
    }
    return best_dist;
}

inline const char* sync_name(SyncType t){
    for(int p=0;p<N_SYNC;p++) if(SYNC_PATTERNS[p].type==t) return SYNC_PATTERNS[p].name;
    return "NONE";
}
inline bool sync_is_voice(SyncType t){
    return t==SyncType::BS_VOICE||t==SyncType::MS_VOICE||
           t==SyncType::DIRECT_VOICE_TS1||t==SyncType::DIRECT_VOICE_TS2;
}
// direct-mode 슬롯 (1/2). repeater/MS 는 0 반환(슬롯은 CACH 로 결정).
inline int sync_direct_slot(SyncType t){
    if(t==SyncType::DIRECT_VOICE_TS1||t==SyncType::DIRECT_DATA_TS1) return 1;
    if(t==SyncType::DIRECT_VOICE_TS2||t==SyncType::DIRECT_DATA_TS2) return 2;
    return 0;
}

} // namespace dmr
