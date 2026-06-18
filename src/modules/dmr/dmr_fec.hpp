#pragma once
// ── DMR FEC 원시함수 (ETSI TS 102 361-1 Annex B) — header-only, zero-dep ─────
// 클린룸: 패리티 방정식/생성다항식은 ETSI 규격 및 dsd(GPLv2)/dsdcc(GPLv3)/
// ok-dmrlib 에서 교차검증한 "수식·상수"만 반영. 소스 verbatim 복사 없음.
//
// 비트 표현: uint8_t 배열, 각 원소 0/1, 인덱스 0 = MSB(전송 선두).
// 디코드는 가능한 한 brute-force(전수 비트반전 → syndrome==0 / 최근접) 로 구현 —
// 손으로 옮긴 syndrome-위치 테이블의 오타 위험 제거. (코드워드 작아 비용 무시가능)
//
// ⚠ VALIDATE 표시: 생성다항식/행렬이 출처마다 달라 실신호로 적합성 확정 필요한 항목.
#include <cstdint>
#include <cstring>

namespace dmr {

// ── 비트 유틸 ───────────────────────────────────────────────────────────────
inline uint32_t bits_to_uint(const uint8_t* b, int n){      // MSB-first
    uint32_t v=0; for(int i=0;i<n;i++) v=(v<<1)|(b[i]&1); return v;
}
inline void uint_to_bits(uint32_t v, uint8_t* b, int n){    // MSB-first
    for(int i=0;i<n;i++) b[n-1-i]=(v>>i)&1;
}

// ════════════════════════ Hamming 계열 (단일오류정정, d=3) ════════════════════
// 패리티 방정식(ETSI Annex B). 각 코드는 [데이터 | 패리티] 체계적 배치.
// 디코드: 패리티 재계산→불일치시 각 비트 1개 반전해 syndrome==0 찾기(brute).

// Hamming(13,9,3): 9 data + 4 parity
inline void h13_9_par(const uint8_t* d, uint8_t* p){
    p[0]=d[0]^d[1]^d[3]^d[5]^d[6];
    p[1]=d[0]^d[1]^d[2]^d[4]^d[6]^d[7];
    p[2]=d[0]^d[1]^d[2]^d[3]^d[5]^d[7]^d[8];
    p[3]=d[0]^d[2]^d[4]^d[5]^d[8];
}
// Hamming(15,11,3): 11 data + 4 parity
inline void h15_11_par(const uint8_t* d, uint8_t* p){
    p[0]=d[0]^d[1]^d[2]^d[3]^d[5]^d[7]^d[8];
    p[1]=d[1]^d[2]^d[3]^d[4]^d[6]^d[8]^d[9];
    p[2]=d[2]^d[3]^d[4]^d[5]^d[7]^d[9]^d[10];
    p[3]=d[0]^d[1]^d[2]^d[4]^d[6]^d[7]^d[10];   // ETSI(ok-dmrlib): d8 항 없음
}
// Hamming(17,12,3): 12 data + 5 parity  (rate-3/4 data, 본 경로 미사용)
inline void h17_12_par(const uint8_t* d, uint8_t* p){
    p[0]=d[0]^d[1]^d[2]^d[3]^d[6]^d[7]^d[9];
    p[1]=d[0]^d[1]^d[2]^d[3]^d[4]^d[7]^d[8]^d[10];
    p[2]=d[0]^d[1]^d[2]^d[3]^d[4]^d[5]^d[8]^d[9]^d[11];
    p[3]=d[0]^d[4]^d[5]^d[7]^d[9]^d[10];
    p[4]=d[1]^d[5]^d[6]^d[8]^d[10]^d[11];
}

// 범용 brute 단일오류정정: par(data)=parity 체계. n_total = n_data+n_par.
template<int NDATA,int NPAR>
inline bool hamming_decode(uint8_t* cw, void(*par)(const uint8_t*,uint8_t*)){
    const int N=NDATA+NPAR;
    uint8_t p[NPAR];
    auto syn0=[&](const uint8_t* w)->bool{
        uint8_t pp[NPAR]; par(w,pp);
        for(int i=0;i<NPAR;i++) if((w[NDATA+i]&1)!=pp[i]) return false;
        return true;
    };
    (void)p;
    if(syn0(cw)) return true;                 // 무오류
    for(int f=0; f<N; f++){                    // 1비트 반전 시도
        cw[f]^=1; if(syn0(cw)) return true; cw[f]^=1;
    }
    return false;                              // 정정실패(>1 오류)
}
inline bool h13_9_decode (uint8_t* cw){ return hamming_decode<9,4>(cw,h13_9_par); }
inline bool h15_11_decode(uint8_t* cw){ return hamming_decode<11,4>(cw,h15_11_par); }

// Hamming(16,11,4): (15,11,3) + 전체 짝수 패리티 1비트 (임베디드 LC BPTC 행)
inline void h16_11_par(const uint8_t* d, uint8_t* p){
    h15_11_par(d,p);
    p[4]=d[0]^d[2]^d[5]^d[6]^d[8]^d[9]^d[10];   // ETSI(ok-dmrlib) 5번째 열
}

// ════════════════════════ CRC / Checksum ════════════════════════════════════
// 임베디드 LC 5비트 체크섬 = Σ(9 옥텟) mod 31  (CRC 아님)
inline uint8_t emb_lc_checksum(const uint8_t* lc72){
    uint16_t sum=0;
    for(int i=0;i<9;i++) sum += (uint16_t)bits_to_uint(lc72+i*8,8);
    return (uint8_t)(sum % 31);
}
// CRC-16/CCITT (poly 0x1021, init 0, non-reflected) + 타입 마스크 XOR. CSBK/Data hdr.
inline uint16_t crc16_ccitt(const uint8_t* bits, int nbits){
    uint16_t crc=0;
    for(int i=0;i<nbits;i++){
        uint16_t b=((crc>>15)&1) ^ (bits[i]&1);
        crc<<=1; if(b) crc^=0x1021;
    }
    return crc;
}
enum CrcMask : uint16_t { CRCM_CSBK=0xA5A5, CRCM_DATAHDR=0xCCCC, CRCM_MBC=0xAAAA, CRCM_PI=0x6969 };

// CRC-9 (poly 0x059) — rate-1/2 데이터 블록 (본 경로 미사용, 구조만)
inline uint16_t crc9(const uint8_t* bits,int nbits){
    uint16_t crc=0;
    for(int i=0;i<nbits;i++){ uint16_t b=((crc>>8)&1)^(bits[i]&1); crc=(crc<<1)&0x1FF; if(b)crc^=0x059; }
    return crc;
}

// ════════════════════════ Golay(20,8,7) — Slot Type ═════════════════════════
// 8 info([CC:4][DataType:4] MSB-first) → 20비트 체계적 [data8|parity12]. 최근접 디코드.
// 생성행렬 G(8×20) ETSI TS 102 361-1 B.3.1 (ok-dmrlib Golay2087, dsdcc 교차검증).
inline uint32_t golay20_8_encode(uint8_t v){
    static const uint8_t GPAR[8][12]={      // G 의 패리티부 (열 8..19)
        {0,0,1,1,1,1,0,1,1,0,1,0},
        {1,1,0,1,1,0,0,1,1,0,0,1},
        {0,1,1,0,1,1,0,0,1,1,0,1},
        {0,0,1,1,0,1,1,0,0,1,1,1},
        {1,1,0,1,1,1,0,0,0,1,1,0},
        {1,0,1,0,1,0,0,1,0,1,1,1},
        {1,0,0,1,0,0,1,1,1,1,1,0},
        {1,0,0,0,1,1,1,0,1,0,1,1}};
    uint8_t d[8]; for(int r=0;r<8;r++) d[r]=(v>>(7-r))&1;
    uint32_t cw=0;
    for(int r=0;r<8;r++) cw=(cw<<1)|d[r];                       // data8 (MSB=CC msb)
    for(int j=0;j<12;j++){ int p=0; for(int r=0;r<8;r++) p^=d[r]&GPAR[r][j]; cw=(cw<<1)|p; }
    return cw;                                                  // 20비트, bit19=CC msb
}
// 최근접 디코드 → data8, 반환=비트오류수(>3 이면 신뢰불가)
inline int golay20_8_decode(uint32_t cw20, uint8_t& out){
    int best=99; out=0;
    for(int d=0; d<256; d++){
        uint32_t c=golay20_8_encode((uint8_t)d);
        int dist=__builtin_popcount((c^cw20)&0xFFFFF);
        if(dist<best){ best=dist; out=(uint8_t)d; }
    }
    return best;
}

// ════════════════════════ QR(16,7,6)  ⚠VALIDATE ═════════════════════════════
// EMB(16): 7 info(CC4|PI1|LCSS2) + 9 parity. 최근접(128 코드워드).
// 생성다항식 g(x)=x^9+x^7+x^6+x^4+x^3+x+1? 출처상이 → 0x2DB 가정.
inline uint16_t qr16_7_encode(uint8_t data7){
    uint16_t reg=(uint16_t)(data7&0x7F)<<9;
    const uint16_t g=0x2DB;             // 10비트 ⚠VALIDATE
    for(int i=15;i>=9;i--) if(reg&(1u<<i)) reg^=g<<(i-9);
    return ((uint16_t)(data7&0x7F)<<9)|(reg&0x1FF);
}
inline int qr16_7_decode(uint16_t cw16, uint8_t& out){
    int best=99; out=0;
    for(int d=0; d<128; d++){
        uint16_t c=qr16_7_encode((uint8_t)d);
        int dist=__builtin_popcount((unsigned)((c^cw16)&0xFFFF));
        if(dist<best){ best=dist; out=(uint8_t)d; }
    }
    return best;
}

// ════════════════════════ BPTC(196,96) ══════════════════════════════════════
// Data/CSBK/Voice-LC-hdr/Term 버스트 페이로드. on-air 196비트(SYNC/EMB 제외한
// INFO 양반 108+108 중 196) → 디인터리브 → 13행×15열 → 행 H(15,11)·열 H(13,9)
// → 96 info. ⚠ 디인터리브 방향은 round-trip 으로 확정.
//
// 입력 onair[196] = 버스트 INFO 비트(앞 98 + 뒤 98 결합). R(3) 무시.
inline bool bptc196_96(const uint8_t onair[196], uint8_t out96[96]){
    // 1) 디인터리브: deint[i] = onair[(i*181) mod 196]   (ETSI; 방향 round-trip 검증)
    uint8_t m[196];
    for(int i=0;i<196;i++) m[i]=onair[(i*181)%196];
    // 2) 13행×15열 행렬. 196 = 1(R 예약) + 195(13*15). m[0]=R, m[1..195]→매트릭스.
    uint8_t M[13][15];
    for(int r=0;r<13;r++) for(int c=0;c<15;c++) M[r][c]=m[1 + r*15 + c];
    // 3) 열 디코드 Hamming(13,9): 각 열 = 13비트(행0..12), data=행0..8 par=행9..12
    for(int c=0;c<15;c++){
        uint8_t col[13]; for(int r=0;r<13;r++) col[r]=M[r][c];
        h13_9_decode(col);            // col[0..8]=data, [9..12]=par
        for(int r=0;r<13;r++) M[r][c]=col[r];
    }
    // 4) 행 디코드 Hamming(15,11): 각 행 = 15비트, data=0..10 par=11..14
    for(int r=0;r<13;r++){
        uint8_t row[15]; for(int c=0;c<15;c++) row[c]=M[r][c];
        h15_11_decode(row);
        for(int c=0;c<15;c++) M[r][c]=row[c];
    }
    // 5) 96 info 추출: ETSI — 행0 열3..10(8비트) + 행1..8 열0..10(88비트) = 96
    int o=0;
    for(int c=3;c<=10;c++) out96[o++]=M[0][c];
    for(int r=1;r<=8;r++) for(int c=0;c<=10;c++) out96[o++]=M[r][c];
    return o==96;
}

// ════════════════════════ RS(12,9) GF(256)  ⚠VALIDATE ═══════════════════════
// Voice LC hdr/Term 의 96비트 BPTC 페이로드 = LC[0:72] ‖ RS_parity[72:96].
// GF(256) prim 0x11D. 3 패리티 바이트. 타입 마스크 XOR.
struct GF256 {
    uint8_t exp[512], logt[256];
    GF256(){
        int x=1; for(int i=0;i<255;i++){ exp[i]=(uint8_t)x; logt[x]=(uint8_t)i; x<<=1; if(x&0x100)x^=0x11D; }
        for(int i=255;i<512;i++) exp[i]=exp[i-255]; logt[0]=0;
    }
    uint8_t mul(uint8_t a,uint8_t b)const{ return (a&&b)?exp[logt[a]+logt[b]]:0; }
};
// 신드롬 검사만(정정 없이): 마스크 적용 후 9데이터 인코드 == 수신 3패리티?
// 마스크: Voice LC hdr 등 타입별. 여기선 검증만(정정은 차후).
inline bool rs12_9_check(const uint8_t data9[9], const uint8_t par3[3], const uint8_t mask3[3]){
    static const GF256 gf;
    // g(x) = (x+α^0)(x+α^1)(x+α^2) 로 계산한 3 패리티 (체계적). ⚠VALIDATE roots.
    uint8_t p[3]={0,0,0};
    for(int i=0;i<9;i++){
        uint8_t fb = data9[i] ^ p[0];
        p[0]=p[1]^gf.mul(fb, gf.exp[ (0+1)%255 ]);   // 계수 ⚠VALIDATE
        p[1]=p[2]^gf.mul(fb, gf.exp[ (1+1)%255 ]);
        p[2]=     gf.mul(fb, gf.exp[ (2+1)%255 ]);
    }
    for(int i=0;i<3;i++) if((par3[i]^mask3[i])!=p[i]) return false;
    return true;
}

} // namespace dmr
