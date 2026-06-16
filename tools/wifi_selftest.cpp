// 코딩 체인 셀프테스트: SIGNAL 필드 encode→interleave→BPSK→(완전채널)→
//   deinterleave→Viterbi→parse 가 rate/len 복원하는지. 동기/CFO/채널 배제.
//   build: g++ -O2 -I src tools/wifi_selftest.cpp -lm -o /tmp/wifi_selftest
#include <cstdio>
#include <vector>
#include <cstdint>
#include "modules/wifi/wifi_ofdm.hpp"
using namespace wifi_ofdm;

// 길쌈부호 encode (K=7, g0=0133,g1=0171), tail 포함 가정 (입력에 6 tail 0)
static void conv_encode(const std::vector<uint8_t>& in, std::vector<uint8_t>& out){
    int reg=0; out.clear();
    auto par=[&](int g){ int x=reg&g,c=0; while(x){c^=x&1;x>>=1;} return c; };
    for(uint8_t b: in){ reg=((reg<<1)|b)&0x7f; out.push_back(par(0133)); out.push_back(par(0171)); }
}
// 인터리버 (N_CBPS=48, BPSK): output[i(k)]=coded[k], i(k)=3*(k%16)+k/16
static void interleave48(const uint8_t* coded, uint8_t* out){
    for(int k=0;k<48;k++){ int i=3*(k%16)+k/16; out[i]=coded[k]; }
}

int main(){
    // SIGNAL 24비트: rate(4)=6Mbps(1101) | rsv(1)=0 | len(12 LSB)=100 | parity(1) | tail(6)=0
    int rate_bits[4]={1,1,0,1};          // R1R2R3R4 = 6 Mbps
    int len=100;
    std::vector<uint8_t> sig(24,0);
    for(int i=0;i<4;i++) sig[i]=rate_bits[i];
    sig[4]=0;
    for(int i=0;i<12;i++) sig[5+i]=(len>>i)&1;
    int par=0; for(int i=0;i<17;i++) par^=sig[i]; sig[17]=par;
    // tail 18..23 = 0

    // encode → 48 coded bits
    std::vector<uint8_t> coded; conv_encode(sig, coded);
    printf("encoded %zu coded bits (기대 48)\n", coded.size());

    // interleave
    uint8_t inter[48]; interleave48(coded.data(), inter);
    // BPSK map (bit0→-1, bit1→+1), 완전채널 → subc[i] = ±1
    // RX: subc 는 subcarrier 순서 = interleaver output 순서. soft=-real(=-map) → bit0=+soft
    float subc[48];
    for(int i=0;i<48;i++) subc[i] = (inter[i]? +1.f : -1.f);   // map
    // RX demap: soft = -map (802.11 BPSK)
    float soft_subc[48]; for(int i=0;i<48;i++) soft_subc[i]=-subc[i];

    // deinterleave: coded[k]=subc[i(k)]
    float coded_soft[48]; for(int k=0;k<48;k++){ int i=3*(k%16)+k/16; coded_soft[k]=soft_subc[i]; }

    // Viterbi
    Viterbi vit; std::vector<uint8_t> bits; vit.decode(coded_soft,24,bits);

    // parse
    int rrate=0; for(int i=0;i<4;i++) rrate|=bits[i]<<i;
    int rlen=0;  for(int i=0;i<12;i++) rlen|=bits[5+i]<<i;
    int rpar=0;  for(int i=0;i<17;i++) rpar^=bits[i];
    printf("decoded: rate=0x%X (기대 0xB) len=%d (기대 100) parity_ok=%d tail0=%d\n",
        rrate, rlen, rpar==bits[17], (bits[18]|bits[19]|bits[20]|bits[21]|bits[22]|bits[23])==0);
    printf("input sig bits:  "); for(int i=0;i<24;i++) printf("%d",sig[i]); printf("\n");
    printf("decoded bits:    "); for(int i=0;i<24;i++) printf("%d",bits[i]); printf("\n");
    int err=0; for(int i=0;i<24;i++) if(sig[i]!=bits[i]) err++;
    printf("bit errors: %d/24  → %s\n", err, err==0?"CHAIN OK":"CHAIN BUG");
    return 0;
}
