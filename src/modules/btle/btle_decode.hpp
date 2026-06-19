#pragma once
// ── BLE 광고 패킷 스트리밍 디코더 (자작, 공개 표준만: Bluetooth Core Vol 6 Part B) ──
// 입력 = 채널 baseband 의 FM 판별기 출력열(순시주파수) d[n], 샘플레이트 fs (≈4 MHz).
//        RF→baseband DDC/데시메이션/FM판별은 워커(btle_decode.cpp). adsb_decode.hpp 미러.
//
// 물리계층 (1M PHY, 광고 채널):
//   • GFSK 1 Mbit/s. bit 1 = 양(+) 주파수편이, bit 0 = 음(-). FM판별 부호 = 비트.
//   • 비트 전송순서 LSB-first. 프리앰블 8b(AA LSB=0 → 0xAA) + AccessAddress 32b.
//     광고 AccessAddress 는 고정 0x8E89BED6 → 40-bit 상관기로 동기.
//   • AA 이후(헤더+페이로드+CRC)는 채널인덱스로 화이트닝(LFSR x^7+x^4+1).
//   • PDU 헤더 2B(type/ChSel/TxAdd/RxAdd/Length) → 페이로드 Length B → CRC 3B.
//   • CRC-24 (poly 0x00065B, 광고 init 0x555555). 통과 시 무오류.
//
// 해독 흐름: FM판별 버퍼 → AA 40-bit 상관(동기) → LSB-first 슬라이스 → 디화이트닝
//            → 헤더(길이) → CRC-24 → AdvA/AD구조 또는 CONNECT_IND 파라미터 추출.
//
// !! 검증 포인트 !! 화이트닝 seed/tap 과 CRC(init/reversed poly 0xDA6000)는 비트단위
//    상수다. 실신호(고정 MAC 비콘)로 CRC-OK 확인 필수. 진단 카운터로 단계별 추적:
//    dg_sync(AA동기) → dg_lenok(길이정상) → dg_ok/dg_fail(CRC). dg_sync>0,dg_ok=0 이면
//    화이트닝/CRC 상수 문제(가능 수정: init 0xAAAAAA 또는 최종 비트역전).
#include <cstdint>
#include <cstring>
#include <cmath>
#include <functional>
#include <vector>
#include "btle_meta.hpp"

class BtleDecoder {
public:
    std::function<void(const BtleRecord&)> on_record;
    // 디버그: 길이정상 후보마다 (pdu,len,dewhitened bytes, calc/rx CRC). 기본 null.
    std::function<void(int,int,const uint8_t*,uint32_t,uint32_t)> on_debug;

    // ── 진단 카운터 (워커가 주기적으로 읽어 로그) ──
    long  dg_samp=0, dg_sync=0, dg_lenok=0, dg_ok=0, dg_fail=0;
    float dg_maxlev=0;   // 최대 |FM판별| (신호 유무)
    float sync_thresh=0.80f;   // AA 정규화 상관 임계 (실측: 진짜 패킷 0.9+, 노이즈 컷)
    void  diag_reset(){ dg_samp=dg_sync=dg_lenok=dg_ok=dg_fail=0; dg_maxlev=0; }

    void reset(double sample_rate, int ch_idx, int adv_channel){
        sr  = sample_rate>1.0 ? sample_rate : 4000000.0;
        spb = sr/1e6;                       // samples per bit (1 Mbit/s)
        ch  = ch_idx;
        adv_chan = adv_channel;
        buf.clear(); samp_base=0;
        build_ref();
    }

    // 워커가 FM판별 샘플 묶음을 투입. 내부에서 AA 동기 스캔 + 해독.
    void process(const float* d, size_t n){
        dg_samp += (long)n;
        for(size_t i=0;i<n;i++){ float a=std::fabs(d[i]); if(a>dg_maxlev) dg_maxlev=a; }
        buf.insert(buf.end(), d, d+n);
        // 최장: 40-bit 동기 + (헤더2+페이로드37+CRC3)=42B → 376 bit
        size_t need = (size_t)((SYNC_BITS + (2+37+3)*8 + 2)*spb) + 8;
        if(buf.size() < need) return;
        size_t limit = buf.size() - need;
        size_t j=0;
        while(j <= limit){
            int pol; float dc;
            if(find_sync(j, pol, dc)){
                dg_sync++;
                int adv = decode_at(j, pol, dc);
                if(adv>0){ j += (size_t)adv; continue; }
            }
            j++;
        }
        if(j>0){ samp_base += (double)j; buf.erase(buf.begin(), buf.begin()+j); }
    }

private:
    static constexpr int    SYNC_BITS   = 40;     // 8 프리앰블 + 32 AA
    static constexpr uint32_t ADV_AA    = 0x8E89BED6u;

    double sr=4000000.0, spb=4.0, samp_base=0;
    int    ch=0, adv_chan=37;
    std::vector<float> buf;
    uint8_t ref_[SYNC_BITS];   // 기대 동기 비트 (프리앰블+AA)

    // ── 비트 k 중심의 FM판별 소프트값 (oversampled) ──
    inline float samp(size_t j, int k) const {
        long idx = (long)j + (long)llround((k+0.5)*spb);
        if(idx<0 || idx>=(long)buf.size()) return 0.f;
        return buf[idx];
    }

    // 동기 기대 비트열: 프리앰블(AA LSB 따라 0xAA/0x55) + AA 4바이트(byte0 LSB-first)
    void build_ref(){
        uint8_t aa[4]={ (uint8_t)(ADV_AA), (uint8_t)(ADV_AA>>8),
                        (uint8_t)(ADV_AA>>16), (uint8_t)(ADV_AA>>24) };
        uint8_t pre = (aa[0]&1) ? 0x55 : 0xAA;
        for(int b=0;b<8;b++)  ref_[b]      = (pre>>b)&1;        // 프리앰블 LSB-first
        for(int B=0;B<4;B++)
            for(int b=0;b<8;b++) ref_[8+B*8+b] = (aa[B]>>b)&1;  // AA LSB-first
    }

    // AA 40-bit 정규화 상관. 성공 시 pol(극성 ±1) / dc(슬라이스 기준) 반환.
    bool find_sync(size_t j, int& pol, float& dc) const {
        float vals[SYNC_BITS], mean=0;
        for(int k=0;k<SYNC_BITS;k++){ vals[k]=samp(j,k); mean+=vals[k]; }
        mean/=SYNC_BITS;
        float corr=0, energy=0;
        for(int k=0;k<SYNC_BITS;k++){
            float s=vals[k]-mean;
            corr   += s*(ref_[k]?1.f:-1.f);
            energy += std::fabs(s);
        }
        if(energy < 1e-5f) return false;
        float score = corr/energy;          // [-1,+1]
        if(std::fabs(score) < sync_thresh) return false;
        pol = (score>0)?1:-1; dc = mean;
        return true;
    }

    // BLE 화이트닝 LFSR (x^7+x^4+1, Vol6 PartB 3.2). 채널인덱스로 seed.
    struct Whiten {
        uint8_t r[7];
        void init(int chan){
            r[0]=1;
            for(int i=0;i<6;i++) r[1+i] = (uint8_t)((chan>>(5-i))&1); // r[1]=bit5(MSB)
        }
        uint8_t step(){
            uint8_t out=r[6], fb=r[6];
            for(int i=6;i>0;i--) r[i]=r[i-1];
            r[0]=fb; r[4]^=fb;       // 시프트 후 x^4 탭
            return out;
        }
    };

    // CRC-24 (BLE, reversed poly 0xDA6000, LSB-first). 무오류 시 수신 CRC 와 일치.
    static uint32_t ble_crc(const uint8_t* data, int len, uint32_t init){
        uint32_t crc = init & 0xFFFFFF;
        for(int i=0;i<len;i++){
            uint8_t d=data[i];
            for(int b=0;b<8;b++){
                uint8_t in=(d>>b)&1, out=crc&1;
                crc>>=1;
                if(in^out) crc^=0xDA6000;
            }
        }
        return crc & 0xFFFFFF;
    }

    // 동기 j 에서 PDU 해독. 성공 시 소비 샘플수(>0), 길이 비정상이면 0.
    int decode_at(size_t j, int pol, float dc){
        Whiten w; w.init(adv_chan);
        uint8_t raw[44]={0};
        auto get_byte=[&](int bytePos)->uint8_t{
            uint8_t v=0;
            for(int bit=0;bit<8;bit++){
                int k = SYNC_BITS + bytePos*8 + bit;
                uint8_t rawbit = ((samp(j,k)-dc)*pol > 0.f) ? 1 : 0;
                uint8_t out = (uint8_t)(rawbit ^ w.step());   // 디화이트닝
                v |= (uint8_t)(out<<bit);                     // LSB-first
            }
            return v;
        };
        raw[0]=get_byte(0); raw[1]=get_byte(1);
        int pdu_type=raw[0]&0x0F;
        int txadd   =(raw[0]>>6)&1;
        int length  = raw[1]&0x3F;                            // 페이로드 길이
        if(length<6 || length>37) return 0;                   // AdvA 6B 미만/과대 → 잡음
        dg_lenok++;
        int total = 2 + length + 3;
        for(int b=2;b<total;b++) raw[b]=get_byte(b);

        // 광고 CRCInit = 0x555555 (Vol6). reflected(LSB-first) LFSR 라 비트역전값 0xAAAAAA 로 로드.
        // (실캡처 검증: seed/tap/poly 0xDA6000 일치, init 0xAAAAAA 에서만 CRC 통과)
        uint32_t calc = ble_crc(raw, 2+length, 0xAAAAAA);
        uint32_t rx   = (uint32_t)raw[2+length] | ((uint32_t)raw[3+length]<<8)
                      | ((uint32_t)raw[4+length]<<16);
        bool ok = (calc==rx);
        if(on_debug) on_debug(pdu_type, length, raw, calc, rx);
        // CRC 통과 시에만 패킷만큼 소비. 실패면 0 반환 → 가짜 sync 가 근처 진짜
        // 패킷을 건너뛰지 않게(다음 샘플부터 재스캔). 실측: 수율 ↑, 임계 민감도 ↓.
        if(!ok){ dg_fail++; return 0; }
        dg_ok++;
        int consumed = (int)((SYNC_BITS + total*8)*spb);

        BtleRecord m{};
        m.crc_ok=true; m.adv_chan=adv_chan; m.pdu_type=pdu_type; m.addr_type=txadd;
        const uint8_t* pl = raw+2;

        if(pdu_type==0x5){                                    // CONNECT_IND
            m.is_connect=true;
            for(int i=0;i<6;i++){ m.init_mac[i]=pl[5-i]; m.mac[i]=pl[11-i]; }
            const uint8_t* ll = pl+12;                        // LLData 22B
            m.access_addr = (uint32_t)ll[0]|((uint32_t)ll[1]<<8)|((uint32_t)ll[2]<<16)|((uint32_t)ll[3]<<24);
            m.crc_init    = (uint32_t)ll[4]|((uint32_t)ll[5]<<8)|((uint32_t)ll[6]<<16);
            m.interval = (int)(ll[10]|(ll[11]<<8));
            m.latency  = (int)(ll[12]|(ll[13]<<8));
            m.timeout  = (int)(ll[14]|(ll[15]<<8));
            m.chan_map = (uint64_t)ll[16]|((uint64_t)ll[17]<<8)|((uint64_t)ll[18]<<16)
                       | ((uint64_t)ll[19]<<24)|((uint64_t)ll[20]<<32);
            m.hop = ll[21]&0x1F; m.sca=(ll[21]>>5)&0x07;
        } else {
            for(int i=0;i<6;i++) m.mac[i]=pl[5-i];            // AdvA
            // AD 구조 보유: ADV_IND(0)/ADV_NONCONN_IND(2)/SCAN_RSP(4)/ADV_SCAN_IND(6)
            if(pdu_type==0x0||pdu_type==0x2||pdu_type==0x4||pdu_type==0x6)
                parse_ad(pl+6, length-6, m);
        }
        if(on_record) on_record(m);
        return consumed;
    }

    // AD 구조 루프 (len/type/value). 이름/플래그/제조사 요약.
    void parse_ad(const uint8_t* p, int n, BtleRecord& m){
        int i=0, nad=0;
        while(i<n){
            int adlen=p[i]; if(adlen==0) break; if(i+1+adlen>n) break;
            int type=p[i+1]; const uint8_t* val=p+i+2; int vlen=adlen-1;
            nad++;
            switch(type){
                case 0x01: if(vlen>=1) m.flags=val[0]; break;          // Flags
                case 0x08: case 0x09: {                                // Name
                    int c = vlen<(int)sizeof(m.name)-1 ? vlen : (int)sizeof(m.name)-1;
                    memcpy(m.name, val, c); m.name[c]=0;
                } break;
                case 0xFF: if(vlen>=2){                                // Manufacturer
                    m.company = (uint16_t)(val[0]|(val[1]<<8));
                    if(m.company==0x004C && vlen>=4 && val[2]==0x02 && val[3]==0x15)
                        snprintf(m.info,sizeof(m.info),"iBeacon");
                } break;
                default: break;
            }
            i += 1+adlen;
        }
        m.n_ad=nad;
        if(!m.info[0] && m.company!=0xFFFF){
            const char* cn=btle_company_name(m.company);
            if(cn[0]) snprintf(m.info,sizeof(m.info),"%s",cn);
            else      snprintf(m.info,sizeof(m.info),"0x%04X",m.company);
        }
    }
};
