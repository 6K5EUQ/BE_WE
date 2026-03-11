#include "fft_viewer.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <chrono>

// ── AIS 복조 상수 ─────────────────────────────────────────────────────────
static constexpr int   AIS_BAUD     = 9600;
static constexpr int   AIS_OVER     = 8;          // 오버샘플링 배수
static constexpr int   AIS_WORK_SR  = AIS_BAUD * AIS_OVER; // 76800 Hz
static constexpr int   AIS_MIN_BITS = 56;          // 최소 프레임 (FCS 제외)

// ── CRC-CCITT (ITU-T, 0x1021 reversed = 0x8408) ──────────────────────────
static uint16_t crc_ccitt(const uint8_t* data, int len){
    uint16_t crc = 0xFFFF;
    for(int i=0;i<len;i++){
        uint8_t b = data[i];
        for(int j=0;j<8;j++){
            if((crc ^ b) & 1) crc = (crc >> 1) ^ 0x8408u;
            else               crc >>= 1;
            b >>= 1;
        }
    }
    return crc ^ 0xFFFF;
}

// ── AIS 비트스트림 → n비트 값 추출 (MSB first) ───────────────────────────
static uint32_t get_u(const uint8_t* b, int off, int n){
    uint32_t v=0;
    for(int i=0;i<n;i++) v=(v<<1)|(b[off+i]&1);
    return v;
}
static int32_t get_i(const uint8_t* b, int off, int n){
    uint32_t u=get_u(b,off,n);
    if(u>>(n-1)) u|=(0xFFFFFFFFu<<n);
    return (int32_t)u;
}
static char ais6(uint8_t v){
    v&=0x3F;
    return (char)(v<40 ? v+48 : v+56);
}

// ── AIS 프레임 디코드 + 터미널 출력 ──────────────────────────────────────
// bits[]: HDLC 비트스터핑 제거 후 MSB-first NRZ 비트 배열
// n    : 비트 수 (FCS 16비트 포함)
static void ais_decode_frame(const uint8_t* bits, int n, int ch_idx){
    if(n < AIS_MIN_BITS + 16) return;

    int payload_bits = n - 16;  // FCS 제외

    // ── FCS 검증 ────────────────────────────────────────────────────────
    // 비트 → 바이트 (LSB first, HDLC 표준)
    int nb = (payload_bits + 7) / 8;
    uint8_t bytes[64]={};
    for(int i=0;i<payload_bits;i++)
        bytes[i/8] |= (bits[i] << (i%8));
    // 수신 FCS (LSB first)
    uint16_t rx_fcs=0;
    for(int i=0;i<16;i++)
        rx_fcs |= ((uint16_t)bits[payload_bits+i] << i);
    uint16_t calc_fcs = crc_ccitt(bytes, nb);
    if(calc_fcs != rx_fcs) return;  // FCS 불일치 → 버림

    // ── AIS 비트스트림: HDLC 바이트를 MSB-first로 재정렬 ───────────────
    // AIS payload는 HDLC 바이트 기준 MSB-first로 인코딩됨
    uint8_t msb[64]={};
    for(int i=0;i<payload_bits;i++){
        int byte_idx = i/8;
        int bit_in_byte = i%8;
        // HDLC는 LSB first → byte를 MSB first로 반전
        uint8_t lsb_byte = bytes[byte_idx];
        uint8_t msb_byte = 0;
        for(int j=0;j<8;j++) msb_byte |= ((lsb_byte>>j)&1)<<(7-j);
        msb[i] = (msb_byte >> (7 - bit_in_byte)) & 1;
    }

    // ── NMEA payload 인코딩 ──────────────────────────────────────────────
    int n_chars = (payload_bits + 5) / 6;
    char payload[64]={};
    for(int i=0;i<n_chars && i<63;i++){
        uint8_t v=0;
        for(int j=0;j<6;j++){
            int bi=i*6+j;
            if(bi<payload_bits) v=(v<<1)|(msb[bi]&1);
            else v<<=1;
        }
        v &= 0x3F;
        payload[i]=(char)(v<40 ? v+48 : v+56);
    }
    int fill = (6-(payload_bits%6))%6;

    // NMEA 문장 체크섬
    char nmea[160];
    snprintf(nmea,sizeof(nmea),"AIVDM,1,1,,A,%s,%d",payload,fill);
    uint8_t cs=0; for(int i=0;nmea[i];i++) cs^=(uint8_t)nmea[i];
    printf("[AIS ch%d] !%s*%02X\n", ch_idx, nmea, cs);
    fflush(stdout);

    // ── 메시지 파싱 ─────────────────────────────────────────────────────
    uint8_t  mtype = (uint8_t)get_u(msb,0,6);
    uint32_t mmsi  = get_u(msb,8,30);

    if((mtype>=1&&mtype<=3) && payload_bits>=168){
        float lon = get_i(msb,61,28)/600000.0f;
        float lat = get_i(msb,89,27)/600000.0f;
        float sog = get_u(msb,50,10)/10.0f;
        float cog = get_u(msb,116,12)/10.0f;
        printf("  Type%u  MMSI=%-9u  lat=%9.5f  lon=%10.5f  SOG=%5.1fkn  COG=%6.1f\xc2\xb0\n",
               mtype,mmsi,lat,lon,sog,cog);
    } else if(mtype==18 && payload_bits>=168){
        float lon = get_i(msb,57,28)/600000.0f;
        float lat = get_i(msb,85,27)/600000.0f;
        float sog = get_u(msb,46,10)/10.0f;
        printf("  Type18 MMSI=%-9u  lat=%9.5f  lon=%10.5f  SOG=%5.1fkn  (ClassB)\n",
               mmsi,lat,lon,sog);
    } else if(mtype==5 && payload_bits>=426){
        char name[21]={},dest[21]={};
        for(int i=0;i<20;i++) name[i]=ais6((uint8_t)get_u(msb,112+i*6,6));
        for(int i=0;i<20;i++) dest[i]=ais6((uint8_t)get_u(msb,302+i*6,6));
        for(int i=19;i>=0&&name[i]==' ';i--) name[i]=0;
        for(int i=19;i>=0&&dest[i]==' ';i--) dest[i]=0;
        printf("  Type5  MMSI=%-9u  NAME=%-20s  DEST=%s\n",mmsi,name,dest);
    } else {
        printf("  Type%u  MMSI=%-9u\n",mtype,mmsi);
    }
    fflush(stdout);
}

// ── HDLC 비트스트림 디코더 ────────────────────────────────────────────────
// AIS/HDLC 규격:
//   FLAG: 0x7E = 01111110 (LSB first: 0,1,1,1,1,1,1,0)
//   비트스터핑: 연속 5개 1 → 뒤에 0 삽입 (수신 시 제거)
//   NRZI: 0=천이, 1=유지 (수신 시 NRZ로 변환 완료된 비트를 입력)
struct HdlcDecoder {
    uint8_t  sreg      = 0;    // 최근 8비트 shift register (LSB-first 입력)
    int      ones      = 0;    // 연속 1 카운트 (비트스터핑 검출)
    bool     in_frame  = false;
    bool     skip_next = false; // 비트스터핑 제거용: 다음 0 버림
    // 비트 버퍼: 최대 512비트 (AIS 최대 프레임 < 1024비트)
    uint8_t  buf[1024] = {};
    int      nbits     = 0;

    void reset(){ in_frame=false; nbits=0; ones=0; skip_next=false; }

    // NRZ 비트 입력 (NRZI 디코딩 완료 후)
    void push(uint8_t bit, int ch_idx){
        // shift register에 비트 누적 (LSB first)
        sreg = (sreg >> 1) | (bit << 7);

        // FLAG 탐지: 01111110
        // LSB-first로 입력 시 0x7E = 0b01111110
        if(sreg == 0x7E){
            if(in_frame && nbits >= AIS_MIN_BITS + 16)
                ais_decode_frame(buf, nbits, ch_idx);
            in_frame  = true;
            nbits     = 0;
            ones      = 0;
            skip_next = false;
            return;
        }

        if(!in_frame) return;

        // 비트스터핑: 연속 5개 1 뒤에 오는 0은 버림
        if(skip_next){
            skip_next = false;
            if(bit != 0){
                // 5개 1 뒤에 1이 오면 → ABORT (invalid frame)
                reset();
            }
            return;
        }

        if(bit == 1){
            ones++;
            if(ones == 5) skip_next = true; // 다음 비트(0) 버릴 준비
        } else {
            ones = 0;
        }

        if(nbits < (int)sizeof(buf)*8)
            buf[nbits++] = bit;
        else
            reset(); // 버퍼 초과 → 리셋
    }
};

// ── AIS 복조 워커 ─────────────────────────────────────────────────────────
void FFTViewer::ais_worker(int ch_idx){
    Channel& ch = channels[ch_idx];
    const uint32_t msr = header.sample_rate;

    // 채널 중심 주파수 오프셋
    float ch_cf_hz = ((ch.s + ch.e) * 0.5f) * 1e6f;
    float off_hz   = ch_cf_hz - (float)header.center_frequency;

    // decimation 계수: msr → AIS_WORK_SR (76800 Hz)
    uint32_t decim = msr / AIS_WORK_SR;
    if(decim < 1) decim = 1;
    uint32_t work_sr = msr / decim;
    float    sps     = (float)work_sr / AIS_BAUD;  // 샘플/심볼 (약 8.0)

    printf("[AIS ch%d] start  cf=%.4f MHz  off=%.0f Hz  decim=%u  work_sr=%u  sps=%.2f\n",
           ch_idx, ch_cf_hz*1e-6f, off_hz, decim, work_sr, sps);
    fflush(stdout);

    // ── 믹서 ────────────────────────────────────────────────────────────
    Oscillator osc;
    osc.set_freq(off_hz, (double)msr);

    // ── 저역통과 IIR (4.8 kHz cut, 정규화 기준 work_sr) ────────────────
    // GMSK BT=0.4, bandwidth ≈ AIS_BAUD*0.5 Hz
    IIR1 lpI, lpQ;
    lpI.set((double)AIS_BAUD * 0.5 / (double)work_sr);
    lpQ.set((double)AIS_BAUD * 0.5 / (double)work_sr);

    // ── FM discriminator 상태 ───────────────────────────────────────────
    float prev_i=1.f, prev_q=0.f;

    // ── 타이밍 복원 (Zero-crossing 기반 Mueller-Müller 단순화) ─────────
    // sps=8이므로 정수 카운터로 심볼 경계 추적
    float   sym_phase = sps * 0.5f;  // 첫 심볼 중앙부터 시작
    float   mu        = 0.f;         // 소수점 위상 보정 (±0.5 sps)
    float   prev_sym  = 0.f;         // 이전 심볼 값 (타이밍 루프)
    float   prev_d    = 0.f;         // 이전 discriminator 출력
    // 샘플 버퍼 (타이밍 보정용 1심볼 지연)
    static constexpr int SBuf = 16;
    float   sbuf[SBuf]={};
    int     sbuf_idx=0;

    // ── NRZI 상태 ────────────────────────────────────────────────────────
    uint8_t nrzi_prev = 0;

    // ── HDLC 디코더 ──────────────────────────────────────────────────────
    HdlcDecoder hdlc;

    // ── decimation 누적 ──────────────────────────────────────────────────
    uint32_t di=0;
    float    acc_i=0.f, acc_q=0.f;
    float    sample_cnt=0.f;  // work_sr 기준 샘플 카운터

    while(!ch.digi_stop_req.load(std::memory_order_relaxed)){
        size_t rp    = ch.digi_rp.load(std::memory_order_relaxed);
        size_t wp    = ring_wp.load(std::memory_order_acquire);
        size_t avail = (wp - rp) & IQ_RING_MASK;

        // 최소 1심볼 분량 대기
        if(avail < decim * 2){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // 1회 처리 한도: 약 100 심볼 분량
        size_t maxp = (size_t)decim * 800;
        if(avail > maxp) avail = maxp;

        for(size_t i=0; i<avail; i++){
            size_t idx = (rp+i) & IQ_RING_MASK;
            // BladeRF SC16Q11: 범위 [-2048, 2048)
            float si = ring[idx*2]   * (1.f/2048.f);
            float sq = ring[idx*2+1] * (1.f/2048.f);

            // 믹서: 채널 중심으로 내리기
            float mi, mq;
            osc.mix(si, sq, mi, mq);

            // decimation (FIR 대신 단순 평균)
            acc_i += mi; acc_q += mq;
            if(++di < decim) continue;
            di=0;

            // ── work_sr 기준 샘플 처리 ──────────────────────────────────
            float ci = lpI.p(acc_i*(1.f/decim));
            float cq = lpQ.p(acc_q*(1.f/decim));
            acc_i=acc_q=0.f;

            // FM discriminator
            float cross = ci*prev_q - cq*prev_i;
            float dot   = ci*prev_i + cq*prev_q;
            float d     = atan2f(cross, dot+1e-12f) * (1.f/(float)M_PI);
            prev_i=ci; prev_q=cq;

            // 샘플 링버퍼에 저장
            sbuf[sbuf_idx & (SBuf-1)] = d;
            sbuf_idx++;

            // ── 심볼 타이밍: sps 카운터 ─────────────────────────────────
            sample_cnt += 1.f;
            if(sample_cnt >= sps + mu){
                sample_cnt -= (sps + mu);

                // 현재 심볼 중앙 샘플 = 현재 d
                float cur = d;

                // Mueller-Müller 타이밍 오차
                float err = prev_sym * (cur > 0.f ? 1.f : -1.f)
                          - cur      * (prev_sym > 0.f ? 1.f : -1.f);
                mu += 0.002f * err;
                if(mu >  0.5f) mu =  0.5f;
                if(mu < -0.5f) mu = -0.5f;
                prev_sym = cur;

                // 비트 결정
                uint8_t rf_bit = (cur > 0.f) ? 1 : 0;

                // NRZI 디코딩: 0=천이→비트1, 1=유지→비트0  (AIS NRZI 정의)
                // AIS: mark=+deviation=1, space=-deviation=0
                // NRZI: 이전과 같으면 0, 다르면 1
                uint8_t nrz = (rf_bit != nrzi_prev) ? 1 : 0;
                // 실제 AIS NRZI: 천이=0, 유지=1 이지만 역전 가능성 있음
                // 표준 ITU-R M.1371: 0→천이, 1→유지
                // 즉: nrz = (rf_bit == nrzi_prev) ? 1 : 0
                nrz = (rf_bit == nrzi_prev) ? 1 : 0;
                nrzi_prev = rf_bit;

                hdlc.push(nrz, ch_idx);
            }
        }

        ch.digi_rp.store((rp+avail) & IQ_RING_MASK, std::memory_order_release);
    }

    printf("[AIS ch%d] worker exited\n", ch_idx);
    fflush(stdout);
}

void FFTViewer::start_digi(int ch_idx, Channel::DigitalMode mode){
    Channel& ch = channels[ch_idx];
    if(ch.digi_run.load() || !ch.filter_active) return;
    ch.digital_mode = mode;
    ch.digi_rp.store(ring_wp.load());
    ch.digi_stop_req.store(false);
    ch.digi_run.store(true);
    if(mode == Channel::DIGI_AIS)
        ch.digi_thr = std::thread(&FFTViewer::ais_worker, this, ch_idx);
    printf("[DIGI ch%d] start mode=%d\n", ch_idx, (int)mode);
    fflush(stdout);
}

void FFTViewer::stop_digi(int ch_idx){
    Channel& ch = channels[ch_idx];
    if(!ch.digi_run.load()) return;
    ch.digi_stop_req.store(true);
    if(ch.digi_thr.joinable()) ch.digi_thr.join();
    ch.digi_run.store(false);
    ch.digital_mode = Channel::DIGI_NONE;
    printf("[DIGI ch%d] stopped\n", ch_idx);
    fflush(stdout);
}
