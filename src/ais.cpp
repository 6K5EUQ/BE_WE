#include "fft_viewer.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <chrono>

static constexpr int AIS_BAUD    = 9600;
static constexpr int AIS_OVER    = 8;
static constexpr int AIS_WORK_SR = AIS_BAUD * AIS_OVER; // 76800

// ── CRC-CCITT (ITU-R M.1371, poly=0x1021 bit-reversed=0x8408) ─────────────
// AIS FCS: CRC-CCITT applied to bytes between FLAGS (excl. stuffed bits)
// Input bytes are LSB-first (as transmitted)
static uint16_t crc_ccitt(const uint8_t* d, int len){
    uint16_t crc=0xFFFF;
    for(int i=0;i<len;i++){
        uint8_t b=d[i];
        for(int j=0;j<8;j++){ crc=((crc^b)&1)?(crc>>1)^0x8408u:(crc>>1); b>>=1; }
    }
    return crc^0xFFFF;
}

// ── AIS 비트(MSB-first per field) → 값 ────────────────────────────────────
static uint32_t bu(const uint8_t* b,int o,int n){
    uint32_t v=0; for(int i=0;i<n;i++) v=(v<<1)|(b[o+i]&1); return v;
}
static int32_t bi(const uint8_t* b,int o,int n){
    uint32_t u=bu(b,o,n); if(u>>(n-1)) u|=(0xFFFFFFFFu<<n); return(int32_t)u;
}
static char a6(uint8_t v){ v&=0x3F; return(char)(v<40?v+48:v+56); }

// ── NMEA + 파싱 출력 ──────────────────────────────────────────────────────
// msb[]: AIS 비트스트림 — 각 필드의 비트가 MSB-first 순서로 나열됨
// (즉 첫 번째 전송 비트가 msb[0], 그 다음이 msb[1], ...)
static void print_frame(const uint8_t* msb, int n, int ch_idx){
    int nc=(n+5)/6;
    char pay[64]={};
    for(int i=0;i<nc&&i<63;i++){
        uint8_t v=0;
        for(int j=0;j<6;j++){ int b=i*6+j; v=(v<<1)|(b<n?msb[b]&1:0); }
        v&=0x3F; pay[i]=(char)(v<40?v+48:v+56);
    }
    int fill=(6-(n%6))%6;
    char nmea[200]; snprintf(nmea,sizeof(nmea),"AIVDM,1,1,,A,%s,%d",pay,fill);
    uint8_t cs=0; for(int i=0;nmea[i];i++) cs^=(uint8_t)nmea[i];
    printf("[AIS ch%d] !%s*%02X\n",ch_idx,nmea,cs);
    uint8_t mt=(uint8_t)bu(msb,0,6); uint32_t mm=bu(msb,8,30);
    if((mt>=1&&mt<=3)&&n>=168){
        printf("  Type%u MMSI=%u lat=%.5f lon=%.5f SOG=%.1f COG=%.1f\n",
               mt,mm,bi(msb,89,27)/600000.f,bi(msb,61,28)/600000.f,
               bu(msb,50,10)/10.f,bu(msb,116,12)/10.f);
    } else if(mt==18&&n>=168){
        printf("  Type18 MMSI=%u lat=%.5f lon=%.5f SOG=%.1f\n",
               mm,bi(msb,85,27)/600000.f,bi(msb,57,28)/600000.f,bu(msb,46,10)/10.f);
    } else if(mt==5&&n>=426){
        char nm[21]={},ds[21]={};
        for(int i=0;i<20;i++) nm[i]=a6((uint8_t)bu(msb,112+i*6,6));
        for(int i=0;i<20;i++) ds[i]=a6((uint8_t)bu(msb,302+i*6,6));
        for(int i=19;i>=0&&nm[i]==' ';i--) nm[i]=0;
        for(int i=19;i>=0&&ds[i]==' ';i--) ds[i]=0;
        printf("  Type5 MMSI=%u NAME='%s' DEST='%s'\n",mm,nm,ds);
    } else { printf("  Type%u MMSI=%u bits=%d\n",mt,mm,n); }
    fflush(stdout);
}

// ── HDLC 디코더 ───────────────────────────────────────────────────────────
// AIS/HDLC: 비트 전송 순서 LSB-first
// FLAG = 0x7E = 0b01111110, 전송 순서(LSB-first): 0,1,1,1,1,1,1,0
//
// shift register 방식: 새 비트를 MSB에 삽입 (sreg = (sreg>>1)|(bit<<7))
// FLAG가 들어오면: 첫 비트(0)이 MSB, 마지막(0)이 LSB → 0b01111110 = 0x7E ✓
//
// 비트스터핑: 연속 5개 1 뒤에 0이 삽입됨 → 수신 시 5개 1 뒤 0은 버림
// raw[]: 비트스터핑 제거 후의 비트들 (LSB-first per byte)
// AIS 비트스트림: raw[i]가 i번째 전송 비트 = 해당 바이트의 bit(i%8) (LSB-first)
//
// AIS 필드 파싱은 바이트 내 MSB-first로 읽음:
// 즉 raw[0]이 첫 번째 바이트의 LSB, raw[7]이 MSB
// → 필드 파싱용 비트 배열: bit_n = raw[byte*8 + (7 - bit_in_byte)]
struct Hdlc {
    uint8_t sreg=0;
    int     ones=0;
    bool    inframe=false;
    bool    skip=false;
    uint8_t raw[800]={};   // 비트스터핑 제거 후 비트 (LSB-first per byte)
    int     nb=0;
    int     flag_cnt=0;    // 진단용

    void reset(){ inframe=false; nb=0; ones=0; skip=false; }

    void push(uint8_t bit, int ch_idx){
        sreg=(uint8_t)((sreg>>1)|(bit<<7));

        if(sreg==0x7E){  // FLAG 탐지
            flag_cnt++;
            if(inframe && nb>=(56+16)){
                try_decode(ch_idx);
            }
            inframe=true; nb=0; ones=0; skip=false;
            return;
        }

        if(!inframe) return;

        // 비트스터핑 처리
        if(skip){
            skip=false;
            if(bit!=0){ reset(); return; }  // ABORT
            return;  // stuffed 0 버림
        }
        if(bit==1){ if(++ones==5) skip=true; }
        else ones=0;

        if(nb<(int)sizeof(raw)*8) raw[nb++]=bit;
        else reset();
    }

    int decode_cnt=0, fcs_ok=0, fcs_fail=0;

    void try_decode(int ch_idx){
        // nb = 페이로드 비트 + FCS 16비트
        int plen = nb - 16;
        if(plen < 56) return;

        int nbytes = (plen + 7) / 8;
        if(nbytes > 64) return;

        decode_cnt++;

        // raw[] (LSB-first) → 바이트 배열
        uint8_t bytes[64]={};
        for(int i=0;i<nbytes*8&&i<nb;i++)
            bytes[i/8] |= (raw[i] << (i%8));

        // FCS: plen부터 16비트 (LSB-first)
        uint16_t rxfcs=0;
        for(int i=0;i<16;i++) rxfcs |= ((uint16_t)raw[plen+i] << i);

        // CRC 계산: 페이로드 바이트에 대해
        int fcs_bytes = (plen + 7) / 8;
        uint16_t calcfcs = crc_ccitt(bytes, fcs_bytes);

        if(calcfcs != rxfcs){ fcs_fail++; return; }  // FCS 불일치
        fcs_ok++;

        // raw[] (LSB-first per byte) → AIS 비트스트림 (MSB-first per field/byte)
        // AIS에서 첫 번째 전송 비트 = 첫 바이트의 LSB = raw[0]
        // 하지만 필드 파싱은 MSB-first: 첫 필드 비트가 해당 바이트의 MSB 쪽
        // 표준: AIS 비트 0 = 첫 번째 전송 비트 = 첫 바이트 bit0 (LSB)
        // 그러므로 ais[i] = raw[i] (비트스트림 그대로)
        // print_frame의 bu()는 인덱스 순서로 MSB-first 읽기를 함
        // → raw[] 비트를 바이트 단위로 비트 역순 변환해야 함
        uint8_t ais[512]={};
        for(int i=0;i<plen;i++){
            int byte_i = i / 8;
            int bit_i  = i % 8;
            // raw[i]는 byte_i의 bit_i (LSB=0)
            // ais에서 같은 바이트의 MSB-first 위치: byte_i*8 + (7-bit_i)
            ais[byte_i*8 + (7-bit_i)] = raw[i];
        }

        print_frame(ais, plen, ch_idx);
    }
};

// ── AIS 워커 ──────────────────────────────────────────────────────────────
void FFTViewer::ais_worker(int ch_idx){
    Channel& ch=channels[ch_idx];
    uint32_t msr=header.sample_rate;

    float off_hz=(((ch.s+ch.e)/2.0f)-(float)(header.center_frequency/1e6f))*1e6f;
    float bw_hz=fabsf(ch.e-ch.s)*1e6f;
    if(bw_hz<20000.f) bw_hz=20000.f;

    // 1단계: off_hz와 BW를 모두 커버하는 중간 SR 계산
    // inter_sr은 off_hz*2 와 bw_hz*3 중 큰 값 이상이어야 함
    float need_sr = std::max(fabsf(off_hz) * 2.2f, bw_hz * 3.f);
    if(need_sr < (float)AIS_WORK_SR) need_sr = (float)AIS_WORK_SR;
    uint32_t inter_sr = AIS_WORK_SR;
    while((float)inter_sr < need_sr && inter_sr < msr) inter_sr *= 2;
    uint32_t cap_decim = msr / inter_sr;
    if(cap_decim < 1) cap_decim = 1;
    uint32_t actual_inter = msr / cap_decim;

    // 2단계: inter_sr → AIS_WORK_SR decimation
    uint32_t fine_decim = actual_inter / AIS_WORK_SR;
    if(fine_decim < 1) fine_decim = 1;
    uint32_t work_sr = actual_inter / fine_decim;
    float    sps = (float)work_sr / (float)AIS_BAUD;

    const size_t MAX_LAG=(size_t)(msr*0.08);
    const size_t BATCH  =(size_t)cap_decim*AIS_BAUD/10;

    printf("[AIS ch%d] start cf=%.4fMHz off=%.0fHz decim=%u inter=%u fine=%u work_sr=%u sps=%.2f\n",
           ch_idx,(ch.s+ch.e)/2.f,off_hz,cap_decim,actual_inter,fine_decim,work_sr,sps);
    fflush(stdout);

    Oscillator osc; osc.set_freq((double)off_hz,(double)msr);

    // LPF를 각 샘플에 적용 후 서브샘플링 (decimation-after-LPF)
    // cutoff = (AIS BW/2) / msr, AIS BW ≈ 16kHz → cutoff = 8000/msr
    IIR1 lpi,lpq;
    { double cn=8000.0/(double)msr; if(cn>0.45)cn=0.45; lpi.set(cn); lpq.set(cn); }
    int cap_cnt=0;

    // 2단계: work_sr로 추가 decimation + narrow LPF
    IIR1 lpi2,lpq2;
    { double cn=9600.0*1.2/(double)work_sr; if(cn>0.45)cn=0.45; lpi2.set(cn); lpq2.set(cn); }
    int fine_cnt=0;

    float prev_i=0,prev_q=0;

    // 타이밍 복원 (M&M)
    float sym_phase=0.f, mu=0.f, p_sym=0.f;

    // NRZI dual decoder: d=0 정방향, d=1 반전
    // AIS NRZI: 천이=1(mark), 유지=0(space) → FM: mark=높은주파수, space=낮은주파수
    // RF bit: disc>0 → mark=1, disc<0 → space=0
    // NRZI decode: bit 천이(현재≠이전) → data bit 1
    uint8_t nrzi_prev[2]={0,0};
    Hdlc hdlc[2];

    // 진단 카운터
    uint64_t sym_count=0;
    float disc_min=1e9f, disc_max=-1e9f;
    float iq_max=0.f;
    int diag_interval=work_sr/2;  // 0.5초마다 진단 출력
    int diag_cnt=0;

    while(!ch.digi_stop_req.load(std::memory_order_relaxed)){
        size_t wp=ring_wp.load(std::memory_order_acquire);
        size_t rp=ch.digi_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;

        if(lag>MAX_LAG){
            rp=(wp-(size_t)(msr*0.02f))&IQ_RING_MASK;
            ch.digi_rp.store(rp,std::memory_order_release);
            cap_cnt=0; fine_cnt=0;
            lpi.s=lpq.s=0; lpi2.s=lpq2.s=0; prev_i=prev_q=0;
            sym_phase=mu=p_sym=0.f;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(500)); continue; }

        size_t avail=std::min(lag,BATCH);

        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/2048.f;
            float sq=ring[pos*2+1]/2048.f;

            float mi,mq; osc.mix(si,sq,mi,mq);

            // 1단계: 각 샘플마다 LPF 적용 → cap_decim마다 서브샘플링
            float fi=lpi.p(mi); float fq=lpq.p(mq);
            if(++cap_cnt<(int)cap_decim) continue;
            cap_cnt=0;

            // 2단계: work_sr로 추가 decimation + narrow LPF
            if(++fine_cnt<(int)fine_decim) continue;
            fine_cnt=0;
            fi=lpi2.p(fi); fq=lpq2.p(fq);

            // FM discriminator: atan2 (진폭 독립적, SNR 낮아도 부호 정확)
            float iq_amp=fi*fi+fq*fq;
            if(iq_amp>iq_max) iq_max=iq_amp;
            float cross=fi*prev_q-fq*prev_i;
            float dot  =fi*prev_i+fq*prev_q;
            float disc=atan2f(cross, dot+1e-30f);
            prev_i=fi; prev_q=fq;

            // 진단
            if(disc<disc_min) disc_min=disc;
            if(disc>disc_max) disc_max=disc;
            diag_cnt++;
            if(diag_cnt>=diag_interval){
                printf("[AIS ch%d] diag: syms=%llu disc=[%.6f,%.6f] iq_max=%.6f "
                       "flags0=%d dec0=%d ok0=%d fail0=%d | flags1=%d dec1=%d ok1=%d fail1=%d\n",
                       ch_idx,(unsigned long long)sym_count,
                       disc_min,disc_max,iq_max,
                       hdlc[0].flag_cnt,hdlc[0].decode_cnt,hdlc[0].fcs_ok,hdlc[0].fcs_fail,
                       hdlc[1].flag_cnt,hdlc[1].decode_cnt,hdlc[1].fcs_ok,hdlc[1].fcs_fail);
                fflush(stdout);
                disc_min=1e9f; disc_max=-1e9f; iq_max=0.f;
                diag_cnt=0;
            }

            // M&M 타이밍
            sym_phase+=1.f;
            if(sym_phase>=(sps+mu)){
                sym_phase-=(sps+mu);
                float cur=disc;
                sym_count++;

                float ec=(cur>0.f?1.f:-1.f)*p_sym-(p_sym>0.f?1.f:-1.f)*cur;
                mu+=0.005f*ec;
                if(mu>1.f) mu=1.f; if(mu<-1.f) mu=-1.f;
                p_sym=cur;

                // RF bit 결정: disc>0 → mark(1), disc<0 → space(0)
                uint8_t rf=(cur>0.f)?1:0;

                // NRZI 디코딩: 천이(현재≠이전) → data=1, 유지(현재==이전) → data=0
                for(int d=0;d<2;d++){
                    uint8_t rf_d=(d==0)?rf:(rf^1);
                    uint8_t nrz=(rf_d!=nrzi_prev[d])?1:0;  // 천이=1 (AIS NRZI)
                    nrzi_prev[d]=rf_d;
                    hdlc[d].push(nrz,ch_idx);
                }
            }
        }
        ch.digi_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    printf("[AIS ch%d] stopped syms=%llu flags0=%d flags1=%d\n",
           ch_idx,(unsigned long long)sym_count,hdlc[0].flag_cnt,hdlc[1].flag_cnt);
    fflush(stdout);
}

void FFTViewer::start_digi(int ch_idx, Channel::DigitalMode mode){
    Channel& ch=channels[ch_idx];
    if(ch.digi_run.load()||!ch.filter_active) return;
    ch.digital_mode=mode;
    ch.digi_rp.store(ring_wp.load());
    ch.digi_stop_req.store(false);
    ch.digi_run.store(true);
    if(mode==Channel::DIGI_AIS)
        ch.digi_thr=std::thread(&FFTViewer::ais_worker,this,ch_idx);
    printf("[DIGI ch%d] start mode=%d\n",ch_idx,(int)mode); fflush(stdout);
}

void FFTViewer::stop_digi(int ch_idx){
    Channel& ch=channels[ch_idx];
    if(!ch.digi_run.load()) return;
    ch.digi_stop_req.store(true);
    if(ch.digi_thr.joinable()) ch.digi_thr.join();
    ch.digi_run.store(false);
    ch.digital_mode=Channel::DIGI_NONE;
    printf("[DIGI ch%d] stopped\n",ch_idx); fflush(stdout);
}
