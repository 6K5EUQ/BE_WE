#include "fft_viewer.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <chrono>

static constexpr int AIS_BAUD = 9600;

// ── CRC-CCITT ──────────────────────────────────────────────────────────────
static uint16_t crc_ccitt(const uint8_t* d, int len){
    uint16_t crc=0xFFFF;
    for(int i=0;i<len;i++){
        uint8_t b=d[i];
        for(int j=0;j<8;j++){ crc=((crc^b)&1)?(crc>>1)^0x8408u:(crc>>1); b>>=1; }
    }
    return crc^0xFFFF;
}

static uint32_t bu(const uint8_t* b,int o,int n){
    uint32_t v=0; for(int i=0;i<n;i++) v=(v<<1)|(b[o+i]&1); return v;
}
static int32_t bi(const uint8_t* b,int o,int n){
    uint32_t u=bu(b,o,n); if(u>>(n-1)) u|=(0xFFFFFFFFu<<n); return(int32_t)u;
}
static char a6(uint8_t v){ v&=0x3F; return(char)(v<40?v+48:v+56); }

static void print_frame(const uint8_t* bits, int nbits, int ch_idx){
    int nc=(nbits+5)/6; char pay[64]={};
    for(int i=0;i<nc&&i<63;i++){
        uint8_t v=0;
        for(int j=0;j<6;j++){ int b=i*6+j; v=(v<<1)|(b<nbits?bits[b]&1:0); }
        v&=0x3F; pay[i]=(char)(v<40?v+48:v+56);
    }
    int fill=(6-(nbits%6))%6;
    char nmea[200]; snprintf(nmea,sizeof(nmea),"AIVDM,1,1,,A,%s,%d",pay,fill);
    uint8_t cs=0; for(int i=0;nmea[i];i++) cs^=(uint8_t)nmea[i];
    printf("[AIS ch%d] !%s*%02X\n",ch_idx,nmea,cs);
    uint8_t mt=(uint8_t)bu(bits,0,6); uint32_t mm=bu(bits,8,30);
    if((mt>=1&&mt<=3)&&nbits>=168)
        printf("  Type%u MMSI=%u lat=%.5f lon=%.5f SOG=%.1f\n",
               mt,mm,bi(bits,89,27)/600000.f,bi(bits,61,28)/600000.f,bu(bits,50,10)/10.f);
    else if(mt==18&&nbits>=168)
        printf("  Type18 MMSI=%u lat=%.5f lon=%.5f\n",
               mm,bi(bits,85,27)/600000.f,bi(bits,57,28)/600000.f);
    else if(mt==5&&nbits>=426){
        char nm[21]={}; for(int i=0;i<20;i++) nm[i]=a6((uint8_t)bu(bits,112+i*6,6));
        for(int i=19;i>=0&&nm[i]==' ';i--) nm[i]=0;
        printf("  Type5 MMSI=%u NAME='%s'\n",mm,nm);
    } else printf("  Type%u MMSI=%u bits=%d\n",mt,mm,nbits);
    fflush(stdout);
}

struct Hdlc {
    uint8_t sreg=0; int ones=0; bool inframe=false,skip=false;
    uint8_t raw[512]={}; int nb=0;
    int flag_cnt=0,fcs_ok=0;
    void reset(){ inframe=false; nb=0; ones=0; skip=false; sreg=0; }
    void push(uint8_t bit, int ch_idx){
        sreg=(uint8_t)((sreg>>1)|(bit<<7));
        if(sreg==0x7E){
            flag_cnt++;
            if(inframe&&nb>=(56+16)) try_decode(ch_idx);
            inframe=true; nb=0; ones=0; skip=false; return;
        }
        if(!inframe) return;
        if(skip){ skip=false; if(bit!=0){reset();return;} return; }
        if(bit==1){ if(++ones==5) skip=true; } else ones=0;
        if(nb<(int)sizeof(raw)*8) raw[nb++]=bit; else reset();
    }
    void try_decode(int ch_idx){
        int plen=nb-16;
        if(plen<56){
            printf("[AIS ch%d] HDLC too_short nb=%d plen=%d\n",ch_idx,nb,plen);
            fflush(stdout); return;
        }
        int nbytes=(plen+7)/8;
        if(nbytes>64){
            printf("[AIS ch%d] HDLC too_long nbytes=%d\n",ch_idx,nbytes);
            fflush(stdout); return;
        }
        uint8_t bytes[64]={};
        for(int i=0;i<nbytes*8&&i<nb;i++) bytes[i/8]|=(raw[i]<<(i%8));
        uint16_t rxfcs=0;
        for(int i=0;i<16;i++) rxfcs|=((uint16_t)raw[plen+i]<<i);
        uint16_t calcfcs=crc_ccitt(bytes,(plen+7)/8);
        if(calcfcs!=rxfcs){
            printf("[AIS ch%d] HDLC crc_fail nb=%d calc=0x%04X rx=0x%04X\n",ch_idx,nb,calcfcs,rxfcs);
            fflush(stdout); return;
        }
        fcs_ok++;
        uint8_t bits[512]={};
        for(int i=0;i<plen;i++) bits[(i/8)*8+(7-(i%8))]=raw[i];
        print_frame(bits,plen,ch_idx);
    }
};

// ── AIS 워커 ──────────────────────────────────────────────────────────────
void FFTViewer::ais_worker(int ch_idx){
    Channel& ch=channels[ch_idx];
    uint32_t msr=header.sample_rate;
    float off_hz=(((ch.s+ch.e)/2.0f)-(float)(header.center_frequency/1e6f))*1e6f;

    // work_sr: AIS_BAUD * sps_target (sps=8)
    // decimation을 한 단계로 단순화: msr → work_sr
    uint32_t total_decim = msr / (AIS_BAUD * 8);
    if(total_decim < 1) total_decim = 1;
    uint32_t work_sr = msr / total_decim;
    float sps = (float)work_sr / (float)AIS_BAUD;

    const size_t MAX_LAG = (size_t)(msr*0.08);
    const size_t BATCH   = (size_t)(msr / 50);

    printf("[AIS ch%d] start cf=%.4fMHz off=%.0fHz decim=%u work_sr=%u sps=%.2f\n",
           ch_idx,(ch.s+ch.e)/2.f,off_hz,total_decim,work_sr,sps);
    fflush(stdout);

    // 믹서: off_hz 만큼 주파수 이동
    Oscillator osc; osc.set_freq((double)off_hz,(double)msr);

    // decimation: 단순 누적 평균 (sinc 등가, aliasing 억제)
    double cap_i=0,cap_q=0; int cap_cnt=0;

    // work_sr 기준 LPF: ±15kHz (오실레이터 오차 여유 포함)
    // 2단 IIR1
    IIR1 lpi1,lpq1,lpi2,lpq2;
    { float cn = 15000.f/(float)work_sr;
      if(cn>0.45f) cn=0.45f;
      lpi1.set(cn); lpq1.set(cn);
      lpi2.set(cn); lpq2.set(cn); }

    float prev_i=0,prev_q=0;
    // disc DC 제거: 시상수 약 5ms (gate 열린 직후 빠르게 수렴)
    // AIS FSK: +2.4kHz/-2.4kHz 대칭이므로 장기 평균은 0에 가까워야 함
    float disc_dc=0.f;
    const float DC_ALPHA = 1.f / (0.005f * (float)work_sr);

    // AGC: disc 진폭 정규화
    float agc_peak=0.1f;
    const float AGC_ATTACK=0.01f, AGC_DECAY=0.0001f;

    // ── Squelch + auto-calibration (demod.cpp와 동일한 방식) ─────────────
    const float SQL_ALPHA  = 0.05f;
    const int   CALIB_SAMP = (int)(work_sr * 0.5f);   // 500ms 분량
    float sql_avg=-120.0f;
    std::vector<float> calib_buf;
    bool calibrated = ch.sq_calibrated.load(std::memory_order_relaxed);
    if(!calibrated) calib_buf.reserve(CALIB_SAMP);
    bool gate_open=false;
    int  sq_ui_tick=0;

    // ── AIS FSK 복조 상태 ──────────────────────────────────────────────────
    float sym_phase=0.f, mu=0.f, p_sym=0.f;
    uint8_t nrzi_prev[2]={0,0};
    Hdlc hdlc[2];

    // 진단
    int diag_cnt=0;
    const int DIAG_INTERVAL=(int)work_sr*5;

    while(!ch.digi_stop_req.load(std::memory_order_relaxed)){
        size_t wp=ring_wp.load(std::memory_order_acquire);
        size_t rp=ch.digi_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;

        if(lag>MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK;
            ch.digi_rp.store(rp,std::memory_order_release);
            cap_i=cap_q=0; cap_cnt=0;
            lpi1.s=lpq1.s=lpi2.s=lpq2.s=0; prev_i=prev_q=0;
            disc_dc=0.f; agc_peak=0.1f;
            sym_phase=mu=p_sym=0.f;
            lag=(wp-rp)&IQ_RING_MASK;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/hw.iq_scale, sq=ring[pos*2+1]/hw.iq_scale;

            float mi,mq; osc.mix(si,sq,mi,mq);
            cap_i+=mi; cap_q+=mq; cap_cnt++;
            if(cap_cnt<(int)total_decim) continue;

            float fi=(float)(cap_i/cap_cnt), fq=(float)(cap_q/cap_cnt);
            cap_i=cap_q=0; cap_cnt=0;

            // 2단 LPF (work_sr 기준, ±6kHz)
            fi=lpi1.p(fi); fq=lpq1.p(fq);
            fi=lpi2.p(fi); fq=lpq2.p(fq);

            // ── Squelch ───────────────────────────────────────────────────
            float p_inst=fi*fi+fq*fq;
            float db_inst=(p_inst>1e-12f)?10.0f*log10f(p_inst):-120.0f;
            sql_avg=SQL_ALPHA*db_inst+(1.0f-SQL_ALPHA)*sql_avg;

            // auto-calibration: 첫 500ms 샘플로 노이즈 플로어 추정 → thr 설정
            if(!calibrated){
                calib_buf.push_back(db_inst);
                if((int)calib_buf.size()>=CALIB_SAMP){
                    std::vector<float> tmp=calib_buf;
                    size_t p20=tmp.size()/5;
                    std::nth_element(tmp.begin(),tmp.begin()+p20,tmp.end());
                    ch.sq_threshold.store(tmp[p20]+10.0f,std::memory_order_relaxed);
                    calibrated=true;
                    ch.sq_calibrated.store(true,std::memory_order_relaxed);
                    calib_buf.clear(); calib_buf.shrink_to_fit();
                }
            }

            float thr=ch.sq_threshold.load(std::memory_order_relaxed);
            bool prev_gate=gate_open;
            if(calibrated){
                if(!gate_open && sql_avg>=thr) gate_open=true;
                if( gate_open && sql_avg< thr-3.0f){
                    gate_open=false;
                    hdlc[0].reset(); hdlc[1].reset();
                    sym_phase=0.f; mu=0.f; p_sym=0.f;
                    nrzi_prev[0]=nrzi_prev[1]=0;
                    prev_i=0.f; prev_q=0.f;
                    disc_dc=0.f; agc_peak=0.1f;
                }
            }
            if(gate_open!=prev_gate){
                printf("[AIS ch%d] gate %s sql=%.1fdB thr=%.1fdB\n",
                       ch_idx,gate_open?"OPEN":"CLOSE",sql_avg,thr);
                fflush(stdout);
                if(gate_open){
                    // gate 열릴 때 disc_dc 즉시 리셋 (이전 노이즈 편향 제거)
                    disc_dc=0.f; agc_peak=0.1f;
                }
            }
            if(++sq_ui_tick>=256){ sq_ui_tick=0;
                ch.sq_sig .store(sql_avg,  std::memory_order_relaxed);
                ch.sq_gate.store(gate_open,std::memory_order_relaxed);
            }

            // ── AIS FSK 복조 (squelch 열릴 때만) ─────────────────────────
            if(!gate_open) continue;

            // FM discriminator
            float cross=fi*prev_q-fq*prev_i;
            float dot  =fi*prev_i+fq*prev_q;
            float disc=(prev_i!=0.f||prev_q!=0.f)?atan2f(cross,dot+1e-30f):0.f;
            prev_i=fi; prev_q=fq;

            // DC 제거: 잔류 주파수 오프셋 제거
            disc_dc = DC_ALPHA*disc + (1.f-DC_ALPHA)*disc_dc;
            disc -= disc_dc;

            // AGC: disc 진폭 정규화 → 비트 결정 임계 0 고정
            float aabs=fabsf(disc);
            if(aabs>agc_peak) agc_peak=aabs*AGC_ATTACK+(1.f-AGC_ATTACK)*agc_peak;
            else              agc_peak=aabs*AGC_DECAY +(1.f-AGC_DECAY )*agc_peak;
            if(agc_peak<1e-6f) agc_peak=1e-6f;
            disc/=agc_peak;

            // 진단: gate 열린 구간에서 200샘플마다 출력
            diag_cnt++;
            if(diag_cnt>=200){
                printf("[AIS ch%d] D fi=%.5f disc=%.3f dc=%.3f agc=%.4f sps=%.2f f0=%d ok0=%d\n",
                       ch_idx,fi,disc,disc_dc,agc_peak,sps,
                       hdlc[0].flag_cnt,hdlc[0].fcs_ok);
                fflush(stdout);
                diag_cnt=0;
            }

            // M&M 타이밍
            sym_phase+=1.f;
            if(sym_phase>=(sps+mu)){
                sym_phase-=(sps+mu);
                float cur=disc;
                float ec=(cur>0.f?1.f:-1.f)*p_sym-(p_sym>0.f?1.f:-1.f)*cur;
                mu+=0.005f*ec; if(mu>1.f)mu=1.f; if(mu<-1.f)mu=-1.f;
                p_sym=cur;

                uint8_t rf=(cur>0.f)?1:0;
                for(int d=0;d<2;d++){
                    uint8_t rf_d=(d==0)?rf:(rf^1);
                    // AIS NRZI: 천이=0(space), 유지=1(mark)
                    uint8_t nrz=(rf_d==nrzi_prev[d])?1:0;
                    nrzi_prev[d]=rf_d;
                    hdlc[d].push(nrz,ch_idx);
                }
            }
        }
        ch.digi_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    printf("[AIS ch%d] stopped f0=%d ok0=%d f1=%d ok1=%d\n",
           ch_idx,hdlc[0].flag_cnt,hdlc[0].fcs_ok,hdlc[1].flag_cnt,hdlc[1].fcs_ok);
    fflush(stdout);
}

void FFTViewer::start_digi(int ch_idx, Channel::DigitalMode mode){
    Channel& ch=channels[ch_idx];
    if(ch.digi_run.load()||!ch.filter_active) return;
    ch.digital_mode=mode;
    ch.sq_calibrated.store(false);   // 재시작 시 auto-calibration 재수행
    ch.sq_threshold.store(-50.0f);   // threshold 초기화 → calibration 후 실제값으로 덮어씀
    ch.digi_rp.store(ring_wp.load());
    ch.digi_stop_req.store(false);
    ch.digi_run.store(true);
    if(mode==Channel::DIGI_AIS)
        ch.digi_thr=std::thread(&FFTViewer::ais_worker,this,ch_idx);
    else if(mode==Channel::DIGI_DMR)
        ch.digi_thr=std::thread(&FFTViewer::dmr_worker,this,ch_idx);
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
