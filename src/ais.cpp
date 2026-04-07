#include "fft_viewer.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <sys/wait.h>
#include <poll.h>
#include <fcntl.h>
#include <signal.h>

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
    bewe_log_push(0,"[AIS ch%d] !%s*%02X\n",ch_idx,nmea,cs);
    uint8_t mt=(uint8_t)bu(bits,0,6); uint32_t mm=bu(bits,8,30);
    if((mt>=1&&mt<=3)&&nbits>=168)
        bewe_log_push(0,"  Type%u MMSI=%u lat=%.5f lon=%.5f SOG=%.1f\n",
               mt,mm,bi(bits,89,27)/600000.f,bi(bits,61,28)/600000.f,bu(bits,50,10)/10.f);
    else if(mt==18&&nbits>=168)
        bewe_log_push(0,"  Type18 MMSI=%u lat=%.5f lon=%.5f\n",
               mm,bi(bits,85,27)/600000.f,bi(bits,57,28)/600000.f);
    else if(mt==5&&nbits>=426){
        char nm[21]={}; for(int i=0;i<20;i++) nm[i]=a6((uint8_t)bu(bits,112+i*6,6));
        for(int i=19;i>=0&&nm[i]==' ';i--) nm[i]=0;
        bewe_log_push(0,"  Type5 MMSI=%u NAME='%s'\n",mm,nm);
    } else bewe_log_push(0,"  Type%u MMSI=%u bits=%d\n",mt,mm,nbits);
}

// ── AIS Python 파이프 관리 ─────────────────────────────────────────────────

void FFTViewer::ais_pipe_start(){
    if(ais_pipe.alive.load()) return;

    int pipe_to[2], pipe_from[2];
    if(pipe(pipe_to) != 0 || pipe(pipe_from) != 0){
        bewe_log_push(0,"[AIS pipe] pipe() failed\n");
        return;
    }

    pid_t pid = fork();
    if(pid < 0){
        bewe_log_push(0,"[AIS pipe] fork() failed\n");
        close(pipe_to[0]); close(pipe_to[1]);
        close(pipe_from[0]); close(pipe_from[1]);
        return;
    }

    if(pid == 0){
        // 자식 프로세스
        close(pipe_to[1]);    // 쓰기 끝 닫기
        close(pipe_from[0]);  // 읽기 끝 닫기
        dup2(pipe_to[0], STDIN_FILENO);
        dup2(pipe_from[1], STDOUT_FILENO);
        close(pipe_to[0]);
        close(pipe_from[1]);

        // Python 실행 경로 결정
        char script_path[512];
        // 실행 파일 기준 상대 경로 시도
        ssize_t len = readlink("/proc/self/exe", script_path, sizeof(script_path)-1);
        if(len > 0){
            script_path[len] = '\0';
            // 실행파일 디렉토리에서 ../decoder/AIS/ais_pipe.py
            char* last_slash = strrchr(script_path, '/');
            if(last_slash){
                // build/ 디렉토리 안에 실행파일이 있으므로 한단계 위로
                *last_slash = '\0';
                last_slash = strrchr(script_path, '/');
                if(last_slash){
                    snprintf(last_slash, sizeof(script_path)-(last_slash-script_path),
                             "/decoder/AIS/ais_pipe.py");
                }
            }
        }
        // fallback: 절대 경로
        if(access(script_path, R_OK) != 0)
            snprintf(script_path, sizeof(script_path), "%s/BE_WE/decoder/AIS/ais_pipe.py",
                     getenv("HOME") ? getenv("HOME") : "/home/dsa");

        execl("/usr/bin/python3", "python3", "-u", script_path, (char*)nullptr);
        _exit(1);
    }

    // 부모 프로세스
    close(pipe_to[0]);    // 읽기 끝 닫기
    close(pipe_from[1]);  // 쓰기 끝 닫기

    ais_pipe.fd_to_py = pipe_to[1];
    ais_pipe.fd_from_py = pipe_from[0];
    ais_pipe.pid = pid;

    // fd_from_py를 non-blocking으로
    int flags = fcntl(ais_pipe.fd_from_py, F_GETFL);
    fcntl(ais_pipe.fd_from_py, F_SETFL, flags | O_NONBLOCK);

    ais_pipe.alive.store(true);
    ais_pipe_reader_stop.store(false);
    ais_pipe_reader_thr = std::thread(&FFTViewer::ais_pipe_reader_loop, this);

    bewe_log_push(0,"[AIS pipe] started pid=%d\n", pid);
}

void FFTViewer::ais_pipe_stop(){
    if(!ais_pipe.alive.load()) return;
    ais_pipe.alive.store(false);

    // 파이프 닫기 → Python이 EOF로 종료
    if(ais_pipe.fd_to_py >= 0){ close(ais_pipe.fd_to_py); ais_pipe.fd_to_py = -1; }

    // reader 스레드 종료
    ais_pipe_reader_stop.store(true);
    if(ais_pipe_reader_thr.joinable()) ais_pipe_reader_thr.join();

    if(ais_pipe.fd_from_py >= 0){ close(ais_pipe.fd_from_py); ais_pipe.fd_from_py = -1; }

    // 자식 프로세스 대기
    if(ais_pipe.pid > 0){
        int st;
        pid_t ret = waitpid(ais_pipe.pid, &st, WNOHANG);
        if(ret == 0){
            // 아직 살아있으면 1초 대기 후 강제 종료
            std::this_thread::sleep_for(std::chrono::seconds(1));
            ret = waitpid(ais_pipe.pid, &st, WNOHANG);
            if(ret == 0){
                kill(ais_pipe.pid, SIGKILL);
                waitpid(ais_pipe.pid, &st, 0);
            }
        }
        ais_pipe.pid = -1;
    }
    bewe_log_push(0,"[AIS pipe] stopped\n");
}

void FFTViewer::ais_pipe_send_frame(const uint8_t* bits, int nbits, int ch_idx){
    if(!ais_pipe.alive.load(std::memory_order_relaxed)) return;
    if(ais_pipe.fd_to_py < 0) return;

    // 비트열을 문자열로 변환
    char buf[600]; // "FRAME X NNN " + 512 bits + "\n"
    int off = snprintf(buf, sizeof(buf), "FRAME %d %d ", ch_idx, nbits);
    for(int i = 0; i < nbits && off < (int)sizeof(buf)-2; i++)
        buf[off++] = (bits[i] & 1) ? '1' : '0';
    buf[off++] = '\n';

    std::lock_guard<std::mutex> lk(ais_pipe.write_mtx);
    ssize_t ret = write(ais_pipe.fd_to_py, buf, off);
    if(ret < 0){
        // EPIPE: Python 프로세스 종료됨
        ais_pipe.alive.store(false);
    }
}

void FFTViewer::ais_pipe_reader_loop(){
    char line_buf[4096];
    int line_pos = 0;
    std::string accum;

    while(!ais_pipe_reader_stop.load(std::memory_order_relaxed)){
        struct pollfd pfd;
        pfd.fd = ais_pipe.fd_from_py;
        pfd.events = POLLIN;
        int pr = poll(&pfd, 1, 100); // 100ms 타임아웃

        if(pr <= 0) continue;
        if(pfd.revents & (POLLHUP|POLLERR)){
            // Python 프로세스 종료 감지
            ais_pipe.alive.store(false);
            break;
        }
        if(!(pfd.revents & POLLIN)) continue;

        char rbuf[2048];
        ssize_t n = read(ais_pipe.fd_from_py, rbuf, sizeof(rbuf));
        if(n <= 0){
            if(n == 0){ ais_pipe.alive.store(false); break; } // EOF
            continue; // EAGAIN
        }

        for(ssize_t i = 0; i < n; i++){
            if(rbuf[i] == '\n'){
                line_buf[line_pos] = '\0';
                std::string line(line_buf, line_pos);
                line_pos = 0;

                if(line == "---END---"){
                    // 누적된 텍스트를 Q 패널에 전달
                    if(!accum.empty()){
                        digi_log_push(0, "%s", accum.c_str());
                    }
                    accum.clear();
                } else {
                    if(!accum.empty()) accum += '\n';
                    accum += line;
                }
            } else {
                if(line_pos < (int)sizeof(line_buf)-1)
                    line_buf[line_pos++] = rbuf[i];
            }
        }
    }
}

// ── HDLC ─────────────────────────────────────────────────────────────────

struct Hdlc {
    FFTViewer* viewer = nullptr;  // 파이프 전송용
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
        if(plen<56) return;
        int nbytes=(plen+7)/8;
        if(nbytes>64) return;
        uint8_t bytes[64]={};
        for(int i=0;i<nbytes*8&&i<nb;i++) bytes[i/8]|=(raw[i]<<(i%8));
        uint16_t rxfcs=0;
        for(int i=0;i<16;i++) rxfcs|=((uint16_t)raw[plen+i]<<i);
        uint16_t calcfcs=crc_ccitt(bytes,(plen+7)/8);
        if(calcfcs!=rxfcs) return;
        fcs_ok++;
        uint8_t bits[512]={};
        for(int i=0;i<plen;i++) bits[(i/8)*8+(7-(i%8))]=raw[i];
        print_frame(bits,plen,ch_idx);
        if(viewer) viewer->ais_pipe_send_frame(bits,plen,ch_idx);
    }
};

// ── AIS 워커 ──────────────────────────────────────────────────────────────
void FFTViewer::ais_worker(int ch_idx){
    Channel& ch=channels[ch_idx];
    uint32_t msr=header.sample_rate;
    uint64_t init_cf = live_cf_hz.load(std::memory_order_acquire);
    float off_hz=(((ch.s+ch.e)/2.0f)-(float)(init_cf/1e6f))*1e6f;

    // sps=16: GMSK BT=0.4 전이폭(~9샘플@8sps)이 심볼 경계를 침범하는 ISI 문제 해결
    // sps=8이면 전이가 인접 심볼까지 걸쳐 stuffed-zero 오판 → 프레임 abort
    uint32_t target_work = AIS_BAUD * 16;  // 153600
    uint32_t total_decim = (msr + target_work/2) / target_work;  // 반올림
    if(total_decim < 1) total_decim = 1;
    uint32_t work_sr = msr / total_decim;
    float sps = (float)work_sr / (float)AIS_BAUD;

    const size_t MAX_LAG = (size_t)(msr*0.08);
    const size_t BATCH   = (size_t)(msr / 50);

    bewe_log_push(0,"[AIS ch%d] start cf=%.4fMHz off=%.0fHz decim=%u work_sr=%u sps=%.2f\n",
           ch_idx,(ch.s+ch.e)/2.f,off_hz,total_decim,work_sr,sps);

    Oscillator osc; osc.set_freq((double)off_hz,(double)msr);
    uint64_t prev_cf = init_cf;

    // ── 진단용: decimation 전 raw IQ + decimation 후(LPF전) IQ 둘 다 덤프
    FILE* dump_raw_fp = nullptr;   // mixer 전 raw IQ (처음 0.01초만)
    FILE* dump_dec_fp = nullptr;   // decimation 후, LPF 전 IQ (5초)
    int dump_raw_cnt = 0;
    int dump_dec_cnt = 0;
    const int DUMP_RAW_MAX = (int)(msr * 0.01); // 0.01초 raw (614k samples)
    const int DUMP_DEC_MAX = work_sr * 5;        // 5초 decimated
    {
        char p1[256], p2[256];
        snprintf(p1,sizeof(p1),"/tmp/ais_raw_ch%d.raw",ch_idx);
        snprintf(p2,sizeof(p2),"/tmp/ais_dec_ch%d.raw",ch_idx);
        dump_raw_fp = fopen(p1,"wb");
        dump_dec_fp = fopen(p2,"wb");
        bewe_log_push(0,"[AIS ch%d] dumping raw IQ (%d samples) + decimated IQ (%d samples)\n",
            ch_idx, DUMP_RAW_MAX, DUMP_DEC_MAX);
    }

    double cap_i=0,cap_q=0; int cap_cnt=0;

    IIR1 lpi1,lpq1,lpi2,lpq2;
    { float cn = 15000.f/(float)work_sr;
      if(cn>0.45f) cn=0.45f;
      lpi1.set(cn); lpq1.set(cn);
      lpi2.set(cn); lpq2.set(cn); }

    float prev_i=0,prev_q=0;
    float disc_dc=0.f;
    const float DC_ALPHA = 1.f / (0.005f * (float)work_sr);

    // post-disc smoothing: GMSK BT=0.4 매칭 필터 근사 (cutoff ~6kHz)
    IIR1 disc_lpf;
    { float cn = 6000.f / (float)work_sr; disc_lpf.set(cn); }

    float agc_peak=0.1f;
    const float AGC_ATTACK=0.01f, AGC_DECAY=0.002f;

    float sym_phase = 0.f;

    uint8_t nrzi_prev[2]={0,0};
    Hdlc hdlc[2];
    hdlc[0].viewer = this;
    hdlc[1].viewer = this;

    // 진단 (10초 간격)
    int diag_cnt=0;
    const int DIAG_INTERVAL=(int)(work_sr*10);
    int total_demod_samples=0;
    int lag_reset_cnt=0;
    bool gate_open=false;

    while(!ch.digi_stop_req.load(std::memory_order_relaxed)){
        // Squelch gate 상태 확인
        bool prev_gate = gate_open;
        gate_open = ch.sq_gate.load(std::memory_order_relaxed);

        // gate CLOSED→OPEN 전환: 전체 DSP 초기화 + 진단
        if(!prev_gate && gate_open){
            // 진단: ring buffer 첫 10 샘플 로그
            size_t twp=ring_wp.load(std::memory_order_acquire);
            size_t trp=ch.digi_rp.load(std::memory_order_relaxed);
            size_t tlag=(twp-trp)&IQ_RING_MASK;
            bewe_log_push(0,"[AIS ch%d] GATE OPEN: rp=%zu wp=%zu lag=%zu\n",ch_idx,trp,twp,tlag);
            for(int ii=0;ii<std::min((size_t)5,tlag);ii++){
                size_t tpos=(trp+ii)&IQ_RING_MASK;
                bewe_log_push(0,"  ring[%zu]: I=%d Q=%d (%.6f, %.6f)\n",
                    tpos, ring[tpos*2], ring[tpos*2+1],
                    ring[tpos*2]/hw.iq_scale, ring[tpos*2+1]/hw.iq_scale);
            }
            cap_i=cap_q=0; cap_cnt=0;
            lpi1.s=lpq1.s=lpi2.s=lpq2.s=0; prev_i=prev_q=0;
            disc_dc=0.f; agc_peak=0.1f; disc_lpf.s=0.f;
            sym_phase=0.f;
            nrzi_prev[0]=nrzi_prev[1]=0;
            hdlc[0].reset(); hdlc[1].reset();
        }

        { uint64_t cur_cf=live_cf_hz.load(std::memory_order_acquire);
          if(cur_cf!=prev_cf){
              off_hz=(((ch.s+ch.e)/2.0f)-(float)(cur_cf/1e6))*1e6f;
              osc.set_freq((double)off_hz,(double)msr);
              prev_cf=cur_cf;
          }
        }
        size_t wp=ring_wp.load(std::memory_order_acquire);
        size_t rp=ch.digi_rp.load(std::memory_order_relaxed);
        size_t lag=(wp-rp)&IQ_RING_MASK;

        if(lag>MAX_LAG){
            size_t keep=(size_t)(msr*0.02);
            rp=(wp-keep)&IQ_RING_MASK;
            ch.digi_rp.store(rp,std::memory_order_release);
            cap_i=cap_q=0; cap_cnt=0;
            lpi1.s=lpq1.s=lpi2.s=lpq2.s=0; prev_i=prev_q=0;
            disc_dc=0.f; agc_peak=0.1f; disc_lpf.s=0.f;
            sym_phase=0.f;
            lag=(wp-rp)&IQ_RING_MASK;
            lag_reset_cnt++;
        }
        if(lag==0){ std::this_thread::sleep_for(std::chrono::microseconds(50)); continue; }

        size_t avail=std::min(lag,BATCH);
        for(size_t s=0;s<avail;s++){
            size_t pos=(rp+s)&IQ_RING_MASK;
            float si=ring[pos*2]/hw.iq_scale, sq=ring[pos*2+1]/hw.iq_scale;

            // raw IQ 덤프 (mixer 전)
            if(dump_raw_fp && dump_raw_cnt < DUMP_RAW_MAX){
                float iq[2]={si,sq};
                fwrite(iq,sizeof(float),2,dump_raw_fp);
                dump_raw_cnt++;
                if(dump_raw_cnt>=DUMP_RAW_MAX){
                    fclose(dump_raw_fp); dump_raw_fp=nullptr;
                    bewe_log_push(0,"[AIS ch%d] raw IQ dump complete\n",ch_idx);
                }
            }

            float mi,mq; osc.mix(si,sq,mi,mq);
            cap_i+=mi; cap_q+=mq; cap_cnt++;
            if(cap_cnt<(int)total_decim) continue;

            float fi=(float)(cap_i/cap_cnt), fq=(float)(cap_q/cap_cnt);
            cap_i=cap_q=0; cap_cnt=0;

            // decimated IQ 덤프 (LPF 전)
            if(dump_dec_fp && dump_dec_cnt < DUMP_DEC_MAX){
                float iq[2]={fi,fq};
                fwrite(iq,sizeof(float),2,dump_dec_fp);
                dump_dec_cnt++;
                if(dump_dec_cnt>=DUMP_DEC_MAX){
                    fclose(dump_dec_fp); dump_dec_fp=nullptr;
                    bewe_log_push(0,"[AIS ch%d] decimated IQ dump complete\n",ch_idx);
                }
            }

            fi=lpi1.p(fi); fq=lpq1.p(fq);
            fi=lpi2.p(fi); fq=lpq2.p(fq);

            // FM discriminator
            float cross=fi*prev_q-fq*prev_i;
            float dot  =fi*prev_i+fq*prev_q;
            float disc=(prev_i!=0.f||prev_q!=0.f)?atan2f(cross,dot+1e-30f):0.f;
            prev_i=fi; prev_q=fq;

            // DC 제거
            disc_dc = DC_ALPHA*disc + (1.f-DC_ALPHA)*disc_dc;
            disc -= disc_dc;

            // post-disc smoothing (GMSK ISI 완화)
            disc = disc_lpf.p(disc);

            // AGC
            float aabs=fabsf(disc);
            if(aabs>agc_peak) agc_peak=aabs*AGC_ATTACK+(1.f-AGC_ATTACK)*agc_peak;
            else              agc_peak=aabs*AGC_DECAY +(1.f-AGC_DECAY )*agc_peak;
            if(agc_peak<1e-6f) agc_peak=1e-6f;
            disc/=agc_peak;

            total_demod_samples++;
            diag_cnt++;
            if(diag_cnt>=DIAG_INTERVAL){
                bewe_log_push(0,"[AIS ch%d] demod=%dk flags=%d+%d decoded=%d+%d lag_rst=%d agc=%.4f\n",
                       ch_idx,total_demod_samples/1000,
                       hdlc[0].flag_cnt,hdlc[1].flag_cnt,
                       hdlc[0].fcs_ok,hdlc[1].fcs_ok,
                       lag_reset_cnt,agc_peak);
                diag_cnt=0;
            }

            // ── 심볼 샘플링 (gate_open일 때만, 고정 간격) ─────────────────────
            if(gate_open){
                sym_phase+=1.f;
                if(sym_phase>=sps){
                    sym_phase-=sps;

                    // 비트 결정 + NRZI + HDLC
                    uint8_t rf=(disc>0.f)?1:0;
                    for(int d=0;d<2;d++){
                        uint8_t rf_d=(d==0)?rf:(rf^1);
                        uint8_t nrz=(rf_d==nrzi_prev[d])?1:0;
                        nrzi_prev[d]=rf_d;
                        hdlc[d].push(nrz,ch_idx);
                    }
                }
            }
        }
        ch.digi_rp.store((rp+avail)&IQ_RING_MASK,std::memory_order_release);
    }
    if(dump_raw_fp) fclose(dump_raw_fp);
    if(dump_dec_fp) fclose(dump_dec_fp);
    bewe_log_push(0,"[AIS ch%d] stopped flags=%d decoded=%d\n",
           ch_idx,hdlc[0].flag_cnt+hdlc[1].flag_cnt,hdlc[0].fcs_ok+hdlc[1].fcs_ok);
}

void FFTViewer::start_digi(int ch_idx, Channel::DigitalMode mode){
    Channel& ch=channels[ch_idx];
    if(ch.digi_run.load()||!ch.filter_active) return;
    ch.digital_mode=mode;
    ch.digi_rp.store(ring_wp.load());
    ch.digi_stop_req.store(false);
    ch.digi_run.store(true);
    if(mode==Channel::DIGI_AIS){
        // Python 파이프 시작 (이미 실행 중이면 무시)
        ais_pipe_start();
        ch.digi_thr=std::thread(&FFTViewer::ais_worker,this,ch_idx);
    }
    bewe_log_push(0,"[DIGI ch%d] start mode=%d\n",ch_idx,(int)mode);
}

void FFTViewer::stop_digi(int ch_idx){
    Channel& ch=channels[ch_idx];
    if(!ch.digi_run.load()) return;
    ch.digi_stop_req.store(true);
    if(ch.digi_thr.joinable()) ch.digi_thr.join();
    ch.digi_run.store(false);

    // 마지막 AIS 채널이면 파이프 정지
    if(ch.digital_mode == Channel::DIGI_AIS){
        bool any_ais = false;
        for(int i = 0; i < MAX_CHANNELS; i++){
            if(i != ch_idx && channels[i].digi_run.load() &&
               channels[i].digital_mode == Channel::DIGI_AIS){
                any_ais = true; break;
            }
        }
        if(!any_ais) ais_pipe_stop();
    }
    ch.digital_mode=Channel::DIGI_NONE;
    bewe_log_push(0,"[DIGI ch%d] stopped\n",ch_idx);
}
