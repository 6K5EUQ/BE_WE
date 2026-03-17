#include "fft_viewer.hpp"
#include "audio.hpp"
#include "net_client.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>

// ── FFT (Radix-2 DIT, 크기 256 고정) ──────────────────────────────────────
static constexpr int NR_N = 256;
static constexpr int NR_H = NR_N / 2 + 1;  // 129 bins

struct CplxF { float r, i; };

static void nr_fft(const float* in, CplxF* out){
    // bit-reverse copy
    CplxF buf[NR_N];
    for(int i=0;i<NR_N;i++){
        int j=0, x=i;
        for(int b=0;b<8;b++){ j=(j<<1)|(x&1); x>>=1; }
        buf[j] = {in[i], 0};
    }
    for(int s=1;s<=8;s++){
        int m=1<<s, mh=m>>1;
        float a = -6.283185307f / (float)m;
        CplxF wm = {cosf(a), sinf(a)};
        for(int k=0;k<NR_N;k+=m){
            CplxF w={1,0};
            for(int j=0;j<mh;j++){
                CplxF t={w.r*buf[k+j+mh].r - w.i*buf[k+j+mh].i,
                         w.r*buf[k+j+mh].i + w.i*buf[k+j+mh].r};
                CplxF u=buf[k+j];
                buf[k+j]    = {u.r+t.r, u.i+t.i};
                buf[k+j+mh] = {u.r-t.r, u.i-t.i};
                float wr=w.r*wm.r - w.i*wm.i;
                w.i = w.r*wm.i + w.i*wm.r;
                w.r = wr;
            }
        }
    }
    for(int i=0;i<NR_H;i++) out[i]=buf[i];
}

static void nr_ifft(const CplxF* freq, float* out){
    // conjugate → FFT → conjugate → /N
    CplxF buf[NR_N];
    // reconstruct full spectrum (conjugate symmetric)
    for(int i=0;i<NR_H;i++) buf[i]={freq[i].r, -freq[i].i};
    for(int i=NR_H;i<NR_N;i++) buf[i]={freq[NR_N-i].r, freq[NR_N-i].i};
    // bit-reverse
    CplxF tmp[NR_N];
    for(int i=0;i<NR_N;i++){
        int j=0, x=i;
        for(int b=0;b<8;b++){ j=(j<<1)|(x&1); x>>=1; }
        tmp[j]=buf[i];
    }
    for(int s=1;s<=8;s++){
        int m=1<<s, mh=m>>1;
        float a = -6.283185307f / (float)m;
        CplxF wm = {cosf(a), sinf(a)};
        for(int k=0;k<NR_N;k+=m){
            CplxF w={1,0};
            for(int j=0;j<mh;j++){
                CplxF t={w.r*tmp[k+j+mh].r - w.i*tmp[k+j+mh].i,
                         w.r*tmp[k+j+mh].i + w.i*tmp[k+j+mh].r};
                CplxF u=tmp[k+j];
                tmp[k+j]    = {u.r+t.r, u.i+t.i};
                tmp[k+j+mh] = {u.r-t.r, u.i-t.i};
                float wr=w.r*wm.r - w.i*wm.i;
                w.i = w.r*wm.i + w.i*wm.r;
                w.r = wr;
            }
        }
    }
    // conjugate + normalize
    float inv = 1.0f / (float)NR_N;
    for(int i=0;i<NR_N;i++) out[i] = tmp[i].r * inv;
}

// ── 채널별 노이즈 제거 상태 ────────────────────────────────────────────────
struct NRState {
    // 공통
    float in_buf[NR_N]  = {};     // 입력 축적 버퍼
    float out_buf[NR_N] = {};     // 출력 버퍼
    float overlap[NR_H] = {};     // overlap-add 이전 프레임 후반
    int   pos   = 0;              // 입력 축적 위치
    int   rd    = 0;              // 출력 읽기 위치
    int   avail = 0;              // 출력 잔여 샘플
    int   prev_mode = 0;

    // Hann 윈도우
    float win[NR_N] = {};
    bool  win_init = false;

    // SS: 잡음 프로파일
    float noise_mag[NR_H] = {};   // 잡음 스펙트럼 크기 평균
    int   noise_frames = 0;       // 수집된 잡음 프레임 수
    bool  noise_ready = false;    // 프로파일 준비 완료
    int   silent_count = 0;       // 연속 무음 프레임 수
    static constexpr int NOISE_LEARN_FRAMES = 30; // ~160ms 학습
    static constexpr float SS_ALPHA = 2.0f;       // 감산 강도
    static constexpr float SS_FLOOR = 0.02f;      // 스펙트럼 하한

    // SG: 게이트 임계값
    float gate_thresh[NR_H] = {}; // 빈별 게이트 임계 (자동 학습)
    int   gate_frames = 0;
    bool  gate_ready = false;
    int   gate_silent = 0;
    static constexpr int GATE_LEARN_FRAMES = 30;
    static constexpr float GATE_MULT = 2.5f;     // 임계값 배수

    void init_win(){
        if(win_init) return;
        for(int i=0;i<NR_N;i++)
            win[i] = 0.5f * (1.0f - cosf(6.283185307f * i / (float)NR_N));
        win_init = true;
    }

    void reset(){
        pos=0; rd=0; avail=0;
        memset(overlap, 0, sizeof(overlap));
        noise_frames=0; noise_ready=false; silent_count=0;
        gate_frames=0; gate_ready=false; gate_silent=0;
        memset(noise_mag, 0, sizeof(noise_mag));
        memset(gate_thresh, 0, sizeof(gate_thresh));
    }

    // 신호 에너지 (무음 판별용)
    float frame_energy(const float* buf, int n){
        float e=0;
        for(int i=0;i<n;i++) e += buf[i]*buf[i];
        return e / (float)n;
    }

    // 처리 (mode: 1=SS, 2=SG)
    void process_frame(int mode){
        init_win();

        // 윈도잉
        float windowed[NR_N];
        for(int i=0;i<NR_N;i++) windowed[i] = in_buf[i] * win[i];

        // FFT
        CplxF freq[NR_H];
        nr_fft(windowed, freq);

        // 빈별 크기
        float mag[NR_H];
        for(int i=0;i<NR_H;i++)
            mag[i] = sqrtf(freq[i].r*freq[i].r + freq[i].i*freq[i].i);

        // 무음 판별 (에너지 기반)
        float energy = frame_energy(in_buf, NR_N);
        bool is_silent = (energy < 1e-6f);

        if(mode == 1){
            // ── Spectral Subtraction ──
            if(is_silent){
                silent_count++;
                if(silent_count >= 3){
                    // 잡음 프로파일 학습/갱신
                    if(noise_frames < NOISE_LEARN_FRAMES){
                        for(int i=0;i<NR_H;i++)
                            noise_mag[i] += mag[i];
                        noise_frames++;
                        if(noise_frames >= NOISE_LEARN_FRAMES){
                            for(int i=0;i<NR_H;i++)
                                noise_mag[i] /= (float)NOISE_LEARN_FRAMES;
                            noise_ready = true;
                        }
                    } else {
                        // 느린 적응 갱신
                        for(int i=0;i<NR_H;i++)
                            noise_mag[i] = noise_mag[i]*0.95f + mag[i]*0.05f;
                    }
                }
            } else {
                silent_count = 0;
            }

            if(noise_ready){
                // 감산
                for(int i=0;i<NR_H;i++){
                    float new_mag = mag[i] - SS_ALPHA * noise_mag[i];
                    if(new_mag < SS_FLOOR * mag[i]) new_mag = SS_FLOOR * mag[i];
                    float gain = (mag[i] > 1e-10f) ? (new_mag / mag[i]) : 0;
                    freq[i].r *= gain;
                    freq[i].i *= gain;
                }
            }
        } else {
            // ── Spectral Gate ──
            if(is_silent){
                gate_silent++;
                if(gate_silent >= 3){
                    if(gate_frames < GATE_LEARN_FRAMES){
                        for(int i=0;i<NR_H;i++)
                            gate_thresh[i] += mag[i];
                        gate_frames++;
                        if(gate_frames >= GATE_LEARN_FRAMES){
                            for(int i=0;i<NR_H;i++)
                                gate_thresh[i] = (gate_thresh[i] / (float)GATE_LEARN_FRAMES) * GATE_MULT;
                            gate_ready = true;
                        }
                    } else {
                        for(int i=0;i<NR_H;i++){
                            float avg = mag[i];
                            gate_thresh[i] = gate_thresh[i]*0.95f + avg*GATE_MULT*0.05f;
                        }
                    }
                }
            } else {
                gate_silent = 0;
            }

            if(gate_ready){
                for(int i=0;i<NR_H;i++){
                    if(mag[i] < gate_thresh[i]){
                        // 부드러운 감쇠 (hard gate가 아닌 soft)
                        float ratio = mag[i] / (gate_thresh[i] + 1e-10f);
                        float gain = ratio * ratio; // 제곱 커브
                        freq[i].r *= gain;
                        freq[i].i *= gain;
                    }
                }
            }
        }

        // IFFT
        float ifft_out[NR_N];
        nr_ifft(freq, ifft_out);

        // 윈도잉 + overlap-add (50% overlap, 출력은 전반 NR_H 샘플)
        for(int i=0;i<NR_H;i++)
            out_buf[i] = overlap[i] + ifft_out[i] * win[i];
        for(int i=0;i<NR_H;i++)
            overlap[i] = ifft_out[NR_H + i] * win[NR_H + i];

        rd = 0;
        avail = NR_H; // 128 샘플 출력
    }

    // 1샘플 처리 (mode: 1=SS, 2=SG)
    float feed(float smp, int mode){
        if(mode != prev_mode){ reset(); prev_mode = mode; }

        in_buf[pos++] = smp;
        if(pos >= NR_N){
            // 50% overlap: 후반부를 앞으로 복사
            process_frame(mode);
            memmove(in_buf, in_buf + NR_H, NR_H * sizeof(float));
            pos = NR_H;
        }
        if(avail > 0){
            float out = out_buf[rd++];
            avail--;
            return out;
        }
        return 0; // 최초 프레임 축적 중
    }
};

// ── Mix worker ─────────────────────────────────────────────────────────────
void FFTViewer::mix_worker(){
    AlsaOut alsa; alsa.open(AUDIO_SR);
    static constexpr int PERIOD=256;
    std::vector<int16_t> sbuf(PERIOD*2,0);

    NRState nr[MAX_CHANNELS];

    while(!mix_stop.load(std::memory_order_relaxed)){
        if(!alsa.pcm){
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            alsa.open(AUDIO_SR);
            if(!alsa.pcm) continue;
        }

        for(int i=0;i<PERIOD;i++){
            float L=0,R=0;
            if(net_cli && remote_mode){
                // ── CONNECT 모드 ──
                for(int c=0;c<MAX_CHANNELS;c++){
                    bool is_muted = (local_ch_out[c]==3);
                    bool rec_on = channels[c].audio_rec_on.load(std::memory_order_relaxed);
                    if(is_muted && !rec_on){
                        float dummy; int8_t p2;
                        net_cli->audio[c].pop(dummy,p2); continue;
                    }
                    float smp=0; int8_t pan=0;
                    if(!net_cli->audio[c].pop(smp, pan)){
                        if(rec_on && channels[c].audio_rec_fp)
                            channels[c].maybe_rec_audio(0.f);
                        continue;
                    }
                    if(rec_on) channels[c].maybe_rec_audio(smp);
                    if(is_muted) continue;

                    int mode = nr_mode[c];
                    if(mode > 0) smp = nr[c].feed(smp, mode);
                    else if(nr[c].prev_mode != 0){ nr[c].reset(); nr[c].prev_mode=0; }

                    int lco = local_ch_out[c];
                    if(lco==0)      { L+=smp; }
                    else if(lco==2) { R+=smp; }
                    else            { L+=smp; R+=smp; }
                }
            } else {
                // ── LOCAL / HOST 모드 ──
                for(int c=0;c<MAX_CHANNELS;c++){
                    if(!channels[c].dem_run.load(std::memory_order_relaxed)
                    && !channels[c].digi_run.load(std::memory_order_relaxed)) continue;
                    if(net_srv && !(channels[c].audio_mask.load() & 0x1u)) continue;
                    if(local_ch_out[c]==3) {
                        float dummy; channels[c].pop_audio(dummy); continue;
                    }
                    float smp=0; channels[c].pop_audio(smp);

                    int mode = nr_mode[c];
                    if(mode > 0) smp = nr[c].feed(smp, mode);
                    else if(nr[c].prev_mode != 0){ nr[c].reset(); nr[c].prev_mode=0; }

                    int lco = local_ch_out[c];
                    if(lco==0)      { L+=smp; }
                    else if(lco==2) { R+=smp; }
                    else            { L+=smp; R+=smp; }
                }
            }
            L=L<-1.0f?-1.0f:L>1.0f?1.0f:L;
            R=R<-1.0f?-1.0f:R>1.0f?1.0f:R;
            sbuf[i*2  ]=(int16_t)(L*32767.0f);
            sbuf[i*2+1]=(int16_t)(R*32767.0f);
        }
        alsa.write(sbuf.data(),PERIOD);
    }
    alsa.close();
    printf("Mix worker exited\n");
}
