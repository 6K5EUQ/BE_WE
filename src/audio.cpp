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
    CplxF buf[NR_N];
    for(int i=0;i<NR_H;i++) buf[i]={freq[i].r, -freq[i].i};
    for(int i=NR_H;i<NR_N;i++) buf[i]={freq[NR_N-i].r, freq[NR_N-i].i};
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
    float inv = 1.0f / (float)NR_N;
    for(int i=0;i<NR_N;i++) out[i] = tmp[i].r * inv;
}

// ── 특수 함수 ──────────────────────────────────────────────────────────────
// expint(x) = E1(x) 지수적분 (Log-MMSE에 필요)
static float expint_e1(float x){
    if(x <= 0) return 20.0f;
    if(x < 1.0f){
        // 급수 전개: E1(x) = -gamma - ln(x) + sum
        float euler = 0.5772156649f;
        float s = -euler - logf(x);
        float term = x;
        for(int n=1;n<=20;n++){
            s += term / (float)(n * n);  // (-1)^(n+1) * x^n / (n*n!)
            // 정확한 급수: (-1)^(n+1) * x^n / (n * n!)
            if(n < 20) term *= -x / (float)(n+1);
        }
        return s;
    }
    // 점근 전개: E1(x) ≈ exp(-x)/x * (1 - 1/x + 2/x^2 - ...)
    float inv = 1.0f / x;
    float s = inv;
    float term = inv;
    for(int n=1;n<=8;n++){
        term *= -(float)n * inv;
        s += term;
    }
    return expf(-x) * s;
}

// ── 채널별 노이즈 제거 상태 ────────────────────────────────────────────────
// mode: 1=SS, 2=SG, 3=WF(Wiener), 4=MS(MMSE-STSA), 5=LM(Log-MMSE)
struct NRState {
    float in_buf[NR_N]  = {};
    float out_buf[NR_N] = {};
    float overlap[NR_H] = {};
    int   pos   = 0;
    int   rd    = 0;
    int   avail = 0;
    int   prev_mode = 0;

    float win[NR_N] = {};
    bool  win_init = false;

    // 잡음 파워 스펙트럼 (공통)
    float noise_pow[NR_H] = {};
    int   noise_frames = 0;
    bool  noise_ready = false;
    int   silent_count = 0;
    static constexpr int LEARN_FRAMES = 30;

    // SS 파라미터
    static constexpr float SS_ALPHA = 2.0f;
    static constexpr float SS_FLOOR = 0.02f;

    // SG 파라미터
    float gate_thresh[NR_H] = {};
    int   gate_frames = 0;
    bool  gate_ready = false;
    int   gate_silent = 0;
    static constexpr int GATE_LEARN_FRAMES = 30;
    static constexpr float GATE_MULT = 2.5f;

    // MMSE/Log-MMSE: 이전 프레임 게인 (스무딩)
    float prev_gain[NR_H] = {};

    void init_win(){
        if(win_init) return;
        for(int i=0;i<NR_N;i++)
            win[i] = 0.5f * (1.0f - cosf(6.283185307f * i / (float)NR_N));
        win_init = true;
        for(int i=0;i<NR_H;i++) prev_gain[i] = 1.0f;
    }

    void reset(){
        pos=0; rd=0; avail=0;
        memset(overlap, 0, sizeof(overlap));
        memset(noise_pow, 0, sizeof(noise_pow));
        noise_frames=0; noise_ready=false; silent_count=0;
        memset(gate_thresh, 0, sizeof(gate_thresh));
        gate_frames=0; gate_ready=false; gate_silent=0;
        for(int i=0;i<NR_H;i++) prev_gain[i] = 1.0f;
    }

    float frame_energy(const float* buf, int n){
        float e=0;
        for(int i=0;i<n;i++) e += buf[i]*buf[i];
        return e / (float)n;
    }

    // 잡음 파워 학습 (SS/WF/MS/LM 공통)
    void learn_noise(const float* mag2, bool is_silent){
        if(is_silent){
            silent_count++;
            if(silent_count >= 3){
                if(noise_frames < LEARN_FRAMES){
                    for(int i=0;i<NR_H;i++)
                        noise_pow[i] += mag2[i];
                    noise_frames++;
                    if(noise_frames >= LEARN_FRAMES){
                        for(int i=0;i<NR_H;i++)
                            noise_pow[i] /= (float)LEARN_FRAMES;
                        noise_ready = true;
                    }
                } else {
                    for(int i=0;i<NR_H;i++)
                        noise_pow[i] = noise_pow[i]*0.95f + mag2[i]*0.05f;
                }
            }
        } else {
            silent_count = 0;
        }
    }

    void process_frame(int mode){
        init_win();

        float windowed[NR_N];
        for(int i=0;i<NR_N;i++) windowed[i] = in_buf[i] * win[i];

        CplxF freq[NR_H];
        nr_fft(windowed, freq);

        float mag[NR_H], mag2[NR_H];
        for(int i=0;i<NR_H;i++){
            mag2[i] = freq[i].r*freq[i].r + freq[i].i*freq[i].i;
            mag[i]  = sqrtf(mag2[i]);
        }

        float energy = frame_energy(in_buf, NR_N);
        bool is_silent = (energy < 1e-6f);

        if(mode == 2){
            // ── SG (Spectral Gate) ──
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
                        for(int i=0;i<NR_H;i++)
                            gate_thresh[i] = gate_thresh[i]*0.95f + mag[i]*GATE_MULT*0.05f;
                    }
                }
            } else {
                gate_silent = 0;
            }
            if(gate_ready){
                for(int i=0;i<NR_H;i++){
                    if(mag[i] < gate_thresh[i]){
                        float ratio = mag[i] / (gate_thresh[i] + 1e-10f);
                        float gain = ratio * ratio;
                        freq[i].r *= gain;
                        freq[i].i *= gain;
                    }
                }
            }
        } else {
            // SS/WF/MS/LM 모두 잡음 파워 학습
            learn_noise(mag2, is_silent);

            if(noise_ready){
                if(mode == 1){
                    // ── SS (Spectral Subtraction) ──
                    for(int i=0;i<NR_H;i++){
                        float new_mag = mag[i] - SS_ALPHA * sqrtf(noise_pow[i]);
                        if(new_mag < SS_FLOOR * mag[i]) new_mag = SS_FLOOR * mag[i];
                        float gain = (mag[i] > 1e-10f) ? (new_mag / mag[i]) : 0;
                        freq[i].r *= gain;
                        freq[i].i *= gain;
                    }
                }
                else if(mode == 3){
                    // ── WF (Wiener Filter) ──
                    // H(w) = max(1 - noise/signal, floor)
                    // 사후 SNR 기반 최적 게인
                    for(int i=0;i<NR_H;i++){
                        float snr_post = mag2[i] / (noise_pow[i] + 1e-10f);
                        float snr_prior = snr_post - 1.0f;
                        if(snr_prior < 0) snr_prior = 0;
                        // Decision-Directed 스무딩
                        snr_prior = 0.98f * prev_gain[i]*prev_gain[i] * (mag2[i] / (noise_pow[i]+1e-10f))
                                  + 0.02f * std::max(snr_post - 1.0f, 0.0f);
                        float gain = snr_prior / (snr_prior + 1.0f);
                        gain = std::max(gain, 0.05f);
                        prev_gain[i] = gain;
                        freq[i].r *= gain;
                        freq[i].i *= gain;
                    }
                }
                else if(mode == 4){
                    // ── MS (MMSE-STSA) ──
                    // Ephraim & Malah (1984)
                    // gain = sqrt(v/gamma) * exp(-v/2) * [(1+v)*I0(v/2) + v*I1(v/2)]
                    // 간소화 근사: gain = sqrt(v/(1+v)) * exp(-v/2) 기반
                    for(int i=0;i<NR_H;i++){
                        float gamma_k = mag2[i] / (noise_pow[i] + 1e-10f);
                        float xi = 0.98f * prev_gain[i]*prev_gain[i] * gamma_k
                                 + 0.02f * std::max(gamma_k - 1.0f, 0.0f);
                        float v = xi * gamma_k / (1.0f + xi);
                        // MMSE-STSA 게인 근사
                        float gain;
                        if(v < 0.01f){
                            gain = 0.05f;
                        } else {
                            // G = sqrt(pi*v / (2*gamma)) * exp(-v/2) * ((1+v)*I0(v/2) + v*I1(v/2))
                            // 근사: G ≈ xi / (1 + xi) * sqrt(1 + 1/max(v, 0.1))
                            gain = (xi / (1.0f + xi)) * sqrtf(1.0f + 1.0f / std::max(v, 0.1f));
                        }
                        gain = std::max(gain, 0.05f);
                        gain = std::min(gain, 1.0f);
                        prev_gain[i] = gain;
                        freq[i].r *= gain;
                        freq[i].i *= gain;
                    }
                }
                else if(mode == 5){
                    // ── LM (Log-MMSE) ──
                    // Ephraim & Malah (1985) - 로그 도메인 MMSE
                    // gain = xi/(1+xi) * exp(0.5 * E1(v))
                    for(int i=0;i<NR_H;i++){
                        float gamma_k = mag2[i] / (noise_pow[i] + 1e-10f);
                        float xi = 0.98f * prev_gain[i]*prev_gain[i] * gamma_k
                                 + 0.02f * std::max(gamma_k - 1.0f, 0.0f);
                        xi = std::max(xi, 1e-4f);
                        float v = xi * gamma_k / (1.0f + xi);
                        v = std::max(v, 1e-4f);
                        float e1 = expint_e1(v);
                        float gain = (xi / (1.0f + xi)) * expf(0.5f * e1);
                        gain = std::max(gain, 0.05f);
                        gain = std::min(gain, 1.0f);
                        prev_gain[i] = gain;
                        freq[i].r *= gain;
                        freq[i].i *= gain;
                    }
                }
            }
        }

        // IFFT + overlap-add
        float ifft_out[NR_N];
        nr_ifft(freq, ifft_out);

        for(int i=0;i<NR_H;i++)
            out_buf[i] = overlap[i] + ifft_out[i] * win[i];
        for(int i=0;i<NR_H;i++)
            overlap[i] = ifft_out[NR_H + i] * win[NR_H + i];

        rd = 0;
        avail = NR_H;
    }

    float feed(float smp, int mode){
        if(mode != prev_mode){ reset(); prev_mode = mode; }

        in_buf[pos++] = smp;
        if(pos >= NR_N){
            process_frame(mode);
            memmove(in_buf, in_buf + NR_H, NR_H * sizeof(float));
            pos = NR_H;
        }
        if(avail > 0){
            float out = out_buf[rd++];
            avail--;
            return out;
        }
        return 0;
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
