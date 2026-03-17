#include "fft_viewer.hpp"
#include "audio.hpp"
#include "net_client.hpp"
#include <rnnoise.h>
#include <vector>
#include <algorithm>

// 채널별 RNNoise 상태
struct RnnCh {
    DenoiseState* st = nullptr;
    float in[480]  = {};
    float out[480] = {};
    int   pos   = 0;   // 입력 축적 위치
    int   rd    = 0;   // 출력 읽기 위치
    int   avail = 0;   // 출력에서 읽을 수 있는 샘플 수
    bool  was_on = false;

    void init()  { if(!st) st = rnnoise_create(NULL); }
    void reset() { pos=0; rd=0; avail=0; }
    void destroy(){ if(st){ rnnoise_destroy(st); st=nullptr; } }

    // dry 원본, wet 디노이즈 비율로 1샘플 처리
    float process(float smp, float wet){
        float dry_smp = smp;
        in[pos] = smp * 32767.0f;
        pos++;
        if(pos >= 480){
            rnnoise_process_frame(st, out, in);
            pos = 0;
            rd  = 0;
            avail = 480;
        }
        if(avail > 0){
            float denoised = out[rd] / 32767.0f;
            rd++; avail--;
            return dry_smp * (1.0f - wet) + denoised * wet;
        }
        return 0; // 첫 프레임 채우는 동안 무음
    }
};

void FFTViewer::mix_worker(){
    AlsaOut alsa; alsa.open(AUDIO_SR);
    static constexpr int PERIOD=256;
    std::vector<int16_t> sbuf(PERIOD*2,0);

    // 채널별 RNNoise 상태
    RnnCh rnn[MAX_CHANNELS];
    for(int c=0;c<MAX_CHANNELS;c++) rnn[c].init();

    while(!mix_stop.load(std::memory_order_relaxed)){
        if(!alsa.pcm){
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            alsa.open(AUDIO_SR);
            if(!alsa.pcm) continue;
        }

        float wet = rnn_wet;

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

                    // 채널별 RNNoise
                    bool ron = rnn_ch[c];
                    if(ron && !rnn[c].was_on) rnn[c].reset();
                    rnn[c].was_on = ron;
                    if(ron) smp = rnn[c].process(smp, wet);

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

                    // 채널별 RNNoise
                    bool ron = rnn_ch[c];
                    if(ron && !rnn[c].was_on) rnn[c].reset();
                    rnn[c].was_on = ron;
                    if(ron) smp = rnn[c].process(smp, wet);

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
    for(int c=0;c<MAX_CHANNELS;c++) rnn[c].destroy();
    alsa.close();
    printf("Mix worker exited\n");
}
