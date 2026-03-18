#include "fft_viewer.hpp"
#include "audio.hpp"
#include "net_client.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>


// ── Mix worker ─────────────────────────────────────────────────────────────
void FFTViewer::mix_worker(){
    AlsaOut alsa; alsa.open(AUDIO_SR);
    static constexpr int PERIOD=256;
    std::vector<int16_t> sbuf(PERIOD*2,0);

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
