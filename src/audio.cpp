#include "fft_viewer.hpp"
#include "audio.hpp"
#include "net_client.hpp"
#include <vector>
#include <algorithm>

void FFTViewer::mix_worker(){
    AlsaOut alsa; alsa.open(AUDIO_SR);
    static constexpr int PERIOD=256;
    std::vector<int16_t> sbuf(PERIOD*2,0);

    while(!mix_stop.load(std::memory_order_relaxed)){
        for(int i=0;i<PERIOD;i++){
            float L=0,R=0;
            if(net_cli && remote_mode){
                // ── CONNECT 모드: 네트워크 오디오 링에서 읽기 ─────────────
                for(int c=0;c<MAX_CHANNELS;c++){
                    if(local_ch_out[c]==3) { // M(mute): drain
                        float dummy; int8_t p2;
                        net_cli->audio[c].pop(dummy,p2); continue;
                    }
                    float smp=0; int8_t pan=0;
                    if(!net_cli->audio[c].pop(smp, pan)) continue;
                    int lco = local_ch_out[c]; // 0=L,1=LR,2=R
                    if(lco==0)      { L+=smp; }
                    else if(lco==2) { R+=smp; }
                    else            { L+=smp; R+=smp; }
                }
            } else {
                // ── LOCAL / HOST 모드: 로컬 채널 오디오 ──────────────────
                for(int c=0;c<MAX_CHANNELS;c++){
                    if(!channels[c].dem_run.load(std::memory_order_relaxed)) continue;
                    if(net_srv && !(channels[c].audio_mask.load() & 0x1u)) continue;
                    if(local_ch_out[c]==3) { // M(mute): drain ring
                        float dummy; channels[c].pop_audio(dummy); continue;
                    }
                    float smp=0; channels[c].pop_audio(smp);
                    int lco = local_ch_out[c]; // 0=L,1=LR,2=R
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