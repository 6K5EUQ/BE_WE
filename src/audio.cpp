#include "fft_viewer.hpp"
#include "audio.hpp"
#include <vector>
#include <algorithm>

void FFTViewer::mix_worker(){
    AlsaOut alsa; alsa.open(AUDIO_SR);
    static constexpr int PERIOD=256;
    std::vector<int16_t> sbuf(PERIOD*2,0);

    while(!mix_stop.load(std::memory_order_relaxed)){
        bool ch_active[MAX_CHANNELS];
        int  ch_pan[MAX_CHANNELS];
        for(int c=0;c<MAX_CHANNELS;c++){
            ch_active[c]=channels[c].dem_run.load(std::memory_order_relaxed);
            ch_pan[c]   =channels[c].pan;
        }
        for(int i=0;i<PERIOD;i++){
            float L=0,R=0;
            for(int c=0;c<MAX_CHANNELS;c++){
                if(!ch_active[c]) continue;
                float smp=0; channels[c].pop_audio(smp);
                if(ch_pan[c]<=0) L+=smp;
                if(ch_pan[c]>=0) R+=smp;
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