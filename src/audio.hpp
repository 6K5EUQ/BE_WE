#pragma once
#include "config.hpp"
#include <alsa/asoundlib.h>
#include <cstdio>
#include <cstdint>

// ── Stereo ALSA output ────────────────────────────────────────────────────
struct AlsaOut {
    snd_pcm_t* pcm=nullptr;

    bool open(uint32_t sr=AUDIO_SR){
        int err=snd_pcm_open(&pcm,AUDIO_DEVICE,SND_PCM_STREAM_PLAYBACK,0);
        if(err<0){ fprintf(stderr,"ALSA open: %s\n",snd_strerror(err)); return false; }

        snd_pcm_hw_params_t* hw; snd_pcm_hw_params_alloca(&hw);
        snd_pcm_hw_params_any(pcm,hw);
        snd_pcm_hw_params_set_access(pcm,hw,SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm,hw,SND_PCM_FORMAT_S16_LE);
        snd_pcm_hw_params_set_channels(pcm,hw,2);
        unsigned rsr=sr; snd_pcm_hw_params_set_rate_near(pcm,hw,&rsr,0);
        snd_pcm_uframes_t buf_sz=8192, per_sz=256;
        snd_pcm_hw_params_set_buffer_size_near(pcm,hw,&buf_sz);
        snd_pcm_hw_params_set_period_size_near(pcm,hw,&per_sz,0);
        err=snd_pcm_hw_params(pcm,hw);
        if(err<0){ fprintf(stderr,"ALSA hw: %s\n",snd_strerror(err)); snd_pcm_close(pcm); pcm=nullptr; return false; }

        snd_pcm_sw_params_t* sw; snd_pcm_sw_params_alloca(&sw);
        snd_pcm_sw_params_current(pcm,sw);
        snd_pcm_sw_params_set_start_threshold(pcm,sw,256);
        snd_pcm_sw_params_set_avail_min(pcm,sw,256);
        snd_pcm_sw_params(pcm,sw);
        printf("ALSA: %u Hz stereo\n",rsr);
        return true;
    }

    // buf = interleaved L,R,L,R,... int16 pairs; frames = number of stereo frames
    void write(const int16_t* buf, int frames){
        if(!pcm) return;
        while(frames>0){
            snd_pcm_sframes_t r=snd_pcm_writei(pcm,buf,frames);
            if(r<0){
                r=snd_pcm_recover(pcm,(int)r,0);
                if(r<0){ fprintf(stderr,"ALSA wr: %s\n",snd_strerror((int)r)); return; }
                continue;
            }
            buf+=r*2; frames-=(int)r;
        }
    }

    void close(){
        if(pcm){ snd_pcm_drain(pcm); snd_pcm_close(pcm); pcm=nullptr; }
    }
};
