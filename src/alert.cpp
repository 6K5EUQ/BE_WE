#include "fft_viewer.hpp"
#include <algorithm>

void FFTViewer::load_alert_mp3(){
    mpg123_handle* mh=nullptr;
    mpg123_init();
    mh=mpg123_new(nullptr,nullptr);
    if(!mh){ fprintf(stderr,"Alert: mpg123_new failed\n"); return; }
    if(mpg123_open(mh,ALERT_MP3_PATH)!=MPG123_OK){
        fprintf(stderr,"Alert: cannot open %s\n",ALERT_MP3_PATH);
        mpg123_delete(mh); return;
    }
    long rate; int channels,enc;
    mpg123_getformat(mh,&rate,&channels,&enc);
    mpg123_format_none(mh);
    mpg123_format(mh,AUDIO_SR,1,MPG123_ENC_FLOAT_32); // force mono float @48kHz
    alert_pcm.clear();
    unsigned char buf[4096]; size_t done;
    while(mpg123_read(mh,buf,sizeof(buf),&done)==MPG123_OK||done>0){
        float* fp=(float*)buf;
        size_t n=done/sizeof(float);
        for(size_t i=0;i<n;i++) alert_pcm.push_back(std::max(-1.0f,std::min(1.0f,fp[i])));
        if(done==0) break;
    }
    mpg123_close(mh); mpg123_delete(mh);
    printf("Alert MP3 loaded: %zu samples (%.2fs)\n",
           alert_pcm.size(),(float)alert_pcm.size()/AUDIO_SR);
}
