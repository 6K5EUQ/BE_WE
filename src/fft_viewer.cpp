#include "fft_viewer.hpp"
#include <algorithm>

// ── Jet colormap LUT (256 entry, 한 번만 계산) ────────────────────────────
static const uint32_t* jet_lut(){
    static uint32_t lut[256];
    static bool init=false;
    if(!init){
        init=true;
        auto c=[](float v)->uint8_t{v=v<0?0:v>1?1:v;return(uint8_t)(v*255);};
        for(int i=0;i<256;i++){
            float t=i/255.0f;
            float r=1.5f-fabsf(4*t-3);
            float g=1.5f-fabsf(4*t-2);
            float b=1.5f-fabsf(4*t-1);
            lut[i]=IM_COL32(c(r),c(g),c(b),255);
        }
    }
    return lut;
}

// ── Waterfall texture ─────────────────────────────────────────────────────
void FFTViewer::create_waterfall_texture(){
    if(waterfall_texture) glDeleteTextures(1,&waterfall_texture);
    glGenTextures(1,&waterfall_texture);
    glBindTexture(GL_TEXTURE_2D,waterfall_texture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    // nullptr로 초기화: 큰 fft_size(16384)에서 163MB 임시 벡터 생성 방지
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,fft_size,MAX_FFTS_MEMORY,0,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);
    glBindTexture(GL_TEXTURE_2D,0);
}

void FFTViewer::update_wf_row(int fi){
    if((int)wf_row_buf.size()!=fft_size) wf_row_buf.resize(fft_size);
    int mi=fi%MAX_FFTS_MEMORY;
    const int8_t* row=fft_data.data()+mi*fft_size;
    float wmin=display_power_min, wmax=display_power_max;
    float wrng_inv=1.0f/std::max(1.0f,wmax-wmin);
    float pscale=(header.power_max-header.power_min)/127.0f;
    float pbase=header.power_min;
    int half=fft_size/2;
    const uint32_t* lut=jet_lut();
    // norm → LUT 인덱스 변환 (float→int 1회, fabsf 연산 제거)
    auto map=[&](int bin)->uint32_t{
        float p=row[bin]*pscale+pbase;
        float v=(p-wmin)*wrng_inv;
        int idx=(int)(v*255.0f);
        idx=idx<0?0:idx>255?255:idx;
        return lut[idx];
    };
    for(int i=0;i<half;i++) wf_row_buf[i]=map(half+i);
    for(int i=0;i<half;i++) wf_row_buf[half+i]=map(i);
    glBindTexture(GL_TEXTURE_2D,waterfall_texture);
    glTexSubImage2D(GL_TEXTURE_2D,0,0,mi,fft_size,1,GL_RGBA,GL_UNSIGNED_BYTE,wf_row_buf.data());
    glBindTexture(GL_TEXTURE_2D,0);
}

// ── Display helpers ───────────────────────────────────────────────────────
void FFTViewer::get_disp(float& ds, float& de) const {
    float nyq=header.sample_rate/2.0f/1e6f, eff=nyq*0.875f, rng=2*eff;
    ds=-eff+freq_pan*rng; de=ds+rng/freq_zoom;
    ds=std::max(-eff,ds); de=std::min(eff,de);
}

float FFTViewer::x_to_abs(float x, float gx, float gw) const {
    float ds,de; get_disp(ds,de);
    float nm=std::max(0.0f,std::min(1.0f,(x-gx)/gw));
    return (float)(header.center_frequency/1e6f)+ds+nm*(de-ds);
}

float FFTViewer::abs_to_x(float abs_mhz, float gx, float gw) const {
    float cf=header.center_frequency/1e6f; float ds,de; get_disp(ds,de);
    return gx+(abs_mhz-cf-ds)/(de-ds)*gw;
}

int FFTViewer::channel_at_x(float mx, float gx, float gw) const {
    for(int i=0;i<MAX_CHANNELS;i++){
        if(!channels[i].filter_active) continue;
        float x0=abs_to_x(std::min(channels[i].s,channels[i].e),gx,gw);
        float x1=abs_to_x(std::max(channels[i].s,channels[i].e),gx,gw);
        x0=std::max(x0,gx); x1=std::min(x1,gx+gw);
        if(mx>=x0&&mx<=x1) return i;
    }
    return -1;
}