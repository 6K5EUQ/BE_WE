#include "fft_viewer.hpp"
#include "net_protocol.hpp"
#include <algorithm>
#include <mutex>

// 전역 DB 목록 (Central server에서 수신, ui.cpp + cli_host.cpp에서 접근)
std::vector<DbFileEntry> g_db_list;
std::mutex g_db_list_mtx;
#include <cfloat>
#include <cstdarg>
#include <ctime>

// FFTViewer::log_push - LOG overlay buffer + console
void FFTViewer::log_push(int col, const char* fmt, ...){
    if(col < 0 || col > 2) col = 0;
    char raw[480];
    va_list ap; va_start(ap, fmt);
    vsnprintf(raw, sizeof(raw), fmt, ap);
    va_end(ap);
    // strip trailing newlines
    size_t len = strlen(raw);
    while(len > 0 && (raw[len-1]=='\n'||raw[len-1]=='\r')) raw[--len]='\0';
    // timestamp prefix
    time_t now = time(nullptr);
    struct tm tm_buf; localtime_r(&now, &tm_buf);
    char ts[12]; strftime(ts, sizeof(ts), "%H:%M:%S", &tm_buf);
    char full[512];
    snprintf(full, sizeof(full), "[%s] %s", ts, raw);
    // console
    fprintf(stdout, "%s\n", full);
    fflush(stdout);
    // buffer
    std::lock_guard<std::mutex> lk(log_mtx);
    LogEntry e{}; strncpy(e.msg, full, 511);
    if(log_buf[col].size() >= (size_t)LOG_MAX) log_buf[col].erase(log_buf[col].begin());
    log_buf[col].push_back(e);
    log_scroll[col] = true;
}

// 글로벌 로그 (FFTViewer 인스턴스 없이도 호출 가능)
FFTViewer* g_log_viewer = nullptr;
void bewe_log_push(int col, const char* fmt, ...){
    char buf[512];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if(g_log_viewer) g_log_viewer->log_push(col, "%s", buf);
    else fputs(buf, stdout);
}

// FFTViewer::digi_log_push - DIGITAL DECODE overlay buffer
void FFTViewer::digi_log_push(int tab, const char* fmt, ...){
    if(tab < 0 || tab > 3) tab = 0;
    char raw[960];
    va_list ap; va_start(ap, fmt);
    vsnprintf(raw, sizeof(raw), fmt, ap);
    va_end(ap);
    size_t len = strlen(raw);
    while(len > 0 && (raw[len-1]=='\n'||raw[len-1]=='\r')) raw[--len]='\0';
    std::lock_guard<std::mutex> lk(digi_log_mtx);
    DigiLogEntry e{}; strncpy(e.msg, raw, 1023);
    if(digi_log_buf[tab].size() >= (size_t)DIGI_LOG_MAX)
        digi_log_buf[tab].erase(digi_log_buf[tab].begin());
    digi_log_buf[tab].push_back(e);
    digi_log_scroll[tab] = true;
}

void bewe_digi_push(int tab, const char* fmt, ...){
    char buf[1024];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if(g_log_viewer) g_log_viewer->digi_log_push(tab, "%s", buf);
}

// ── Jet colormap LUT (COLORMAP_LUT_SIZE entry, 한 번만 계산) ────────────
static uint32_t g_jet_lut[COLORMAP_LUT_SIZE];
static bool g_jet_init=false;
static const uint32_t* jet_lut(){
    if(!g_jet_init){
        g_jet_init=true;
        auto c=[](float v)->uint8_t{v=v<0?0:v>1?1:v;return(uint8_t)(v*255);};
        for(int i=0;i<COLORMAP_LUT_SIZE;i++){
            float t=i/(float)(COLORMAP_LUT_SIZE-1);
            float r=1.5f-fabsf(4*t-3);
            float g=1.5f-fabsf(4*t-2);
            float b=1.5f-fabsf(4*t-1);
            g_jet_lut[i]=IM_COL32(c(r),c(g),c(b),255);
        }
    }
    return g_jet_lut;
}

// 텍스처 폭: GPU 한계(보통 16384) 이내로 제한
static constexpr int WF_TEX_MAX = 16384;

// ── Waterfall texture ─────────────────────────────────────────────────────
void FFTViewer::create_waterfall_texture(){
#ifdef BEWE_HEADLESS
    int tex_w = std::min(fft_size, WF_TEX_MAX);
    if(tex_w < 1) tex_w = 1;
    wf_row_buf.assign(tex_w, 0);
    return;
#else
    if(waterfall_texture) glDeleteTextures(1,&waterfall_texture);
    int tex_w = std::min(fft_size, WF_TEX_MAX);
    if(tex_w < 1) tex_w = 1;
    glGenTextures(1,&waterfall_texture);
    glBindTexture(GL_TEXTURE_2D,waterfall_texture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,tex_w,MAX_FFTS_MEMORY,0,GL_RGBA,GL_UNSIGNED_BYTE,nullptr);
    glBindTexture(GL_TEXTURE_2D,0);
#endif
}

void FFTViewer::update_wf_row(int fi){
#ifdef BEWE_HEADLESS
    (void)fi;
    return;
#else
    std::lock_guard<std::mutex> lk(data_mtx);
    int tex_w = std::min(fft_size, WF_TEX_MAX);
    if(tex_w < 1) tex_w = 1;
    if((int)wf_row_buf.size()!=tex_w) wf_row_buf.resize(tex_w);
    int mi=fi%MAX_FFTS_MEMORY;
    const float* row=fft_data.data()+mi*fft_size;
    float wmin=display_power_min, wmax=display_power_max;
    float wrng_inv=1.0f/std::max(1.0f,wmax-wmin);
    const uint32_t* lut=jet_lut();

    int fft_half=fft_size/2;
    int tex_half=tex_w/2;

    float ratio = (float)fft_half / (float)tex_half;

    auto map_peak=[&](float bin_start_f, float bin_end_f)->uint32_t{
        int bs=std::max(0, (int)bin_start_f);
        int be=std::min(fft_size-1, (int)bin_end_f);
        float mx=-200.0f;
        for(int b=bs;b<=be;b++) if(row[b]>mx) mx=row[b];
        float t=(mx-wmin)*wrng_inv;
        int idx=(int)(t*(COLORMAP_LUT_SIZE-1));
        idx=idx<0?0:idx>=(COLORMAP_LUT_SIZE)?COLORMAP_LUT_SIZE-1:idx;
        return lut[idx];
    };

    for(int i=0;i<tex_half;i++){
        float b0 = fft_half + i * ratio;
        float b1 = fft_half + (i+1) * ratio - 1;
        wf_row_buf[i]=map_peak(b0, b1);
    }
    for(int i=0;i<tex_half;i++){
        float b0 = i * ratio;
        float b1 = (i+1) * ratio - 1;
        wf_row_buf[tex_half+i]=map_peak(b0, b1);
    }

    glBindTexture(GL_TEXTURE_2D,waterfall_texture);
    glTexSubImage2D(GL_TEXTURE_2D,0,0,mi,tex_w,1,GL_RGBA,GL_UNSIGNED_BYTE,wf_row_buf.data());
    glBindTexture(GL_TEXTURE_2D,0);
#endif
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
