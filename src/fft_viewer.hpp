#pragma once
#include "config.hpp"
#include "channel.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <libbladeRF.h>
#include <fftw3.h>
#include <mpg123.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <algorithm>

// ── FFTViewer ─────────────────────────────────────────────────────────────
class FFTViewer {
public:
    // ── FFT / waterfall data ──────────────────────────────────────────────
    FFTHeader            header;
    std::vector<int8_t>  fft_data;
    GLuint               waterfall_texture=0;
    std::vector<uint32_t> wf_row_buf;

    int   fft_size=DEFAULT_FFT_SIZE, time_average=TIME_AVERAGE;
    bool  fft_size_change_req=false; int pending_fft_size=DEFAULT_FFT_SIZE;
    bool  texture_needs_recreate=false;
    int   current_fft_idx=0, last_wf_update_idx=-1;
    float freq_zoom=1, freq_pan=0;
    float display_power_min=-80, display_power_max=0;
    float spectrum_height_ratio=0.2f;
    std::vector<float> current_spectrum;
    int   cached_sp_idx=-1; float cached_pan=-999, cached_zoom=-999;
    int   cached_px=-1;     float cached_pmin=-999, cached_pmax=-999;
    std::vector<float> autoscale_accum;
    std::chrono::steady_clock::time_point autoscale_last;
    bool  autoscale_init=false, autoscale_active=true;
    std::atomic<bool> spectrum_pause{false};

    // ── BladeRF ───────────────────────────────────────────────────────────
    struct bladerf* dev=nullptr;
    fftwf_plan      fft_plan=nullptr;
    fftwf_complex  *fft_in=nullptr, *fft_out=nullptr;
    bool  is_running=true;
    int   total_ffts=0;
    std::string window_title;
    std::mutex  data_mtx;
    float pending_cf=0; bool freq_req=false, freq_prog=false;

    // ── IQ Ring ───────────────────────────────────────────────────────────
    std::vector<int16_t> ring;
    std::atomic<size_t>  ring_wp{0};

    // ── Channels ──────────────────────────────────────────────────────────
    Channel channels[MAX_CHANNELS];
    int     selected_ch=-1;
    bool    topbar_sel_this_frame=false;

    struct NewDrag{ bool active=false; float anch=0,s=0,e=0; } new_drag;

    // ── IQ Recording ──────────────────────────────────────────────────────
    std::atomic<bool>     rec_on{false}, rec_stop{false};
    std::thread           rec_thr;
    std::atomic<size_t>   rec_rp{0};
    float                 rec_cf_mhz=0;
    uint32_t              rec_sr=0;
    int                   rec_ch=-1;
    std::string           rec_filename;
    std::atomic<uint64_t> rec_frames{0};
    std::chrono::steady_clock::time_point rec_t0;

    // ── Audio mix ─────────────────────────────────────────────────────────
    std::atomic<bool> mix_stop{false};
    std::thread       mix_thr;

    // ── Detection alert ───────────────────────────────────────────────────
    // ★ MP3 경로를 여기서 변경하세요:
    static constexpr const char* ALERT_MP3_PATH = "/home/dsa/BE_WE/lock_on.mp3";
    std::vector<float> alert_pcm;

    // ── alert.cpp ─────────────────────────────────────────────────────────
    void load_alert_mp3();

    // ── bladerf_io.cpp ────────────────────────────────────────────────────
    bool initialize_bladerf(float cf_mhz, float sr_msps);
    void capture_and_process();

    // ── demod.cpp ─────────────────────────────────────────────────────────
    void dem_worker(int ch_idx);
    void start_dem(int ch_idx, Channel::DemodMode mode);
    void stop_dem(int ch_idx);
    void stop_all_dem();

    // ── iq_record.cpp ─────────────────────────────────────────────────────
    void rec_worker();
    void start_rec();
    void stop_rec();

    // ── audio.cpp ─────────────────────────────────────────────────────────
    void mix_worker();

    // ── fft_viewer.cpp (waterfall + display helpers) ──────────────────────
    void create_waterfall_texture();
    void update_wf_row(int fi);
    void get_disp(float& ds, float& de) const;
    float x_to_abs(float x, float gx, float gw) const;
    float abs_to_x(float abs_mhz, float gx, float gw) const;
    int   channel_at_x(float mx, float gx, float gw) const;

    // ── ui.cpp ────────────────────────────────────────────────────────────
    void handle_new_channel_drag(float gx, float gw);
    void handle_channel_interactions(float gx, float gw, float gy, float gh);
    void draw_all_channels(ImDrawList* dl, float gx, float gw, float gy, float gh, bool show_label);
    void draw_freq_axis(ImDrawList* dl, float gx, float gw, float gy, float gh, bool ticks_only=false);
    void handle_zoom_scroll(float gx, float gw, float mouse_x);
    void draw_spectrum_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h);
    void draw_waterfall_area(ImDrawList* dl, float full_x, float full_y, float total_w, float total_h);
};

// ── Entry point ───────────────────────────────────────────────────────────
void run_streaming_viewer();
