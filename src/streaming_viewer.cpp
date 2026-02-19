#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <libbladeRF.h>
#include <fftw3.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <time.h>

#define RX_GAIN 30
#define CHANNEL BLADERF_CHANNEL_RX(0)
#define DEFAULT_FFT_SIZE 8192
#define TIME_AVERAGE 50
#define MAX_FFTS_MEMORY 1000
#define FFT_UPDATE_FPS 15
#define FFT_UPDATE_INTERVAL_MS (1000 / FFT_UPDATE_FPS)

#define HANN_WINDOW_CORRECTION 2.67f

#define AXIS_LABEL_WIDTH 50
#define SLIDER_WIDTH 40
#define BOTTOM_LABEL_HEIGHT 30

// IQ ring buffer capacity (power of 2, complex samples)
#define IQ_RING_CAPACITY (1 << 22)  // 4M samples = ~16MB, ~65ms at 61.44MSPS

// BW/SR tables (6 options)
static const int     IQ_BW_KHZ[6]   = {10, 30, 50, 100, 500, 1000};
static const uint32_t IQ_SR_TABLE[6] = {25000, 75000, 125000, 200000, 1000000, 2000000};
static const char*  IQ_BW_LABELS[6] = {"10 kHz","30 kHz","50 kHz","100 kHz","500 kHz","1 MHz"};

// ─────────────────────────────────────────────────────────────────────────────
struct FFTHeader {
    char magic[4];
    uint32_t version;
    uint32_t fft_size;
    uint32_t sample_rate;
    uint64_t center_frequency;
    uint32_t num_ffts;
    uint32_t time_average;
    float power_min;
    float power_max;
    float reserved[8];
};

// ─────────────────────────────────────────────────────────────────────────────
// Simple WAV writer (16-bit stereo PCM, I=left Q=right)
struct WAVWriter {
    FILE*    fp         = nullptr;
    uint32_t sample_rate = 0;
    uint64_t num_samples = 0;   // stereo frames written

    std::vector<int16_t> buf;
    static constexpr size_t BUF_FRAMES = 65536;

    bool open(const std::string& filename, uint32_t sr) {
        fp = fopen(filename.c_str(), "wb");
        if (!fp) return false;
        sample_rate = sr;
        num_samples = 0;
        buf.reserve(BUF_FRAMES * 2);
        write_header_impl();
        return true;
    }

    void push_sample(int16_t i, int16_t q) {
        buf.push_back(i);
        buf.push_back(q);
        num_samples++;
        if (buf.size() >= BUF_FRAMES * 2) flush_buf();
    }

    void flush_buf() {
        if (!fp || buf.empty()) return;
        fwrite(buf.data(), 2, buf.size(), fp);
        buf.clear();
    }

    void close() {
        flush_buf();
        if (!fp) return;
        fseek(fp, 0, SEEK_SET);
        write_header_impl();
        fclose(fp);
        fp = nullptr;
    }

private:
    void write_header_impl() {
        auto w32 = [&](uint32_t v){ fwrite(&v, 4, 1, fp); };
        auto w16 = [&](uint16_t v){ fwrite(&v, 2, 1, fp); };

        uint32_t data_bytes  = (uint32_t)(num_samples * 4);
        uint32_t riff_size   = 36 + data_bytes;
        uint16_t num_ch      = 2;
        uint16_t bps         = 16;
        uint32_t byte_rate   = sample_rate * 4;
        uint16_t blk_align   = 4;
        uint16_t fmt_pcm     = 1;

        fwrite("RIFF", 1, 4, fp); w32(riff_size);
        fwrite("WAVE", 1, 4, fp);
        fwrite("fmt ", 1, 4, fp); w32(16);
        w16(fmt_pcm); w16(num_ch); w32(sample_rate); w32(byte_rate);
        w16(blk_align); w16(bps);
        fwrite("data", 1, 4, fp); w32(data_bytes);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
void apply_hann_window(fftwf_complex *fft_in, int fft_size) {
    for (int i = 0; i < fft_size; i++) {
        float window = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (fft_size - 1)));
        fft_in[i][0] *= window;
        fft_in[i][1] *= window;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
class FFTViewer {
public:
    FFTHeader header;
    std::vector<int8_t>  fft_data;
    std::vector<float>   waterfall_texture_data;
    GLuint waterfall_texture = 0;

    int   fft_size   = DEFAULT_FFT_SIZE;
    int   time_average = TIME_AVERAGE;
    bool  fft_size_change_requested = false;
    int   pending_fft_size = DEFAULT_FFT_SIZE;
    bool  texture_needs_recreate = false;

    int   current_fft_idx = 0;
    int   last_waterfall_update_idx = -1;
    int   fft_index_step = 1;
    float freq_zoom = 1.0f;
    float freq_pan  = 0.0f;
    float display_power_min = 0.0f;
    float display_power_max = 0.0f;
    float spectrum_height_ratio = 0.2f;

    bool   is_playing = false;
    bool   is_looping = false;
    std::chrono::steady_clock::time_point play_start_time;
    double total_duration = 0.0;

    std::vector<float> current_spectrum;
    int   cached_spectrum_idx    = -1;
    float cached_spectrum_freq_pan  = -999.0f;
    float cached_spectrum_freq_zoom = -999.0f;
    int   cached_spectrum_pixels = -1;
    float cached_spectrum_power_min = -999.0f;
    float cached_spectrum_power_max = -999.0f;

    std::vector<float> autoscale_accum;
    std::chrono::steady_clock::time_point autoscale_last_update;
    bool autoscale_initialized = false;
    bool autoscale_active = true;

    std::chrono::steady_clock::time_point last_input_time;
    std::chrono::steady_clock::time_point last_fft_update_time;
    bool high_fps_mode = true;

    struct bladerf *dev = nullptr;
    fftwf_plan fft_plan = nullptr;
    fftwf_complex *fft_in  = nullptr;
    fftwf_complex *fft_out = nullptr;
    bool is_running = true;
    int  total_ffts_captured = 0;

    std::string window_title;
    std::mutex  data_mutex;
    int pending_new_fft_idx = -1;

    float pending_center_freq   = 0.0f;
    bool  freq_change_requested    = false;
    bool  freq_change_in_progress  = false;

    enum ColorMapType { COLORMAP_JET=0, COLORMAP_COOL=1, COLORMAP_HOT=2, COLORMAP_VIRIDIS=3 };
    ColorMapType color_map = COLORMAP_COOL;

    // ── IQ Selection ────────────────────────────────────────────────────────
    struct IQSelection {
        bool  active          = false;
        bool  dragging        = false;
        float start_mhz       = 0.0f;  // absolute frequency
        float end_mhz         = 0.0f;
        float drag_anchor_mhz = 0.0f;
    };
    IQSelection iq_sel;

    int iq_bw_idx = 3;   // default 100 kHz

    // ── IQ Recording ────────────────────────────────────────────────────────
    std::atomic<bool>     iq_recording{false};
    std::atomic<bool>     iq_record_stop{false};
    std::thread           iq_record_thread;

    // Lock-free ring buffer (raw IQ from BladeRF)
    std::vector<int16_t>  iq_ring_buf;      // IQ_RING_CAPACITY × 2 int16_t
    std::atomic<size_t>   iq_ring_write_pos{0};
    std::atomic<size_t>   iq_ring_read_pos{0};

    float       iq_record_center_mhz  = 0.0f;
    uint32_t    iq_record_target_sr   = 0;
    std::string iq_record_filename;
    std::atomic<uint64_t> iq_samples_written{0};

    // ── initialize_bladerf ──────────────────────────────────────────────────
    bool initialize_bladerf(float center_freq_mhz, float sample_rate_msps) {
        int status = bladerf_open(&dev, nullptr);
        if (status != 0) {
            fprintf(stderr, "Failed to open device: %s\n", bladerf_strerror(status));
            return false;
        }

        status = bladerf_set_frequency(dev, CHANNEL, (uint64_t)(center_freq_mhz * 1e6));
        if (status != 0) {
            fprintf(stderr, "Failed to set frequency: %s\n", bladerf_strerror(status));
            bladerf_close(dev); return false;
        }

        uint32_t actual_rate = 0;
        status = bladerf_set_sample_rate(dev, CHANNEL, (uint32_t)(sample_rate_msps * 1e6), &actual_rate);
        if (status != 0) {
            fprintf(stderr, "Failed to set sample rate: %s\n", bladerf_strerror(status));
            bladerf_close(dev); return false;
        }
        if (actual_rate != (uint32_t)(sample_rate_msps * 1e6))
            fprintf(stderr, "Warning: Requested %.2f MSPS, got %.2f MSPS\n",
                    sample_rate_msps, actual_rate / 1e6f);

        uint32_t bw = (uint32_t)(sample_rate_msps * 1e6 * 0.8f);
        status = bladerf_set_bandwidth(dev, CHANNEL, bw, nullptr);
        if (status != 0) {
            fprintf(stderr, "Failed to set bandwidth: %s\n", bladerf_strerror(status));
            bladerf_close(dev); return false;
        }

        status = bladerf_set_gain(dev, CHANNEL, RX_GAIN);
        if (status != 0) {
            fprintf(stderr, "Failed to set gain: %s\n", bladerf_strerror(status));
            bladerf_close(dev); return false;
        }

        status = bladerf_enable_module(dev, CHANNEL, true);
        if (status != 0) {
            fprintf(stderr, "Failed to enable RX: %s\n", bladerf_strerror(status));
            bladerf_close(dev); return false;
        }

        status = bladerf_sync_config(dev, BLADERF_RX_X1, BLADERF_FORMAT_SC16_Q11,
                                     512, 16384, 128, 10000);
        if (status != 0) {
            fprintf(stderr, "Failed to configure sync: %s\n", bladerf_strerror(status));
            bladerf_close(dev); return false;
        }

        printf("BladeRF initialized: %.2f MHz, %.2f MSPS\n", center_freq_mhz, sample_rate_msps);

        std::memcpy(header.magic, "FFTD", 4);
        header.version          = 1;
        header.fft_size         = fft_size;
        header.sample_rate      = actual_rate;
        header.center_frequency = (uint64_t)(center_freq_mhz * 1e6);
        header.time_average     = TIME_AVERAGE;
        header.power_min        = -80.0f;
        header.power_max        = -30.0f;
        header.num_ffts         = 0;

        fft_data.resize(MAX_FFTS_MEMORY * fft_size);
        waterfall_texture_data.resize(MAX_FFTS_MEMORY * fft_size, 0.0f);
        current_spectrum.resize(fft_size, -80.0f);

        char title[256];
        snprintf(title, sizeof(title), "Real-time FFT Viewer - %.2f MHz", center_freq_mhz);
        window_title = title;

        display_power_min = -80.0f;
        display_power_max =   0.0f;

        fft_in  = fftwf_alloc_complex(fft_size);
        fft_out = fftwf_alloc_complex(fft_size);
        fft_plan = fftwf_plan_dft_1d(fft_size, fft_in, fft_out, FFTW_FORWARD, FFTW_MEASURE);

        last_input_time    = std::chrono::steady_clock::now();
        last_fft_update_time = std::chrono::steady_clock::now();

        // Allocate IQ ring buffer
        iq_ring_buf.resize(IQ_RING_CAPACITY * 2, 0);

        return true;
    }

    // ── create_waterfall_texture ────────────────────────────────────────────
    void create_waterfall_texture() {
        if (waterfall_texture != 0) glDeleteTextures(1, &waterfall_texture);
        glGenTextures(1, &waterfall_texture);
        glBindTexture(GL_TEXTURE_2D, waterfall_texture);
        std::vector<uint32_t> init(fft_size * MAX_FFTS_MEMORY, IM_COL32(0,0,0,255));
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, fft_size, MAX_FFTS_MEMORY,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, init.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    // ── update_waterfall_row ────────────────────────────────────────────────
    void update_waterfall_row(int fft_idx) {
        if (waterfall_texture == 0) return;
        int mem_idx = fft_idx % MAX_FFTS_MEMORY;
        int8_t *fft_row = fft_data.data() + mem_idx * fft_size;
        std::vector<uint32_t> row_u32(fft_size);

        auto jet = [](float v) -> uint32_t {
            float r, g, b;
            if      (v < 0.04f) { r=0; g=0; b=v/0.04f*0.5f; }
            else if (v < 0.15f) { float t=(v-0.04f)/0.11f; r=0; g=0; b=0.5f+t*0.5f; }
            else if (v < 0.35f) { float t=(v-0.15f)/0.2f;  r=0; g=t; b=1.0f-t*0.3f; }
            else if (v < 0.55f) { float t=(v-0.35f)/0.2f;  r=t; g=1; b=0; }
            else if (v < 0.75f) { float t=(v-0.55f)/0.2f;  r=1; g=1.0f-t*0.5f; b=0; }
            else if (v < 0.95f) { float t=(v-0.75f)/0.2f;  r=1; g=0.5f-t*0.5f; b=0; }
            else                { float t=(v-0.95f)/0.05f;  r=1; g=t*0.3f; b=t*0.3f; }
            return IM_COL32((uint8_t)(r*255),(uint8_t)(g*255),(uint8_t)(b*255),255);
        };

        float wf_min   = display_power_min;
        float wf_range = display_power_max - wf_min;
        if (wf_range < 1.0f) wf_range = 1.0f;
        int half = fft_size / 2;

        for (int i = 0; i < half; i++) {
            int bin = half + 1 + i;
            float p = (fft_row[bin]/127.0f)*(header.power_max-header.power_min)+header.power_min;
            float n = std::max(0.0f,std::min(1.0f,(p-wf_min)/wf_range));
            row_u32[i] = jet(n);
        }
        float p0 = (fft_row[0]/127.0f)*(header.power_max-header.power_min)+header.power_min;
        row_u32[half] = jet(std::max(0.0f,std::min(1.0f,(p0-wf_min)/wf_range)));
        for (int i = 1; i <= half; i++) {
            float p = (fft_row[i]/127.0f)*(header.power_max-header.power_min)+header.power_min;
            float n = std::max(0.0f,std::min(1.0f,(p-wf_min)/wf_range));
            row_u32[half+i] = jet(n);
        }

        glBindTexture(GL_TEXTURE_2D, waterfall_texture);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, mem_idx, fft_size, 1,
                        GL_RGBA, GL_UNSIGNED_BYTE, row_u32.data());
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    // ── capture_and_process ─────────────────────────────────────────────────
    void capture_and_process() {
        int16_t *iq_buffer = new int16_t[fft_size * 2];
        std::vector<float> power_accum(fft_size, 0.0f);
        int fft_count = 0;

        while (is_running) {
            // FFT size change
            if (fft_size_change_requested) {
                fft_size_change_requested = false;
                int new_size = pending_fft_size;
                fftwf_destroy_plan(fft_plan);
                fftwf_free(fft_in); fftwf_free(fft_out);
                fft_size   = new_size;
                time_average = TIME_AVERAGE * DEFAULT_FFT_SIZE / fft_size;
                if (time_average < 1) time_average = 1;
                fft_in  = fftwf_alloc_complex(fft_size);
                fft_out = fftwf_alloc_complex(fft_size);
                fft_plan = fftwf_plan_dft_1d(fft_size,fft_in,fft_out,FFTW_FORWARD,FFTW_MEASURE);
                delete[] iq_buffer;
                iq_buffer = new int16_t[fft_size * 2];
                power_accum.assign(fft_size, 0.0f);
                fft_count = 0;
                {
                    std::lock_guard<std::mutex> lk(data_mutex);
                    header.fft_size = fft_size;
                    fft_data.assign(MAX_FFTS_MEMORY * fft_size, 0);
                    waterfall_texture_data.assign(MAX_FFTS_MEMORY * fft_size, 0.0f);
                    current_spectrum.assign(fft_size, -80.0f);
                    total_ffts_captured = 0; current_fft_idx = 0;
                    cached_spectrum_idx = -1;
                    autoscale_accum.clear(); autoscale_initialized=false; autoscale_active=true;
                }
                printf("FFT size changed to %d\n", fft_size);
                texture_needs_recreate = true;
                continue;
            }

            // Frequency change
            if (freq_change_requested && !freq_change_in_progress) {
                freq_change_in_progress = true;
                int s = bladerf_set_frequency(dev, CHANNEL, (uint64_t)(pending_center_freq*1e6));
                if (s == 0) {
                    {
                        std::lock_guard<std::mutex> lk(data_mutex);
                        header.center_frequency = (uint64_t)(pending_center_freq*1e6);
                    }
                    printf("Frequency changed to: %.2f MHz\n", pending_center_freq);
                    char t[256]; snprintf(t,sizeof(t),"Real-time FFT Viewer - %.2f MHz",pending_center_freq);
                    window_title = t;
                    autoscale_accum.clear(); autoscale_initialized=false; autoscale_active=true;
                } else {
                    fprintf(stderr,"Failed to change frequency: %s\n",bladerf_strerror(s));
                }
                freq_change_requested = false;
                freq_change_in_progress = false;
            }

            int status = bladerf_sync_rx(dev, iq_buffer, fft_size, nullptr, 10000);
            if (status != 0) {
                fprintf(stderr,"RX error: %s\n",bladerf_strerror(status));
                continue;
            }

            // ── Push raw IQ to ring buffer for IQ recording ──────────────
            if (iq_recording.load(std::memory_order_relaxed)) {
                for (int i = 0; i < fft_size; i++) {
                    size_t wp   = iq_ring_write_pos.load(std::memory_order_relaxed);
                    size_t next = (wp + 1) & (IQ_RING_CAPACITY - 1);
                    if (next != iq_ring_read_pos.load(std::memory_order_acquire)) {
                        iq_ring_buf[wp * 2]     = iq_buffer[i * 2];
                        iq_ring_buf[wp * 2 + 1] = iq_buffer[i * 2 + 1];
                        iq_ring_write_pos.store(next, std::memory_order_release);
                    }
                    // overflow: drop sample silently
                }
            }

            // ── FFT processing ───────────────────────────────────────────
            for (int i = 0; i < fft_size; i++) {
                fft_in[i][0] = iq_buffer[i*2]   / 2048.0f;
                fft_in[i][1] = iq_buffer[i*2+1] / 2048.0f;
            }
            apply_hann_window(fft_in, fft_size);
            fftwf_execute(fft_plan);

            for (int i = 0; i < fft_size; i++) {
                float ms = fft_out[i][0]*fft_out[i][0] + fft_out[i][1]*fft_out[i][1];
                float np = ms / (float)(fft_size * fft_size) * HANN_WINDOW_CORRECTION;
                power_accum[i] += 10.0f * log10f(np + 1e-10f);
            }
            power_accum[0] = (power_accum[1] + power_accum[fft_size-1]) / 2.0f;
            fft_count++;

            if (fft_count >= time_average) {
                int fft_idx = total_ffts_captured % MAX_FFTS_MEMORY;
                int8_t *row = fft_data.data() + fft_idx * fft_size;
                {
                    std::lock_guard<std::mutex> lk(data_mutex);
                    for (int i = 0; i < fft_size; i++) {
                        float avg = power_accum[i] / fft_count;
                        float n = (avg - header.power_min) / (header.power_max - header.power_min);
                        n = std::max(-1.0f, std::min(1.0f, n));
                        row[i] = (int8_t)(n * 127);
                        current_spectrum[i] = avg;
                    }
                    if (autoscale_active) {
                        if (!autoscale_initialized) {
                            autoscale_accum.reserve(fft_size * 200);
                            autoscale_last_update = std::chrono::steady_clock::now();
                            autoscale_initialized = true;
                        }
                        for (int i = 1; i < fft_size; i++) autoscale_accum.push_back(current_spectrum[i]);
                        auto now = std::chrono::steady_clock::now();
                        float el = std::chrono::duration<float>(now-autoscale_last_update).count();
                        if (el >= 1.0f && !autoscale_accum.empty()) {
                            size_t idx = (size_t)(autoscale_accum.size() * 0.15f);
                            std::nth_element(autoscale_accum.begin(),
                                             autoscale_accum.begin()+idx,
                                             autoscale_accum.end());
                            display_power_min = autoscale_accum[idx] - 10.0f;
                            autoscale_accum.clear();
                            autoscale_active = false;
                            cached_spectrum_idx = -1;
                        }
                    }
                    total_ffts_captured++;
                    current_fft_idx = total_ffts_captured - 1;
                    header.num_ffts = std::min(total_ffts_captured, MAX_FFTS_MEMORY);
                    cached_spectrum_idx = -1;
                }
                std::fill(power_accum.begin(), power_accum.end(), 0.0f);
                fft_count = 0;
            }
        }
        delete[] iq_buffer;
    }

    // ── iq_record_worker ────────────────────────────────────────────────────
    void iq_record_worker() {
        uint32_t main_sr   = header.sample_rate;
        uint32_t target_sr = iq_record_target_sr;
        int      decim     = std::max(1, (int)(main_sr / target_sr));
        uint32_t actual_sr = main_sr / (uint32_t)decim;

        float offset_hz = (iq_record_center_mhz - (float)(header.center_frequency / 1e6f)) * 1e6f;

        WAVWriter wav;
        if (!wav.open(iq_record_filename, actual_sr)) {
            fprintf(stderr, "IQ record: cannot open %s\n", iq_record_filename.c_str());
            iq_recording.store(false);
            return;
        }

        printf("IQ record worker: center=%.3f MHz, offset=%.0f Hz, decim=%d, SR=%u SPS\n",
               iq_record_center_mhz, offset_hz, decim, actual_sr);

        double phase      = 0.0;
        double phase_step = -2.0 * M_PI * (double)offset_hz / (double)main_sr;

        float accum_i = 0.0f, accum_q = 0.0f;
        int   count   = 0;

        while (!iq_record_stop.load(std::memory_order_relaxed)) {
            size_t wp = iq_ring_write_pos.load(std::memory_order_acquire);
            size_t rp = iq_ring_read_pos.load(std::memory_order_relaxed);

            if (rp == wp) {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
                continue;
            }

            // Batch-drain available samples
            size_t avail   = (wp - rp) & (IQ_RING_CAPACITY - 1);
            size_t process = std::min(avail, (size_t)65536);

            for (size_t s = 0; s < process; s++) {
                size_t pos = (rp + s) & (IQ_RING_CAPACITY - 1);
                int16_t i_raw = iq_ring_buf[pos * 2];
                int16_t q_raw = iq_ring_buf[pos * 2 + 1];

                float i_f = (float)i_raw / 2048.0f;
                float q_f = (float)q_raw / 2048.0f;

                float cp = (float)cos(phase);
                float sp = (float)sin(phase);
                float i_mix = i_f * cp - q_f * sp;
                float q_mix = i_f * sp + q_f * cp;

                phase += phase_step;
                if (phase >  M_PI) phase -= 2.0 * M_PI;
                if (phase < -M_PI) phase += 2.0 * M_PI;

                accum_i += i_mix;
                accum_q += q_mix;
                count++;

                if (count >= decim) {
                    float ai = accum_i / (float)decim;
                    float aq = accum_q / (float)decim;
                    auto clamp16 = [](float v) -> int16_t {
                        if (v >  1.0f) v =  1.0f;
                        if (v < -1.0f) v = -1.0f;
                        return (int16_t)(v * 32767.0f);
                    };
                    wav.push_sample(clamp16(ai), clamp16(aq));
                    iq_samples_written.fetch_add(1, std::memory_order_relaxed);
                    accum_i = accum_q = 0.0f;
                    count = 0;
                }
            }

            iq_ring_read_pos.store((rp + process) & (IQ_RING_CAPACITY - 1),
                                   std::memory_order_release);
        }

        wav.close();
        printf("IQ record done: %s  (%llu frames)\n",
               iq_record_filename.c_str(),
               (unsigned long long)iq_samples_written.load());
    }

    // ── start_iq_recording ──────────────────────────────────────────────────
    void start_iq_recording() {
        if (iq_recording.load()) return;
        if (!iq_sel.active) return;

        iq_record_center_mhz = (iq_sel.start_mhz + iq_sel.end_mhz) / 2.0f;
        iq_record_target_sr  = IQ_SR_TABLE[iq_bw_idx];

        // Generate filename
        time_t t = time(nullptr);
        struct tm tmi;
        localtime_r(&t, &tmi);
        char fname[256];
        snprintf(fname, sizeof(fname),
                 "iq_%.3fMHz_%dkHz_%04d%02d%02d_%02d%02d%02d.wav",
                 iq_record_center_mhz, IQ_BW_KHZ[iq_bw_idx],
                 tmi.tm_year+1900, tmi.tm_mon+1, tmi.tm_mday,
                 tmi.tm_hour, tmi.tm_min, tmi.tm_sec);
        iq_record_filename = fname;

        iq_samples_written.store(0);
        // Sync ring read to current write so worker only processes new data
        iq_ring_read_pos.store(iq_ring_write_pos.load());

        iq_record_stop.store(false);
        iq_recording.store(true);
        iq_record_thread = std::thread(&FFTViewer::iq_record_worker, this);

        printf("IQ Recording started → %s\n", fname);
    }

    // ── stop_iq_recording ───────────────────────────────────────────────────
    void stop_iq_recording() {
        if (!iq_recording.load()) return;
        iq_record_stop.store(true);
        if (iq_record_thread.joinable()) iq_record_thread.join();
        iq_recording.store(false);
        printf("IQ Recording stopped. Frames: %llu\n",
               (unsigned long long)iq_samples_written.load());
    }

    // ── Color helpers ────────────────────────────────────────────────────────
    ImU32 get_color(float value) {
        if (value < 0.0f) value = 0.0f;
        if (value > 1.0f) value = 1.0f;
        float r=0,g=0,b=0;
        switch(color_map) {
            case COLORMAP_JET: {
                float h=value*240.0f/360.0f,s=1,v=value;
                float c=v*s,x=c*(1-fabsf(fmodf(h*6,2)-1)),m=v-c;
                if(h<1/6.f){r=c;g=x;} else if(h<2/6.f){r=x;g=c;}
                else if(h<3/6.f){g=c;b=x;} else if(h<4/6.f){g=x;b=c;}
                else if(h<5/6.f){r=x;b=c;} else{r=c;b=x;}
                r+=m;g+=m;b+=m; break;
            }
            case COLORMAP_COOL:
                r=value*0.3f; g=value*0.7f+0.2f; b=0.8f+value*0.2f; break;
            case COLORMAP_HOT:
                if(value<0.33f){r=value*3;}
                else if(value<0.67f){r=1;g=(value-0.33f)*3;}
                else{r=1;g=1;b=(value-0.67f)*3;} break;
            case COLORMAP_VIRIDIS:
                if(value<0.25f){r=0.267+value*0.5f;g=0.004+value*0.2f;b=0.329+value*0.8f;}
                else if(value<0.5f){r=0.293+(value-0.25f)*0.4f;g=0.058+(value-0.25f)*0.8f;b=0.633-(value-0.25f)*1.0f;}
                else if(value<0.75f){r=0.553+(value-0.5f)*1.0f;g=0.258+(value-0.5f)*1.5f;b=0.029+(value-0.5f)*0.2f;}
                else{r=0.993;g=0.906+(value-0.75f)*0.2f;b=0.145;} break;
        }
        return IM_COL32((ImU32)(r*255),(ImU32)(g*255),(ImU32)(b*255),255);
    }

    // ── compute_spectrum_line ───────────────────────────────────────────────
    void compute_spectrum_line(int num_pixels, float sr_mhz,
                               float disp_start, float disp_end) {
        current_spectrum.assign(num_pixels, -80.0f);
        float nyquist = sr_mhz / 2.0f;
        int half_fft  = header.fft_size / 2;
        int mem_idx   = current_fft_idx % MAX_FFTS_MEMORY;
        for (int px = 0; px < num_pixels; px++) {
            float fn  = (float)px / num_pixels;
            float fd  = disp_start + fn * (disp_end - disp_start);
            int bin;
            if (fd >= 0.0f) bin = (int)((fd / nyquist) * half_fft);
            else             bin = fft_size + (int)((fd / nyquist) * half_fft);
            if (bin >= 0 && bin < fft_size) {
                int8_t raw = fft_data[mem_idx * fft_size + bin];
                current_spectrum[px] = (raw/127.0f)*(header.power_max-header.power_min)+header.power_min;
            }
        }
    }

    // ── Shared freq conversion ──────────────────────────────────────────────
    // Returns absolute frequency (MHz) from screen X inside graph area
    float x_to_abs_freq(float x, float graph_x, float graph_w) const {
        float nyq    = header.sample_rate / 2.0f / 1e6f;
        float eff    = nyq * 0.875f;
        float range  = 2.0f * eff;
        float ds     = -eff + freq_pan * range;
        float de     = ds + range / freq_zoom;
        float norm   = (x - graph_x) / graph_w;
        norm = std::max(0.0f, std::min(1.0f, norm));
        float rel = ds + norm * (de - ds);
        return (float)(header.center_frequency / 1e6f) + rel;
    }

    // ── IQ selection overlay (shared by spectrum and waterfall) ─────────────
    void draw_iq_selection_overlay(ImDrawList* dl,
                                   float graph_x, float graph_w,
                                   float graph_y, float graph_h) {
        if (!iq_sel.active) return;

        float cf    = (float)(header.center_frequency / 1e6f);
        float nyq   = header.sample_rate / 2.0f / 1e6f;
        float eff   = nyq * 0.875f;
        float range = 2.0f * eff;
        float ds    = -eff + freq_pan * range;
        float de    = ds + range / freq_zoom;
        float dw    = de - ds;

        float sel_s = std::min(iq_sel.start_mhz, iq_sel.end_mhz);
        float sel_e = std::max(iq_sel.start_mhz, iq_sel.end_mhz);

        // relative to center freq
        float rs = sel_s - cf;
        float re = sel_e - cf;

        float x0 = graph_x + (rs - ds) / dw * graph_w;
        float x1 = graph_x + (re - ds) / dw * graph_w;

        float clip0 = std::max(graph_x, x0);
        float clip1 = std::min(graph_x + graph_w, x1);
        if (clip1 <= clip0) return;

        bool  rec = iq_recording.load();
        ImU32 fill  = rec ? IM_COL32(255, 60, 60, 45)  : IM_COL32(0, 180, 255, 40);
        ImU32 bord  = rec ? IM_COL32(255, 100, 80, 220) : IM_COL32(0, 210, 255, 200);
        ImU32 tcol  = rec ? IM_COL32(255, 140, 100, 255): IM_COL32(0, 230, 255, 255);

        dl->AddRectFilled(ImVec2(clip0, graph_y), ImVec2(clip1, graph_y+graph_h), fill);

        // Dashed left border
        if (x0 >= graph_x - 1 && x0 <= graph_x + graph_w + 1) {
            for (float yy = graph_y; yy < graph_y + graph_h; yy += 10.0f) {
                float ye = std::min(yy + 5.0f, graph_y + graph_h);
                dl->AddLine(ImVec2(x0, yy), ImVec2(x0, ye), bord, 1.5f);
            }
        }
        // Dashed right border
        if (x1 >= graph_x - 1 && x1 <= graph_x + graph_w + 1) {
            for (float yy = graph_y; yy < graph_y + graph_h; yy += 10.0f) {
                float ye = std::min(yy + 5.0f, graph_y + graph_h);
                dl->AddLine(ImVec2(x1, yy), ImVec2(x1, ye), bord, 1.5f);
            }
        }

        // Label
        float bw_khz   = (sel_e - sel_s) * 1000.0f;
        float center_f = (sel_s + sel_e) / 2.0f;
        char  label[80];
        if (rec)
            snprintf(label, sizeof(label), "● REC  %.3f MHz / %.1f kHz", center_f, bw_khz);
        else
            snprintf(label, sizeof(label), "%.3f MHz / %.1f kHz", center_f, bw_khz);

        ImVec2 ts  = ImGui::CalcTextSize(label);
        float  cxv = (clip0 + clip1) / 2.0f - ts.x / 2.0f;
        cxv = std::max(graph_x, std::min(graph_x + graph_w - ts.x, cxv));
        float  lyv = graph_y + 4.0f;

        if (ts.x < (clip1 - clip0) + 20.0f) {
            dl->AddRectFilled(ImVec2(cxv-2, lyv), ImVec2(cxv+ts.x+2, lyv+ts.y+2),
                              IM_COL32(0,0,0,180));
            dl->AddText(ImVec2(cxv, lyv+1), tcol, label);
        }
    }

    // ── Handle right-click drag (call in both spectrum and waterfall) ────────
    // graph_x, graph_w must be the actual drawing area (excluding axis label)
    void handle_iq_selection_drag(float graph_x, float graph_w, bool is_hovered) {
        ImGuiIO& io = ImGui::GetIO();
        ImVec2 mouse = io.MousePos;

        float abs_freq = x_to_abs_freq(mouse.x, graph_x, graph_w);

        if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
            iq_sel.dragging        = true;
            iq_sel.drag_anchor_mhz = abs_freq;
            iq_sel.start_mhz       = abs_freq;
            iq_sel.end_mhz         = abs_freq;
            iq_sel.active          = true;
        }

        if (iq_sel.dragging) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                float f = x_to_abs_freq(mouse.x, graph_x, graph_w);
                iq_sel.start_mhz = std::min(iq_sel.drag_anchor_mhz, f);
                iq_sel.end_mhz   = std::max(iq_sel.drag_anchor_mhz, f);
            }
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
                iq_sel.dragging = false;
                if (fabsf(iq_sel.end_mhz - iq_sel.start_mhz) < 1e-5f)
                    iq_sel.active = false;
            }
        }
    }

    // ── draw_spectrum ────────────────────────────────────────────────────────
    void draw_spectrum(float w, float h) {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,  ImVec2(0,0));
        ImGui::BeginChild("spectrum_plot", ImVec2(w, h), false, ImGuiWindowFlags_NoScrollbar);
        ImDrawList *dl = ImGui::GetWindowDrawList();
        ImVec2 pos = ImGui::GetCursorScreenPos();

        dl->AddRectFilled(pos, ImVec2(pos.x+w, pos.y+h), IM_COL32(10,10,10,255));

        float nyquist       = header.sample_rate / 2.0f / 1e6f;
        float eff_nyq       = nyquist * 0.875f;
        float eff_range     = 2.0f * eff_nyq;
        float disp_start    = -eff_nyq + freq_pan * eff_range;
        float disp_width    = eff_range / freq_zoom;
        float disp_end      = disp_start + disp_width;
        disp_start = std::max(-eff_nyq, disp_start);
        disp_end   = std::min( eff_nyq, disp_end);
        float sr_mhz = header.sample_rate / 1e6f;

        float graph_x = pos.x + AXIS_LABEL_WIDTH;
        float graph_y = pos.y;
        float graph_w = w - AXIS_LABEL_WIDTH;
        float graph_h = h - BOTTOM_LABEL_HEIGHT;

        bool cache_valid = (cached_spectrum_idx    == current_fft_idx     &&
                            cached_spectrum_freq_pan  == freq_pan          &&
                            cached_spectrum_freq_zoom == freq_zoom         &&
                            cached_spectrum_pixels    == (int)graph_w      &&
                            cached_spectrum_power_min == display_power_min &&
                            cached_spectrum_power_max == display_power_max);
        if (!cache_valid) {
            compute_spectrum_line((int)graph_w, sr_mhz, disp_start, disp_end);
            cached_spectrum_idx      = current_fft_idx;
            cached_spectrum_freq_pan = freq_pan;
            cached_spectrum_freq_zoom= freq_zoom;
            cached_spectrum_pixels   = (int)graph_w;
            cached_spectrum_power_min= display_power_min;
            cached_spectrum_power_max= display_power_max;
        }

        float prange = display_power_max - display_power_min;
        int npx = (int)graph_w;
        for (int px = 0; px < npx-1; px++) {
            if (px   >= (int)current_spectrum.size()) break;
            if (px+1 >= (int)current_spectrum.size()) break;
            float p1 = (current_spectrum[px]   - display_power_min) / prange;
            float p2 = (current_spectrum[px+1] - display_power_min) / prange;
            p1 = std::max(0.0f,std::min(1.0f,p1));
            p2 = std::max(0.0f,std::min(1.0f,p2));
            ImVec2 a(graph_x+px,   graph_y+(1.0f-p1)*graph_h);
            ImVec2 b(graph_x+px+1, graph_y+(1.0f-p2)*graph_h);
            dl->AddLine(a, b, IM_COL32(0,255,0,255), 1.5f);
        }

        // dB grid
        for (int i = 0; i <= 10; i++) {
            float np = (float)i/10.0f;
            float y  = graph_y + (1.0f-np)*graph_h;
            dl->AddLine(ImVec2(graph_x,y), ImVec2(graph_x+graph_w,y), IM_COL32(60,60,60,100), 1.0f);
        }
        for (int i = 1; i <= 9; i++) {
            float pl = -(float)i/10.0f*80.0f;
            float np = (float)i/10.0f;
            float y  = graph_y + np*graph_h;
            dl->AddLine(ImVec2(graph_x-5,y), ImVec2(graph_x,y), IM_COL32(100,100,100,200), 1.0f);
            char lb[16]; snprintf(lb,sizeof(lb),"%.0f",pl);
            ImVec2 ts = ImGui::CalcTextSize(lb);
            dl->AddText(ImVec2(graph_x-10-ts.x,y-7), IM_COL32(200,200,200,255), lb);
        }

        // Freq grid
        {
            float cf_mhz   = header.center_frequency / 1e6f;
            float disp_range = disp_end - disp_start;
            if (freq_zoom <= 1.0f) {
                float abs_s = disp_start + cf_mhz, abs_e = disp_end + cf_mhz;
                float step  = 5.0f;
                float first = ceilf(abs_s / step) * step;
                for (float af = first; af <= abs_e+1e-4f; af += step) {
                    float rel = af - cf_mhz;
                    float x   = graph_x + (rel-disp_start)/disp_range*graph_w;
                    if (x < graph_x || x > graph_x+graph_w) continue;
                    dl->AddLine(ImVec2(x,graph_y), ImVec2(x,graph_y+graph_h), IM_COL32(60,60,60,100), 1.0f);
                    dl->AddLine(ImVec2(x,graph_y+graph_h), ImVec2(x,graph_y+graph_h+5), IM_COL32(100,100,100,200), 1.0f);
                    char lb[32]; snprintf(lb,sizeof(lb),"%.0f",af);
                    ImVec2 ts=ImGui::CalcTextSize(lb);
                    dl->AddText(ImVec2(x-ts.x/2, graph_y+graph_h+8), IM_COL32(0,255,0,255), lb);
                }
            } else {
                for (int i = 0; i <= 10; i++) {
                    float fn = (float)i/10.0f;
                    float x  = graph_x + fn*graph_w;
                    float af = cf_mhz + disp_start + fn*disp_range;
                    dl->AddLine(ImVec2(x,graph_y), ImVec2(x,graph_y+graph_h), IM_COL32(60,60,60,100), 1.0f);
                    dl->AddLine(ImVec2(x,graph_y+graph_h), ImVec2(x,graph_y+graph_h+5), IM_COL32(100,100,100,200), 1.0f);
                    char lb[32]; snprintf(lb,sizeof(lb),"%.3f",af);
                    ImVec2 ts=ImGui::CalcTextSize(lb);
                    dl->AddText(ImVec2(x-ts.x/2,graph_y+graph_h+8), IM_COL32(0,255,0,255), lb);
                }
            }
        }

        // ── IQ selection overlay (drawn before InvisibleButton) ──────────
        draw_iq_selection_overlay(dl, graph_x, graph_w, graph_y, graph_h);

        // InvisibleButton for hover/scroll
        ImGui::SetCursorScreenPos(ImVec2(graph_x, graph_y));
        ImGui::InvisibleButton("spectrum_canvas", ImVec2(graph_w, graph_h));
        bool hovered = ImGui::IsItemHovered();

        // ── Right-click drag for IQ selection ───────────────────────────
        handle_iq_selection_drag(graph_x, graph_w, hovered);

        // Mouse hover info
        if (hovered) {
            ImGuiIO& io = ImGui::GetIO();
            ImVec2 mouse = io.MousePos;
            int px = (int)((mouse.x - graph_x) + 0.5f);
            px = std::max(0, std::min(npx-1, px));
            float abs_freq = x_to_abs_freq(mouse.x, graph_x, graph_w);
            float power_db = (px < (int)current_spectrum.size()) ? current_spectrum[px] : -80.0f;
            char info[64]; snprintf(info,sizeof(info),"%.3f MHz | %.1f dB", abs_freq, power_db);
            ImVec2 ts = ImGui::CalcTextSize(info);
            float tx = graph_x + graph_w - ts.x - 4;
            float ty = graph_y + 2;
            dl->AddRectFilled(ImVec2(tx-2,ty), ImVec2(tx+ts.x+2,ty+ts.y+4), IM_COL32(20,20,20,220));
            dl->AddRect(ImVec2(tx-2,ty), ImVec2(tx+ts.x+2,ty+ts.y+4), IM_COL32(100,100,100,255));
            dl->AddText(ImVec2(tx,ty+2), IM_COL32(0,255,0,255), info);

            // Mouse wheel zoom
            if (io.MouseWheel != 0.0f) {
                float mx = (mouse.x - graph_x) / graph_w;
                mx = std::max(0.0f,std::min(1.0f,mx));
                float nyq2 = header.sample_rate/2.0f/1e6f;
                float eff2 = nyq2*0.875f;
                float rng2 = 2.0f*eff2;
                float ds2  = -eff2 + freq_pan*rng2;
                float fmx  = ds2 + mx*(rng2/freq_zoom);
                freq_zoom *= (1.0f + io.MouseWheel*0.1f);
                freq_zoom  = std::max(1.0f,std::min(10.0f,freq_zoom));
                float nw   = rng2/freq_zoom;
                float ns   = fmx - mx*nw;
                freq_pan   = (ns + eff2) / rng2;
                freq_pan   = std::max(0.0f, std::min(1.0f-1.0f/freq_zoom, freq_pan));
            }
        }

        // Y-axis power drag
        ImGui::SetCursorScreenPos(ImVec2(pos.x, graph_y));
        ImGui::InvisibleButton("power_axis_drag", ImVec2(AXIS_LABEL_WIDTH, graph_h));
        static float drag_start_y=0, drag_start_min=0, drag_start_max=0;
        static bool drag_lower=false;
        if (ImGui::IsItemActive()) {
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                ImVec2 m = ImGui::GetMousePos();
                float mid_p = (display_power_min+display_power_max)/2.0f;
                float mid_y = graph_y+graph_h*(1.0f-(mid_p-display_power_min)/(display_power_max-display_power_min));
                drag_start_y=m.y; drag_start_min=display_power_min; drag_start_max=display_power_max;
                drag_lower=(m.y>mid_y);
            }
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                ImVec2 m = ImGui::GetMousePos();
                float dy = m.y - drag_start_y;
                float mid_p = (drag_start_min+drag_start_max)/2.0f;
                if (drag_lower) {
                    float mid_yy = graph_y+graph_h*(1.0f-(mid_p-drag_start_min)/(drag_start_max-drag_start_min));
                    float norm = dy / (graph_y+graph_h - mid_yy);
                    norm = std::max(-1.0f,std::min(1.0f,norm));
                    display_power_min = mid_p - norm*50.0f;
                } else {
                    float mid_yy = graph_y+graph_h*(1.0f-(mid_p-drag_start_min)/(drag_start_max-drag_start_min));
                    float norm = -dy / mid_yy;
                    norm = std::max(-1.0f,std::min(1.0f,norm));
                    display_power_max = mid_p + norm*50.0f;
                }
                if (display_power_max - display_power_min < 5.0f) {
                    float mid=(display_power_min+display_power_max)/2.0f;
                    display_power_min=mid-2.5f; display_power_max=mid+2.5f;
                }
                cached_spectrum_idx = -1;
            }
        }

        ImGui::EndChild();
        ImGui::PopStyleVar(2);
    }

    // ── draw_waterfall_canvas ────────────────────────────────────────────────
    void draw_waterfall_canvas(ImDrawList *dl, ImVec2 plot_pos, ImVec2 plot_size) {
        dl->AddRectFilled(plot_pos, ImVec2(plot_pos.x+plot_size.x, plot_pos.y+plot_size.y),
                          IM_COL32(10,10,10,255));

        float nyquist = header.sample_rate / 2.0f / 1e6f;
        float eff_nyq = nyquist * 0.875f;
        float eff_rng = 2.0f * eff_nyq;
        float disp_s  = -eff_nyq + freq_pan * eff_rng;
        float disp_e  = disp_s + eff_rng / freq_zoom;
        disp_s = std::max(-eff_nyq, disp_s);
        disp_e = std::min( eff_nyq, disp_e);

        float graph_x = plot_pos.x + AXIS_LABEL_WIDTH;
        float graph_y = plot_pos.y;
        float graph_w = plot_size.x - AXIS_LABEL_WIDTH;
        float graph_h = plot_size.y - BOTTOM_LABEL_HEIGHT;

        if (waterfall_texture == 0) create_waterfall_texture();

        if (total_ffts_captured > 0 && last_waterfall_update_idx != current_fft_idx) {
            update_waterfall_row(current_fft_idx);
            last_waterfall_update_idx = current_fft_idx;
        }

        if (waterfall_texture != 0) {
            ImTextureID tid = (ImTextureID)(intptr_t)waterfall_texture;
            int display_rows = std::min((int)total_ffts_captured, MAX_FFTS_MEMORY);

            float u_s = (disp_s + nyquist) / (2.0f * nyquist);
            float u_e = (disp_e + nyquist) / (2.0f * nyquist);
            float v_newest = (float)(current_fft_idx % MAX_FFTS_MEMORY) / MAX_FFTS_MEMORY;
            float v_top    = v_newest + 1.0f / MAX_FFTS_MEMORY;
            float draw_h   = (display_rows >= (int)graph_h) ? graph_h : (float)display_rows;
            float v_draw_bottom = v_top - (float)display_rows / MAX_FFTS_MEMORY;

            dl->AddImage(tid,
                         ImVec2(graph_x, graph_y),
                         ImVec2(graph_x+graph_w, graph_y+draw_h),
                         ImVec2(u_s, v_top), ImVec2(u_e, v_draw_bottom),
                         IM_COL32(255,255,255,255));
        }

        // Freq axis labels (bottom)
        {
            float cf    = header.center_frequency / 1e6f;
            float drng  = disp_e - disp_s;
            if (freq_zoom <= 1.0f) {
                float as = disp_s+cf, ae = disp_e+cf, step=5.0f;
                float first=ceilf(as/step)*step;
                for (float af=first; af<=ae+1e-4f; af+=step) {
                    float x=graph_x+(af-cf-disp_s)/drng*graph_w;
                    if(x<graph_x||x>graph_x+graph_w) continue;
                    dl->AddLine(ImVec2(x,graph_y+graph_h), ImVec2(x,graph_y+graph_h+5), IM_COL32(100,100,100,200),1.0f);
                    char lb[32]; snprintf(lb,sizeof(lb),"%.0f",af);
                    ImVec2 ts=ImGui::CalcTextSize(lb);
                    dl->AddText(ImVec2(x-ts.x/2,graph_y+graph_h+8), IM_COL32(0,255,0,255), lb);
                }
            } else {
                for (int i=0;i<=10;i++) {
                    float fn=(float)i/10.0f, x=graph_x+fn*graph_w;
                    float af=cf+disp_s+fn*drng;
                    dl->AddLine(ImVec2(x,graph_y+graph_h), ImVec2(x,graph_y+graph_h+5), IM_COL32(100,100,100,200),1.0f);
                    char lb[32]; snprintf(lb,sizeof(lb),"%.3f",af);
                    ImVec2 ts=ImGui::CalcTextSize(lb);
                    dl->AddText(ImVec2(x-ts.x/2,graph_y+graph_h+8), IM_COL32(0,255,0,255), lb);
                }
            }
        }

        // ── IQ selection overlay ─────────────────────────────────────────
        draw_iq_selection_overlay(dl, graph_x, graph_w, graph_y, graph_h);

        // InvisibleButton for interaction
        ImGui::InvisibleButton("waterfall_canvas", plot_size);
        bool hovered = ImGui::IsItemHovered();

        // ── Right-click drag for IQ selection ───────────────────────────
        handle_iq_selection_drag(graph_x, graph_w, hovered);

        // Mouse hover info
        if (hovered) {
            ImGuiIO& io = ImGui::GetIO();
            ImVec2 mouse = io.MousePos;
            int px = (int)((mouse.x - graph_x) + 0.5f);
            px = std::max(0, std::min((int)graph_w-1, px));
            float abs_freq = x_to_abs_freq(mouse.x, graph_x, graph_w);

            char info[64]; snprintf(info,sizeof(info),"%.3f MHz", abs_freq);
            ImVec2 ts=ImGui::CalcTextSize(info);
            float tx=graph_x+graph_w-ts.x, ty=graph_y;
            dl->AddRectFilled(ImVec2(tx,ty), ImVec2(tx+ts.x,ty+ts.y+5), IM_COL32(20,20,20,220));
            dl->AddRect(ImVec2(tx,ty), ImVec2(tx+ts.x,ty+ts.y+5), IM_COL32(100,100,100,255));
            dl->AddText(ImVec2(tx,ty+2), IM_COL32(0,255,0,255), info);

            // Mouse wheel zoom
            if (io.MouseWheel != 0.0f) {
                float mx = (mouse.x - graph_x) / graph_w;
                mx = std::max(0.0f,std::min(1.0f,mx));
                float nyq2=header.sample_rate/2.0f/1e6f, eff2=nyq2*0.875f, rng2=2.0f*eff2;
                float ds2=-eff2+freq_pan*rng2;
                float fmx=ds2+mx*(rng2/freq_zoom);
                freq_zoom*=(1.0f+io.MouseWheel*0.1f);
                freq_zoom=std::max(1.0f,std::min(10.0f,freq_zoom));
                float nw=rng2/freq_zoom, ns=fmx-mx*nw;
                freq_pan=(ns+eff2)/rng2;
                freq_pan=std::max(0.0f,std::min(1.0f-1.0f/freq_zoom,freq_pan));
            }
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
void run_streaming_viewer() {
    float center_freq  = 450.0f;
    float sample_rate  = 61.44f;

    FFTViewer viewer;
    if (!viewer.initialize_bladerf(center_freq, sample_rate)) {
        printf("Failed to initialize BladeRF\n");
        return;
    }

    std::thread capture_thread(&FFTViewer::capture_and_process, &viewer);

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow *window = glfwCreateWindow(1400, 900, viewer.window_title.c_str(), nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);
    glewExperimental = GL_TRUE;
    glewInit();

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    viewer.create_waterfall_texture();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (viewer.texture_needs_recreate) {
            viewer.texture_needs_recreate = false;
            viewer.create_waterfall_texture();
        }

        // ── Main FFT window ───────────────────────────────────────────────
        ImGui::SetNextWindowPos(ImVec2(0,0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);

        if (ImGui::Begin("##fft_viewer", nullptr,
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoTitleBar)) {
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,   ImVec2(0,0));
            ImGui::PopStyleVar(2);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(5,5));
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,   ImVec2(8,5));

            // Freq input
            static float new_freq = 450.0f;
            static bool  freq_deactivate = false;
            if (!ImGui::IsAnyItemActive()) {
                if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter))
                    ImGui::SetKeyboardFocusHere();
            }
            if (freq_deactivate) { freq_deactivate=false; ImGui::SetWindowFocus(nullptr); }
            ImGui::SetNextItemWidth(120);
            ImGui::InputFloat("Freq (MHz)", &new_freq, 0,0,"%.3f");
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                viewer.pending_center_freq = new_freq;
                viewer.freq_change_requested = true;
                freq_deactivate = true;
            }

            ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();

            // FFT size dropdown
            static const int fft_sizes[] = {512,1024,2048,4096,8192,16384,32768};
            static const char* fft_labels[] = {"512","1024","2048","4096","8192","16384","32768"};
            static int fft_sz_idx = 4;
            ImGui::Text("FFT Size:"); ImGui::SameLine();
            ImGui::SetNextItemWidth(90);
            if (ImGui::BeginCombo("##fftsize", fft_labels[fft_sz_idx])) {
                for (int i=0;i<7;i++) {
                    bool sel=(fft_sz_idx==i);
                    if (ImGui::Selectable(fft_labels[i],sel)) {
                        fft_sz_idx=i;
                        viewer.pending_fft_size=fft_sizes[i];
                        viewer.fft_size_change_requested=true;
                    }
                    if (sel) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            ImGui::PopStyleVar(2);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,   ImVec2(0,0));

            float w       = ImGui::GetContentRegionAvail().x;
            float total_h = ImGui::GetIO().DisplaySize.y - 35;
            float divider_h = 15.0f;
            float h1      = (total_h - divider_h) * viewer.spectrum_height_ratio;

            viewer.draw_spectrum(w, h1);

            ImGui::InvisibleButton("divider", ImVec2(w, divider_h));
            if (ImGui::IsItemActive()) {
                float delta = ImGui::GetIO().MouseDelta.y;
                viewer.spectrum_height_ratio += delta / total_h;
                viewer.spectrum_height_ratio = std::max(0.1f, std::min(0.9f, viewer.spectrum_height_ratio));
            }

            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,  ImVec2(0,0));
            float remaining_h = ImGui::GetContentRegionAvail().y;
            ImGui::BeginChild("waterfall_plot", ImVec2(w, remaining_h), false, ImGuiWindowFlags_NoScrollbar);
            ImDrawList *wf_draw = ImGui::GetWindowDrawList();
            ImVec2 canvas_pos  = ImGui::GetCursorScreenPos();
            ImVec2 canvas_size(w, remaining_h);
            viewer.draw_waterfall_canvas(wf_draw, canvas_pos, canvas_size);
            ImGui::EndChild();

            ImGui::PopStyleVar(4);
            ImGui::End();
        }

        // ── IQ Recording Control Panel ────────────────────────────────────
        if (viewer.iq_sel.active || viewer.iq_recording.load()) {
            ImVec2 disp = ImGui::GetIO().DisplaySize;
            ImGui::SetNextWindowPos(ImVec2(disp.x - 310, 40), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(300, 0));
            ImGui::SetNextWindowBgAlpha(0.88f);
            if (ImGui::Begin("IQ Recording", nullptr,
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize |
                             ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings)) {

                float sel_s  = std::min(viewer.iq_sel.start_mhz, viewer.iq_sel.end_mhz);
                float sel_e  = std::max(viewer.iq_sel.start_mhz, viewer.iq_sel.end_mhz);
                float sel_cf = (sel_s + sel_e) / 2.0f;
                float sel_bw = (sel_e - sel_s) * 1000.0f;

                ImGui::Text("Selected center : %.4f MHz", sel_cf);
                ImGui::Text("Selected BW     : %.2f kHz", sel_bw);
                ImGui::Separator();

                // BW selector
                ImGui::Text("Record BW:");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(110);
                if (!viewer.iq_recording.load()) {
                    ImGui::Combo("##recbw", &viewer.iq_bw_idx, IQ_BW_LABELS, 6);
                } else {
                    ImGui::Text("%s", IQ_BW_LABELS[viewer.iq_bw_idx]);
                }

                // Show decimation info
                {
                    uint32_t sr = viewer.header.sample_rate;
                    int decim = std::max(1, (int)(sr / IQ_SR_TABLE[viewer.iq_bw_idx]));
                    uint32_t actual = sr / decim;
                    ImGui::TextDisabled("SR: %u SPS  (decim ÷%d)", actual, decim);
                }

                ImGui::Separator();

                if (!viewer.iq_recording.load()) {
                    if (viewer.iq_sel.active && sel_bw > 0.001f) {
                        ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(30,140,30,255));
                        if (ImGui::Button("▶  Start Recording", ImVec2(-1, 0)))
                            viewer.start_iq_recording();
                        ImGui::PopStyleColor();
                    } else {
                        ImGui::TextDisabled("Right-drag on spectrum/waterfall");
                        ImGui::TextDisabled("to select a region first.");
                    }
                } else {
                    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(160,30,30,255));
                    if (ImGui::Button("■  Stop Recording", ImVec2(-1, 0)))
                        viewer.stop_iq_recording();
                    ImGui::PopStyleColor();

                    uint64_t samps = viewer.iq_samples_written.load();
                    uint32_t sr2   = IQ_SR_TABLE[viewer.iq_bw_idx];
                    int      decim2= std::max(1,(int)(viewer.header.sample_rate / sr2));
                    uint32_t asr   = viewer.header.sample_rate / decim2;
                    float dur = asr > 0 ? (float)samps / (float)asr : 0.0f;
                    float mb  = (float)(samps * 4) / (1024.0f * 1024.0f);

                    ImGui::Text("Frames  : %llu", (unsigned long long)samps);
                    ImGui::Text("Duration: %.2f s", dur);
                    ImGui::Text("Size    : %.2f MB", mb);

                    // Filename (truncated)
                    const char* fn = viewer.iq_record_filename.c_str();
                    // Show last part if long
                    size_t fnlen = viewer.iq_record_filename.size();
                    const char* display_fn = (fnlen > 28) ? fn + fnlen - 28 : fn;
                    ImGui::TextDisabled("…%s", display_fn);
                }

                ImGui::Separator();
                if (ImGui::SmallButton("Clear Selection")) {
                    if (!viewer.iq_recording.load()) {
                        viewer.iq_sel.active = false;
                    }
                }
                ImGui::SameLine();
                ImGui::TextDisabled("(right-drag to re-select)");
            }
            ImGui::End();
        }

        ImGui::Render();
        int dw, dh;
        glfwGetFramebufferSize(window, &dw, &dh);
        glViewport(0, 0, dw, dh);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    viewer.is_running = false;
    if (viewer.iq_recording.load()) viewer.stop_iq_recording();
    capture_thread.join();

    if (viewer.waterfall_texture) glDeleteTextures(1, &viewer.waterfall_texture);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();

    printf("Streaming viewer closed\n");
}

// ─────────────────────────────────────────────────────────────────────────────
int main() {
    run_streaming_viewer();
    return 0;
}