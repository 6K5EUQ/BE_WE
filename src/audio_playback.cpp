#include "audio_playback.hpp"
#include "fft_viewer.hpp"  // bewe_log_push
#include "sigmf.hpp"       // SigMF::open_source (SigMF .sigmf-data + legacy .wav)
#include <chrono>
#include <cstdio>
#include <cstring>
#include <algorithm>

AudioPlayback::AudioPlayback() {}
AudioPlayback::~AudioPlayback(){ stop(); }

bool AudioPlayback::start(const std::string& path, uint32_t out_sr, double offset_sec){
    stop(); // 기존 재생 정리

    SigMF::Source src;
    if(!SigMF::open_source(path, src)){
        bewe_log_push(2,"[AudioPlay] open/parse failed: %s\n", path.c_str());
        return false;
    }
    FILE* f = src.f;

    size_t bytes_per_frame = (size_t)src.nch * 2; // int16
    int64_t total_frames   = src.data_size / (int64_t)bytes_per_frame;
    float   total_s        = (src.sample_rate > 0)
                             ? (float)total_frames / (float)src.sample_rate : 0.f;

    // offset 클램프
    if(offset_sec < 0.0) offset_sec = 0.0;
    if(offset_sec > total_s) offset_sec = total_s;
    int64_t start_frame = (int64_t)(offset_sec * (double)src.sample_rate);
    if(start_frame >= total_frames){
        fclose(f);
        bewe_log_push(0,"[AudioPlay] offset past EOF\n");
        return false;
    }

    total_sec_.store(total_s, std::memory_order_relaxed);
    pos_sec_.store((float)offset_sec, std::memory_order_relaxed);

    path_ = path;
    size_t slash = path.find_last_of('/');
    filename_ = (slash == std::string::npos) ? path : path.substr(slash+1);

    // 약 0.5s buffer
    ring_n_ = std::max<size_t>(4096, out_sr / 2);
    ring_.assign(ring_n_ * 2, 0);
    wr_.store(0, std::memory_order_relaxed);
    rd_.store(0, std::memory_order_relaxed);

    stop_req_.store(false, std::memory_order_relaxed);
    paused_.store(false, std::memory_order_relaxed);
    active_.store(true,  std::memory_order_release);

    worker_ = std::thread([this, src, out_sr, start_frame](){
        FILE* f = src.f;
        // start offset으로 seek
        long seek_pos = src.data_offset + start_frame * (long)src.nch * 2;
        fseek(f, seek_pos, SEEK_SET);

        double phase = 0.0;
        double ratio = (double)src.sample_rate / (double)out_sr;
        int16_t prev_L = 0, prev_R = 0;
        int16_t cur_L  = 0, cur_R  = 0;

        int64_t frames_emitted = 0;
        double  base_sec       = (double)start_frame / (double)src.sample_rate;

        auto read_next_input_frame = [&](int16_t& L, int16_t& R) -> bool {
            int16_t frame[8];
            size_t got = fread(frame, sizeof(int16_t), src.nch, f);
            if(got != (size_t)src.nch) return false;
            if(src.nch == 1){ L = R = frame[0]; }
            else            { L = frame[0]; R = frame[1]; }
            return true;
        };

        auto push_stereo_frame = [&](int16_t L, int16_t R) -> bool {
            while(!stop_req_.load(std::memory_order_relaxed)){
                // pause 중에도 ring 공급은 중단 (mix가 못 빼가도록)
                if(paused_.load(std::memory_order_relaxed)){
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
                size_t w = wr_.load(std::memory_order_relaxed);
                size_t r = rd_.load(std::memory_order_acquire);
                size_t next = (w + 1) % ring_n_;
                if(next != r){
                    ring_[w*2]   = L;
                    ring_[w*2+1] = R;
                    wr_.store(next, std::memory_order_release);
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            return false;
        };

        if(!read_next_input_frame(cur_L, cur_R)){
            fclose(f);
            active_.store(false, std::memory_order_release);
            return;
        }
        prev_L = cur_L; prev_R = cur_R;

        while(!stop_req_.load(std::memory_order_relaxed)){
            while(phase >= 1.0){
                prev_L = cur_L;
                prev_R = cur_R;
                if(!read_next_input_frame(cur_L, cur_R)){
                    fclose(f);
                    active_.store(false, std::memory_order_release);
                    return;
                }
                phase -= 1.0;
            }
            float fL = (float)prev_L + (float)(cur_L - prev_L) * (float)phase;
            float fR = (float)prev_R + (float)(cur_R - prev_R) * (float)phase;
            int16_t oL = (int16_t)(fL < -32768 ? -32768 : fL > 32767 ? 32767 : fL);
            int16_t oR = (int16_t)(fR < -32768 ? -32768 : fR > 32767 ? 32767 : fR);
            if(!push_stereo_frame(oL, oR)) break;

            frames_emitted++;
            if((frames_emitted & 0x7FF) == 0){
                pos_sec_.store(
                    (float)(base_sec + (double)frames_emitted / (double)out_sr),
                    std::memory_order_relaxed);
            }
            phase += ratio;
        }

        fclose(f);
        active_.store(false, std::memory_order_release);
    });

    bewe_log_push(0,"[AudioPlay] %s @%.2fs (%u Hz, %u ch)\n",
                  filename_.c_str(), offset_sec, src.sample_rate, (unsigned)src.nch);
    return true;
}

void AudioPlayback::stop(){
    stop_req_.store(true, std::memory_order_release);
    paused_.store(false, std::memory_order_release); // worker가 sleep loop에서 빠져나오게
    if(worker_.joinable()) worker_.join();
    active_.store(false, std::memory_order_release);
    pos_sec_.store(0.f, std::memory_order_relaxed);
    total_sec_.store(0.f, std::memory_order_relaxed);
    path_.clear();
    filename_.clear();
    wr_.store(0, std::memory_order_relaxed);
    rd_.store(0, std::memory_order_relaxed);
}

void AudioPlayback::pause(){
    paused_.store(true, std::memory_order_release);
}
void AudioPlayback::resume(){
    paused_.store(false, std::memory_order_release);
}

bool AudioPlayback::pop_stereo(float& L, float& R){
    if(!active_.load(std::memory_order_acquire)) return false;
    if(paused_.load(std::memory_order_acquire))  return false;
    size_t w = wr_.load(std::memory_order_acquire);
    size_t r = rd_.load(std::memory_order_relaxed);
    if(r == w) return false; // empty
    int16_t iL = ring_[r*2];
    int16_t iR = ring_[r*2+1];
    rd_.store((r + 1) % ring_n_, std::memory_order_release);
    L = (float)iL / 32768.0f;
    R = (float)iR / 32768.0f;
    return true;
}
