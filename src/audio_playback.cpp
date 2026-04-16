#include "audio_playback.hpp"
#include "fft_viewer.hpp"  // bewe_log_push
#include <chrono>
#include <cstdio>
#include <cstring>
#include <algorithm>

namespace {

struct WavInfo {
    uint32_t sample_rate  = 0;
    uint16_t channels     = 0;
    uint16_t bits         = 0;
    long     data_offset  = 0;
    long     data_size    = 0;
};

// sa_compute.cpp의 RIFF 파서를 단순화 + bewe sub-chunk skip
bool parse_wav_header(FILE* f, WavInfo& info){
    char riff[4]={}, wave[4]={};
    if(fread(riff,1,4,f)!=4) return false;
    fseek(f, 4, SEEK_CUR); // file size
    if(fread(wave,1,4,f)!=4) return false;
    if(strncmp(riff,"RIFF",4)!=0 || strncmp(wave,"WAVE",4)!=0) return false;

    while(true){
        char id[4]={};
        uint32_t csz=0;
        if(fread(id,1,4,f)!=4) break;
        if(fread(&csz,4,1,f)!=1) break;
        long chunk_pos = ftell(f);
        if(strncmp(id,"fmt ",4)==0 && csz >= 16){
            uint16_t fmt_tag=0;
            fread(&fmt_tag, 2, 1, f);
            fread(&info.channels, 2, 1, f);
            fread(&info.sample_rate, 4, 1, f);
            fseek(f, 6, SEEK_CUR); // byte rate + block align
            fread(&info.bits, 2, 1, f);
        } else if(strncmp(id,"data",4)==0){
            info.data_offset = chunk_pos;
            info.data_size   = (long)csz;
            // data 청크 안에 bewe sub-chunk가 끼어있을 수 있음 → skip
            char peek[4]={};
            if(fread(peek,1,4,f)==4 && strncmp(peek,"bewe",4)==0){
                uint32_t bsz=0; fread(&bsz,4,1,f);
                long new_off = ftell(f) + (long)bsz + ((long)bsz & 1);
                long skipped = new_off - info.data_offset;
                info.data_size   -= skipped;
                info.data_offset  = new_off;
            }
            break;
        }
        long next = chunk_pos + (long)csz + ((long)csz & 1);
        if(fseek(f, next, SEEK_SET) != 0) break;
    }
    return (info.sample_rate > 0 && info.channels > 0 && info.data_size > 0);
}

} // namespace

AudioPlayback::AudioPlayback() {}
AudioPlayback::~AudioPlayback(){ stop(); }

bool AudioPlayback::start(const std::string& path, uint32_t out_sr, double offset_sec){
    stop(); // 기존 재생 정리

    FILE* f = fopen(path.c_str(), "rb");
    if(!f){ bewe_log_push(2,"[AudioPlay] open failed: %s\n", path.c_str()); return false; }

    WavInfo info;
    if(!parse_wav_header(f, info)){
        fclose(f);
        bewe_log_push(2,"[AudioPlay] WAV parse failed: %s\n", path.c_str());
        return false;
    }
    if(info.bits != 16){
        fclose(f);
        bewe_log_push(2,"[AudioPlay] only 16-bit PCM supported (got %u)\n", (unsigned)info.bits);
        return false;
    }

    size_t bytes_per_frame = (size_t)info.channels * 2; // int16
    int64_t total_frames   = info.data_size / (int64_t)bytes_per_frame;
    float   total_s        = (info.sample_rate > 0)
                             ? (float)total_frames / (float)info.sample_rate : 0.f;

    // offset 클램프
    if(offset_sec < 0.0) offset_sec = 0.0;
    if(offset_sec > total_s) offset_sec = total_s;
    int64_t start_frame = (int64_t)(offset_sec * (double)info.sample_rate);
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

    worker_ = std::thread([this, f, info, out_sr, start_frame](){
        // start offset으로 seek
        long seek_pos = info.data_offset + start_frame * (long)info.channels * 2;
        fseek(f, seek_pos, SEEK_SET);

        double phase = 0.0;
        double ratio = (double)info.sample_rate / (double)out_sr;
        int16_t prev_L = 0, prev_R = 0;
        int16_t cur_L  = 0, cur_R  = 0;

        int64_t frames_emitted = 0;
        double  base_sec       = (double)start_frame / (double)info.sample_rate;

        auto read_next_input_frame = [&](int16_t& L, int16_t& R) -> bool {
            int16_t frame[8];
            size_t got = fread(frame, sizeof(int16_t), info.channels, f);
            if(got != info.channels) return false;
            if(info.channels == 1){ L = R = frame[0]; }
            else                    { L = frame[0]; R = frame[1]; }
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
                  filename_.c_str(), offset_sec, info.sample_rate, (unsigned)info.channels);
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
