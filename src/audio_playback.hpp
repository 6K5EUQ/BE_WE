#pragma once
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

// ── Audio 재생: WAV 파일을 stereo ring으로 흘려보냄 ───────────────────────
// EID 패널의 Audio 탭 전용. mix_worker가 pop_stereo(L,R)을 frame 단위로 호출
// 해 기존 스테레오 믹스에 합산. 로컬 ALSA 출력만 (네트워크 송출 없음).
class AudioPlayback {
public:
    AudioPlayback();
    ~AudioPlayback();

    // 새 WAV 파일 재생 시작. offset_sec 위치에서 시작.
    // 기존 재생/일시정지 상태는 자동 정리.
    bool start(const std::string& path, uint32_t out_sr, double offset_sec = 0.0);

    void stop();          // 정지 + 리소스 해제
    void pause();         // 일시정지 (worker 유지, ring 공급만 멈춤)
    void resume();        // 재개

    bool active() const { return active_.load(std::memory_order_acquire); }
    bool paused() const { return paused_.load(std::memory_order_acquire); }

    // mix_worker가 frame 단위로 호출. paused이면 false 반환.
    bool pop_stereo(float& L, float& R);

    float position_sec() const { return pos_sec_.load(std::memory_order_relaxed); }
    float total_sec()    const { return total_sec_.load(std::memory_order_relaxed); }
    const std::string& filename() const { return filename_; }
    const std::string& path()     const { return path_; }

private:
    void worker_loop();

    std::atomic<bool> active_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> stop_req_{false};
    std::thread       worker_;

    // SPSC stereo ring — worker push, mix_worker pop
    std::vector<int16_t> ring_;   // 2 * N (interleaved L,R,L,R...)
    size_t               ring_n_ = 0;
    std::atomic<size_t>  wr_{0};
    std::atomic<size_t>  rd_{0};

    std::string path_;
    std::string filename_;
    std::atomic<float> pos_sec_{0.f};
    std::atomic<float> total_sec_{0.f};
};
