#include <ctime>
#include "fft_viewer.hpp"
#include "net_server.hpp"

// ── 브로드캐스트 전용 스레드 ─────────────────────────────────────────────
// 캡처 스레드에서 TCP send를 절대 호출하지 않도록 분리.
// net_bcast_seq가 증가하면 깨어나서 최신 FFT 행을 복사 후 전송.
void FFTViewer::net_bcast_worker(){
    int last_seq = -1;
    // 전송 버퍼 (로컬 복사 → send 중 data_mtx 불필요)
    std::vector<int8_t> local_fft;
    uint64_t local_cf  = 0;
    uint32_t local_sr  = 0;
    float    local_min = -80.f, local_max = 0.f;
    int      local_sz  = 0;
    int64_t  local_wt  = 0;

    while(!net_bcast_stop.load(std::memory_order_relaxed)){
        // 새 FFT 행 대기
        {
            std::unique_lock<std::mutex> lk(net_bcast_mtx);
            net_bcast_cv.wait_for(lk, std::chrono::milliseconds(100), [&]{
                return net_bcast_seq.load(std::memory_order_acquire) != last_seq
                    || net_bcast_stop.load(std::memory_order_relaxed);
            });
        }
        if(net_bcast_stop.load(std::memory_order_relaxed)) break;

        int cur_seq = net_bcast_seq.load(std::memory_order_acquire);
        if(cur_seq == last_seq) continue;
        last_seq = cur_seq;

        if(!net_srv || net_srv->client_count() == 0) continue;

        // 최신 FFT 행을 로컬 버퍼로 빠르게 복사 (data_mtx는 최소 시간만 점유)
        {
            std::lock_guard<std::mutex> lk(data_mtx);
            local_sz  = fft_size;
            local_cf  = header.center_frequency;
            local_sr  = header.sample_rate;
            local_min = display_power_min;
            local_max = display_power_max;
            local_wt  = (int64_t)time(nullptr);
            int fi    = (current_fft_idx) % MAX_FFTS_MEMORY;
            const int8_t* rowp = fft_data.data() + fi * fft_size;
            local_fft.assign(rowp, rowp + fft_size);
        }

        // TCP 전송 (블로킹이어도 캡처 스레드와 무관)
        net_srv->broadcast_fft(local_fft.data(), local_sz,
                               local_wt,
                               local_cf, local_sr,
                               local_min, local_max);
    }
}