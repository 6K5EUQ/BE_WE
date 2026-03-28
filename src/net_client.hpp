#pragma once
#include "config.hpp"
#include "net_protocol.hpp"
#include "channel.hpp"
#include <string>
#include <vector>
#include <tuple>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <netinet/in.h>

// ── Net audio ring per channel (with jitter buffer) ──────────────────────
struct NetAudioRing {
    static constexpr size_t SZ   = 32768;
    static constexpr size_t MASK = SZ - 1;
    // 재생 시작 전 최소 축적량 (48kHz 기준 ~20ms)
    static constexpr size_t JITTER_FILL = 960;
    // 언더런 발생 시 재축적 임계값 (JITTER_FILL의 절반)
    static constexpr size_t JITTER_RESUME = JITTER_FILL / 2;

    float                   buf[SZ]{};
    int8_t                  pan[SZ]{};
    std::atomic<size_t>     wp{0}, rp{0};
    bool                    primed = false; // jitter buffer 축적 완료 여부

    std::atomic<uint64_t> stat_underruns{0};  // 언더런 횟수

    void push(float v, int8_t p){
        size_t w = wp.load(std::memory_order_relaxed);
        buf[w & MASK] = v;
        pan[w & MASK] = p;
        wp.store(w+1, std::memory_order_release);
    }

    // jitter buffer를 거쳐 pop
    // primed 전: 데이터가 JITTER_FILL 이상 쌓일 때까지 false 반환
    // primed 후: 정상 pop. 언더런 시 primed 해제 → 재축적
    bool pop(float& v, int8_t& p){
        size_t w = wp.load(std::memory_order_acquire);
        size_t r = rp.load(std::memory_order_relaxed);
        size_t avail = w - r;

        if(!primed){
            if(avail < JITTER_FILL) return false;
            primed = true;
        }
        if(avail == 0){
            // 언더런: 재축적 모드로 전환
            primed = false;
            stat_underruns.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        v = buf[r & MASK]; p = pan[r & MASK];
        rp.store(r+1, std::memory_order_release);
        return true;
    }

    void clear(){
        rp.store(wp.load(std::memory_order_acquire));
        primed = false;
    }
};

// ── NetClient ─────────────────────────────────────────────────────────────
class NetClient {
public:
    // ── Connection ───────────────────────────────────────────────────────
    bool connect(const char* host, int port,
                 const char* id, const char* pw, uint8_t tier);
    // relay 모드: 이미 연결된 fd로 AUTH만 수행
    bool connect_fd(int fd, const char* id, const char* pw, uint8_t tier);
    void disconnect();
    bool is_connected() const { return connected_.load(); }

    // ── Remote FFT data (written by recv thread, read by UI) ─────────────
    // UI calls pop_fft_frame() to get buffered frames (1s delay for smooth playback)
    struct FftFrame {
        std::vector<float> data;
        uint64_t cf_hz;
        uint32_t sr;
        uint32_t fft_sz;
        float    pmin, pmax;
        int64_t  wall_time;  // seconds since epoch
        int64_t  recv_us;    // steady_clock microseconds at receive time
    };

    mutable std::mutex   fft_mtx;
    // Legacy single-frame access (kept for compatibility)
    std::vector<float>   fft_data;
    uint64_t             cf_hz   = 0;
    uint32_t             sr      = 0;
    uint16_t             fft_sz  = 0;
    float                pmin    = -80.f, pmax = 0.f;
    std::atomic<int>     fft_seq{0};   // incremented each new buffered frame

    // Buffer queue: recv thread enqueues, UI dequeues after DISPLAY_DELAY_US
    static constexpr int64_t DISPLAY_DELAY_US = 50'000;  // 50ms
    static constexpr size_t  FFT_QUEUE_MAX     = 512;        // max buffered frames

    // Returns true if a frame was dequeued (ready to display)
    bool pop_fft_frame(FftFrame& out);

    // ── HW status (from STATUS packets) ──────────────────────────────────
    // ── File transfer receive state ───────────────────────────────────────
    struct FileRecv {
        std::string filename;
        uint64_t    total_bytes = 0;
        uint64_t    recv_bytes  = 0;
        uint8_t     transfer_id = 0;
        bool        done        = false;
        std::string save_path;
        FILE*       fp          = nullptr;
    };
    std::mutex              file_recv_mtx;
    std::vector<FileRecv>   file_recv_list;  // history for STAT display

    int64_t               fft_wall_time = 0;  // wall_time from last FFT frame
    std::atomic<float>    remote_cf_mhz{450.f};
    std::atomic<float>    remote_gain_db{0.f};
    std::atomic<uint32_t> remote_sr{0};
    std::atomic<uint8_t>  remote_hw{0};

    // Heartbeat state: 0=OK, 1=CHASSIS_RESETTING, 2=SPECTRUM_PAUSED; -1=no HB yet
    std::atomic<int>      host_state{-1};
    std::atomic<uint8_t>  remote_sdr_temp_c{0};      // HOST SDR 온도 (°C, 0=미측정)
    std::atomic<uint8_t>  remote_sdr_state{0};        // 0=OK, 1=stream error
    std::atomic<uint8_t>  remote_iq_on{0};            // HOST IQ 롤링 상태 (0=off, 1=on)
    std::atomic<double>   last_heartbeat_time{0.0};  // glfwGetTime() at last HB

    // ── Channel sync (from CHANNEL_SYNC packets) ──────────────────────────
    // Applied directly to a FFTViewer's channels array via callback
    std::function<void(const PktChannelSync&)> on_channel_sync;
    std::function<void(const PktWfEvent&)>     on_wf_event;
    std::function<void(const std::string& name, uint64_t total)> on_file_meta;
    std::function<void(const std::string& name, uint64_t done, uint64_t total)> on_file_progress;
    std::function<void(const std::string& path,
                       const std::string& name)> on_file_done;
    // Optional: override save directory for incoming file (return "" to use default)
    std::function<std::string(const std::string& filename)> on_get_save_dir;
    std::function<void(bool allowed)> on_region_response;
    // IQ_PROGRESS: HOST→relay→JOIN 파이프 전송 진행상황 (phase: 0=REC,1=Transferring,2=Done)
    std::function<void(const PktIqProgress&)> on_iq_progress;
    // IQ_CHUNK: HOST→central MUX→JOIN 청크 스트리밍 수신
    std::function<void(uint32_t req_id, uint32_t seq,
                       const char* filename, uint64_t filesize,
                       const uint8_t* data, uint32_t data_len)> on_iq_chunk;
    // SHARE_LIST: called when HOST sends updated share file list
    // vector of (filename, size_bytes, uploader)
    std::function<void(const std::vector<std::tuple<std::string,uint64_t,std::string>>&)> on_share_list;

    // ── Operator list ─────────────────────────────────────────────────────
    mutable std::mutex   op_mtx;
    PktOperatorList      op_list{};
    std::atomic<bool>    op_list_updated{false};

    // ── Audio rings (one per channel) ─────────────────────────────────────
    NetAudioRing audio[MAX_CHANNELS];

    // ── Chat ──────────────────────────────────────────────────────────────
    struct ChatMsg { char from[32]; char msg[256]; };
    std::mutex              chat_mtx;
    std::vector<ChatMsg>    chat_log;
    static constexpr int    CHAT_LOG_MAX = 200;
    std::atomic<bool>       chat_updated{false};

    // JOIN이 수직바를 왼쪽 끝으로 밀면 FFT 수신을 멈춤 (서버로부터 패킷 미사용)
    std::atomic<bool> fft_recv_enabled{true};

    // ── Traffic stats ────────────────────────────────────────────────────
    std::atomic<uint64_t> stat_rx_bytes{0};   // 총 수신 바이트
    std::atomic<uint64_t> stat_tx_bytes{0};   // 총 송신 바이트

    struct NetStats {
        uint64_t rx_bytes    = 0;
        uint64_t tx_bytes    = 0;
        uint64_t underruns   = 0;   // 전체 채널 언더런 합계
        size_t   jitter_fill = 0;   // 현재 지터버퍼 샘플 수 (max across channels)
    };
    NetStats collect_stats() const {
        NetStats s;
        s.rx_bytes = stat_rx_bytes.load(std::memory_order_relaxed);
        s.tx_bytes = stat_tx_bytes.load(std::memory_order_relaxed);
        for(int i = 0; i < MAX_CHANNELS; i++){
            s.underruns += audio[i].stat_underruns.load(std::memory_order_relaxed);
            size_t avail = audio[i].wp.load(std::memory_order_relaxed)
                         - audio[i].rp.load(std::memory_order_relaxed);
            if(avail > s.jitter_fill) s.jitter_fill = avail;
        }
        return s;
    }

    // ── Auth result ───────────────────────────────────────────────────────
    uint8_t my_op_index = 0;
    uint8_t my_tier     = 0;
    char    my_name[32] = {};

    // ── Commands to server ────────────────────────────────────────────────
    bool send_cmd(const PktCmd& cmd);
    bool send_chat(const char* msg);

    // Convenience helpers
    bool cmd_set_freq(float cf_mhz);
    bool cmd_set_gain(float db);
    bool cmd_create_ch(int idx, float s, float e);
    bool cmd_delete_ch(int idx);
    bool cmd_set_ch_mode(int idx, int mode);
    bool cmd_set_ch_audio(int idx, uint32_t mask);
    bool cmd_set_ch_pan(int idx, int pan);
    bool cmd_set_sq_thresh(int idx, float thr);
    bool cmd_set_autoscale();
    bool cmd_toggle_recv(int ch_idx, bool enable);
    bool cmd_update_ch_range(int idx, float s, float e);
    bool cmd_toggle_tm_iq();
    bool cmd_set_capture_pause(bool pause);
    bool cmd_set_spectrum_pause(bool pause);
    bool cmd_request_region(int32_t fft_top, int32_t fft_bot,
                             float freq_lo, float freq_hi,
                             int32_t time_start, int32_t time_end);
    bool cmd_request_share_download(const char* filename);
    bool cmd_share_upload(const char* filepath, uint8_t transfer_id);
    bool cmd_chassis_reset();            // JOIN → HOST: trigger chassis 1 reset
    bool cmd_net_reset();               // JOIN → HOST: trigger chassis 2 (net-only) reset
    bool cmd_delete_pub_file(const char* filename);  // JOIN → HOST: delete public file
    bool cmd_rx_stop();                   // JOIN → HOST: /rx stop
    bool cmd_rx_start();                  // JOIN → HOST: /rx start
    bool cmd_set_fft_size(uint32_t size); // JOIN → HOST: FFT 크기 변경
    bool cmd_set_sr(float msps);          // JOIN → HOST: SR 변경

    // ── UDP Discovery Listener ────────────────────────────────────────────
    // Note: DiscoveryAnnounce is defined in net_protocol.hpp (already included)
    bool start_discovery_listen(
        std::function<void(const DiscoveryAnnounce&)> callback);
    void stop_discovery_listen();

private:
    int  fd_ = -1;
    std::atomic<bool> connected_{false};
    std::thread       recv_thr_;

    mutable std::mutex send_mtx_;

    // Discovery listener (forward declared to avoid pulling udp_discovery.hpp)
    class DiscoveryListener* discovery_listener_ = nullptr;

    // FFT buffer queue (1s delay)
    std::mutex              fft_queue_mtx_;
    std::vector<FftFrame>   fft_queue_;

    void recv_loop();
    void handle_packet(PacketType type, const uint8_t* payload, uint32_t len);
    bool raw_send(PacketType type, const void* payload, uint32_t len);
};