#pragma once
#include "net_protocol.hpp"
#include "channel.hpp"
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <functional>
#include <netinet/in.h>

// ── Per-client connection ─────────────────────────────────────────────────
struct ClientConn {
    int     fd       = -1;
    uint8_t op_index = 0;   // 1-based
    uint8_t tier     = 0;
    char    name[32] = {};
    bool    authed   = false;
    std::mutex      send_mtx;
    std::atomic<bool> alive{false};
    std::thread     thr;

    ClientConn() = default;
    ClientConn(const ClientConn&) = delete;
    ClientConn& operator=(const ClientConn&) = delete;
};

// ── Server callback table ─────────────────────────────────────────────────
struct ServerCallbacks {
    // Returns true + assigns op_index if auth ok
    std::function<bool(const char* id, const char* pw,
                       uint8_t tier, uint8_t& op_index)> on_auth;

    std::function<void(float cf_mhz)>               on_set_freq;
    std::function<void(float db)>                    on_set_gain;
    std::function<void(int idx, float s, float e)>   on_create_ch;
    std::function<void(int idx)>                     on_delete_ch;
    std::function<void(int idx, int mode)>           on_set_ch_mode;
    std::function<void(int idx, uint32_t mask)>      on_set_ch_audio;
    std::function<void(int ch_idx)>                  on_start_rec;
    std::function<void()>                            on_stop_rec;
    std::function<void(int idx, int pan)>            on_set_ch_pan;
    std::function<void(int idx, float thr)>          on_set_sq_thresh;
    std::function<void()>                            on_set_autoscale;
    std::function<void(int ch_idx, uint8_t op_idx, bool enable)> on_toggle_recv;
    std::function<void(int idx, float s, float e)>   on_update_ch_range;
    std::function<void()>                            on_toggle_tm_iq;
    std::function<void(bool pause)>                  on_set_capture_pause;
    std::function<void(bool pause)>                  on_set_spectrum_pause;
    std::function<void(uint8_t op_idx, int32_t fft_top, int32_t fft_bot,
                       float freq_lo, float freq_hi)> on_request_region;
    std::function<void(const char* from, const char* msg)> on_chat;
};

// ── NetServer ─────────────────────────────────────────────────────────────
class NetServer {
public:
    ServerCallbacks cb;

    bool start(int port = BEWE_DEFAULT_PORT);
    void stop();
    bool is_running() const { return running_.load(); }
    int  client_count() const;
    std::vector<OpEntry> get_operators() const;

    // ── Broadcast / Send ─────────────────────────────────────────────────
    // FFT frame → all clients
    void broadcast_fft(const int8_t* data, int fft_size,
                        int64_t wall_time,
                       uint64_t center_hz, uint32_t sr,
                       float pmin, float pmax);

    // Audio → operators matching mask (bit0=host, bit1=op1, bit2=op2, ...)
    void send_audio(uint32_t op_mask, uint8_t ch_idx, int8_t pan,
                    const float* pcm, uint32_t n_samples);

    void broadcast_wf_event(int32_t fft_offset, int64_t wall_time,
                            uint8_t type, const char* label);
    void send_file_to(int op_index, const char* path, uint8_t transfer_id);

    // Channel state → all clients
    void broadcast_channel_sync(const Channel* chs, int n);

    // Chat → all clients
    void broadcast_chat(const char* from, const char* msg);

    // HW status → all clients
    void broadcast_status(float cf_mhz, float gain_db,
                          uint32_t sr, uint8_t hw_type);

    // Operator list → all clients
    void broadcast_operator_list();

    // Get current operator list (for UI)
    
private:
    int  server_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread accept_thr_;

    mutable std::mutex            clients_mtx_;
    std::vector<std::shared_ptr<ClientConn>> clients_;
    std::atomic<uint8_t>          next_idx_{1};

    void accept_loop();
    void client_loop(std::shared_ptr<ClientConn> c);
    void handle_packet(std::shared_ptr<ClientConn> c,
                       PacketType type,
                       const uint8_t* payload, uint32_t len);
    void send_to(ClientConn& c, PacketType type,
                 const void* payload, uint32_t len);
    void drop_client(std::shared_ptr<ClientConn> c);
};