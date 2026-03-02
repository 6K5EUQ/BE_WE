#pragma once
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

// ── Net audio ring per channel ────────────────────────────────────────────
struct NetAudioRing {
    static constexpr size_t SZ   = 32768;
    static constexpr size_t MASK = SZ - 1;
    float                   buf[SZ]{};
    int8_t                  pan[SZ]{};
    std::atomic<size_t>     wp{0}, rp{0};

    void push(float v, int8_t p){
        size_t w = wp.load(std::memory_order_relaxed);
        buf[w & MASK] = v;
        pan[w & MASK] = p;
        wp.store(w+1, std::memory_order_release);
    }
    bool pop(float& v, int8_t& p){
        size_t r = rp.load(std::memory_order_relaxed);
        if(r == wp.load(std::memory_order_acquire)) return false;
        v = buf[r & MASK]; p = pan[r & MASK];
        rp.store(r+1, std::memory_order_release);
        return true;
    }
    void clear(){ rp.store(wp.load(std::memory_order_acquire)); }
};

// ── NetClient ─────────────────────────────────────────────────────────────
class NetClient {
public:
    // ── Connection ───────────────────────────────────────────────────────
    bool connect(const char* host, int port,
                 const char* id, const char* pw, uint8_t tier);
    void disconnect();
    bool is_connected() const { return connected_.load(); }

    // ── Remote FFT data (written by recv thread, read by UI) ─────────────
    // UI should lock fft_mtx when reading
    mutable std::mutex   fft_mtx;
    std::vector<int8_t>  fft_data;     // latest FFT row
    uint64_t             cf_hz   = 0;
    uint32_t             sr      = 0;
    uint16_t             fft_sz  = 0;
    float                pmin    = -80.f, pmax = 0.f;
    std::atomic<int>     fft_seq{0};   // incremented each new frame

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
    // SHARE_LIST: called when HOST sends updated share file list
    // vector of (filename, size_bytes, uploader)
    std::function<void(const std::vector<std::tuple<std::string,uint64_t,std::string>>&)> on_share_list;

    // ── Operator list ─────────────────────────────────────────────────────
    mutable std::mutex   op_mtx;
    PktOperatorList      op_list{};
    std::atomic<bool>    op_list_updated{false};

    // ── Audio rings (one per channel) ─────────────────────────────────────
    NetAudioRing audio[5];  // MAX_CHANNELS

    // ── Chat ──────────────────────────────────────────────────────────────
    struct ChatMsg { char from[32]; char msg[256]; };
    std::mutex              chat_mtx;
    std::vector<ChatMsg>    chat_log;
    static constexpr int    CHAT_LOG_MAX = 200;
    std::atomic<bool>       chat_updated{false};

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

    void recv_loop();
    void handle_packet(PacketType type, const uint8_t* payload, uint32_t len);
    bool raw_send(PacketType type, const void* payload, uint32_t len);
};