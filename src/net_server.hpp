#pragma once
#include "net_protocol.hpp"
#include "channel.hpp"
#include <string>
#include <vector>
#include <deque>
#include <tuple>
#include <thread>
#include <mutex>
#include <condition_variable>
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
    bool     is_relay  = false;  // 중앙서버 MUX 경유 클라이언트 (inject_fd로 추가됨)
    std::atomic<bool> alive{false};
    std::thread     thr;

    // ── 독립 송신 큐: FFT/제어 + 오디오 각각 전용 스레드 ─────────────────
    static constexpr size_t SEND_QUEUE_MAX     = 512;
    static constexpr size_t AUDIO_QUEUE_MAX    = 256;

    // FFT/제어 큐 + 스레드
    std::deque<std::vector<uint8_t>> send_queue;
    std::mutex              send_mtx;
    std::condition_variable send_cv;
    std::thread             send_thr;

    // 오디오 큐 + 스레드
    std::deque<std::vector<uint8_t>> audio_queue;
    std::mutex              audio_mtx;
    std::condition_variable audio_cv;
    std::thread             audio_thr;

    std::atomic<bool>       send_stop{false};
    std::mutex              fd_write_mtx;  // fd write 직렬화 (send/audio/send_file_to)

    // fd로 패킷 전송 (non-blocking: socketpair 버퍼 가득 차면 드롭)
    void send_raw(const std::vector<uint8_t>& pkt){
        if(fd < 0 || !alive.load()) return;
        std::lock_guard<std::mutex> wlk(fd_write_mtx);
        size_t sent = 0;
        while(sent < pkt.size()){
            ssize_t r = ::send(fd, pkt.data()+sent, pkt.size()-sent, MSG_NOSIGNAL | MSG_DONTWAIT);
            if(r < 0){
                if(errno == EAGAIN || errno == EWOULDBLOCK)
                    return;  // 버퍼 가득 → 이 패킷 드롭 (실시간 스트림)
                alive.store(false); return;
            }
            if(r == 0){ alive.store(false); return; }
            sent += (size_t)r;
        }
    }

    // FFT/제어 전용 스레드
    void send_worker(){
        while(true){
            std::vector<uint8_t> pkt;
            {
                std::unique_lock<std::mutex> lk(send_mtx);
                send_cv.wait(lk, [this]{ return !send_queue.empty() || send_stop.load(); });
                if(send_stop.load() && send_queue.empty()) break;
                pkt = std::move(send_queue.front());
                send_queue.pop_front();
            }
            send_raw(pkt);
        }
    }

    // 오디오 전용 스레드
    void audio_worker(){
        while(true){
            std::vector<uint8_t> pkt;
            {
                std::unique_lock<std::mutex> lk(audio_mtx);
                audio_cv.wait(lk, [this]{ return !audio_queue.empty() || send_stop.load(); });
                if(send_stop.load() && audio_queue.empty()) break;
                pkt = std::move(audio_queue.front());
                audio_queue.pop_front();
            }
            send_raw(pkt);
        }
    }

    void start_send_worker(){
        send_thr  = std::thread(&ClientConn::send_worker, this);
        audio_thr = std::thread(&ClientConn::audio_worker, this);
    }
    void stop_send_worker(){
        send_stop.store(true);
        send_cv.notify_all();
        audio_cv.notify_all();
        if(send_thr.joinable())  send_thr.join();
        if(audio_thr.joinable()) audio_thr.join();
    }

    void enqueue(std::vector<uint8_t> pkt, bool is_fft = false, bool is_audio = false){
        if(is_audio){
            std::lock_guard<std::mutex> lk(audio_mtx);
            if(audio_queue.size() >= AUDIO_QUEUE_MAX)
                audio_queue.pop_front();
            audio_queue.push_back(std::move(pkt));
            audio_cv.notify_one();
        } else {
            std::lock_guard<std::mutex> lk(send_mtx);
            if(send_queue.size() >= SEND_QUEUE_MAX){
                if(is_fft) send_queue.pop_front();
                else return;
            }
            send_queue.push_back(std::move(pkt));
            send_cv.notify_one();
        }
    }

    // 업로드 수신 상태 (SHARE_UPLOAD_META/DATA)
    struct UploadRecv {
        char     filename[128] = {};
        uint64_t total_bytes   = 0;
        uint64_t recv_bytes    = 0;
        uint8_t  transfer_id   = 0;
        FILE*    fp            = nullptr;
        bool     active        = false;
        char     save_path[512]= {};
    } upload{};

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
    std::function<void(int idx, float s, float e, const char* creator)> on_create_ch;
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
    std::function<void(uint8_t op_idx, const char* op_name,
                       int32_t fft_top, int32_t fft_bot,
                       float freq_lo, float freq_hi,
                       int32_t time_start, int32_t time_end)> on_request_region;
    std::function<void(const char* from, const char* msg)> on_chat;
    std::function<void(uint8_t op_idx, const char* filename)> on_share_download_req;
    // JOIN이 파일 업로드 완료: op_idx, op_name, 저장된 절대경로
    std::function<void(uint8_t op_idx, const char* op_name, const char* saved_path)> on_share_upload_done;
    std::function<void()> on_chassis_reset;  // JOIN이 /chassis 1 reset 명령 전송
    std::function<void()> on_net_reset;      // JOIN이 /chassis 2 reset 명령 전송
    std::function<void()> on_rx_stop;        // JOIN이 /rx stop 명령 전송
    std::function<void()> on_rx_start;       // JOIN이 /rx start 명령 전송
    std::function<void(uint32_t size)>       on_set_fft_size; // JOIN → HOST: FFT 크기 변경
    std::function<void(float msps)>          on_set_sr;       // JOIN → HOST: SR 변경
    // JOIN이 public 파일 삭제 요청: op_name, filename (소유자 검증은 ui.cpp에서)
    std::function<void(const char* op_name, const char* filename)> on_pub_delete_req;
    // 중앙서버 relay 브로드캐스트 콜백: BEWE 패킷 1회 전달 → 중앙서버가 N명에게 fan-out
    // 이 콜백을 통해 FFT/오디오/채팅 등이 relay 클라이언트로 전달됨 (N× 대역폭 문제 해결)
    std::function<void(const uint8_t*, size_t)> on_relay_broadcast;
};

// ── NetServer ─────────────────────────────────────────────────────────────
class NetServer {
public:
    ServerCallbacks cb;

    bool start(int port = BEWE_DEFAULT_PORT);
    void stop();
    bool is_running() const { return running_.load(); }
    int  client_count() const;
    int  listen_port() const { return listen_port_; }
    bool has_relay() const { return relay_client_count_.load() > 0; }

    // relay MUX 모드: socketpair의 local_fd를 새 클라이언트로 inject
    void inject_fd(int fd);
    std::vector<OpEntry> get_operators() const;

    // ── Broadcast / Send ─────────────────────────────────────────────────
    // FFT frame → all clients
    void broadcast_fft(const float* data, int fft_size,
                        int64_t wall_time,
                       uint64_t center_hz, uint32_t sr,
                       float pmin, float pmax);

    // Audio → operators matching mask (bit0=host, bit1=op1, bit2=op2, ...)
    void send_audio(uint32_t op_mask, uint8_t ch_idx, int8_t pan,
                    const float* pcm, uint32_t n_samples);

    // Audio → all clients unconditionally (릴레이 서버가 per-JOIN 필터링)
    void broadcast_audio_all(uint8_t ch_idx, int8_t pan,
                             const float* pcm, uint32_t n_samples);



    void broadcast_wf_event(int32_t fft_offset, int64_t wall_time,
                            uint8_t type, const char* label);
    void send_file_to(int op_index, const char* path, uint8_t transfer_id,
                      std::function<void(uint64_t done, uint64_t total)> progress_cb = nullptr);
    void send_region_response(int op_index, bool allowed);

    // Channel state → all clients
    void broadcast_channel_sync(const Channel* chs, int n);

    // Heartbeat → all clients
    // host_state: 0=OK, 1=CHASSIS_RESETTING, 2=SPECTRUM_PAUSED
    // sdr_state:  0=streaming OK, 1=stream error
    void broadcast_heartbeat(uint8_t host_state = 0, uint8_t sdr_temp_c = 0, uint8_t sdr_state = 0, uint8_t iq_on = 0);

    // /chassis 2 reset: FFT+오디오 방송 일시 중단 / 재개
    void pause_broadcast()  { bcast_pause_.store(true,  std::memory_order_relaxed); }
    void resume_broadcast() { bcast_pause_.store(false, std::memory_order_relaxed); }

    // 모든 클라이언트 송신 큐 flush (쌓인 데이터 버리기)
    void flush_clients(){
        std::lock_guard<std::mutex> lk(clients_mtx_);
        for(auto& c : clients_){
            { std::lock_guard<std::mutex> qlk(c->send_mtx);  c->send_queue.clear();  }
            { std::lock_guard<std::mutex> qlk(c->audio_mtx); c->audio_queue.clear(); }
        }
    }

    // Chat → all clients
    void broadcast_chat(const char* from, const char* msg);

    // Share list → specific client (or all clients if op_index==-1)
    // tuple: (filename, size_bytes, uploader_name)
    void send_share_list(int op_index,
                         const std::vector<std::tuple<std::string,uint64_t,std::string>>& files);

    // HW status → all clients
    void broadcast_status(float cf_mhz, float gain_db,
                          uint32_t sr, uint8_t hw_type);

    // Operator list → all clients
    void broadcast_operator_list();

    // ── UDP Discovery Broadcast ───────────────────────────────────────────
    void start_discovery_broadcast(const char* station_name, float lat, float lon,
                                    uint16_t tcp_port, const char* host_ip,
                                    uint8_t host_tier = 1);
    void stop_discovery_broadcast();
    void update_discovery_user_count();

    // HOST 본인 정보 설정 (op_list index=0 으로 브로드캐스트)
    void set_host_info(const char* name, uint8_t tier){
        strncpy(host_name_, name, 31); host_name_[31]='\0';
        host_tier_ = tier;
    }

    // Get current operator list (for UI)
    
private:
    // Forward declaration to avoid pulling udp_discovery.hpp into every TU
    class DiscoveryBroadcaster* discovery_bcast_ = nullptr;
    std::atomic<bool> bcast_pause_{false}; // /chassis 2 reset: 방송 일시 중단 플래그

    char    host_name_[32] = {};
    uint8_t host_tier_     = 1;
    int  server_fd_ = -1;
    int  listen_port_ = 0;
    std::atomic<bool> running_{false};
    std::atomic<int>  relay_client_count_{0};  // inject_fd로 추가된 relay 클라이언트 수
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