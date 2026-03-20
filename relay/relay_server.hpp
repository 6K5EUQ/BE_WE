#pragma once
#include "relay_proto.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <deque>
#include <string>
#include <memory>
#include <chrono>
#include <condition_variable>

struct JoinEntry {
    uint16_t          conn_id = 0;
    int               fd      = -1;
    std::atomic<bool> alive{true};
    std::thread       thr;      // JOIN→HOST recv 스레드

    // ── 독립 송신 큐: FFT/제어 + 오디오 각각 전용 스레드 ─────────────────
    static constexpr size_t SEND_QUEUE_MAX  = 512;
    static constexpr size_t AUDIO_QUEUE_MAX = 256;

    // BEWE PacketType (relay는 net_protocol.hpp를 include하지 않으므로 상수만)
    static constexpr uint8_t BEWE_TYPE_AUDIO = 0x04;

    // FFT/제어 큐
    std::deque<std::vector<uint8_t>> send_queue;
    std::mutex              send_mtx;
    std::condition_variable send_cv;
    std::thread             send_thr;

    // 오디오 큐
    std::deque<std::vector<uint8_t>> audio_queue;
    std::mutex              audio_mtx;
    std::condition_variable audio_cv;
    std::thread             audio_thr;

    std::atomic<bool>       send_stop{false};
    std::mutex              fd_write_mtx;  // fd write 직렬화

    // fd로 패킷 전송 (공통)
    void send_raw(const std::vector<uint8_t>& pkt){
        if(fd < 0 || !alive.load()) return;
        std::lock_guard<std::mutex> wlk(fd_write_mtx);
        const uint8_t* p = pkt.data();
        size_t rem = pkt.size();
        while(rem > 0){
            ssize_t r = ::send(fd, p, rem, MSG_NOSIGNAL);
            if(r <= 0){ alive.store(false); break; }
            p += r; rem -= r;
        }
    }

    void start_send_worker(){
        send_thr = std::thread([this](){
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
        });
        audio_thr = std::thread([this](){
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
        });
    }

    void stop_send_worker(){
        send_stop.store(true);
        send_cv.notify_all();
        audio_cv.notify_all();
        if(send_thr.joinable())  send_thr.join();
        if(audio_thr.joinable()) audio_thr.join();
    }

    // BEWE 패킷 데이터를 받아 타입에 따라 적절한 큐에 push
    // BEWE 패킷 형식: magic[4] + type[1] + len[4] + payload
    void enqueue(const uint8_t* data, size_t len){
        // BEWE 헤더에서 type 판별 (offset 4)
        bool is_audio = (len >= 5 && data[4] == BEWE_TYPE_AUDIO);

        if(is_audio){
            std::lock_guard<std::mutex> lk(audio_mtx);
            if(audio_queue.size() >= AUDIO_QUEUE_MAX)
                audio_queue.pop_front();
            audio_queue.emplace_back(data, data + len);
            audio_cv.notify_one();
        } else {
            std::lock_guard<std::mutex> lk(send_mtx);
            if(send_queue.size() >= SEND_QUEUE_MAX)
                send_queue.pop_front();
            send_queue.emplace_back(data, data + len);
            send_cv.notify_one();
        }
    }
};

struct HostRoom {
    std::string               station_id;
    RelayStation              info;
    int                       fd = -1;       // HOST ctrl+mux 소켓
    std::atomic<bool>         alive{true};
    std::atomic<bool>         resetting{false};  // chassis 2 reset: LIST에서 제외
    std::chrono::steady_clock::time_point last_hb;

    mutable std::mutex                    host_send_mtx; // HOST fd write 직렬화
    mutable std::mutex                    joins_mtx;
    std::vector<std::shared_ptr<JoinEntry>> joins;
    uint16_t                              next_conn_id = 1;
};

class RelayServer {
public:
    bool start(int port = RELAY_PORT);
    void run();
    void stop();

private:
    int               listen_fd_ = -1;
    std::atomic<bool> running_{false};

    mutable std::mutex                     rooms_mtx_;
    std::vector<std::shared_ptr<HostRoom>> rooms_;

    std::thread accept_thr_;
    std::thread watchdog_thr_;

    int  make_listen_sock(int port);
    void accept_loop();
    void watchdog_loop();
    void handshake(int fd);

    // HOST: mux 스트림 수신 루프 (HOST→relay→JOIN fanout)
    void host_mux_loop(std::shared_ptr<HostRoom> room);

    // JOIN: BEWE 스트림 수신 루프 (JOIN→relay→HOST forward)
    void join_loop(std::shared_ptr<JoinEntry> je, std::shared_ptr<HostRoom> room);

    void handle_list_req(int fd);
    std::shared_ptr<HostRoom> find_room(const std::string& id) const;
};
