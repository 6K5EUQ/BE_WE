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

    // ── per-JOIN 비동기 send 큐 (HOST→JOIN 병렬 전송) ──────────────────
    static constexpr size_t SEND_QUEUE_MAX = 512;
    std::deque<std::vector<uint8_t>> send_queue;
    std::mutex              send_mtx;
    std::condition_variable send_cv;
    std::thread             send_thr;
    std::atomic<bool>       send_stop{false};

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
                if(fd < 0 || !alive.load()) continue;
                // blocking send — 이 JOIN 전용 스레드이므로 다른 JOIN에 영향 없음
                const uint8_t* p = pkt.data();
                size_t rem = pkt.size();
                while(rem > 0){
                    ssize_t r = ::send(fd, p, rem, MSG_NOSIGNAL);
                    if(r <= 0){ alive.store(false); break; }
                    p += r; rem -= r;
                }
            }
        });
    }

    void stop_send_worker(){
        send_stop.store(true);
        send_cv.notify_all();
        if(send_thr.joinable()) send_thr.join();
    }

    // 큐에 데이터 push (non-blocking, 큐 포화 시 oldest 드롭)
    void enqueue(const uint8_t* data, size_t len){
        std::lock_guard<std::mutex> lk(send_mtx);
        if(send_queue.size() >= SEND_QUEUE_MAX)
            send_queue.pop_front();  // oldest 드롭 (실시간 스트림)
        send_queue.emplace_back(data, data + len);
        send_cv.notify_one();
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
