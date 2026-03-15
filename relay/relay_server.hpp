#pragma once
#include "relay_proto.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

struct JoinEntry {
    uint16_t          conn_id = 0;
    int               fd      = -1;
    std::atomic<bool> alive{true};
    std::thread       thr;
};

struct HostRoom {
    std::string               station_id;
    RelayStation              info;
    int                       fd = -1;       // HOST ctrl+mux 소켓
    std::atomic<bool>         alive{true};
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

    // JOIN fd로 데이터 전송 (직렬화 불필요 — join별 단일 스레드가 write)
    static bool send_to_join(int fd, const uint8_t* buf, size_t len);
};
