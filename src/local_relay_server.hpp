#pragma once
#include "relay_proto.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

// ── LocalRelayServer ────────────────────────────────────────────────────────
// HOST가 시작 시 자동으로 켜는 LAN 전용 미니 relay 서버.
// 같은 망(공유기)에 있는 클라이언트는 외부 relay 없이 이 서버를 통해 연결.
// BEWE_LOCAL_RELAY_PORT(7702)에서 listen.

static constexpr int BEWE_LOCAL_RELAY_PORT = 7702;

struct LocalJoinEntry {
    uint16_t          conn_id = 0;
    int               fd      = -1;
    std::atomic<bool> alive{true};
    std::thread       thr;
};

struct LocalHostRoom {
    std::string               station_id;
    RelayStation              info;
    int                       fd = -1;
    std::atomic<bool>         alive{true};
    std::chrono::steady_clock::time_point last_hb;

    mutable std::mutex                        host_send_mtx;
    mutable std::mutex                        joins_mtx;
    std::vector<std::shared_ptr<LocalJoinEntry>> joins;
    uint16_t                                  next_conn_id = 1;
};

class LocalRelayServer {
public:
    bool start(int port = BEWE_LOCAL_RELAY_PORT);
    void stop();
    bool is_running() const { return running_.load(); }
    int  listen_port() const { return listen_port_val_; }

private:
    int               listen_fd_ = -1;
    int               listen_port_val_ = 0;
    std::atomic<bool> running_{false};

    mutable std::mutex                         rooms_mtx_;
    std::vector<std::shared_ptr<LocalHostRoom>> rooms_;

    std::thread accept_thr_;
    std::thread watchdog_thr_;

    int  make_listen_sock(int port);
    void accept_loop();
    void watchdog_loop();
    void handshake(int fd);
    void host_mux_loop(std::shared_ptr<LocalHostRoom> room);
    void join_loop(std::shared_ptr<LocalJoinEntry> je, std::shared_ptr<LocalHostRoom> room);
    void handle_list_req(int fd);
    std::shared_ptr<LocalHostRoom> find_room(const std::string& id) const;
    static bool send_to_join(int fd, const uint8_t* buf, size_t len);
};
