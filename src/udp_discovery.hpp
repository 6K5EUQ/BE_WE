#pragma once
#include "net_protocol.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>

// ── DiscoveryBroadcaster ──────────────────────────────────────────────────
// Sends DiscoveryAnnounce via UDP broadcast (255.255.255.255:7701) every 2 s.
// Thread-safe: set_user_count() may be called from any thread.
class DiscoveryBroadcaster {
public:
    void set_info(const char* station_name, float lat, float lon,
                  uint16_t tcp_port, const char* host_ip);
    void set_user_count(uint8_t n);

    bool start();
    void stop();

private:
    DiscoveryAnnounce pkt_{};
    std::mutex        pkt_mtx_;
    int               sock_ = -1;
    std::atomic<bool> running_{false};
    std::thread       thr_;
    void              broadcast_loop();
};

// ── DiscoveryListener ─────────────────────────────────────────────────────
// Binds 0.0.0.0:7701, receives DiscoveryAnnounce packets from LAN.
// Calls on_station_found from listener thread — callback must be thread-safe.
class DiscoveryListener {
public:
    std::function<void(const DiscoveryAnnounce&)> on_station_found;

    bool start();
    void stop();

private:
    int               sock_ = -1;
    std::atomic<bool> running_{false};
    std::thread       thr_;
    void              listen_loop();
};
