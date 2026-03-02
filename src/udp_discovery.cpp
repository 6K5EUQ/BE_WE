#include "udp_discovery.hpp"
#include <cstring>
#include <cstdio>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h>

// ── DiscoveryBroadcaster ──────────────────────────────────────────────────

void DiscoveryBroadcaster::set_info(const char* name, float lat, float lon,
                                     uint16_t port, const char* ip) {
    std::lock_guard<std::mutex> lk(pkt_mtx_);
    memset(&pkt_, 0, sizeof(pkt_));
    pkt_.magic[0]='B'; pkt_.magic[1]='E'; pkt_.magic[2]='W'; pkt_.magic[3]='G';
    strncpy(pkt_.station_name, name, 63);
    pkt_.lat      = lat;
    pkt_.lon      = lon;
    pkt_.tcp_port = port;
    strncpy(pkt_.host_ip, ip, 15);
}

void DiscoveryBroadcaster::set_user_count(uint8_t n) {
    std::lock_guard<std::mutex> lk(pkt_mtx_);
    pkt_.user_count = n;
}

bool DiscoveryBroadcaster::start() {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) { perror("[DiscoveryBroadcaster] socket"); return false; }
    int bcast = 1;
    setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));
    int reuse = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    running_.store(true);
    thr_ = std::thread(&DiscoveryBroadcaster::broadcast_loop, this);
    printf("[DiscoveryBroadcaster] started\n");
    return true;
}

void DiscoveryBroadcaster::stop() {
    running_.store(false);
    if (thr_.joinable()) thr_.join();
    if (sock_ >= 0) { close(sock_); sock_ = -1; }
    printf("[DiscoveryBroadcaster] stopped\n");
}

void DiscoveryBroadcaster::broadcast_loop() {
    sockaddr_in dest{};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(BEWE_DISCOVERY_PORT);
    dest.sin_addr.s_addr = INADDR_BROADCAST;

    while (running_.load()) {
        {
            std::lock_guard<std::mutex> lk(pkt_mtx_);
            sendto(sock_, &pkt_, sizeof(pkt_), 0,
                   reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
        }
        // 2-second interval in 100 ms increments for responsive stop
        for (int i = 0; i < 20 && running_.load(); i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── DiscoveryListener ─────────────────────────────────────────────────────

bool DiscoveryListener::start() {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) { perror("[DiscoveryListener] socket"); return false; }
    int reuse = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    int bcast = 1;
    setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(BEWE_DISCOVERY_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("[DiscoveryListener] bind");
        close(sock_); sock_ = -1; return false;
    }
    running_.store(true);
    thr_ = std::thread(&DiscoveryListener::listen_loop, this);
    printf("[DiscoveryListener] listening on port %d\n", BEWE_DISCOVERY_PORT);
    return true;
}

void DiscoveryListener::stop() {
    running_.store(false);
    if (sock_ >= 0) { shutdown(sock_, SHUT_RDWR); close(sock_); sock_ = -1; }
    if (thr_.joinable()) thr_.join();
    printf("[DiscoveryListener] stopped\n");
}

void DiscoveryListener::listen_loop() {
    while (running_.load()) {
        if (sock_ < 0) break;
        fd_set fds; FD_ZERO(&fds); FD_SET(sock_, &fds);
        timeval tv{0, 200000}; // 200 ms timeout
        int r = select(sock_ + 1, &fds, nullptr, nullptr, &tv);
        if (r <= 0) continue;

        DiscoveryAnnounce ann{};
        sockaddr_in src{}; socklen_t srclen = sizeof(src);
        ssize_t n = recvfrom(sock_, &ann, sizeof(ann), 0,
                             reinterpret_cast<sockaddr*>(&src), &srclen);
        if (n != static_cast<ssize_t>(sizeof(DiscoveryAnnounce))) continue;
        if (memcmp(ann.magic, "BEWG", 4) != 0) continue;
        ann.station_name[63] = '\0';
        ann.host_ip[15]      = '\0';
        if (on_station_found) on_station_found(ann);
    }
}
