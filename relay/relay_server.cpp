#include "relay_server.hpp"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <unistd.h>

static constexpr int HOST_TIMEOUT_SEC  = 15;
static constexpr int HANDSHAKE_TIMEOUT = 10;
static constexpr size_t PIPE_BUF       = 65536;

int RelayServer::make_listen_sock(int port){
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0){ perror("socket"); return -1; }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)port);
    if(bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0){ perror("bind"); close(fd); return -1; }
    if(listen(fd, 64) < 0){ perror("listen"); close(fd); return -1; }
    printf("[Relay] listening on port %d\n", port);
    return fd;
}

bool RelayServer::start(int port){
    listen_fd_ = make_listen_sock(port);
    if(listen_fd_ < 0) return false;
    running_.store(true);
    accept_thr_   = std::thread(&RelayServer::accept_loop,   this);
    watchdog_thr_ = std::thread(&RelayServer::watchdog_loop, this);
    return true;
}

void RelayServer::run(){
    while(running_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void RelayServer::stop(){
    running_.store(false);
    if(listen_fd_ >= 0){ shutdown(listen_fd_, SHUT_RDWR); close(listen_fd_); listen_fd_=-1; }
    if(accept_thr_.joinable())   accept_thr_.join();
    if(watchdog_thr_.joinable()) watchdog_thr_.join();
    std::lock_guard<std::mutex> lk(rooms_mtx_);
    for(auto& r : rooms_){
        r->alive.store(false);
        if(r->fd >= 0){ shutdown(r->fd, SHUT_RDWR); close(r->fd); r->fd=-1; }
        std::lock_guard<std::mutex> jlk(r->joins_mtx);
        for(auto& j : r->joins){
            j->alive.store(false);
            if(j->fd >= 0){ shutdown(j->fd, SHUT_RDWR); close(j->fd); j->fd=-1; }
        }
    }
    rooms_.clear();
}

void RelayServer::accept_loop(){
    while(running_.load()){
        sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
        int cfd = accept(listen_fd_, (sockaddr*)&caddr, &clen);
        if(cfd < 0){ if(running_.load()) perror("accept"); break; }
        int ka = 1; setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        std::thread([this, cfd](){ handshake(cfd); }).detach();
    }
}

// ── 핸드셰이크 ────────────────────────────────────────────────────────────
void RelayServer::handshake(int fd){
    timeval tv{HANDSHAKE_TIMEOUT, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    RelayPktHdr hdr{};
    std::vector<uint8_t> payload;
    if(!relay_recv_pkt(fd, hdr, payload, 65536)){ close(fd); return; }

    // 타임아웃 해제
    timeval tv0{0,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv0, sizeof(tv0));

    auto type = static_cast<RelayPktType>(hdr.type);

    if(type == RelayPktType::HOST_OPEN){
        if(payload.size() < sizeof(RelayHostOpen)){ close(fd); return; }
        auto* op = reinterpret_cast<const RelayHostOpen*>(payload.data());

        auto room = std::make_shared<HostRoom>();
        room->fd      = fd;
        room->last_hb = std::chrono::steady_clock::now();
        room->station_id = std::string(op->station_id,
                               strnlen(op->station_id, sizeof(op->station_id)));
        strncpy(room->info.station_id,   op->station_id,   sizeof(room->info.station_id)-1);
        strncpy(room->info.station_name, op->station_name, sizeof(room->info.station_name)-1);
        room->info.lat        = op->lat;
        room->info.lon        = op->lon;
        room->info.host_tier  = op->host_tier;
        room->info.user_count = 0;

        {
            std::lock_guard<std::mutex> lk(rooms_mtx_);
            rooms_.erase(std::remove_if(rooms_.begin(), rooms_.end(),
                [&](const std::shared_ptr<HostRoom>& r){
                    return r->station_id == room->station_id;
                }), rooms_.end());
            rooms_.push_back(room);
        }
        printf("[Relay] HOST room '%s' (%s) opened\n",
               room->station_id.c_str(), room->info.station_name);

        host_mux_loop(room);

    } else if(type == RelayPktType::JOIN_ROOM){
        if(payload.size() < sizeof(RelayJoinRoom)){ close(fd); return; }
        auto* jr = reinterpret_cast<const RelayJoinRoom*>(payload.data());
        std::string sid(jr->station_id, strnlen(jr->station_id, sizeof(jr->station_id)));

        auto room = find_room(sid);
        if(!room){
            RelayError err{}; strncpy(err.msg, "Room not found", 63);
            relay_send_pkt(fd, RelayPktType::ERROR, &err, sizeof(err));
            close(fd); return;
        }

        auto je = std::make_shared<JoinEntry>();
        je->fd = fd;
        {
            std::lock_guard<std::mutex> lk(room->joins_mtx);
            je->conn_id = room->next_conn_id++;
            if(room->next_conn_id == 0xFFFF) room->next_conn_id = 1;
            room->joins.push_back(je);
            room->info.user_count = (uint8_t)room->joins.size();
        }

        // HOST에게 CONN_OPEN 알림
        {
            std::lock_guard<std::mutex> lk(room->host_send_mtx);
            relay_send_mux(room->fd, je->conn_id, RelayMuxType::CONN_OPEN, nullptr, 0);
        }

        printf("[Relay] JOIN conn_id=%u entered room '%s' (%zu users)\n",
               je->conn_id, sid.c_str(), room->joins.size());

        join_loop(je, room);

    } else if(type == RelayPktType::LIST_REQ){
        handle_list_req(fd);
        close(fd);
    } else {
        close(fd);
    }
}

// ── HOST mux 수신 루프: HOST→relay→JOIN ───────────────────────────────────
// HOST가 보내는 형식: RelayMuxHdr + BEWE 패킷
// conn_id=0xFFFF: 모든 JOIN에게 broadcast
// conn_id=특정값: 해당 JOIN에게만 전송
void RelayServer::host_mux_loop(std::shared_ptr<HostRoom> room){
    std::vector<uint8_t> buf(PIPE_BUF);
    uint64_t mux_pkts = 0;

    while(room->alive.load()){
        RelayMuxHdr mux{};
        if(!relay_recv_all(room->fd, &mux, RELAY_MUX_HDR_SIZE)){
            int e = errno;
            if(room->alive.load())
                printf("[Relay] host_mux_loop recv failed room='%s' errno=%d(%s) pkts=%llu\n",
                       room->station_id.c_str(), e, strerror(e), (unsigned long long)mux_pkts);
            break;
        }

        // HOST_HB인지 확인 (relay 제어 패킷)
        // HOST가 heartbeat를 MUX 스트림 밖으로 보낼 수도 있음 →
        // MUX 헤더 type=0인 경우 relay_pkt 헤더로 재해석
        // 단순화: HOST는 HB도 MUX로 보냄 (type=0x00 예약)
        // → RelayMuxType::DATA=1, CONN_OPEN=2, CONN_CLOSE=3
        // → type=0이면 heartbeat로 처리

        if(mux.type == 0x00){
            // heartbeat
            room->last_hb = std::chrono::steady_clock::now();
            if(mux.len > 0){
                if(buf.size() < mux.len) buf.resize(mux.len);
                if(!relay_recv_all(room->fd, buf.data(), mux.len)) break;
                if(mux.len >= sizeof(RelayHostHb)){
                    auto* hb = reinterpret_cast<const RelayHostHb*>(buf.data());
                    std::lock_guard<std::mutex> jlk(room->joins_mtx);
                    room->info.user_count = hb->user_count;
                }
            }
            continue;
        }

        room->last_hb = std::chrono::steady_clock::now();

        // NET_RESET: 네트워크 리셋 신호
        if(mux.type == static_cast<uint8_t>(RelayMuxType::NET_RESET)){
            if(mux.len > 0){
                uint8_t flag = 0;
                if(!relay_recv_all(room->fd, &flag, 1)) break;
                if(mux.len > 1){
                    // 나머지 바이트 버리기
                    std::vector<uint8_t> discard(mux.len - 1);
                    if(!relay_recv_all(room->fd, discard.data(), mux.len - 1)) break;
                }
                if(flag == 0){
                    room->resetting.store(true);
                    printf("[Relay] room '%s' (%s) Server reset ...\n",
                           room->station_id.c_str(), room->info.station_name);
                } else {
                    room->resetting.store(false);
                    printf("[Relay] room '%s' (%s) Server open ...\n",
                           room->station_id.c_str(), room->info.station_name);
                }
            }
            continue;
        }

        if(mux.len > 4*1024*1024){
            printf("[Relay] host_mux_loop oversized mux.len=%u room='%s'\n",
                   mux.len, room->station_id.c_str());
            break;
        }
        if(buf.size() < mux.len) buf.resize(mux.len);
        if(mux.len > 0 && !relay_recv_all(room->fd, buf.data(), mux.len)){
            int e = errno;
            printf("[Relay] host_mux_loop recv data failed room='%s' len=%u errno=%d(%s)\n",
                   room->station_id.c_str(), mux.len, e, strerror(e));
            break;
        }
        mux_pkts++;

        auto mux_type = static_cast<RelayMuxType>(mux.type);
        if(mux_type != RelayMuxType::DATA || mux.len == 0) continue;

        // JOIN들에게 전달
        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        for(auto& je : room->joins){
            if(!je->alive.load() || je->fd < 0) continue;
            if(mux.conn_id != 0xFFFF && mux.conn_id != je->conn_id) continue;
            send_to_join(je->fd, buf.data(), mux.len);
        }
    }

    // 룸 닫기
    room->alive.store(false);
    if(room->fd >= 0){ shutdown(room->fd, SHUT_RDWR); close(room->fd); room->fd=-1; }
    {
        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        for(auto& je : room->joins){
            je->alive.store(false);
            if(je->fd >= 0){ shutdown(je->fd, SHUT_RDWR); close(je->fd); je->fd=-1; }
        }
        room->joins.clear();
    }
    {
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        rooms_.erase(std::remove_if(rooms_.begin(), rooms_.end(),
            [&](const std::shared_ptr<HostRoom>& r){ return r.get()==room.get(); }),
            rooms_.end());
    }
    printf("[Relay] HOST room '%s' closed\n", room->station_id.c_str());
}

// ── JOIN 수신 루프: JOIN→relay→HOST ───────────────────────────────────────
// JOIN이 보내는 BEWE 패킷을 MUX 헤더 붙여서 HOST에게 전달
void RelayServer::join_loop(std::shared_ptr<JoinEntry> je, std::shared_ptr<HostRoom> room){
    std::vector<uint8_t> buf(PIPE_BUF);
    uint64_t pkt_count = 0;
    const char* disc_reason = "unknown";

    while(je->alive.load() && room->alive.load()){
        // BEWE 패킷 헤더 수신 (9바이트: magic[4]+type[1]+len[4])
        if(!relay_recv_all(je->fd, buf.data(), 9)){
            int e = errno;
            if(je->alive.load())
                printf("[Relay] join_loop recv hdr failed conn_id=%u errno=%d(%s) pkts=%llu\n",
                       je->conn_id, e, strerror(e), (unsigned long long)pkt_count);
            disc_reason = "recv_hdr_fail";
            break;
        }
        uint32_t bewe_len = *reinterpret_cast<uint32_t*>(buf.data() + 5);
        if(bewe_len > 4*1024*1024){
            printf("[Relay] join_loop oversized bewe_len=%u conn_id=%u\n", bewe_len, je->conn_id);
            disc_reason = "oversized";
            break;
        }
        if(buf.size() < 9 + bewe_len) buf.resize(9 + bewe_len);
        if(bewe_len > 0 && !relay_recv_all(je->fd, buf.data()+9, bewe_len)){
            int e = errno;
            printf("[Relay] join_loop recv data failed conn_id=%u bewe_len=%u errno=%d(%s)\n",
                   je->conn_id, bewe_len, e, strerror(e));
            disc_reason = "recv_data_fail";
            break;
        }

        // HOST에게 MUX 헤더 + BEWE 패킷 전달
        std::lock_guard<std::mutex> lk(room->host_send_mtx);
        if(!room->alive.load() || room->fd < 0){
            disc_reason = "host_room_closed";
            break;
        }
        if(!relay_send_mux(room->fd, je->conn_id, RelayMuxType::DATA,
                           buf.data(), 9 + bewe_len)){
            printf("[Relay] join_loop send to host failed conn_id=%u\n", je->conn_id);
            disc_reason = "send_host_fail";
            break;
        }
        pkt_count++;
    }
    printf("[Relay] join_loop exit conn_id=%u reason=%s pkts=%llu\n",
           je->conn_id, disc_reason, (unsigned long long)pkt_count);

    je->alive.store(false);
    if(je->fd >= 0){ shutdown(je->fd, SHUT_RDWR); close(je->fd); je->fd=-1; }

    // HOST에게 CONN_CLOSE 알림
    if(room->alive.load() && room->fd >= 0){
        std::lock_guard<std::mutex> lk(room->host_send_mtx);
        relay_send_mux(room->fd, je->conn_id, RelayMuxType::CONN_CLOSE, nullptr, 0);
    }

    {
        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        room->joins.erase(std::remove_if(room->joins.begin(), room->joins.end(),
            [&](const std::shared_ptr<JoinEntry>& j){ return j.get()==je.get(); }),
            room->joins.end());
        room->info.user_count = (uint8_t)room->joins.size();
        printf("[Relay] JOIN conn_id=%u left room '%s' (%zu users)\n",
               je->conn_id, room->station_id.c_str(), room->joins.size());
    }
}

bool RelayServer::send_to_join(int fd, const uint8_t* buf, size_t len){
    while(len > 0){
        ssize_t r = send(fd, buf, len, MSG_NOSIGNAL);
        if(r <= 0) return false;
        buf += r; len -= r;
    }
    return true;
}

// 자신의 LAN IPv4 주소 목록 수집 (루프백·링크로컬 제외)
static void collect_lan_ips(RelayListResp& resp){
    resp.lan_ip_count = 0;
    memset(resp.lan_ips, 0, sizeof(resp.lan_ips));
    ifaddrs* ifa = nullptr;
    if(getifaddrs(&ifa) != 0) return;
    for(ifaddrs* p = ifa; p; p = p->ifa_next){
        if(!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
        auto* sin = reinterpret_cast<sockaddr_in*>(p->ifa_addr);
        uint32_t ip = ntohl(sin->sin_addr.s_addr);
        // 루프백(127.x), 링크로컬(169.254.x) 제외
        if((ip >> 24) == 127) continue;
        if((ip >> 16) == 0xA9FE) continue; // 169.254.x.x
        if(resp.lan_ip_count >= RELAY_MAX_LAN_IPS) break;
        const char* s = inet_ntoa(sin->sin_addr);
        strncpy(resp.lan_ips[resp.lan_ip_count], s, 15);
        resp.lan_ip_count++;
    }
    freeifaddrs(ifa);
}

void RelayServer::handle_list_req(int fd){
    std::vector<RelayStation> stations;
    {
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        for(auto& r : rooms_)
            if(r->alive.load() && !r->resetting.load() && r->info.station_name[0] != '\0')
                stations.push_back(r->info);
    }
    uint16_t cnt = (uint16_t)stations.size();
    uint32_t plen = sizeof(RelayListResp) + cnt * sizeof(RelayStation);
    std::vector<uint8_t> payload(plen);
    auto* resp = reinterpret_cast<RelayListResp*>(payload.data());
    resp->count = cnt;
    collect_lan_ips(*resp);
    if(cnt > 0)
        memcpy(payload.data()+sizeof(RelayListResp),
               stations.data(), cnt*sizeof(RelayStation));
    relay_send_pkt(fd, RelayPktType::LIST_RESP, payload.data(), plen);
}

std::shared_ptr<HostRoom> RelayServer::find_room(const std::string& id) const {
    std::lock_guard<std::mutex> lk(rooms_mtx_);
    for(auto& r : rooms_)
        if(r->station_id == id && r->alive.load()) return r;
    return nullptr;
}

void RelayServer::watchdog_loop(){
    while(running_.load()){
        std::this_thread::sleep_for(std::chrono::seconds(5));
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        for(auto& r : rooms_){
            if(!r->alive.load()) continue;
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - r->last_hb).count();
            if(age > HOST_TIMEOUT_SEC && r->fd >= 0){
                printf("[Relay] timeout: closing room '%s'\n", r->station_id.c_str());
                shutdown(r->fd, SHUT_RDWR); close(r->fd); r->fd=-1;
                r->alive.store(false);
            }
        }
    }
}
