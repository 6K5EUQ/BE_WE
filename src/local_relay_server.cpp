#include "local_relay_server.hpp"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

static constexpr int LRS_HOST_TIMEOUT_SEC  = 15;
static constexpr int LRS_HANDSHAKE_TIMEOUT = 10;
static constexpr size_t LRS_PIPE_BUF       = 65536;

int LocalRelayServer::make_listen_sock(int port){
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0){ perror("socket"); return -1; }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)port);
    if(bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0){ perror("bind"); close(fd); return -1; }
    // port==0 이면 OS가 할당한 실제 포트를 가져옴
    {
        sockaddr_in bound{};
        socklen_t blen = sizeof(bound);
        if(getsockname(fd, (sockaddr*)&bound, &blen) == 0)
            listen_port_val_ = ntohs(bound.sin_port);
        else
            listen_port_val_ = port;
    }
    if(listen(fd, 64) < 0){ perror("listen"); close(fd); return -1; }
    printf("[LocalRelay] listening on port %d\n", listen_port_val_);
    return fd;
}

bool LocalRelayServer::start(int port){
    listen_fd_ = make_listen_sock(port);
    if(listen_fd_ < 0) return false;
    running_.store(true);
    accept_thr_   = std::thread(&LocalRelayServer::accept_loop,   this);
    watchdog_thr_ = std::thread(&LocalRelayServer::watchdog_loop, this);
    return true;
}

void LocalRelayServer::stop(){
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

void LocalRelayServer::accept_loop(){
    while(running_.load()){
        sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
        int cfd = accept(listen_fd_, (sockaddr*)&caddr, &clen);
        if(cfd < 0){ if(running_.load()) perror("accept"); break; }
        int ka = 1; setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        std::thread([this, cfd](){ handshake(cfd); }).detach();
    }
}

void LocalRelayServer::handshake(int fd){
    timeval tv{LRS_HANDSHAKE_TIMEOUT, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    RelayPktHdr hdr{};
    std::vector<uint8_t> payload;
    if(!relay_recv_pkt(fd, hdr, payload, 65536)){ close(fd); return; }

    timeval tv0{0,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv0, sizeof(tv0));

    auto type = static_cast<RelayPktType>(hdr.type);

    if(type == RelayPktType::HOST_OPEN){
        if(payload.size() < sizeof(RelayHostOpen)){ close(fd); return; }
        auto* op = reinterpret_cast<const RelayHostOpen*>(payload.data());

        auto room = std::make_shared<LocalHostRoom>();
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
                [&](const std::shared_ptr<LocalHostRoom>& r){
                    return r->station_id == room->station_id;
                }), rooms_.end());
            rooms_.push_back(room);
        }
        printf("[LocalRelay] HOST room '%s' opened\n", room->station_id.c_str());
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

        auto je = std::make_shared<LocalJoinEntry>();
        je->fd = fd;
        {
            std::lock_guard<std::mutex> lk(room->joins_mtx);
            je->conn_id = room->next_conn_id++;
            if(room->next_conn_id == 0xFFFF) room->next_conn_id = 1;
            room->joins.push_back(je);
            room->info.user_count = (uint8_t)room->joins.size();
        }

        {
            std::lock_guard<std::mutex> lk(room->host_send_mtx);
            relay_send_mux(room->fd, je->conn_id, RelayMuxType::CONN_OPEN, nullptr, 0);
        }

        printf("[LocalRelay] JOIN conn_id=%u entered room '%s'\n",
               je->conn_id, sid.c_str());
        join_loop(je, room);

    } else if(type == RelayPktType::LIST_REQ){
        handle_list_req(fd);
        close(fd);
    } else {
        close(fd);
    }
}

void LocalRelayServer::host_mux_loop(std::shared_ptr<LocalHostRoom> room){
    std::vector<uint8_t> buf(LRS_PIPE_BUF);

    while(room->alive.load()){
        RelayMuxHdr mux{};
        if(!relay_recv_all(room->fd, &mux, RELAY_MUX_HDR_SIZE)){
            if(room->alive.load())
                printf("[LocalRelay] host_mux_loop recv failed room='%s' errno=%d\n",
                       room->station_id.c_str(), errno);
            break;
        }

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

        // NET_RESET: 데이터 소비 후 skip (로컬 릴레이에서는 로그만)
        if(mux.type == static_cast<uint8_t>(RelayMuxType::NET_RESET)){
            if(mux.len > 0){
                if(buf.size() < mux.len) buf.resize(mux.len);
                if(!relay_recv_all(room->fd, buf.data(), mux.len)) break;
            }
            continue;
        }

        if(mux.len > 4*1024*1024) break;
        if(buf.size() < mux.len) buf.resize(mux.len);
        if(mux.len > 0 && !relay_recv_all(room->fd, buf.data(), mux.len)) break;

        auto mux_type = static_cast<RelayMuxType>(mux.type);
        if(mux_type != RelayMuxType::DATA || mux.len == 0) continue;

        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        for(auto& je : room->joins){
            if(!je->alive.load() || je->fd < 0) continue;
            if(mux.conn_id != 0xFFFF && mux.conn_id != je->conn_id) continue;
            send_to_join(je->fd, buf.data(), mux.len);
        }
    }

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
            [&](const std::shared_ptr<LocalHostRoom>& r){ return r.get()==room.get(); }),
            rooms_.end());
    }
    printf("[LocalRelay] HOST room '%s' closed\n", room->station_id.c_str());
}

void LocalRelayServer::join_loop(std::shared_ptr<LocalJoinEntry> je,
                                  std::shared_ptr<LocalHostRoom> room){
    std::vector<uint8_t> buf(LRS_PIPE_BUF);

    while(je->alive.load() && room->alive.load()){
        if(!relay_recv_all(je->fd, buf.data(), 9)){
            if(je->alive.load())
                printf("[LocalRelay] join_loop recv hdr failed conn_id=%u errno=%d\n",
                       je->conn_id, errno);
            break;
        }
        uint32_t bewe_len = *reinterpret_cast<uint32_t*>(buf.data() + 5);
        if(bewe_len > 4*1024*1024) break;
        if(buf.size() < 9 + bewe_len) buf.resize(9 + bewe_len);
        if(bewe_len > 0 && !relay_recv_all(je->fd, buf.data()+9, bewe_len)) break;

        std::lock_guard<std::mutex> lk(room->host_send_mtx);
        if(!room->alive.load() || room->fd < 0) break;
        if(!relay_send_mux(room->fd, je->conn_id, RelayMuxType::DATA,
                           buf.data(), 9 + bewe_len)) break;
    }

    je->alive.store(false);
    if(je->fd >= 0){ shutdown(je->fd, SHUT_RDWR); close(je->fd); je->fd=-1; }

    if(room->alive.load() && room->fd >= 0){
        std::lock_guard<std::mutex> lk(room->host_send_mtx);
        relay_send_mux(room->fd, je->conn_id, RelayMuxType::CONN_CLOSE, nullptr, 0);
    }

    {
        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        room->joins.erase(std::remove_if(room->joins.begin(), room->joins.end(),
            [&](const std::shared_ptr<LocalJoinEntry>& j){ return j.get()==je.get(); }),
            room->joins.end());
        room->info.user_count = (uint8_t)room->joins.size();
        printf("[LocalRelay] JOIN conn_id=%u left room '%s'\n",
               je->conn_id, room->station_id.c_str());
    }
}

bool LocalRelayServer::send_to_join(int fd, const uint8_t* buf, size_t len){
    while(len > 0){
        ssize_t r = send(fd, buf, len, MSG_NOSIGNAL);
        if(r <= 0) return false;
        buf += r; len -= r;
    }
    return true;
}

void LocalRelayServer::handle_list_req(int fd){
    std::vector<RelayStation> stations;
    {
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        for(auto& r : rooms_)
            if(r->alive.load() && r->info.station_name[0] != '\0')
                stations.push_back(r->info);
    }
    uint16_t cnt = (uint16_t)stations.size();
    uint32_t plen = sizeof(RelayListResp) + cnt * sizeof(RelayStation);
    std::vector<uint8_t> payload(plen);
    auto* resp = reinterpret_cast<RelayListResp*>(payload.data());
    resp->count = cnt;
    resp->lan_ip_count = 0; // local relay: LAN IP 불필요
    if(cnt > 0)
        memcpy(payload.data()+sizeof(RelayListResp),
               stations.data(), cnt*sizeof(RelayStation));
    relay_send_pkt(fd, RelayPktType::LIST_RESP, payload.data(), plen);
}

std::shared_ptr<LocalHostRoom> LocalRelayServer::find_room(const std::string& id) const {
    std::lock_guard<std::mutex> lk(rooms_mtx_);
    for(auto& r : rooms_)
        if(r->station_id == id && r->alive.load()) return r;
    return nullptr;
}

void LocalRelayServer::watchdog_loop(){
    while(running_.load()){
        std::this_thread::sleep_for(std::chrono::seconds(5));
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        for(auto& r : rooms_){
            if(!r->alive.load()) continue;
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - r->last_hb).count();
            if(age > LRS_HOST_TIMEOUT_SEC && r->fd >= 0){
                printf("[LocalRelay] timeout: closing room '%s'\n", r->station_id.c_str());
                shutdown(r->fd, SHUT_RDWR); close(r->fd); r->fd=-1;
                r->alive.store(false);
            }
        }
    }
}
