#include "relay_client.hpp"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <chrono>

// ── TCP 연결 (5초 타임아웃) ───────────────────────────────────────────────
int RelayClient::tcp_connect(const std::string& host, int port){
    addrinfo hints{}, *res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    char ps[8]; snprintf(ps, sizeof(ps), "%d", port);
    if(getaddrinfo(host.c_str(), ps, &hints, &res) != 0) return -1;
    int fd = socket(res->ai_family, res->ai_socktype, 0);
    if(fd < 0){ freeaddrinfo(res); return -1; }
    timeval tv{5,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if(connect(fd, res->ai_addr, res->ai_addrlen) < 0){
        close(fd); freeaddrinfo(res); return -1;
    }
    freeaddrinfo(res);
    timeval tv0{0,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv0, sizeof(tv0));
    return fd;
}

// ── 목록 조회 ─────────────────────────────────────────────────────────────
std::vector<RelayClient::Station>
RelayClient::fetch_stations(const std::string& host, int port){
    std::vector<Station> out;
    int fd = tcp_connect(host, port);
    if(fd < 0) return out;
    timeval tv{5,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if(!relay_send_pkt(fd, RelayPktType::LIST_REQ, nullptr, 0)){ close(fd); return out; }
    RelayPktHdr hdr{}; std::vector<uint8_t> payload;
    if(!relay_recv_pkt(fd, hdr, payload, 256*1024)){ close(fd); return out; }
    close(fd);
    if(static_cast<RelayPktType>(hdr.type) != RelayPktType::LIST_RESP) return out;
    if(payload.size() < sizeof(RelayListResp)) return out;
    auto* resp = reinterpret_cast<const RelayListResp*>(payload.data());
    uint16_t cnt = resp->count;
    const RelayStation* arr = reinterpret_cast<const RelayStation*>(
        payload.data() + sizeof(RelayListResp));
    uint16_t max_cnt = (uint16_t)((payload.size()-sizeof(RelayListResp))/sizeof(RelayStation));
    if(cnt > max_cnt) cnt = max_cnt;
    for(int i = 0; i < (int)cnt; i++){
        Station s;
        s.station_id  = std::string(arr[i].station_id,
                            strnlen(arr[i].station_id, sizeof(arr[i].station_id)));
        s.name        = std::string(arr[i].station_name,
                            strnlen(arr[i].station_name, sizeof(arr[i].station_name)));
        s.lat         = arr[i].lat;
        s.lon         = arr[i].lon;
        s.host_tier   = arr[i].host_tier;
        s.user_count  = arr[i].user_count;
        out.push_back(std::move(s));
    }
    return out;
}

void RelayClient::start_polling(const std::string& host, int port,
                                 std::function<void(const std::vector<Station>&)> cb){
    stop_polling();
    poll_running_.store(true);
    poll_thr_ = std::thread(&RelayClient::poll_loop, this, host, port, std::move(cb));
}

void RelayClient::stop_polling(){
    poll_running_.store(false);
    if(poll_thr_.joinable()) poll_thr_.join();
}

void RelayClient::poll_loop(std::string host, int port,
                             std::function<void(const std::vector<Station>&)> cb){
    while(poll_running_.load()){
        auto st = fetch_stations(host, port);
        cb(st); // 빈 벡터도 전달 (stale 정리용)
        for(int i = 0; i < 100 && poll_running_.load(); i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── HOST: 룸 열기 ─────────────────────────────────────────────────────────
int RelayClient::open_room(const std::string& relay_host, int relay_port,
                            const std::string& station_id, const std::string& station_name,
                            float lat, float lon, uint8_t host_tier){
    int fd = tcp_connect(relay_host, relay_port);
    if(fd < 0){ printf("[RelayClient] open_room: connect failed\n"); return -1; }

    RelayHostOpen op{};
    strncpy(op.station_id,   station_id.c_str(),   sizeof(op.station_id)-1);
    strncpy(op.station_name, station_name.c_str(), sizeof(op.station_name)-1);
    op.lat = lat; op.lon = lon;
    op.host_tier = host_tier; op.user_count = 0;

    if(!relay_send_pkt(fd, RelayPktType::HOST_OPEN, &op, sizeof(op))){
        close(fd); return -1;
    }
    printf("[RelayClient] room '%s' opened\n", station_id.c_str());
    return fd;
}

// ── HOST: MUX 어댑터 ──────────────────────────────────────────────────────
// relay_fd에서 MUX 스트림을 읽어 각 JOIN의 socketpair에 라우팅.
// 새 JOIN(CONN_OPEN) → socketpair 생성 → on_new_join(local_fd) 호출
// 데이터(DATA)        → 해당 join의 remote_fd로 전달
// JOIN 종료(CONN_CLOSE) → socketpair 닫기
// local_fd에서 JOIN이 보내는 데이터 → relay_fd로 MUX해서 전송
void RelayClient::start_mux_adapter(int relay_fd,
                                     std::function<void(int)> on_new_join,
                                     std::function<uint8_t()> user_count_fn){
    stop_mux_adapter();
    mux_relay_fd_ = relay_fd;
    mux_running_.store(true);
    mux_thr_ = std::thread(&RelayClient::mux_loop, this,
                            relay_fd, std::move(on_new_join), std::move(user_count_fn));
}

void RelayClient::stop_mux_adapter(){
    mux_running_.store(false);
    if(mux_relay_fd_ >= 0){ shutdown(mux_relay_fd_, SHUT_RDWR); }
    if(mux_thr_.joinable()) mux_thr_.join();
    std::lock_guard<std::mutex> lk(mux_joins_mtx_);
    for(auto& [id, jp] : mux_joins_){
        if(jp->local_fd  >= 0){ shutdown(jp->local_fd,  SHUT_RDWR); close(jp->local_fd);  }
        if(jp->remote_fd >= 0){ shutdown(jp->remote_fd, SHUT_RDWR); close(jp->remote_fd); }
        if(jp->thr.joinable()) jp->thr.detach();
    }
    mux_joins_.clear();
    mux_relay_fd_ = -1;
}

void RelayClient::mux_loop(int relay_fd,
                            std::function<void(int)> on_new_join,
                            std::function<uint8_t()> count_fn){
    std::vector<uint8_t> buf(256*1024);

    // keepalive heartbeat 타이머
    auto last_hb = std::chrono::steady_clock::now();

    while(mux_running_.load()){
        // MUX 헤더 수신 (non-blocking 타임아웃 3초로 HB 전송 기회 확보)
        timeval tv{3,0};
        setsockopt(relay_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        RelayMuxHdr mux{};
        ssize_t r = recv(relay_fd, &mux, RELAY_MUX_HDR_SIZE, MSG_WAITALL);
        if(r <= 0){
            if(r == 0){
                printf("[RelayClient] mux_loop: relay connection closed (EOF)\n");
                break;
            }
            if(errno != EAGAIN && errno != EWOULDBLOCK && errno != ETIMEDOUT){
                printf("[RelayClient] mux_loop: recv error errno=%d(%s)\n",
                       errno, strerror(errno));
                break;
            }
            // EAGAIN/ETIMEDOUT → heartbeat 전송 후 계속
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(now-last_hb).count() >= 3){
                last_hb = now;
                RelayMuxHdr hb{}; hb.type = 0x00; hb.len = sizeof(RelayHostHb);
                RelayHostHb hbp{}; hbp.user_count = count_fn ? count_fn() : 0;
                {
                    std::lock_guard<std::mutex> wlk(mux_relay_write_mtx_);
                    relay_send_all(relay_fd, &hb, RELAY_MUX_HDR_SIZE);
                    relay_send_all(relay_fd, &hbp, sizeof(hbp));
                }
                printf("[RelayClient] HB sent (users=%u)\n", hbp.user_count);
            }
            continue;
        }
        if(r != RELAY_MUX_HDR_SIZE){
            printf("[RelayClient] mux_loop: short read %zd/%d\n", r, RELAY_MUX_HDR_SIZE);
            break;
        }

        // 타임아웃 해제
        timeval tv0{0,0};
        setsockopt(relay_fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));

        auto mux_type = static_cast<RelayMuxType>(mux.type);
        uint16_t cid  = mux.conn_id;

        if(mux_type == RelayMuxType::CONN_OPEN){
            // 새 JOIN → socketpair 생성
            int sv[2];
            if(socketpair(AF_UNIX, SOCK_STREAM, 0, sv) < 0) continue;
            // sv[0]: NetServer가 accept 대신 직접 client_loop에 inject (local_fd)
            // sv[1]: relay로 데이터 보내는 쪽 (remote_fd)
            auto jp = std::make_shared<JoinPair>();
            jp->local_fd  = sv[0];
            jp->remote_fd = sv[1];
            {
                std::lock_guard<std::mutex> lk(mux_joins_mtx_);
                mux_joins_[cid] = jp;
            }

            // sv[1] → relay_fd pump (JOIN이 local_fd에 쓰는 데이터를 relay로)
            int rfd = relay_fd;
            jp->thr = std::thread([jp, cid, rfd, this](){
                std::vector<uint8_t> tbuf(65536);
                while(jp->local_fd >= 0){
                    ssize_t n = recv(jp->remote_fd, tbuf.data(), tbuf.size(), 0);
                    if(n <= 0) break;
                    RelayMuxHdr mh{}; mh.conn_id=0xFFFF;
                    mh.type = static_cast<uint8_t>(RelayMuxType::DATA);
                    mh.len  = (uint32_t)n;
                    // relay_fd write: mux_loop(HB)와 동시 접근 방지 → mutex
                    std::lock_guard<std::mutex> wlk(mux_relay_write_mtx_);
                    relay_send_all(rfd, &mh, RELAY_MUX_HDR_SIZE);
                    relay_send_all(rfd, tbuf.data(), n);
                }
            });
            jp->thr.detach();

            // NetServer에 새 JOIN 알림 (sv[0]를 client 소켓으로 inject)
            on_new_join(sv[0]);
            printf("[RelayClient] new JOIN conn_id=%u, injected fd=%d\n", cid, sv[0]);

        } else if(mux_type == RelayMuxType::DATA){
            if(mux.len > (uint32_t)buf.size()) buf.resize(mux.len);
            if(mux.len > 0 && !relay_recv_all(relay_fd, buf.data(), mux.len)) break;

            // 특정 JOIN 또는 broadcast
            std::lock_guard<std::mutex> lk(mux_joins_mtx_);
            for(auto& [id, jp] : mux_joins_){
                if(cid != 0xFFFF && id != cid) continue;
                if(jp->remote_fd >= 0)
                    send(jp->remote_fd, buf.data(), mux.len, MSG_NOSIGNAL);
            }

        } else if(mux_type == RelayMuxType::CONN_CLOSE){
            std::lock_guard<std::mutex> lk(mux_joins_mtx_);
            auto it = mux_joins_.find(cid);
            if(it != mux_joins_.end()){
                auto& jp = it->second;
                if(jp->local_fd  >= 0){ shutdown(jp->local_fd,  SHUT_RDWR); close(jp->local_fd);  jp->local_fd=-1;  }
                if(jp->remote_fd >= 0){ shutdown(jp->remote_fd, SHUT_RDWR); close(jp->remote_fd); jp->remote_fd=-1; }
                mux_joins_.erase(it);
                printf("[RelayClient] JOIN conn_id=%u disconnected\n", cid);
            }
        }
    }
    mux_running_.store(false);
}

// ── JOIN 모드: 룸 입장 ────────────────────────────────────────────────────
int RelayClient::join_room(const std::string& relay_host, int relay_port,
                            const std::string& station_id){
    int fd = tcp_connect(relay_host, relay_port);
    if(fd < 0){ printf("[RelayClient] join_room: connect failed\n"); return -1; }

    RelayJoinRoom jr{};
    strncpy(jr.station_id, station_id.c_str(), sizeof(jr.station_id)-1);
    if(!relay_send_pkt(fd, RelayPktType::JOIN_ROOM, &jr, sizeof(jr))){
        close(fd); return -1;
    }

    // ERROR 체크 (룸 없으면 relay가 즉시 ERROR 전송)
    timeval tv{2,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    uint8_t peek[4] = {};
    ssize_t pn = recv(fd, peek, sizeof(peek), MSG_PEEK | MSG_DONTWAIT);
    if(pn >= 4 && memcmp(peek, RELAY_MAGIC, 4) == 0){
        RelayPktHdr ehdr{}; std::vector<uint8_t> ep;
        if(relay_recv_pkt(fd, ehdr, ep, 256)){
            if(static_cast<RelayPktType>(ehdr.type) == RelayPktType::ERROR){
                printf("[RelayClient] join_room error: %s\n",
                       ep.size()>=sizeof(RelayError)
                       ? reinterpret_cast<const RelayError*>(ep.data())->msg : "unknown");
                close(fd); return -1;
            }
        }
    }
    timeval tv0{0,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));

    printf("[RelayClient] joined room '%s'\n", station_id.c_str());
    return fd;
}
