#include "central_server.hpp"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <unistd.h>

static constexpr int HOST_TIMEOUT_SEC  = 20;  // HB 간격(3s) + 데이터 프레임 SNDTIMEO(3s) + 마진
static constexpr int HANDSHAKE_TIMEOUT = 10;
static constexpr size_t PIPE_BUF       = 65536;

// HOST fd에 보낼 MUX 패킷을 큐에 enqueue (non-blocking, host_mux_loop이 flush)
static void enqueue_host_send(std::shared_ptr<HostRoom>& room, uint16_t conn_id,
                              CentralMuxType type, const void* data, uint32_t len){
    CentralMuxHdr mh{};
    mh.conn_id = conn_id;
    mh.type = static_cast<uint8_t>(type);
    mh.len = len;
    std::vector<uint8_t> pkt(CENTRAL_MUX_HDR_SIZE + len);
    memcpy(pkt.data(), &mh, CENTRAL_MUX_HDR_SIZE);
    if(len > 0 && data) memcpy(pkt.data() + CENTRAL_MUX_HDR_SIZE, data, len);
    std::lock_guard<std::mutex> lk(room->host_send_mtx);
    room->host_send_queue.push_back(std::move(pkt));
}

// HOST send 큐를 flush (host_mux_loop에서 호출)
// blocking send: CONN_OPEN/CLOSE 같은 제어 패킷은 절대 드롭하면 안 됨
// SO_SNDTIMEO=5s 설정 되어 있으므로 무한 블로킹 없음
static void flush_host_send_queue(std::shared_ptr<HostRoom>& room){
    std::deque<std::vector<uint8_t>> batch;
    {
        std::lock_guard<std::mutex> lk(room->host_send_mtx);
        batch.swap(room->host_send_queue);
    }
    for(auto& pkt : batch){
        if(room->fd < 0 || !room->alive.load()) break;
        size_t sent = 0;
        while(sent < pkt.size()){
            ssize_t r = ::send(room->fd, pkt.data()+sent, pkt.size()-sent, MSG_NOSIGNAL);
            if(r > 0){ sent += r; continue; }
            if(r < 0 && errno == EINTR) continue;
            // 에러 (EPIPE, ETIMEDOUT 등) → 룸 종료
            printf("[Central] flush_host_send: send error errno=%d(%s) room='%s'\n",
                   errno, strerror(errno), room->station_id.c_str());
            room->alive.store(false);
            break;
        }
        if(!room->alive.load()) break;
    }
}

// ── BEWE 타입 이름 (디버그용) ──────────────────────────────────────────────
static const char* bewe_type_name(uint8_t t){
    switch(t){
        case BEWE_TYPE_AUTH_REQ:  return "AUTH_REQ";
        case BEWE_TYPE_AUTH_ACK:  return "AUTH_ACK";
        case BEWE_TYPE_FFT:      return "FFT";
        case BEWE_TYPE_AUDIO:    return "AUDIO";
        case BEWE_TYPE_CMD:      return "CMD";
        case BEWE_TYPE_CHAT:     return "CHAT";
        case BEWE_TYPE_STATUS:   return "STATUS";
        case BEWE_TYPE_OP_LIST:  return "OP_LIST";
        case BEWE_TYPE_CH_SYNC:  return "CH_SYNC";
        case BEWE_TYPE_HEARTBEAT:return "HEARTBEAT";
        default: return "UNKNOWN";
    }
}

static const char* mux_type_name(uint8_t t){
    switch(t){
        case 0x00: return "HB";
        case (uint8_t)CentralMuxType::DATA:       return "DATA";
        case (uint8_t)CentralMuxType::CONN_OPEN:  return "CONN_OPEN";
        case (uint8_t)CentralMuxType::CONN_CLOSE: return "CONN_CLOSE";
        case (uint8_t)CentralMuxType::NET_RESET:  return "NET_RESET";
        default: return "UNKNOWN_MUX";
    }
}

int CentralServer::make_listen_sock(int port){
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
    printf("[Central] listening on port %d\n", port);
    return fd;
}

bool CentralServer::start(int port){
    listen_fd_ = make_listen_sock(port);
    if(listen_fd_ < 0) return false;
    running_.store(true);
    accept_thr_   = std::thread(&CentralServer::accept_loop,   this);
    watchdog_thr_ = std::thread(&CentralServer::watchdog_loop, this);
    return true;
}

void CentralServer::run(){
    while(running_.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void CentralServer::stop(){
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
            j->stop_send_worker();
        }
    }
    rooms_.clear();
}

void CentralServer::accept_loop(){
    while(running_.load()){
        sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
        int cfd = accept(listen_fd_, (sockaddr*)&caddr, &clen);
        if(cfd < 0){ if(running_.load()) perror("accept"); break; }

        char ip_str[INET_ADDRSTRLEN] = {};
        inet_ntop(AF_INET, &caddr.sin_addr, ip_str, sizeof(ip_str));
        printf("[Central] accept: fd=%d from %s:%d\n", cfd, ip_str, ntohs(caddr.sin_port));

        int ka = 1; setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        int bufsize = 4 * 1024 * 1024;  // 4MB (HOST FFT 스트림 버스트 흡수)
        setsockopt(cfd, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize));
        setsockopt(cfd, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
        int nd = 1; setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
        std::thread([this, cfd](){ handshake(cfd); }).detach();
    }
}

// ── 핸드셰이크 ────────────────────────────────────────────────────────────
void CentralServer::handshake(int fd){
    timeval tv{HANDSHAKE_TIMEOUT, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    CentralPktHdr hdr{};
    std::vector<uint8_t> payload;
    if(!central_recv_pkt(fd, hdr, payload, 65536)){
        printf("[Central] handshake: recv failed fd=%d errno=%d(%s)\n", fd, errno, strerror(errno));
        close(fd); return;
    }

    // recv: 타임아웃 해제 (host_mux_loop에서 개별 설정)
    // send: 5s 타임아웃 유지 (join_loop→HOST send가 무한 블로킹되는 것 방지)
    timeval tv0{0,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));
    timeval stv{5,0};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

    auto type = static_cast<CentralPktType>(hdr.type);

    if(type == CentralPktType::HOST_OPEN){
        if(payload.size() < sizeof(CentralHostOpen)){ close(fd); return; }
        auto* op = reinterpret_cast<const CentralHostOpen*>(payload.data());

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
        // 중앙 서버: HOST 정보 저장
        strncpy(room->host_name, op->station_name, 31);
        room->host_tier = op->host_tier;

        {
            std::lock_guard<std::mutex> lk(rooms_mtx_);
            rooms_.erase(std::remove_if(rooms_.begin(), rooms_.end(),
                [&](const std::shared_ptr<HostRoom>& r){
                    return r->station_id == room->station_id;
                }), rooms_.end());
            rooms_.push_back(room);
        }
        printf("[Central] HOST room '%s' (%s) opened  fd=%d\n",
               room->station_id.c_str(), room->info.station_name, fd);

        host_mux_loop(room);

    } else if(type == CentralPktType::JOIN_ROOM){
        if(payload.size() < sizeof(CentralJoinRoom)){ close(fd); return; }
        auto* jr = reinterpret_cast<const CentralJoinRoom*>(payload.data());
        std::string sid(jr->station_id, strnlen(jr->station_id, sizeof(jr->station_id)));

        auto room = find_room(sid);
        if(!room){
            CentralError err{}; strncpy(err.msg, "Room not found", 63);
            central_send_pkt(fd, CentralPktType::ERROR, &err, sizeof(err));
            printf("[Central] JOIN rejected: room '%s' not found\n", sid.c_str());
            close(fd); return;
        }

        auto je = std::make_shared<JoinEntry>();
        je->fd = fd;
        // JOIN fd에 send 타임아웃 설정: 느린 JOIN이 send worker를 무한 블로킹하지 않도록
        timeval jtv{5, 0};
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &jtv, sizeof(jtv));
        je->start_send_worker();
        {
            std::lock_guard<std::mutex> lk(room->joins_mtx);
            je->conn_id = room->next_conn_id++;
            if(room->next_conn_id == 0xFFFF) room->next_conn_id = 1;
            room->joins.push_back(je);
            room->info.user_count = (uint8_t)room->joins.size();
        }

        // HOST에게 CONN_OPEN 알림 (큐 경유 → host_mux_loop이 flush)
        enqueue_host_send(room, je->conn_id, CentralMuxType::CONN_OPEN, nullptr, 0);

        printf("[Central] JOIN conn_id=%u entered room '%s' (%zu users) fd=%d\n",
               je->conn_id, sid.c_str(), room->joins.size(), fd);

        // 캐시 전송은 AUTH_ACK 통과 후 (dispatch_to_joins에서 처리)
        // JOIN이 AUTH_ACK를 동기 대기하므로 그 전에 다른 패킷을 보내면 안 됨

        join_loop(je, room);

    } else if(type == CentralPktType::LIST_REQ){
        handle_list_req(fd);
        close(fd);
    } else {
        printf("[Central] handshake: unknown type=0x%02x fd=%d\n", hdr.type, fd);
        close(fd);
    }
}

// ── HOST mux 수신 루프: HOST→relay→JOIN ───────────────────────────────────
void CentralServer::host_mux_loop(std::shared_ptr<HostRoom> room){
    std::vector<uint8_t> buf(PIPE_BUF);
    uint64_t mux_pkts = 0;
    uint64_t hb_count = 0;
    uint64_t fft_count = 0, audio_count = 0, other_count = 0;
    uint64_t recv_bytes = 0;
    auto last_stat = std::chrono::steady_clock::now();
    auto start_time = last_stat;

    printf("[Central] host_mux_loop started room='%s' fd=%d\n",
           room->station_id.c_str(), room->fd);

    while(room->alive.load()){
        // HOST send 큐 flush (join_loop 등이 enqueue한 제어 패킷을 HOST에 전달)
        flush_host_send_queue(room);

        CentralMuxHdr mux{};
        if(!central_recv_all(room->fd, &mux, CENTRAL_MUX_HDR_SIZE)){
            int e = errno;
            if(room->alive.load())
                printf("[Central] host_mux_loop recv FAILED room='%s' errno=%d(%s) pkts=%llu hb=%llu\n",
                       room->station_id.c_str(), e, strerror(e),
                       (unsigned long long)mux_pkts, (unsigned long long)hb_count);
            break;
        }
        recv_bytes += CENTRAL_MUX_HDR_SIZE;

        // 10초마다 통계
        auto now_s = std::chrono::steady_clock::now();
        auto stat_elapsed = std::chrono::duration_cast<std::chrono::seconds>(now_s - last_stat).count();
        if(stat_elapsed >= 10){
            auto total_sec = std::chrono::duration_cast<std::chrono::seconds>(now_s - start_time).count();
            size_t join_count = 0;
            {
                std::lock_guard<std::mutex> jlk(room->joins_mtx);
                join_count = room->joins.size();
            }
            printf("[Central] [STATS] room='%s' uptime=%llds | recv: pkts=%llu bytes=%llu (%.1f KB/s) | "
                   "hb=%llu fft=%llu audio=%llu other=%llu | joins=%zu\n",
                   room->station_id.c_str(),
                   (long long)total_sec,
                   (unsigned long long)mux_pkts,
                   (unsigned long long)recv_bytes,
                   (double)recv_bytes / std::max((long long)1, (long long)total_sec) / 1024.0,
                   (unsigned long long)hb_count,
                   (unsigned long long)fft_count,
                   (unsigned long long)audio_count,
                   (unsigned long long)other_count,
                   join_count);
            last_stat = now_s;
        }

        // ── HB ─────────────────────────────────────────────────────────────
        if(mux.type == 0x00){
            room->last_hb = std::chrono::steady_clock::now();
            hb_count++;
            if(mux.len > 0){
                if(buf.size() < mux.len) buf.resize(mux.len);
                if(!central_recv_all(room->fd, buf.data(), mux.len)){
                    printf("[Central] host_mux_loop HB payload recv failed room='%s'\n",
                           room->station_id.c_str());
                    break;
                }
                recv_bytes += mux.len;
                if(mux.len >= sizeof(CentralHostHb)){
                    auto* hb = reinterpret_cast<const CentralHostHb*>(buf.data());
                    std::lock_guard<std::mutex> jlk(room->joins_mtx);
                    room->info.user_count = hb->user_count;
                }
            }
            continue;
        }

        room->last_hb = std::chrono::steady_clock::now();

        // ── NET_RESET ───────────────────────────────────────────────────────
        if(mux.type == static_cast<uint8_t>(CentralMuxType::NET_RESET)){
            if(mux.len > 0){
                uint8_t flag = 0;
                if(!central_recv_all(room->fd, &flag, 1)) break;
                recv_bytes += 1;
                if(mux.len > 1){
                    std::vector<uint8_t> discard(mux.len - 1);
                    if(!central_recv_all(room->fd, discard.data(), mux.len - 1)) break;
                    recv_bytes += mux.len - 1;
                }
                if(flag == 0){
                    room->resetting.store(true);
                    printf("[Central] room '%s' (%s) NET_RESET start\n",
                           room->station_id.c_str(), room->info.station_name);
                } else {
                    room->resetting.store(false);
                    printf("[Central] room '%s' (%s) NET_RESET open\n",
                           room->station_id.c_str(), room->info.station_name);
                }
            }
            continue;
        }

        // ── DATA 검증 ──────────────────────────────────────────────────────
        if(mux.len > 4*1024*1024){
            printf("[Central] host_mux_loop OVERSIZED mux: type=%s conn_id=%u len=%u room='%s'\n",
                   mux_type_name(mux.type), mux.conn_id, mux.len, room->station_id.c_str());
            break;
        }
        if(buf.size() < mux.len) buf.resize(mux.len);
        if(mux.len > 0 && !central_recv_all(room->fd, buf.data(), mux.len)){
            int e = errno;
            printf("[Central] host_mux_loop recv DATA failed room='%s' mux_type=%s len=%u errno=%d(%s)\n",
                   room->station_id.c_str(), mux_type_name(mux.type), mux.len, e, strerror(e));
            break;
        }
        recv_bytes += mux.len;
        mux_pkts++;

        auto mux_type = static_cast<CentralMuxType>(mux.type);
        if(mux_type != CentralMuxType::DATA || mux.len == 0) continue;

        // BEWE 타입별 카운트
        if(mux.len >= BEWE_HDR_SIZE){
            uint8_t bt = buf[4];
            if(bt == BEWE_TYPE_FFT)        fft_count++;
            else if(bt == BEWE_TYPE_AUDIO) audio_count++;
            else                           other_count++;
        }

        // BEWE 패킷 중앙 처리: 오디오 필터링, CHANNEL_SYNC 인터셉트
        dispatch_to_joins(room, mux.conn_id, buf.data(), mux.len);
    }

    // 룸 닫기
    printf("[Central] host_mux_loop EXIT room='%s' pkts=%llu hb=%llu fft=%llu audio=%llu recv=%.1fMB\n",
           room->station_id.c_str(),
           (unsigned long long)mux_pkts, (unsigned long long)hb_count,
           (unsigned long long)fft_count, (unsigned long long)audio_count,
           (double)recv_bytes / (1024.0*1024.0));

    room->alive.store(false);
    if(room->fd >= 0){ shutdown(room->fd, SHUT_RDWR); close(room->fd); room->fd=-1; }
    {
        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        for(auto& je : room->joins){
            je->alive.store(false);
            if(je->fd >= 0){ shutdown(je->fd, SHUT_RDWR); close(je->fd); je->fd=-1; }
            je->stop_send_worker();
        }
        room->joins.clear();
    }
    {
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        rooms_.erase(std::remove_if(rooms_.begin(), rooms_.end(),
            [&](const std::shared_ptr<HostRoom>& r){ return r.get()==room.get(); }),
            rooms_.end());
    }
    printf("[Central] HOST room '%s' closed\n", room->station_id.c_str());
}

// ── JOIN 수신 루프: JOIN→relay→HOST ───────────────────────────────────────
void CentralServer::join_loop(std::shared_ptr<JoinEntry> je, std::shared_ptr<HostRoom> room){
    std::vector<uint8_t> buf(PIPE_BUF);
    uint64_t pkt_count = 0;
    const char* disc_reason = "unknown";

    while(je->alive.load() && room->alive.load()){
        // BEWE 패킷 헤더 수신 (9바이트: magic[4]+type[1]+len[4])
        if(!central_recv_all(je->fd, buf.data(), 9)){
            int e = errno;
            if(je->alive.load())
                printf("[Central] join_loop recv hdr failed conn_id=%u errno=%d(%s) pkts=%llu\n",
                       je->conn_id, e, strerror(e), (unsigned long long)pkt_count);
            disc_reason = "recv_hdr_fail";
            break;
        }
        uint32_t bewe_len = *reinterpret_cast<uint32_t*>(buf.data() + 5);
        if(bewe_len > 4*1024*1024){
            printf("[Central] join_loop oversized bewe_len=%u conn_id=%u\n", bewe_len, je->conn_id);
            disc_reason = "oversized";
            break;
        }
        if(buf.size() < 9 + bewe_len) buf.resize(9 + bewe_len);
        if(bewe_len > 0 && !central_recv_all(je->fd, buf.data()+9, bewe_len)){
            int e = errno;
            printf("[Central] join_loop recv data failed conn_id=%u bewe_len=%u errno=%d(%s)\n",
                   je->conn_id, bewe_len, e, strerror(e));
            disc_reason = "recv_data_fail";
            break;
        }

        uint8_t bewe_type = (9 + bewe_len > 4) ? buf[4] : 0xFF;
        printf("[Central] JOIN→HOST conn_id=%u bewe_type=%s(0x%02x) len=%u\n",
               je->conn_id, bewe_type_name(bewe_type), bewe_type, 9 + bewe_len);

        // JOIN→HOST: 릴레이가 처리할 명령은 인터셉트
        bool consumed = intercept_join_cmd(je, room, buf.data(), 9 + bewe_len);
        if(consumed) { pkt_count++; continue; }

        // HOST에게 MUX 헤더 + BEWE 패킷 전달 (큐 경유)
        if(!room->alive.load() || room->fd < 0){
            disc_reason = "host_room_closed";
            break;
        }
        enqueue_host_send(room, je->conn_id, CentralMuxType::DATA, buf.data(), 9 + bewe_len);
        pkt_count++;
    }
    printf("[Central] join_loop EXIT conn_id=%u '%s' reason=%s pkts=%llu\n",
           je->conn_id, je->name, disc_reason, (unsigned long long)pkt_count);

    je->alive.store(false);
    je->stop_send_worker();
    if(je->fd >= 0){ shutdown(je->fd, SHUT_RDWR); close(je->fd); je->fd=-1; }

    // HOST에게 CONN_CLOSE 알림 (큐 경유)
    if(room->alive.load() && room->fd >= 0)
        enqueue_host_send(room, je->conn_id, CentralMuxType::CONN_CLOSE, nullptr, 0);

    {
        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        room->joins.erase(std::remove_if(room->joins.begin(), room->joins.end(),
            [&](const std::shared_ptr<JoinEntry>& j){ return j.get()==je.get(); }),
            room->joins.end());
        room->info.user_count = (uint8_t)room->joins.size();
        printf("[Central] JOIN conn_id=%u '%s' left room '%s' (%zu users)\n",
               je->conn_id, je->name, room->station_id.c_str(), room->joins.size());
    }
    // OPERATOR_LIST 갱신 (JOIN 나간 후)
    if(room->alive.load() && je->authed)
        build_and_broadcast_op_list(room);
}

// ── BEWE 패킷 중앙 처리: HOST→JOIN 방향 ──────────────────────────────────
void CentralServer::dispatch_to_joins(std::shared_ptr<HostRoom> room,
                                     uint16_t conn_id,
                                     const uint8_t* bewe_pkt, size_t bewe_len){
    if(bewe_len < BEWE_HDR_SIZE) return;
    uint8_t bewe_type = bewe_pkt[4]; // BEWE 패킷 타입

    // ── AUTH_ACK: HOST → JOIN 통과. 릴레이는 op_index 캐시 + 캐시 전송
    if(bewe_type == BEWE_TYPE_AUTH_ACK && bewe_len >= BEWE_HDR_SIZE + 2){
        uint8_t ok = bewe_pkt[BEWE_HDR_SIZE];
        uint8_t op_idx = bewe_pkt[BEWE_HDR_SIZE + 1];
        if(ok && conn_id != 0xFFFF){
            std::shared_ptr<JoinEntry> target;
            {
                std::lock_guard<std::mutex> jlk(room->joins_mtx);
                for(auto& je : room->joins){
                    if(je->conn_id == conn_id){
                        je->op_index = op_idx;
                        je->authed   = true;
                        target = je;
                        printf("[Central] AUTH_ACK ok=%u op=%u '%s' room='%s' conn_id=%u\n",
                               ok, op_idx, je->name, room->station_id.c_str(), conn_id);
                        break;
                    }
                }
            }
            // AUTH_ACK를 먼저 전달한 후, 캐시된 상태 패킷 전송 (모두 ctrl 큐로 → 즉시 전송)
            if(target){
                target->enqueue_ctrl(bewe_pkt, bewe_len);  // AUTH_ACK 먼저
                std::vector<uint8_t> c_hb, c_st, c_ch, c_op;
                {
                    std::lock_guard<std::mutex> clk(room->cache_mtx);
                    c_hb = room->cached_heartbeat;
                    c_st = room->cached_status;
                    c_ch = room->cached_ch_sync;
                    c_op = room->cached_op_list;
                }
                int cache_count = 0;
                if(!c_hb.empty()){ target->enqueue_ctrl(c_hb.data(), c_hb.size()); cache_count++; }
                if(!c_st.empty()){ target->enqueue_ctrl(c_st.data(), c_st.size()); cache_count++; }
                if(!c_ch.empty()){ target->enqueue_ctrl(c_ch.data(), c_ch.size()); cache_count++; }
                if(!c_op.empty()){ target->enqueue_ctrl(c_op.data(), c_op.size()); cache_count++; }
                printf("[Central] sent %d cached packets to conn_id=%u\n", cache_count, conn_id);
                // OPERATOR_LIST 갱신 (새 유저 반영)
                build_and_broadcast_op_list(room);
                return;
            }
        }
        // ok=0 또는 broadcast → 그대로 전달
    }

    // ── 상태 패킷 캐시 (새 JOIN 접속 시 즉시 전송용) ────────────────
    if(bewe_type == BEWE_TYPE_HEARTBEAT || bewe_type == BEWE_TYPE_STATUS){
        std::lock_guard<std::mutex> clk(room->cache_mtx);
        auto& cache = (bewe_type == BEWE_TYPE_HEARTBEAT) ? room->cached_heartbeat
                                                         : room->cached_status;
        cache.assign(bewe_pkt, bewe_pkt + bewe_len);
    }

    // ── CHANNEL_SYNC 인터셉트: 캐시 저장 → audio_mask 재작성 후 broadcast
    // (rebuild_and_broadcast_ch_sync 내부에서 cache_mtx를 잡으므로 여기서 중복 잠금 없이 처리)
    if(bewe_type == BEWE_TYPE_CH_SYNC){
        {
            std::lock_guard<std::mutex> clk(room->cache_mtx);
            room->cached_ch_sync.assign(bewe_pkt, bewe_pkt + bewe_len);
        }
        // host_mux_loop 경유: HOST에 send하면 데드락 → send_to_host=false
        rebuild_and_broadcast_ch_sync(room, /*send_to_host=*/false);
        return;
    }

    // ── AUDIO_FRAME: 릴레이가 뮤트 테이블 기반으로 필터링 ─────────────
    if(bewe_type == BEWE_TYPE_AUDIO){
        if(bewe_len < BEWE_HDR_SIZE + 1) return;
        uint8_t ch_idx = bewe_pkt[BEWE_HDR_SIZE];
        if(ch_idx >= MAX_CHANNELS_RELAY) return;

        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        for(auto& je : room->joins){
            if(!je->alive.load() || je->fd < 0 || !je->authed) continue;
            if(conn_id != 0xFFFF && conn_id != je->conn_id) continue;
            if(!je->recv_audio[ch_idx]) continue;
            je->enqueue_audio(bewe_pkt, bewe_len);
        }
        return;
    }

    // ── CHAT: 전역 브로드캐스트 (모든 방의 JOIN + 다른 방 HOST) ─────────
    if(bewe_type == BEWE_TYPE_CHAT){
        broadcast_global_chat(bewe_pkt, bewe_len, room.get());
        return;
    }

    // ── 기타 패킷 (FFT, HEARTBEAT, STATUS 등): 타입에 따라 분류 ─────────
    // FFT: auth 완료된 JOIN에게만, 전용 send_queue (대용량, 드롭 허용)
    // 제어(HEARTBEAT/STATUS/CMD_ACK 등): ctrl_queue (우선 전송, 드롭 없음)
    bool is_fft = (bewe_type == BEWE_TYPE_FFT);
    bool is_ctrl = (bewe_type == BEWE_TYPE_HEARTBEAT || bewe_type == BEWE_TYPE_STATUS ||
                    bewe_type == BEWE_TYPE_CMD || bewe_type == BEWE_TYPE_OP_LIST ||
                    bewe_type == BEWE_TYPE_AUTH_ACK);
    std::lock_guard<std::mutex> jlk(room->joins_mtx);
    for(auto& je : room->joins){
        if(!je->alive.load() || je->fd < 0) continue;
        if(conn_id != 0xFFFF && conn_id != je->conn_id) continue;
        if(is_fft && !je->authed) continue;
        if(is_ctrl)
            je->enqueue_ctrl(bewe_pkt, bewe_len);
        else
            je->enqueue_data(bewe_pkt, bewe_len);
    }
}

// ── JOIN→HOST 방향: 릴레이 인터셉트 ──────────────────────────────────────
bool CentralServer::intercept_join_cmd(std::shared_ptr<JoinEntry> je,
                                      std::shared_ptr<HostRoom> room,
                                      const uint8_t* bewe_pkt, size_t bewe_len){
    if(bewe_len < BEWE_HDR_SIZE) return false;
    uint8_t bewe_type = bewe_pkt[4];

    // ── AUTH_REQ: HOST에 포워드 (HOST가 처리), 릴레이는 이름만 캐시 ──
    if(bewe_type == BEWE_TYPE_AUTH_REQ){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(plen >= (size_t)BEWE_AUTH_REQ_SIZE){
            memcpy(je->name, payload, 31);  // id
            je->tier = payload[96];          // tier
            printf("[Central] AUTH_REQ intercepted: conn_id=%u name='%s' tier=%u\n",
                   je->conn_id, je->name, je->tier);
        }
        return false;  // HOST에 포워드
    }

    // ── CMD 패킷 인터셉트 ────────────────────────────────────────────
    if(bewe_type == BEWE_TYPE_CMD){
        const uint8_t* cmd_payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t cmd_len = bewe_len - BEWE_HDR_SIZE;
        if(cmd_len < 4) return false;

        uint8_t cmd_type = cmd_payload[0];

        // TOGGLE_RECV: 릴레이에서만 처리 (HOST에 포워드 안 함)
        if(cmd_type == BEWE_CMD_TOGGLE_RECV && cmd_len >= 6){
            uint8_t ch_idx = cmd_payload[4];
            uint8_t enable = cmd_payload[5];
            if(ch_idx < MAX_CHANNELS_RELAY){
                je->recv_audio[ch_idx] = (enable != 0);
                printf("[Central] TOGGLE_RECV conn_id=%u ch=%u enable=%u\n",
                       je->conn_id, ch_idx, enable);
                // join_loop 경유: HOST에 send OK
                rebuild_and_broadcast_ch_sync(room, /*send_to_host=*/true);
            }
            return true;  // HOST에 포워드 안 함
        }
        return false;  // 다른 CMD는 HOST에 포워드
    }

    // ── CHAT: 전역 브로드캐스트 (모든 방 JOIN + 다른 방 HOST) ────────
    if(bewe_type == BEWE_TYPE_CHAT){
        broadcast_global_chat(bewe_pkt, bewe_len, room.get());
        return false;  // 소스 방 HOST에도 포워드
    }

    return false;
}

// ── BEWE 패킷 빌드 ──────────────────────────────────────────────────────
std::vector<uint8_t> CentralServer::make_bewe_packet(uint8_t type,
                                                     const void* payload, uint32_t plen){
    std::vector<uint8_t> pkt(BEWE_HDR_SIZE + plen);
    pkt[0]='B'; pkt[1]='E'; pkt[2]='W'; pkt[3]='E';
    pkt[4] = type;
    memcpy(pkt.data()+5, &plen, 4);
    if(plen > 0 && payload)
        memcpy(pkt.data()+BEWE_HDR_SIZE, payload, plen);
    return pkt;
}

// ── OPERATOR_LIST 빌드 + broadcast ───────────────────────────────────────
void CentralServer::build_and_broadcast_op_list(std::shared_ptr<HostRoom> room){
    uint8_t buf[1 + BEWE_MAX_OPERATORS * BEWE_OP_ENTRY_SIZE] = {};
    int count = 0;

    uint8_t* p = buf + 1;
    p[0] = 0;  // index=0 (HOST)
    p[1] = room->host_tier;
    strncpy((char*)(p+2), room->host_name, 31);
    p += BEWE_OP_ENTRY_SIZE;
    count++;

    std::lock_guard<std::mutex> jlk(room->joins_mtx);
    for(auto& je : room->joins){
        if(!je->authed || !je->alive.load()) continue;
        if(count >= BEWE_MAX_OPERATORS) break;
        p[0] = je->op_index;
        p[1] = je->tier;
        strncpy((char*)(p+2), je->name, 31);
        p += BEWE_OP_ENTRY_SIZE;
        count++;
    }
    buf[0] = (uint8_t)count;

    uint32_t plen = 1 + count * BEWE_OP_ENTRY_SIZE;
    auto pkt = make_bewe_packet(BEWE_TYPE_OP_LIST, buf, plen);

    {
        std::lock_guard<std::mutex> clk(room->cache_mtx);
        room->cached_op_list = pkt;
    }

    for(auto& je : room->joins){
        if(!je->alive.load() || je->fd < 0) continue;
        je->enqueue_ctrl(pkt.data(), pkt.size());
    }
    printf("[Central] OP_LIST broadcast: %d operators room='%s'\n", count, room->station_id.c_str());
}

// ── CHANNEL_SYNC audio_mask 재작성 + broadcast ───────────────────────────
void CentralServer::rebuild_and_broadcast_ch_sync(std::shared_ptr<HostRoom> room, bool send_to_host){
    std::vector<uint8_t> base_sync;
    {
        std::lock_guard<std::mutex> clk(room->cache_mtx);
        if(room->cached_ch_sync.empty()) return;
        base_sync = room->cached_ch_sync;
    }
    if(base_sync.size() < BEWE_HDR_SIZE + CH_SYNC_ENTRY_SIZE * MAX_CHANNELS_RELAY) return;

    uint8_t* payload = base_sync.data() + BEWE_HDR_SIZE;
    {
        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        for(int ch = 0; ch < MAX_CHANNELS_RELAY; ch++){
            uint8_t* entry = payload + ch * CH_SYNC_ENTRY_SIZE;
            uint32_t orig_mask;
            memcpy(&orig_mask, entry + CH_SYNC_MASK_OFFSET, sizeof(orig_mask));
            uint32_t new_mask = orig_mask & 0x1u;

            for(auto& je : room->joins){
                if(!je->authed || !je->alive.load()) continue;
                if(je->recv_audio[ch])
                    new_mask |= (1u << je->op_index);
            }
            memcpy(entry + CH_SYNC_MASK_OFFSET, &new_mask, sizeof(new_mask));
        }

        for(auto& je : room->joins){
            if(!je->alive.load() || je->fd < 0) continue;
            je->enqueue_ctrl(base_sync.data(), base_sync.size());
        }
    }  // joins_mtx 해제 후 cache/host 작업

    {
        std::lock_guard<std::mutex> clk(room->cache_mtx);
        room->cached_ch_sync = base_sync;
    }

    // HOST에도 재작성된 CHANNEL_SYNC 전송 (큐 경유)
    if(send_to_host && room->alive.load() && room->fd >= 0)
        enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA, base_sync.data(), (uint32_t)base_sync.size());
}

// 자신의 LAN IPv4 주소 목록 수집 (루프백·링크로컬 제외)
static void collect_lan_ips(CentralListResp& resp){
    resp.lan_ip_count = 0;
    memset(resp.lan_ips, 0, sizeof(resp.lan_ips));
    ifaddrs* ifa = nullptr;
    if(getifaddrs(&ifa) != 0) return;
    for(ifaddrs* p = ifa; p; p = p->ifa_next){
        if(!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
        auto* sin = reinterpret_cast<sockaddr_in*>(p->ifa_addr);
        uint32_t ip = ntohl(sin->sin_addr.s_addr);
        if((ip >> 24) == 127) continue;
        if((ip >> 16) == 0xA9FE) continue;
        if(resp.lan_ip_count >= CENTRAL_MAX_LAN_IPS) break;
        const char* s = inet_ntoa(sin->sin_addr);
        strncpy(resp.lan_ips[resp.lan_ip_count], s, 15);
        resp.lan_ip_count++;
    }
    freeifaddrs(ifa);
}

void CentralServer::handle_list_req(int fd){
    std::vector<CentralStation> stations;
    {
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        for(auto& r : rooms_)
            if(r->alive.load() && !r->resetting.load() && r->info.station_name[0] != '\0')
                stations.push_back(r->info);
    }
    uint16_t cnt = (uint16_t)stations.size();
    uint32_t plen = sizeof(CentralListResp) + cnt * sizeof(CentralStation);
    std::vector<uint8_t> payload(plen);
    auto* resp = reinterpret_cast<CentralListResp*>(payload.data());
    resp->count = cnt;
    collect_lan_ips(*resp);
    if(cnt > 0)
        memcpy(payload.data()+sizeof(CentralListResp),
               stations.data(), cnt*sizeof(CentralStation));
    central_send_pkt(fd, CentralPktType::LIST_RESP, payload.data(), plen);
}

std::shared_ptr<HostRoom> CentralServer::find_room(const std::string& id) const {
    std::lock_guard<std::mutex> lk(rooms_mtx_);
    for(auto& r : rooms_)
        if(r->station_id == id && r->alive.load()) return r;
    return nullptr;
}

void CentralServer::watchdog_loop(){
    while(running_.load()){
        std::this_thread::sleep_for(std::chrono::seconds(3));
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        for(auto& r : rooms_){
            if(!r->alive.load()) continue;
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - r->last_hb).count();
            if(age > HOST_TIMEOUT_SEC && r->fd >= 0){
                printf("[Central] WATCHDOG timeout: room='%s' age=%llds (limit=%ds) — closing\n",
                       r->station_id.c_str(), (long long)age, HOST_TIMEOUT_SEC);
                shutdown(r->fd, SHUT_RDWR); close(r->fd); r->fd=-1;
                r->alive.store(false);
            }
        }
    }
}

// ── 전역 채팅 브로드캐스트 ───────────────────────────────────────────────
void CentralServer::broadcast_global_chat(const uint8_t* bewe_pkt, size_t bewe_len,
                                          HostRoom* skip_host_room){
    struct Target {
        std::shared_ptr<HostRoom> room;
        bool send_to_host;
        std::vector<std::shared_ptr<JoinEntry>> joins;
    };
    std::vector<Target> targets;
    {
        std::lock_guard<std::mutex> lk(rooms_mtx_);
        for(auto& room : rooms_){
            if(!room->alive.load()) continue;
            Target t;
            t.room = room;
            t.send_to_host = (room.get() != skip_host_room && room->fd >= 0);
            {
                std::lock_guard<std::mutex> jlk(room->joins_mtx);
                for(auto& je : room->joins)
                    if(je->alive.load() && je->authed && je->fd >= 0)
                        t.joins.push_back(je);
            }
            targets.push_back(std::move(t));
        }
    }

    for(auto& t : targets){
        if(t.send_to_host && t.room->fd >= 0){
            enqueue_host_send(t.room, 0xFFFF, CentralMuxType::DATA, bewe_pkt, (uint32_t)bewe_len);
        }
        for(auto& je : t.joins)
            je->enqueue_data(bewe_pkt, bewe_len);
    }
}
