#include "central_server.hpp"
#include "../src/net_protocol.hpp"
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
#include <sys/stat.h>
#include <dirent.h>

static constexpr int HOST_TIMEOUT_SEC  = 20;  // HB 간격(3s) + 데이터 프레임 SNDTIMEO(3s) + 마진
static constexpr int HANDSHAKE_TIMEOUT = 10;
static constexpr size_t PIPE_BUF_SZ    = 65536;

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
    accept_thr_   = std::thread(&CentralServer::accept_loop,  this);
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

    // recv/send 타임아웃 모두 해제 (host_mux_loop / send_worker에서 블로킹 운용)
    timeval tv0{0,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv0, sizeof(tv0));

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

        // HOST 연결 직후 첫 OP_LIST(HOST만) 전송 → HOST UI 초기화
        build_and_broadcast_op_list(room);

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
        // SO_SNDTIMEO 제거: 타임아웃 시 EAGAIN→alive=false로 오연결 끊김 발생
        // send_worker는 per-JOIN 전용 스레드이므로 블로킹이 길어져도 다른 JOIN에 무관
        // TCP 스택이 실제 연결 끊김 시 EPIPE/ECONNRESET으로 정상 감지
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
    std::vector<uint8_t> buf(PIPE_BUF_SZ);
    uint64_t mux_pkts = 0;
    uint64_t hb_count = 0;
    uint64_t fft_count = 0, audio_count = 0, other_count = 0;
    uint64_t recv_bytes = 0;
    // 3초 윈도우 카운터 (구간 값)
    uint64_t win_hb = 0, win_fft = 0, win_audio = 0;
    uint64_t win_bytes = 0;
    uint64_t win_hb_bytes = 0, win_fft_bytes = 0, win_audio_bytes = 0;
    auto last_stat = std::chrono::steady_clock::now();
    auto start_time = last_stat;

    printf("[Central] host_mux_loop started room='%s' fd=%d\n",
           room->station_id.c_str(), room->fd);

    // HOST 접속 시 DB + Report 목록 초기 전송
    broadcast_db_list(room);
    broadcast_report_list_central(room);

    // flush 전용 스레드: recv 블로킹과 분리하여 JOIN→HOST 패킷 지연 제거
    std::thread flush_thr([room](){
        std::shared_ptr<HostRoom> r = room;
        while(r->alive.load()){
            flush_host_send_queue(r);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    while(room->alive.load()){
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

        // 3초마다 통계 (구간 값)
        auto now_s = std::chrono::steady_clock::now();
        auto stat_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_s - last_stat).count();
        if(stat_elapsed >= 3000){
            auto total_sec = std::chrono::duration_cast<std::chrono::seconds>(now_s - start_time).count();
            size_t join_count = 0;
            {
                std::lock_guard<std::mutex> jlk(room->joins_mtx);
                join_count = room->joins.size();
            }
            double win_sec = stat_elapsed / 1000.0;
            printf("[Central] [STATS] room='%s' uptime=%llds | recv: %.1f KB/s | "
                   "hb=%.1f KB/s fft=%.1f KB/s audio=%.1f KB/s | joins=%zu\n",
                   room->station_id.c_str(),
                   (long long)total_sec,
                   (double)win_bytes / win_sec / 1024.0,
                   (double)win_hb_bytes / win_sec / 1024.0,
                   (double)win_fft_bytes / win_sec / 1024.0,
                   (double)win_audio_bytes / win_sec / 1024.0,
                   join_count);
            last_stat = now_s;
            win_hb = win_fft = win_audio = win_bytes = 0;
            win_hb_bytes = win_fft_bytes = win_audio_bytes = 0;
        }

        // ── HB ─────────────────────────────────────────────────────────────
        if(mux.type == 0x00){
            room->last_hb = std::chrono::steady_clock::now();
            // 첫 HB 수신 시 DB+Report 목록 재전송 (HOST mux_loop 안정화 후)
            if(hb_count == 0){
                broadcast_db_list(room);
                broadcast_report_list_central(room);
            }
            hb_count++; win_hb++;
            win_hb_bytes += CENTRAL_MUX_HDR_SIZE + mux.len;
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
        win_bytes  += mux.len;
        mux_pkts++;

        auto mux_type = static_cast<CentralMuxType>(mux.type);
        if(mux_type != CentralMuxType::DATA || mux.len == 0) continue;

        // BEWE 타입별 카운트
        if(mux.len >= BEWE_HDR_SIZE){
            uint8_t bt = buf[4];
            if(bt == BEWE_TYPE_FFT)        { fft_count++;   win_fft++;   win_fft_bytes   += mux.len; }
            else if(bt == BEWE_TYPE_AUDIO) { audio_count++; win_audio++; win_audio_bytes += mux.len; }
            else                           other_count++;
        }

        // BEWE 패킷 중앙 처리: 오디오 필터링, CHANNEL_SYNC 인터셉트
        dispatch_to_joins(room, mux.conn_id, buf.data(), mux.len);
    }

    // flush 스레드 종료 대기
    if(flush_thr.joinable()) flush_thr.join();

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
    std::vector<uint8_t> buf(PIPE_BUF_SZ);
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
                // DB + Report 목록 전송
                broadcast_db_list(room);
                broadcast_report_list_central(room);
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

    // ── DB_SAVE: HOST가 보내도 Central에서 인터셉트하여 저장 ───────────
    if(bewe_type == BEWE_TYPE_DB_SAVE_META || bewe_type == BEWE_TYPE_DB_SAVE_DATA){
        // HOST 방향에서 온 DB_SAVE — JOIN의 intercept와 동일한 로직 재사용
        // room에 임시 db_fp 저장 (HOST 전용)
        static FILE* host_db_fp = nullptr;
        static std::string host_db_path;
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(bewe_type == BEWE_TYPE_DB_SAVE_META && plen >= 128+8+1+32){
            char filename[129]={}; memcpy(filename, payload, 128);
            uint64_t total_bytes=0; memcpy(&total_bytes, payload+128, 8);
            char op_name[33]={}; memcpy(op_name, payload+128+8+1, 32);
            const char* info = (plen>=128+8+1+32+1)?(const char*)(payload+128+8+1+32):"";
            const char* home=getenv("HOME");
            std::string db_base=home?std::string(home)+"/BE_WE/DataBase":"/tmp/BE_WE/DataBase";
            mkdir(db_base.c_str(),0755);
            std::string dst=db_base+"/"+filename;
            printf("[Central] DB_SAVE_META(HOST): '%s' by '%s' → %s\n",filename,op_name,dst.c_str());
            if(info[0]){ FILE* fi=fopen((dst+".info").c_str(),"w");
                if(fi){fwrite(info,1,strnlen(info,511),fi);fclose(fi);} }
            if(host_db_fp) fclose(host_db_fp);
            host_db_fp=fopen(dst.c_str(),"wb"); host_db_path=dst;
        } else if(bewe_type == BEWE_TYPE_DB_SAVE_DATA && plen>=6 && host_db_fp){
            uint8_t is_last=payload[1]; uint32_t cb=0; memcpy(&cb,payload+2,4);
            if(plen>=6+cb) fwrite(payload+6,1,cb,host_db_fp);
            if(is_last){fclose(host_db_fp);host_db_fp=nullptr;
                printf("[Central] DB_SAVE complete(HOST): %s\n",host_db_path.c_str());host_db_path.clear();
                broadcast_db_list(room); }
        }
        return; // JOIN에 포워드 안 함
    }

    // ── DB_DOWNLOAD_REQ from HOST: Central에서 파일 읽어 HOST에 전송 ──
    if(bewe_type == BEWE_TYPE_DB_DL_REQ){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(plen >= 128+32){
            char fn[129]={}; memcpy(fn, payload, 128);
            char op[33]={}; memcpy(op, payload+128, 32);
            const char* home = getenv("HOME");
            std::string db_base = home ? std::string(home)+"/BE_WE/DataBase" : "/tmp/BE_WE/DataBase";
            std::string fpath = db_base + "/" + fn;
            printf("[Central] DB_DOWNLOAD_REQ(HOST): '%s' by '%s'\n", fn, op);
            FILE* fp = fopen(fpath.c_str(), "rb");
            if(fp){
                fseek(fp, 0, SEEK_END); uint64_t fsz = (uint64_t)ftell(fp); fseek(fp, 0, SEEK_SET);
                // 1) .info 먼저 전송 (DB_DL_INFO)
                {
                    PktDbDownloadInfo dinfo{};
                    strncpy(dinfo.filename, fn, 127);
                    FILE* fi = fopen((fpath + ".info").c_str(), "r");
                    if(fi){ fread(dinfo.info_data, 1, sizeof(dinfo.info_data)-1, fi); fclose(fi); }
                    std::vector<uint8_t> ibewe(9 + sizeof(PktDbDownloadInfo));
                    memcpy(ibewe.data(), "BEWE", 4);
                    ibewe[4] = BEWE_TYPE_DB_DL_INFO;
                    uint32_t ilen = (uint32_t)sizeof(PktDbDownloadInfo);
                    memcpy(ibewe.data()+5, &ilen, 4);
                    memcpy(ibewe.data()+9, &dinfo, sizeof(PktDbDownloadInfo));
                    enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA, ibewe.data(), (uint32_t)ibewe.size());
                }
                // 2) .wav 데이터 청크 전송 (DB_DL_DATA)
                const size_t CHUNK = 64*1024;
                std::vector<uint8_t> buf(sizeof(PktDbDownloadData) + CHUNK);
                bool first = true;
                while(true){
                    size_t n = fread(buf.data() + sizeof(PktDbDownloadData), 1, CHUNK, fp);
                    bool last = (n == 0 || feof(fp));
                    if(n == 0 && !first) break;
                    auto* d = reinterpret_cast<PktDbDownloadData*>(buf.data());
                    memset(d, 0, sizeof(PktDbDownloadData));
                    strncpy(d->filename, fn, 127);
                    d->total_bytes = fsz;
                    d->chunk_bytes = (uint32_t)n;
                    d->is_first = first ? 1 : 0;
                    d->is_last = (last || n == 0) ? 1 : 0;
                    first = false;
                    size_t pkt_len = sizeof(PktDbDownloadData) + n;
                    std::vector<uint8_t> bewe(9 + pkt_len);
                    memcpy(bewe.data(), "BEWE", 4);
                    bewe[4] = BEWE_TYPE_DB_DL_DATA;
                    uint32_t blen = (uint32_t)pkt_len;
                    memcpy(bewe.data()+5, &blen, 4);
                    memcpy(bewe.data()+9, buf.data(), pkt_len);
                    enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA, bewe.data(), (uint32_t)bewe.size());
                    if(n == 0 || last) break;
                }
                fclose(fp);
                printf("[Central] DB_DOWNLOAD sent(HOST): '%s' (%.1fMB)\n", fn, fsz/1048576.0);
            }
        }
        return;
    }

    // ── DB_DELETE from HOST ──────────────────────────────────────────────
    if(bewe_type == BEWE_TYPE_DB_DELETE){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(plen >= 128+32){
            char fn[129]={}; memcpy(fn, payload, 128);
            char op[33]={}; memcpy(op, payload+128, 32);
            (void)op; // operator는 로그용으로만 사용 (flat 저장이라 경로엔 불필요)
            const char* home = getenv("HOME");
            std::string db_base = home ? std::string(home)+"/BE_WE/DataBase" : "/tmp/BE_WE/DataBase";
            std::string fpath = db_base+"/"+fn;
            std::string ipath = fpath + ".info";
            int r1 = remove(fpath.c_str());
            int r2 = remove(ipath.c_str());
            if(r1 != 0)
                printf("[Central] DB_DELETE(HOST) FAILED: '%s' errno=%d (%s)\n", fpath.c_str(), errno, strerror(errno));
            else
                printf("[Central] DB_DELETE(HOST) OK: '%s'\n", fpath.c_str());
            if(r2 != 0 && errno != ENOENT)
                printf("[Central] DB_DELETE(HOST) .info FAILED: '%s' errno=%d (%s)\n", ipath.c_str(), errno, strerror(errno));
            broadcast_db_list(room);
        }
        return;
    }

    // ── Report from HOST (REPORT_ADD/DELETE/UPDATE) ────────────────────
    if(bewe_type == 0x23 || bewe_type == BEWE_TYPE_RPT_DELETE || bewe_type == BEWE_TYPE_RPT_UPDATE){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        const char* home = getenv("HOME");
        std::string rpt_dir = home ? std::string(home)+"/BE_WE/DataBase/_reports" : "/tmp/BE_WE/DataBase/_reports";
        mkdir(rpt_dir.c_str(), 0755);
        if(bewe_type == 0x23 && plen >= sizeof(PktReportAdd)){
            auto* ra = reinterpret_cast<const PktReportAdd*>(payload);
            FILE* fi = fopen((rpt_dir+"/"+ra->filename+".info").c_str(), "w");
            if(fi){ if(ra->info_summary[0]) fprintf(fi,"%s",ra->info_summary); else fprintf(fi,"Operator: %s\n",ra->reporter); fclose(fi); }
            printf("[Central] REPORT_ADD(HOST): '%s'\n", ra->filename);
        } else if(bewe_type == BEWE_TYPE_RPT_DELETE && plen >= 128){
            char fn[129]={}; memcpy(fn,payload,128);
            remove((rpt_dir+"/"+fn+".info").c_str());
            printf("[Central] REPORT_DELETE(HOST): '%s'\n", fn);
        } else if(bewe_type == BEWE_TYPE_RPT_UPDATE && plen >= 128+512){
            char fn[129]={}; memcpy(fn,payload,128);
            FILE* fi = fopen((rpt_dir+"/"+fn+".info").c_str(), "w");
            if(fi){ fwrite(payload+128,1,strnlen((const char*)(payload+128),511),fi); fclose(fi); }
            printf("[Central] REPORT_UPDATE(HOST): '%s'\n", fn);
        }
        broadcast_report_list_central(room);
        return;
    }

    // ── 기타 패킷 (FFT, HEARTBEAT, STATUS 등): 타입에 따라 분류 ─────────
    // FFT: auth 완료된 JOIN에게만, 전용 send_queue (대용량, 드롭 허용)
    // 제어(HEARTBEAT/STATUS/CMD_ACK 등): ctrl_queue (우선 전송, 드롭 없음)
    bool is_fft = (bewe_type == BEWE_TYPE_FFT);
    bool is_ctrl = (bewe_type == BEWE_TYPE_HEARTBEAT || bewe_type == BEWE_TYPE_STATUS ||
                    bewe_type == BEWE_TYPE_CMD || bewe_type == BEWE_TYPE_OP_LIST ||
                    bewe_type == BEWE_TYPE_AUTH_ACK ||
                    bewe_type == BEWE_TYPE_IQ_CHUNK);   // 드롭 불가
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

    // ── DB_SAVE: Central server에 직접 저장 (HOST에 포워드 안 함) ──
    if(bewe_type == BEWE_TYPE_DB_SAVE_META){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        // PktDbSaveMeta: filename[128]+total_bytes[8]+transfer_id[1]+operator_name[32]+info_data[512]
        if(plen >= 128+8+1+32){
            char filename[129]={}; memcpy(filename, payload, 128);
            uint64_t total_bytes = 0; memcpy(&total_bytes, payload+128, 8);
            char operator_name[33]={}; memcpy(operator_name, payload+128+8+1, 32);
            const char* info_data = (plen >= 128+8+1+32+1) ? (const char*)(payload+128+8+1+32) : "";

            // ~/BE_WE/DataBase/  (flat — operator는 .info 의 Operator: 필드로 보존)
            const char* home = getenv("HOME");
            std::string db_base = home ? std::string(home)+"/BE_WE/DataBase" : "/tmp/BE_WE/DataBase";
            mkdir(db_base.c_str(), 0755);
            std::string dst = db_base + "/" + filename;

            printf("[Central] DB_SAVE_META: '%s' by '%s' (%.1fMB) → %s\n",
                   filename, operator_name, total_bytes/1048576.0, dst.c_str());

            // .info 저장
            if(info_data[0]){
                FILE* fi = fopen((dst+".info").c_str(), "w");
                if(fi){ fwrite(info_data, 1, strnlen(info_data, 511), fi); fclose(fi); }
            }
            // 파일 열기 (je에 저장)
            if(je->db_fp) fclose(je->db_fp);
            je->db_fp = fopen(dst.c_str(), "wb");
            je->db_path = dst;
        }
        return true;  // HOST에 포워드 안 함
    }
    if(bewe_type == BEWE_TYPE_DB_SAVE_DATA){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        // PktDbSaveData: transfer_id[1]+is_last[1]+chunk_bytes[4]+data[]
        if(plen >= 6 && je->db_fp){
            uint8_t is_last = payload[1];
            uint32_t chunk_bytes = 0; memcpy(&chunk_bytes, payload+2, 4);
            if(plen >= 6 + chunk_bytes)
                fwrite(payload+6, 1, chunk_bytes, je->db_fp);
            if(is_last){
                fclose(je->db_fp);
                je->db_fp = nullptr;
                printf("[Central] DB_SAVE complete: %s\n", je->db_path.c_str());
                je->db_path.clear();
                broadcast_db_list(room);
            }
        }
        return true;  // HOST에 포워드 안 함
    }

    // ── DB_DOWNLOAD_REQ: Central에서 파일 읽어 요청자에게 전송 ────────
    if(bewe_type == BEWE_TYPE_DB_DL_REQ){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(plen >= 128+32){
            char fn[129]={}; memcpy(fn, payload, 128);
            char op[33]={}; memcpy(op, payload+128, 32);
            const char* home = getenv("HOME");
            std::string db_base = home ? std::string(home)+"/BE_WE/DataBase" : "/tmp/BE_WE/DataBase";
            std::string fpath = db_base + "/" + fn;
            printf("[Central] DB_DOWNLOAD_REQ: '%s' by '%s' → conn_id=%u\n", fn, op, je->conn_id);

            FILE* fp = fopen(fpath.c_str(), "rb");
            if(fp){
                fseek(fp, 0, SEEK_END); uint64_t fsz = (uint64_t)ftell(fp); fseek(fp, 0, SEEK_SET);
                // 1) .info 먼저 전송 (DB_DL_INFO)
                {
                    PktDbDownloadInfo dinfo{};
                    strncpy(dinfo.filename, fn, 127);
                    FILE* fi = fopen((fpath + ".info").c_str(), "r");
                    if(fi){ fread(dinfo.info_data, 1, sizeof(dinfo.info_data)-1, fi); fclose(fi); }
                    std::vector<uint8_t> ibewe(9 + sizeof(PktDbDownloadInfo));
                    memcpy(ibewe.data(), "BEWE", 4);
                    ibewe[4] = BEWE_TYPE_DB_DL_INFO;
                    uint32_t ilen = (uint32_t)sizeof(PktDbDownloadInfo);
                    memcpy(ibewe.data()+5, &ilen, 4);
                    memcpy(ibewe.data()+9, &dinfo, sizeof(PktDbDownloadInfo));
                    je->enqueue_ctrl(ibewe.data(), ibewe.size());
                }
                // 2) .wav 데이터 청크 전송 (DB_DL_DATA)
                const size_t CHUNK = 64*1024;
                std::vector<uint8_t> buf(sizeof(PktDbDownloadData) + CHUNK);
                bool first = true;
                while(true){
                    size_t n = fread(buf.data() + sizeof(PktDbDownloadData), 1, CHUNK, fp);
                    bool last = (n == 0 || feof(fp));
                    if(n == 0 && !first) break;
                    auto* d = reinterpret_cast<PktDbDownloadData*>(buf.data());
                    memset(d, 0, sizeof(PktDbDownloadData));
                    strncpy(d->filename, fn, 127);
                    d->total_bytes = fsz;
                    d->chunk_bytes = (uint32_t)n;
                    d->is_first = first ? 1 : 0;
                    d->is_last = (last || n == 0) ? 1 : 0;
                    first = false;
                    size_t pkt_len = sizeof(PktDbDownloadData) + n;
                    // BEWE 패킷으로 감싸기
                    std::vector<uint8_t> bewe(9 + pkt_len);
                    memcpy(bewe.data(), "BEWE", 4);
                    bewe[4] = BEWE_TYPE_DB_DL_DATA;
                    uint32_t blen = (uint32_t)pkt_len;
                    memcpy(bewe.data()+5, &blen, 4);
                    memcpy(bewe.data()+9, buf.data(), pkt_len);
                    je->enqueue_ctrl(bewe.data(), bewe.size());
                    if(n == 0 || last) break;
                }
                fclose(fp);
                printf("[Central] DB_DOWNLOAD sent: '%s' (%.1fMB)\n", fn, fsz/1048576.0);
            } else {
                printf("[Central] DB_DOWNLOAD_REQ: file not found '%s'\n", fpath.c_str());
            }
        }
        return true;  // HOST에 포워드 안 함
    }

    // ── DB_DELETE_REQ: Central에서 파일 삭제 ────────────────────────
    if(bewe_type == BEWE_TYPE_DB_DELETE){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(plen >= 128+32){
            char fn[129]={}; memcpy(fn, payload, 128);
            char op[33]={}; memcpy(op, payload+128, 32);
            (void)op; // operator는 로그용으로만 사용 (flat 저장이라 경로엔 불필요)
            const char* home = getenv("HOME");
            std::string db_base = home ? std::string(home)+"/BE_WE/DataBase" : "/tmp/BE_WE/DataBase";
            std::string fpath = db_base + "/" + fn;
            std::string ipath = fpath + ".info";
            int r1 = remove(fpath.c_str());
            int r2 = remove(ipath.c_str());
            if(r1 != 0)
                printf("[Central] DB_DELETE FAILED: '%s' errno=%d (%s)\n", fpath.c_str(), errno, strerror(errno));
            else
                printf("[Central] DB_DELETE OK: '%s'\n", fpath.c_str());
            if(r2 != 0 && errno != ENOENT)
                printf("[Central] DB_DELETE .info FAILED: '%s' errno=%d (%s)\n", ipath.c_str(), errno, strerror(errno));
            broadcast_db_list(room);
        }
        return true;
    }

    // ── REPORT_ADD: Central _reports/ 에 .info 저장 ────────────────
    if(bewe_type == 0x23){ // REPORT_ADD
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(plen >= sizeof(PktReportAdd)){
            auto* ra = reinterpret_cast<const PktReportAdd*>(payload);
            const char* home = getenv("HOME");
            std::string rpt_dir = home ? std::string(home)+"/BE_WE/DataBase/_reports" : "/tmp/BE_WE/DataBase/_reports";
            mkdir(rpt_dir.c_str(), 0755);
            std::string ip = rpt_dir + "/" + std::string(ra->filename) + ".info";
            FILE* fi = fopen(ip.c_str(), "w");
            if(fi){
                // info_summary에 기본 정보 저장
                if(ra->info_summary[0]) fprintf(fi, "%s", ra->info_summary);
                else fprintf(fi, "Operator: %s\n", ra->reporter);
                fclose(fi);
            }
            printf("[Central] REPORT_ADD: '%s' by '%s'\n", ra->filename, ra->reporter);
            broadcast_report_list_central(room);
        }
        return true; // HOST에 포워드 안 함
    }

    // ── REPORT_DELETE: Central _reports/ 에서 삭제 ───────────────
    if(bewe_type == BEWE_TYPE_RPT_DELETE){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(plen >= 128){
            char fn[129]={}; memcpy(fn, payload, 128);
            const char* home = getenv("HOME");
            std::string rpt_dir = home ? std::string(home)+"/BE_WE/DataBase/_reports" : "/tmp/BE_WE/DataBase/_reports";
            std::string ip = rpt_dir + "/" + fn + ".info";
            remove(ip.c_str());
            printf("[Central] REPORT_DELETE: '%s'\n", fn);
            broadcast_report_list_central(room);
        }
        return true;
    }

    // ── REPORT_UPDATE: Central _reports/ .info 갱신 ──────────────
    if(bewe_type == BEWE_TYPE_RPT_UPDATE){
        const uint8_t* payload = bewe_pkt + BEWE_HDR_SIZE;
        size_t plen = bewe_len - BEWE_HDR_SIZE;
        if(plen >= 128+512){
            char fn[129]={}; memcpy(fn, payload, 128);
            const char* info = (const char*)(payload+128);
            const char* home = getenv("HOME");
            std::string rpt_dir = home ? std::string(home)+"/BE_WE/DataBase/_reports" : "/tmp/BE_WE/DataBase/_reports";
            std::string ip = rpt_dir + "/" + fn + ".info";
            FILE* fi = fopen(ip.c_str(), "w");
            if(fi){ fwrite(info, 1, strnlen(info,511), fi); fclose(fi); }
            printf("[Central] REPORT_UPDATE: '%s'\n", fn);
            broadcast_report_list_central(room);
        }
        return true;
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

    // JOIN들에게 전송
    for(auto& je : room->joins){
        if(!je->alive.load() || je->fd < 0) continue;
        je->enqueue_ctrl(pkt.data(), pkt.size());
    }
    // HOST에게도 MUX broadcast (conn_id=0xFFFF)로 전송 → HOST UI 오퍼레이터 목록 갱신
    if(room->alive.load() && room->fd >= 0)
        enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA, pkt.data(), pkt.size());
    printf("[Central] OP_LIST broadcast: %d operators room='%s'\n", count, room->station_id.c_str());
}

// ── REPORT_LIST broadcast (Central _reports/ 기반) ────────────────────────
void CentralServer::broadcast_report_list_central(std::shared_ptr<HostRoom> room){
    const char* home = getenv("HOME");
    std::string rpt_dir = home ? std::string(home)+"/BE_WE/DataBase/_reports" : "/tmp/BE_WE/DataBase/_reports";
    mkdir(rpt_dir.c_str(), 0755);

    std::vector<ReportFileEntry> entries;
    DIR* d = opendir(rpt_dir.c_str());
    if(d){
        struct dirent* de;
        while((de = readdir(d))){
            if(de->d_name[0]=='.') continue;
            std::string n(de->d_name);
            if(n.size()<6 || n.substr(n.size()-5)!=".info") continue;
            // filename = .info 제거
            std::string fn = n.substr(0, n.size()-5);
            ReportFileEntry re{};
            strncpy(re.filename, fn.c_str(), 127);
            // .info에서 reporter, info_summary 읽기
            std::string ip = rpt_dir + "/" + n;
            FILE* fi = fopen(ip.c_str(), "r");
            if(fi){
                char line[256]; int pos=0;
                while(fgets(line,sizeof(line),fi)){
                    char k[64]={},val[128]={};
                    if(sscanf(line,"%63[^:]: %127[^\n]",k,val)>=1){
                        if(strcmp(k,"Operator")==0) strncpy(re.reporter,val,31);
                    }
                    int l=(int)strlen(line);
                    if(pos+l<255){ memcpy(re.info_summary+pos,line,l); pos+=l; }
                }
                fclose(fi);
            }
            entries.push_back(re);
        }
        closedir(d);
    }

    uint16_t cnt = (uint16_t)std::min(entries.size(), (size_t)200);
    size_t payload_sz = sizeof(PktReportList) + cnt * sizeof(ReportFileEntry);
    std::vector<uint8_t> payload(payload_sz, 0);
    auto* hdr = reinterpret_cast<PktReportList*>(payload.data());
    hdr->count = cnt;
    if(cnt > 0) memcpy(payload.data()+sizeof(PktReportList), entries.data(), cnt*sizeof(ReportFileEntry));

    std::vector<uint8_t> bewe(9 + payload_sz);
    memcpy(bewe.data(), "BEWE", 4);
    bewe[4] = 0x22; // REPORT_LIST
    uint32_t plen = (uint32_t)payload_sz;
    memcpy(bewe.data()+5, &plen, 4);
    memcpy(bewe.data()+9, payload.data(), payload_sz);

    enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA, bewe.data(), (uint32_t)bewe.size());
    std::lock_guard<std::mutex> jlk(room->joins_mtx);
    for(auto& je : room->joins){
        if(!je->alive.load() || je->fd < 0 || !je->authed) continue;
        je->enqueue_ctrl(bewe.data(), bewe.size());
    }
    printf("[Central] REPORT_LIST broadcast: %u reports\n", cnt);
}

// ── DB_LIST broadcast ─────────────────────────────────────────────────────
void CentralServer::broadcast_db_list(std::shared_ptr<HostRoom> room){
    // ~/BE_WE/DataBase/ 스캔
    const char* home = getenv("HOME");
    std::string db_base = home ? std::string(home)+"/BE_WE/DataBase" : "/tmp/BE_WE/DataBase";

    std::vector<DbFileEntry> entries;
    DIR* top = opendir(db_base.c_str());
    if(top){
        struct dirent* de;
        while((de = readdir(top))){
            if(de->d_name[0]=='.') continue;
            // flat 구조: .wav 정규 파일만 수집 (_reports/ 등 디렉토리는 자동 제외)
            std::string fn(de->d_name);
            if(fn.size()<5 || fn.substr(fn.size()-4)!=".wav") continue;
            std::string fp = db_base + "/" + fn;
            struct stat st{};
            if(stat(fp.c_str(),&st)!=0 || !S_ISREG(st.st_mode)) continue;
            DbFileEntry e{};
            strncpy(e.filename, fn.c_str(), 127);
            e.size_bytes = (uint64_t)st.st_size;
            // .info 에서 Operator: 값을 읽어 e.operator_name 채움
            FILE* fi = fopen((fp+".info").c_str(), "r");
            if(fi){
                char line[256];
                while(fgets(line,sizeof(line),fi)){
                    char k[64]={},val[128]={};
                    if(sscanf(line,"%63[^:]: %127[^\n]",k,val)>=2){
                        if(strcmp(k,"Operator")==0){ strncpy(e.operator_name,val,31); break; }
                    }
                }
                fclose(fi);
            }
            entries.push_back(e);
        }
        closedir(top);
    }

    // BEWE 패킷 빌드
    uint16_t cnt = (uint16_t)std::min(entries.size(), (size_t)500);
    size_t payload_sz = sizeof(PktDbList) + cnt * sizeof(DbFileEntry);
    std::vector<uint8_t> payload(payload_sz, 0);
    auto* hdr = reinterpret_cast<PktDbList*>(payload.data());
    hdr->count = cnt;
    if(cnt > 0)
        memcpy(payload.data() + sizeof(PktDbList), entries.data(), cnt * sizeof(DbFileEntry));

    // BEWE 패킷으로 감싸기
    std::vector<uint8_t> bewe(9 + payload_sz);
    memcpy(bewe.data(), "BEWE", 4);
    bewe[4] = BEWE_TYPE_DB_LIST;
    uint32_t plen = (uint32_t)payload_sz;
    memcpy(bewe.data()+5, &plen, 4);
    memcpy(bewe.data()+9, payload.data(), payload_sz);

    // HOST에 전송
    enqueue_host_send(room, 0xFFFF, CentralMuxType::DATA, bewe.data(), (uint32_t)bewe.size());

    // 모든 JOIN에 전송
    std::lock_guard<std::mutex> jlk(room->joins_mtx);
    for(auto& je : room->joins){
        if(!je->alive.load() || je->fd < 0 || !je->authed) continue;
        je->enqueue_ctrl(bewe.data(), bewe.size());
    }

    printf("[Central] DB_LIST broadcast: %u files\n", cnt);
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
