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

static constexpr int HOST_TIMEOUT_SEC  = 10;
static constexpr int HANDSHAKE_TIMEOUT = 10;
static constexpr size_t PIPE_BUF       = 65536;

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
    if(!central_recv_pkt(fd, hdr, payload, 65536)){ close(fd); return; }

    // 타임아웃 해제
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
        printf("[Central] HOST room '%s' (%s) opened\n",
               room->station_id.c_str(), room->info.station_name);

        host_mux_loop(room);

    } else if(type == CentralPktType::JOIN_ROOM){
        if(payload.size() < sizeof(CentralJoinRoom)){ close(fd); return; }
        auto* jr = reinterpret_cast<const CentralJoinRoom*>(payload.data());
        std::string sid(jr->station_id, strnlen(jr->station_id, sizeof(jr->station_id)));

        auto room = find_room(sid);
        if(!room){
            CentralError err{}; strncpy(err.msg, "Room not found", 63);
            central_send_pkt(fd, CentralPktType::ERROR, &err, sizeof(err));
            close(fd); return;
        }

        auto je = std::make_shared<JoinEntry>();
        je->fd = fd;
        je->start_send_worker();
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
            central_send_mux(room->fd, je->conn_id, CentralMuxType::CONN_OPEN, nullptr, 0);
        }

        printf("[Central] JOIN conn_id=%u entered room '%s' (%zu users)\n",
               je->conn_id, sid.c_str(), room->joins.size());

        // 캐시 전송은 AUTH_ACK 통과 후 (dispatch_to_joins에서 처리)
        // JOIN이 AUTH_ACK를 동기 대기하므로 그 전에 다른 패킷을 보내면 안 됨

        join_loop(je, room);

    } else if(type == CentralPktType::LIST_REQ){
        handle_list_req(fd);
        close(fd);
    } else {
        close(fd);
    }
}

// ── HOST mux 수신 루프: HOST→relay→JOIN ───────────────────────────────────
// HOST가 보내는 형식: CentralMuxHdr + BEWE 패킷
// conn_id=0xFFFF: 모든 JOIN에게 broadcast
// conn_id=특정값: 해당 JOIN에게만 전송
void CentralServer::host_mux_loop(std::shared_ptr<HostRoom> room){
    std::vector<uint8_t> buf(PIPE_BUF);
    uint64_t mux_pkts = 0;

    while(room->alive.load()){
        CentralMuxHdr mux{};
        if(!central_recv_all(room->fd, &mux, CENTRAL_MUX_HDR_SIZE)){
            int e = errno;
            if(room->alive.load())
                printf("[Central] host_mux_loop recv failed room='%s' errno=%d(%s) pkts=%llu\n",
                       room->station_id.c_str(), e, strerror(e), (unsigned long long)mux_pkts);
            break;
        }

        // HOST_HB인지 확인 (relay 제어 패킷)
        // HOST가 heartbeat를 MUX 스트림 밖으로 보낼 수도 있음 →
        // MUX 헤더 type=0인 경우 relay_pkt 헤더로 재해석
        // 단순화: HOST는 HB도 MUX로 보냄 (type=0x00 예약)
        // → CentralMuxType::DATA=1, CONN_OPEN=2, CONN_CLOSE=3
        // → type=0이면 heartbeat로 처리

        if(mux.type == 0x00){
            // heartbeat
            room->last_hb = std::chrono::steady_clock::now();
            if(mux.len > 0){
                if(buf.size() < mux.len) buf.resize(mux.len);
                if(!central_recv_all(room->fd, buf.data(), mux.len)) break;
                if(mux.len >= sizeof(CentralHostHb)){
                    auto* hb = reinterpret_cast<const CentralHostHb*>(buf.data());
                    std::lock_guard<std::mutex> jlk(room->joins_mtx);
                    room->info.user_count = hb->user_count;
                }
            }
            continue;
        }

        room->last_hb = std::chrono::steady_clock::now();

        // NET_RESET: 네트워크 리셋 신호
        if(mux.type == static_cast<uint8_t>(CentralMuxType::NET_RESET)){
            if(mux.len > 0){
                uint8_t flag = 0;
                if(!central_recv_all(room->fd, &flag, 1)) break;
                if(mux.len > 1){
                    // 나머지 바이트 버리기
                    std::vector<uint8_t> discard(mux.len - 1);
                    if(!central_recv_all(room->fd, discard.data(), mux.len - 1)) break;
                }
                if(flag == 0){
                    room->resetting.store(true);
                    printf("[Central] room '%s' (%s) Server reset ...\n",
                           room->station_id.c_str(), room->info.station_name);
                } else {
                    room->resetting.store(false);
                    printf("[Central] room '%s' (%s) Server open ...\n",
                           room->station_id.c_str(), room->info.station_name);
                }
            }
            continue;
        }

        if(mux.len > 4*1024*1024){
            printf("[Central] host_mux_loop oversized mux.len=%u room='%s'\n",
                   mux.len, room->station_id.c_str());
            break;
        }
        if(buf.size() < mux.len) buf.resize(mux.len);
        if(mux.len > 0 && !central_recv_all(room->fd, buf.data(), mux.len)){
            int e = errno;
            printf("[Central] host_mux_loop recv data failed room='%s' len=%u errno=%d(%s)\n",
                   room->station_id.c_str(), mux.len, e, strerror(e));
            break;
        }
        mux_pkts++;

        auto mux_type = static_cast<CentralMuxType>(mux.type);
        if(mux_type != CentralMuxType::DATA || mux.len == 0) continue;

        // BEWE 패킷 중앙 처리: 오디오 필터링, CHANNEL_SYNC 인터셉트
        dispatch_to_joins(room, mux.conn_id, buf.data(), mux.len);
    }

    // 룸 닫기
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
// JOIN이 보내는 BEWE 패킷을 MUX 헤더 붙여서 HOST에게 전달
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

        // JOIN→HOST: 릴레이가 처리할 명령은 인터셉트
        bool consumed = intercept_join_cmd(je, room, buf.data(), 9 + bewe_len);
        if(consumed) { pkt_count++; continue; }  // 릴레이에서 처리 완료, HOST에 포워드 안 함

        // HOST에게 MUX 헤더 + BEWE 패킷 전달 (하드웨어 제어 명령 등)
        std::lock_guard<std::mutex> lk(room->host_send_mtx);
        if(!room->alive.load() || room->fd < 0){
            disc_reason = "host_room_closed";
            break;
        }
        if(!central_send_mux(room->fd, je->conn_id, CentralMuxType::DATA,
                           buf.data(), 9 + bewe_len)){
            printf("[Central] join_loop send to host failed conn_id=%u\n", je->conn_id);
            disc_reason = "send_host_fail";
            break;
        }
        pkt_count++;
    }
    printf("[Central] join_loop exit conn_id=%u reason=%s pkts=%llu\n",
           je->conn_id, disc_reason, (unsigned long long)pkt_count);

    je->alive.store(false);
    je->stop_send_worker();
    if(je->fd >= 0){ shutdown(je->fd, SHUT_RDWR); close(je->fd); je->fd=-1; }

    // HOST에게 CONN_CLOSE 알림
    if(room->alive.load() && room->fd >= 0){
        std::lock_guard<std::mutex> lk(room->host_send_mtx);
        central_send_mux(room->fd, je->conn_id, CentralMuxType::CONN_CLOSE, nullptr, 0);
    }

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
                        printf("[Central] AUTH_ACK op=%u '%s' room='%s'\n",
                               op_idx, je->name, room->station_id.c_str());
                        break;
                    }
                }
            }
            // AUTH_ACK를 먼저 전달한 후, 캐시된 상태 패킷 전송
            if(target){
                target->enqueue_data(bewe_pkt, bewe_len);  // AUTH_ACK 먼저
                std::lock_guard<std::mutex> clk(room->cache_mtx);
                if(!room->cached_heartbeat.empty())
                    target->enqueue_data(room->cached_heartbeat.data(), room->cached_heartbeat.size());
                if(!room->cached_status.empty())
                    target->enqueue_data(room->cached_status.data(), room->cached_status.size());
                if(!room->cached_ch_sync.empty())
                    target->enqueue_data(room->cached_ch_sync.data(), room->cached_ch_sync.size());
                if(!room->cached_op_list.empty())
                    target->enqueue_data(room->cached_op_list.data(), room->cached_op_list.size());
                // OPERATOR_LIST 갱신 (새 유저 반영)
                build_and_broadcast_op_list(room);
                return;  // AUTH_ACK는 이미 전송함, 아래 기타 경로 타지 않도록
            }
        }
        // ok=0 또는 broadcast → 그대로 전달
    }

    // ── 상태 패킷 캐시 (새 JOIN 접속 시 즉시 전송용) ────────────────
    if(bewe_type == BEWE_TYPE_HEARTBEAT || bewe_type == BEWE_TYPE_STATUS ||
       bewe_type == BEWE_TYPE_CH_SYNC){
        std::lock_guard<std::mutex> clk(room->cache_mtx);
        auto& cache = (bewe_type == BEWE_TYPE_HEARTBEAT) ? room->cached_heartbeat :
                      (bewe_type == BEWE_TYPE_STATUS)    ? room->cached_status :
                                                           room->cached_ch_sync;
        cache.assign(bewe_pkt, bewe_pkt + bewe_len);
    }

    // ── CHANNEL_SYNC 인터셉트: 캐시 저장 → audio_mask 재작성 후 broadcast
    if(bewe_type == BEWE_TYPE_CH_SYNC){
        // 원본 캐시 (audio_mask 재작성의 base)
        {
            std::lock_guard<std::mutex> clk(room->cache_mtx);
            room->cached_ch_sync.assign(bewe_pkt, bewe_pkt + bewe_len);
        }
        // audio_mask를 릴레이의 recv_audio[] 기반으로 재작성 + broadcast
        rebuild_and_broadcast_ch_sync(room);
        return;
    }

    // ── AUDIO_FRAME: 릴레이가 뮤트 테이블 기반으로 필터링 ─────────────
    if(bewe_type == BEWE_TYPE_AUDIO){
        if(bewe_len < BEWE_HDR_SIZE + 1) return;
        uint8_t ch_idx = bewe_pkt[BEWE_HDR_SIZE]; // PktAudioFrame.ch_idx
        if(ch_idx >= MAX_CHANNELS_RELAY) return;

        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        for(auto& je : room->joins){
            if(!je->alive.load() || je->fd < 0) continue;
            if(conn_id != 0xFFFF && conn_id != je->conn_id) continue;
            // 뮤트 테이블 체크: 이 JOIN이 이 채널을 수신하는가?
            if(!je->recv_audio[ch_idx]) continue;
            je->enqueue_audio(bewe_pkt, bewe_len);
        }
        return;
    }

    // ── 기타 패킷 (FFT, STATUS, HEARTBEAT 등): 그대로 전달 ───────────
    std::lock_guard<std::mutex> jlk(room->joins_mtx);
    for(auto& je : room->joins){
        if(!je->alive.load() || je->fd < 0) continue;
        if(conn_id != 0xFFFF && conn_id != je->conn_id) continue;
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
                // audio_mask 재계산 + CHANNEL_SYNC 재작성 broadcast
                rebuild_and_broadcast_ch_sync(room);
            }
            return true;  // HOST에 포워드 안 함
        }
        return false;  // 다른 CMD는 HOST에 포워드
    }

    // ── CHAT: 릴레이가 broadcast, HOST에도 포워드 ────────────────────
    if(bewe_type == BEWE_TYPE_CHAT){
        // 모든 JOIN에게 broadcast
        std::lock_guard<std::mutex> jlk(room->joins_mtx);
        for(auto& j : room->joins){
            if(!j->alive.load() || !j->authed || j->fd < 0) continue;
            j->enqueue_data(bewe_pkt, bewe_len);
        }
        return false;  // HOST에도 포워드 (HOST UI에 표시)
    }

    return false;
}

// ── BEWE 패킷 빌드 ──────────────────────────────────────────────────────
std::vector<uint8_t> CentralServer::make_bewe_packet(uint8_t type,
                                                     const void* payload, uint32_t plen){
    std::vector<uint8_t> pkt(BEWE_HDR_SIZE + plen);
    // magic
    pkt[0]='B'; pkt[1]='E'; pkt[2]='W'; pkt[3]='E';
    pkt[4] = type;
    memcpy(pkt.data()+5, &plen, 4);
    if(plen > 0 && payload)
        memcpy(pkt.data()+BEWE_HDR_SIZE, payload, plen);
    return pkt;
}

// ── OPERATOR_LIST 빌드 + broadcast ───────────────────────────────────────
void CentralServer::build_and_broadcast_op_list(std::shared_ptr<HostRoom> room){
    // OpEntry[]: index(1) + tier(1) + name(32) = 34 bytes each
    // PktOperatorList: count(1) + OpEntry[16]
    uint8_t buf[1 + BEWE_MAX_OPERATORS * BEWE_OP_ENTRY_SIZE] = {};
    int count = 0;

    // HOST를 index=0으로 첫 번째 엔트리
    uint8_t* p = buf + 1;
    p[0] = 0;  // index=0 (HOST)
    p[1] = room->host_tier;
    strncpy((char*)(p+2), room->host_name, 31);
    p += BEWE_OP_ENTRY_SIZE;
    count++;

    // JOIN들
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

    // 캐시 갱신
    {
        std::lock_guard<std::mutex> clk(room->cache_mtx);
        room->cached_op_list = pkt;
    }

    // 모든 JOIN에게 broadcast
    for(auto& je : room->joins){
        if(!je->alive.load() || je->fd < 0) continue;
        je->enqueue_data(pkt.data(), pkt.size());
    }
}

// ── CHANNEL_SYNC audio_mask 재작성 + broadcast ───────────────────────────
// HOST가 보낸 원본 CHANNEL_SYNC에서 audio_mask를 릴레이의 recv_audio[] 기반으로 재계산
void CentralServer::rebuild_and_broadcast_ch_sync(std::shared_ptr<HostRoom> room){
    std::vector<uint8_t> base_sync;
    {
        std::lock_guard<std::mutex> clk(room->cache_mtx);
        if(room->cached_ch_sync.empty()) return;
        base_sync = room->cached_ch_sync;  // 복사
    }
    if(base_sync.size() < BEWE_HDR_SIZE + CH_SYNC_ENTRY_SIZE * MAX_CHANNELS_RELAY) return;

    // audio_mask 재계산: bit0=HOST(항상 원본 유지), bit_i=JOIN op_index i의 recv_audio[ch]
    uint8_t* payload = base_sync.data() + BEWE_HDR_SIZE;
    std::lock_guard<std::mutex> jlk(room->joins_mtx);
    for(int ch = 0; ch < MAX_CHANNELS_RELAY; ch++){
        uint8_t* entry = payload + ch * CH_SYNC_ENTRY_SIZE;
        // 원본에서 HOST bit(0) 보존
        uint32_t orig_mask;
        memcpy(&orig_mask, entry + CH_SYNC_MASK_OFFSET, sizeof(orig_mask));
        uint32_t new_mask = orig_mask & 0x1u;  // bit0(HOST) 유지

        for(auto& je : room->joins){
            if(!je->authed || !je->alive.load()) continue;
            if(je->recv_audio[ch])
                new_mask |= (1u << je->op_index);
        }
        memcpy(entry + CH_SYNC_MASK_OFFSET, &new_mask, sizeof(new_mask));
    }

    // 캐시도 갱신
    {
        std::lock_guard<std::mutex> clk(room->cache_mtx);
        room->cached_ch_sync = base_sync;
    }

    // 모든 JOIN에게 broadcast
    for(auto& je : room->joins){
        if(!je->alive.load() || je->fd < 0) continue;
        je->enqueue_data(base_sync.data(), base_sync.size());
    }

    // HOST에도 재작성된 CHANNEL_SYNC 전송 (HOST 툴팁의 listener 반영)
    if(room->alive.load() && room->fd >= 0){
        std::lock_guard<std::mutex> hlk(room->host_send_mtx);
        central_send_mux(room->fd, 0xFFFF, CentralMuxType::DATA,
                       base_sync.data(), base_sync.size());
    }
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
        // 루프백(127.x), 링크로컬(169.254.x) 제외
        if((ip >> 24) == 127) continue;
        if((ip >> 16) == 0xA9FE) continue; // 169.254.x.x
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
                printf("[Central] timeout: closing room '%s'\n", r->station_id.c_str());
                shutdown(r->fd, SHUT_RDWR); close(r->fd); r->fd=-1;
                r->alive.store(false);
            }
        }
    }
}
