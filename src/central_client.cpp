#include "central_client.hpp"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <poll.h>
#include <chrono>

// ── TCP 연결 (non-blocking connect, 3초 타임아웃) ─────────────────────────
int CentralClient::tcp_connect(const std::string& host, int port){
    addrinfo hints{}, *res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    char ps[8]; snprintf(ps, sizeof(ps), "%d", port);
    if(getaddrinfo(host.c_str(), ps, &hints, &res) != 0) return -1;
    int fd = socket(res->ai_family, res->ai_socktype, 0);
    if(fd < 0){ freeaddrinfo(res); return -1; }

    // non-blocking connect → poll로 3초 타임아웃
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int cr = ::connect(fd, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if(cr < 0 && errno != EINPROGRESS){ close(fd); return -1; }
    if(cr < 0){
        pollfd pfd{fd, POLLOUT, 0};
        int pr = poll(&pfd, 1, 3000);   // 3초 대기
        if(pr <= 0){ close(fd); return -1; }
        int so_err = 0; socklen_t sl = sizeof(so_err);
        getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_err, &sl);
        if(so_err != 0){ close(fd); return -1; }
    }
    // blocking 모드로 복원
    fcntl(fd, F_SETFL, flags);

    timeval tv{3,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int nd = 1; setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
    return fd;
}

// ── 여러 후보 IP를 순차 시도해 첫 번째 성공한 fd 반환 ────────────────────
// candidates: {primary_host, lan_ip1, lan_ip2, ...} 순서
// LAN IP가 먼저 성공하면 더 빠른 경로로 접속됨
int CentralClient::tcp_connect_any(const std::vector<std::string>& candidates, int port){
    for(const auto& h : candidates){
        if(h.empty()) continue;
        int fd = tcp_connect(h, port);
        if(fd >= 0){
            printf("[CentralClient] connected via %s:%d\n", h.c_str(), port);
            return fd;
        }
    }
    return -1;
}

// 현재 캐시된 LAN IP + 고정 host를 합친 후보 목록 반환
// LAN IP를 앞에 배치해 같은 망이면 우선 시도
std::vector<std::string> CentralClient::make_candidates(const std::string& primary_host){
    std::vector<std::string> cands;
    {
        std::lock_guard<std::mutex> lk(central_lan_ips_mtx);
        for(const auto& ip : central_lan_ips)
            if(ip != primary_host)
                cands.push_back(ip);
    }
    cands.push_back(primary_host);
    return cands;
}

// ── 목록 조회 ─────────────────────────────────────────────────────────────
std::vector<CentralClient::Station>
CentralClient::fetch_stations(const std::string& host, int port){
    std::vector<Station> out;
    // 후보 IP 순서로 접속 시도 (LAN → WAN)
    auto cands = make_candidates(host);
    int fd = tcp_connect_any(cands, port);
    if(fd < 0) return out;
    timeval tv{5,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if(!central_send_pkt(fd, CentralPktType::LIST_REQ, nullptr, 0)){ close(fd); return out; }
    CentralPktHdr hdr{}; std::vector<uint8_t> payload;
    if(!central_recv_pkt(fd, hdr, payload, 256*1024)){ close(fd); return out; }
    close(fd);
    if(static_cast<CentralPktType>(hdr.type) != CentralPktType::LIST_RESP) return out;
    if(payload.size() < sizeof(CentralListResp)) return out;
    auto* resp = reinterpret_cast<const CentralListResp*>(payload.data());

    // LAN IP 캐시 업데이트 (relay 서버가 자신의 LAN IP를 알려줌)
    {
        std::lock_guard<std::mutex> lk(central_lan_ips_mtx);
        central_lan_ips.clear();
        for(int li = 0; li < (int)resp->lan_ip_count && li < CENTRAL_MAX_LAN_IPS; li++){
            std::string ip(resp->lan_ips[li], strnlen(resp->lan_ips[li], 15));
            if(!ip.empty()) central_lan_ips.push_back(ip);
        }
    }

    uint16_t cnt = resp->count;
    const CentralStation* arr = reinterpret_cast<const CentralStation*>(
        payload.data() + sizeof(CentralListResp));
    uint16_t max_cnt = (uint16_t)((payload.size()-sizeof(CentralListResp))/sizeof(CentralStation));
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

void CentralClient::start_polling(const std::string& host, int port,
                                 std::function<void(const std::vector<Station>&)> cb){
    stop_polling();
    poll_running_.store(true);
    poll_thr_ = std::thread(&CentralClient::poll_loop, this, host, port, std::move(cb));
}

void CentralClient::stop_polling(){
    poll_running_.store(false);
    if(poll_thr_.joinable()) poll_thr_.join();
}

void CentralClient::poll_loop(std::string host, int port,
                             std::function<void(const std::vector<Station>&)> cb){
    while(poll_running_.load()){
        auto st = fetch_stations(host, port);
        cb(st); // 빈 벡터도 전달 (stale 정리용)
        for(int i = 0; i < 100 && poll_running_.load(); i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── HOST: 룸 열기 ─────────────────────────────────────────────────────────
int CentralClient::open_room(const std::string& central_host, int central_port,
                            const std::string& station_id, const std::string& station_name,
                            float lat, float lon, uint8_t host_tier){
    auto cands = make_candidates(central_host);
    int fd = tcp_connect_any(cands, central_port);
    if(fd < 0){ printf("[CentralClient] open_room: connect failed\n"); return -1; }

    // TCP 최적화: send/recv 버퍼 확대 + Nagle 비활성화
    int bufsize = 4 * 1024 * 1024;
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize));
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
    int nd = 1; setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));

    CentralHostOpen op{};
    strncpy(op.station_id,   station_id.c_str(),   sizeof(op.station_id)-1);
    strncpy(op.station_name, station_name.c_str(), sizeof(op.station_name)-1);
    op.lat = lat; op.lon = lon;
    op.host_tier = host_tier; op.user_count = 0;

    if(!central_send_pkt(fd, CentralPktType::HOST_OPEN, &op, sizeof(op))){
        close(fd); return -1;
    }
    printf("[CentralClient] room '%s' opened\n", station_id.c_str());
    return fd;
}

// ── HOST: MUX 어댑터 ──────────────────────────────────────────────────────
// relay_fd에서 MUX 스트림을 읽어 각 JOIN의 socketpair에 라우팅.
// 새 JOIN(CONN_OPEN) → socketpair 생성 → on_new_join(local_fd) 호출
// 데이터(DATA)        → 해당 join의 remote_fd로 전달
// JOIN 종료(CONN_CLOSE) → socketpair 닫기
// local_fd에서 JOIN이 보내는 데이터 → relay_fd로 MUX해서 전송
// ── central_fd 전용 송신 큐 ─────────────────────────────────────────────────
void CentralClient::enqueue_central(const void* hdr, size_t hdr_len,
                                 const void* data, size_t data_len){
    if(!central_sender_running_.load()) return;
    size_t total = hdr_len + data_len;
    std::lock_guard<std::mutex> lk(central_queue_mtx_);
    // 큐 오버플로: 오래된 항목 드롭 (FFT 프레임은 스트리밍이라 일부 유실 허용)
    int overflow_drops = 0;
    while(central_queue_bytes_ + total > CENTRAL_QUEUE_MAX_BYTES && !central_send_queue_.empty()){
        central_queue_bytes_ -= central_send_queue_.front().size();
        central_send_queue_.pop_front();
        overflow_drops++;
    }
    if(overflow_drops > 0){
        static int drop_warn = 0;
        if(drop_warn++ % 60 == 0)
            printf("[CentralClient] enqueue overflow: dropped %d, queueBytes=%zu/%zu, queueSize=%zu\n",
                   overflow_drops, central_queue_bytes_, CENTRAL_QUEUE_MAX_BYTES,
                   central_send_queue_.size());
    }
    std::vector<uint8_t> pkt(total);
    if(hdr_len && hdr)   memcpy(pkt.data(),           hdr,  hdr_len);
    if(data_len && data) memcpy(pkt.data() + hdr_len, data, data_len);
    central_send_queue_.push_back(std::move(pkt));
    central_queue_bytes_ += total;
    central_queue_cv_.notify_one();
}

// 오디오 전용 enqueue (FFT 큐와 분리, 16MB 초과 시 드롭)
void CentralClient::enqueue_central_audio(const void* hdr, size_t hdr_len,
                                          const void* data, size_t data_len){
    if(!central_sender_running_.load()) return;
    size_t total = hdr_len + data_len;
    std::lock_guard<std::mutex> lk(central_queue_mtx_);
    while(central_audio_bytes_ + total > CENTRAL_AUDIO_MAX_BYTES && !central_audio_queue_.empty()){
        central_audio_bytes_ -= central_audio_queue_.front().size();
        central_audio_queue_.pop_front();
    }
    std::vector<uint8_t> pkt(total);
    if(hdr_len && hdr)   memcpy(pkt.data(),           hdr,  hdr_len);
    if(data_len && data) memcpy(pkt.data() + hdr_len, data, data_len);
    central_audio_queue_.push_back(std::move(pkt));
    central_audio_bytes_ += total;
    central_queue_cv_.notify_one();
}

// JOIN→HOST 방향 전용 enqueue (4MB 초과 시 드롭)
void CentralClient::enqueue_join_data(const void* hdr, size_t hdr_len,
                                      const void* data, size_t data_len){
    if(!central_sender_running_.load()) return;
    size_t total = hdr_len + data_len;
    std::lock_guard<std::mutex> lk(central_queue_mtx_);
    int overflow_drops = 0;
    while(central_join_bytes_ + total > CENTRAL_JOIN_MAX_BYTES && !central_join_queue_.empty()){
        central_join_bytes_ -= central_join_queue_.front().size();
        central_join_queue_.pop_front();
        overflow_drops++;
    }
    if(overflow_drops > 0){
        static int drop_warn = 0;
        if(drop_warn++ % 60 == 0)
            printf("[CentralClient] join enqueue overflow: dropped %d, joinBytes=%zu/%zu\n",
                   overflow_drops, central_join_bytes_, CENTRAL_JOIN_MAX_BYTES);
    }
    std::vector<uint8_t> pkt(total);
    if(hdr_len && hdr)   memcpy(pkt.data(),           hdr,  hdr_len);
    if(data_len && data) memcpy(pkt.data() + hdr_len, data, data_len);
    central_join_queue_.push_back(std::move(pkt));
    central_join_bytes_ += total;
    central_queue_cv_.notify_one();
}

// HB/제어 전용 enqueue (우선순위 큐, 드롭 없음)
// 제어 패킷(CMD, ACK, CH_SYNC 등) + HB를 FFT/오디오보다 먼저 전송
void CentralClient::enqueue_hb(const void* hdr, size_t hdr_len,
                               const void* data, size_t data_len){
    if(!central_sender_running_.load()) return;
    size_t total = hdr_len + data_len;
    std::vector<uint8_t> pkt(total);
    if(hdr_len && hdr)   memcpy(pkt.data(),           hdr,  hdr_len);
    if(data_len && data) memcpy(pkt.data() + hdr_len, data, data_len);
    {
        std::lock_guard<std::mutex> lk(central_queue_mtx_);
        // 순수 HB(≤16바이트)는 최대 4개까지만 유지 (stale HB 방지)
        // 제어 패킷(CMD/ACK/CH_SYNC 등)은 드롭 없음
        bool is_pure_hb = (total <= 16 + CENTRAL_MUX_HDR_SIZE);
        if(is_pure_hb){
            while(central_hb_queue_.size() >= 4)
                central_hb_queue_.pop_front();
        }
        central_hb_queue_.push_back(std::move(pkt));
    }
    central_queue_cv_.notify_one();
}

// central_fd 단독 write 스레드
// blocking send: 네트워크 속도가 충분하면 가장 빠르고 간단한 방식
// mux_loop는 별도 스레드에서 recv → 동일 fd blocking send 가능
void CentralClient::central_sender_loop(int central_fd){
    // SO_SNDTIMEO: 10초 블로킹 상한 (연결 끊김 감지용)
    timeval tv{10, 0};
    setsockopt(central_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    uint64_t data_sent = 0, data_dropped = 0;
    auto last_stat = std::chrono::steady_clock::now();

    while(central_sender_running_.load()){
        // 배치 처리: 한 번에 최대 32개 패킷을 꺼내 연속 전송
        // → 큐 lock 횟수 감소 + 연속 write로 TCP throughput 향상
        std::vector<std::vector<uint8_t>> batch;
        batch.reserve(32);
        {
            std::unique_lock<std::mutex> lk(central_queue_mtx_);
            central_queue_cv_.wait_for(lk, std::chrono::milliseconds(200),
                [this]{ return !central_hb_queue_.empty() ||
                               !central_send_queue_.empty() ||
                               !central_audio_queue_.empty() ||
                               !central_join_queue_.empty() ||
                               !central_sender_running_.load(); });
            if(!central_sender_running_.load()) break;

            // 우선순위: HB/제어 → FFT → 오디오 → JOIN
            // HB는 전부 꺼냄 (보통 1~2개), 나머지는 최대 32개씩
            while(!central_hb_queue_.empty()){
                batch.push_back(std::move(central_hb_queue_.front()));
                central_hb_queue_.pop_front();
            }
            int n = 0;
            while(!central_send_queue_.empty() && n++ < 32){
                central_queue_bytes_ -= central_send_queue_.front().size();
                batch.push_back(std::move(central_send_queue_.front()));
                central_send_queue_.pop_front();
            }
            n = 0;
            while(!central_audio_queue_.empty() && n++ < 32){
                central_audio_bytes_ -= central_audio_queue_.front().size();
                batch.push_back(std::move(central_audio_queue_.front()));
                central_audio_queue_.pop_front();
            }
            n = 0;
            while(!central_join_queue_.empty() && n++ < 16){
                central_join_bytes_ -= central_join_queue_.front().size();
                batch.push_back(std::move(central_join_queue_.front()));
                central_join_queue_.pop_front();
            }
        }
        if(batch.empty()) continue;

        bool failed = false;
        for(auto& pkt : batch){
            const uint8_t* p = pkt.data();
            size_t rem = pkt.size();
            while(rem > 0){
                ssize_t r = ::send(central_fd, p, rem, MSG_NOSIGNAL);
                if(r > 0){ p += r; rem -= r; continue; }
                if(r < 0 && errno == EINTR) continue;
                printf("[CentralClient] sender: send error errno=%d(%s)\n",
                       errno, strerror(errno));
                failed = true; break;
            }
            if(failed) break;
            data_sent++;
        }

        if(failed){ shutdown(central_fd, SHUT_RDWR); break; }

        // 30초마다 통계
        auto now = std::chrono::steady_clock::now();
        if(std::chrono::duration_cast<std::chrono::seconds>(now-last_stat).count() >= 30){
            printf("[CentralClient] sender stats: sent=%llu dropped=%llu fftQ=%zu audioQ=%zu joinQ=%zu\n",
                   (unsigned long long)data_sent, (unsigned long long)data_dropped,
                   central_queue_bytes_, central_audio_bytes_, central_join_bytes_);
            last_stat = now;
        }
    }
    central_sender_running_.store(false);
}

void CentralClient::start_mux_adapter(int central_fd,
                                     std::function<void(int)> on_new_join,
                                     std::function<uint8_t()> user_count_fn,
                                     std::function<void()> on_disconnect){
    stop_mux_adapter();
    mux_central_fd_ = central_fd;
    on_central_disconnect_ = std::move(on_disconnect);

    // central_fd: 초기에는 타임아웃 없음 (sender_loop에서 프레임 타입별로 설정)
    timeval tv0{0, 0};
    setsockopt(central_fd, SOL_SOCKET, SO_SNDTIMEO, &tv0, sizeof(tv0));

    // relay_sender 먼저 시작 (pump/HB enqueue 직전)
    central_queue_bytes_ = 0;
    central_audio_bytes_ = 0;
    central_join_bytes_  = 0;
    central_send_queue_.clear();
    central_hb_queue_.clear();
    central_audio_queue_.clear();
    central_join_queue_.clear();
    central_sender_running_.store(true);
    central_sender_thr_ = std::thread(&CentralClient::central_sender_loop, this, central_fd);

    mux_running_.store(true);
    mux_thr_ = std::thread(&CentralClient::mux_loop, this,
                            central_fd, std::move(on_new_join), std::move(user_count_fn));
}

void CentralClient::stop_mux_adapter(){
    mux_running_.store(false);
    central_sender_running_.store(false);
    central_queue_cv_.notify_all();  // relay_sender 대기 중이면 깨우기
    if(mux_central_fd_ >= 0){ shutdown(mux_central_fd_, SHUT_RDWR); }
    if(mux_thr_.joinable()) mux_thr_.join();
    if(central_sender_thr_.joinable()) central_sender_thr_.join();
    {
        std::lock_guard<std::mutex> qlk(central_queue_mtx_);
        central_send_queue_.clear();
        central_hb_queue_.clear();
        central_audio_queue_.clear();
        central_join_queue_.clear();
        central_queue_bytes_ = 0;
        central_audio_bytes_ = 0;
        central_join_bytes_  = 0;
    }
    std::lock_guard<std::mutex> lk(mux_joins_mtx_);
    for(auto& [id, jp] : mux_joins_){
        if(jp->local_fd  >= 0){ shutdown(jp->local_fd,  SHUT_RDWR); close(jp->local_fd);  }
        if(jp->remote_fd >= 0){ shutdown(jp->remote_fd, SHUT_RDWR); close(jp->remote_fd); }
        if(jp->thr.joinable()) jp->thr.detach();
    }
    mux_joins_.clear();
    mux_central_fd_ = -1;
}

void CentralClient::mux_loop(int central_fd,
                            std::function<void(int)> on_new_join,
                            std::function<uint8_t()> count_fn){
    std::vector<uint8_t> buf(256*1024);

    // keepalive heartbeat 타이머
    auto last_hb = std::chrono::steady_clock::now();

    while(mux_running_.load()){
        // MUX 헤더 수신 (non-blocking 타임아웃 3초로 HB 전송 기회 확보)
        timeval tv{3,0};
        setsockopt(central_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        CentralMuxHdr mux{};
        ssize_t r = recv(central_fd, &mux, CENTRAL_MUX_HDR_SIZE, MSG_WAITALL);
        if(r <= 0){
            if(r == 0){
                printf("[CentralClient] mux_loop: relay connection closed (EOF)\n");
                break;
            }
            if(errno != EAGAIN && errno != EWOULDBLOCK && errno != ETIMEDOUT){
                printf("[CentralClient] mux_loop: recv error errno=%d(%s)\n",
                       errno, strerror(errno));
                break;
            }
            // EAGAIN/ETIMEDOUT → heartbeat 우선 큐에 enqueue
            // enqueue_hb는 non-blocking, sender_loop에서 데이터보다 먼저 전송됨
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(now-last_hb).count() >= 3){
                last_hb = now;
                CentralMuxHdr hb{}; hb.type = 0x00; hb.len = sizeof(CentralHostHb);
                CentralHostHb hbp{}; hbp.user_count = count_fn ? count_fn() : 0;
                enqueue_hb(&hb, CENTRAL_MUX_HDR_SIZE, &hbp, sizeof(hbp));
                printf("[CentralClient] HB enqueued (priority, users=%u)\n", hbp.user_count);
            }
            continue;
        }
        if(r != CENTRAL_MUX_HDR_SIZE){
            printf("[CentralClient] mux_loop: short read %zd/%d\n", r, CENTRAL_MUX_HDR_SIZE);
            break;
        }

        // payload 수신용 타임아웃 (5초) — 무한 블로킹 방지
        timeval tvp{5,0};
        setsockopt(central_fd, SOL_SOCKET, SO_RCVTIMEO, &tvp, sizeof(tvp));

        auto mux_type = static_cast<CentralMuxType>(mux.type);
        uint16_t cid  = mux.conn_id;

        if(mux_type == CentralMuxType::CONN_OPEN){
            // 새 JOIN → socketpair 생성
            int sv[2];
            if(socketpair(AF_UNIX, SOCK_STREAM, 0, sv) < 0) continue;
            // socketpair 버퍼 확대 (relay 경유 시 burst 흡수)
            int spbuf = 212992;
            setsockopt(sv[0], SOL_SOCKET, SO_SNDBUF, &spbuf, sizeof(spbuf));
            setsockopt(sv[0], SOL_SOCKET, SO_RCVBUF, &spbuf, sizeof(spbuf));
            setsockopt(sv[1], SOL_SOCKET, SO_SNDBUF, &spbuf, sizeof(spbuf));
            setsockopt(sv[1], SOL_SOCKET, SO_RCVBUF, &spbuf, sizeof(spbuf));
            // remote_fd(sv[1])에 send 타임아웃: 느린 JOIN이 mux_loop를 블로킹하지 않도록
            timeval sp_tv{0, 100000}; // 100ms
            setsockopt(sv[1], SOL_SOCKET, SO_SNDTIMEO, &sp_tv, sizeof(sp_tv));
            // sv[0]: NetServer가 accept 대신 직접 client_loop에 inject (local_fd)
            // sv[1]: relay로 데이터 보내는 쪽 (remote_fd)
            auto jp = std::make_shared<JoinPair>();
            jp->local_fd  = sv[0];
            jp->remote_fd = sv[1];
            {
                std::lock_guard<std::mutex> lk(mux_joins_mtx_);
                mux_joins_[cid] = jp;
            }

            // sv[1] → central_fd pump (JOIN이 local_fd에 쓰는 데이터를 relay로)
            // enqueue_join_data 사용: JOIN→HOST 전용 큐로 분리 → FFT/오디오 큐 방해 없음
            jp->thr = std::thread([jp, cid, this](){
                std::vector<uint8_t> tbuf(65536);
                while(jp->local_fd >= 0 && mux_running_.load()){
                    ssize_t n = recv(jp->remote_fd, tbuf.data(), tbuf.size(), 0);
                    if(n <= 0) break;
                    CentralMuxHdr mh{}; mh.conn_id=cid;
                    mh.type = static_cast<uint8_t>(CentralMuxType::DATA);
                    mh.len  = (uint32_t)n;
                    enqueue_join_data(&mh, CENTRAL_MUX_HDR_SIZE, tbuf.data(), n);
                }
                printf("[CentralClient] pump exit conn_id=%u\n", cid);
            });
            jp->thr.detach();

            // NetServer에 새 JOIN 알림 (sv[0]를 client 소켓으로 inject)
            on_new_join(sv[0]);
            printf("[CentralClient] new JOIN conn_id=%u, injected fd=%d\n", cid, sv[0]);

        } else if(mux_type == CentralMuxType::DATA){
            if(mux.len > (uint32_t)buf.size()) buf.resize(mux.len);
            if(mux.len > 0 && !central_recv_all(central_fd, buf.data(), mux.len)) break;

            // 릴레이→HOST 방향 CHANNEL_SYNC (audio_mask 재작성본): HOST가 직접 처리
            // conn_id=0xFFFF + BEWE type=0x0A → HOST의 audio_mask 갱신
            if(cid == 0xFFFF && mux.len >= 9 && buf[4] == 0x0A){
                if(on_central_ch_sync_)
                    on_central_ch_sync_(buf.data(), mux.len);
                continue;  // socketpair에 전달 안 함
            }

            // 중앙서버→HOST CHAT: relay 루프 방지를 위해 socketpair 전달 않음
            // (JOIN들은 중앙서버에서 직접 CHAT을 수신)
            // BEWE 패킷: magic[4]+type[1]+len[4]+payload → buf[4]=type
            if(mux.len >= 9 && buf[4] == 0x07){  // BEWE_TYPE_CHAT
                if(on_central_chat_ && mux.len >= 9 + 32){
                    const char* from = reinterpret_cast<const char*>(buf.data() + 9);
                    const char* msg  = reinterpret_cast<const char*>(buf.data() + 9 + 32);
                    on_central_chat_(from, msg);
                }
                continue;
            }
            // 특정 JOIN 또는 broadcast → socketpair로 전달
            {
                uint8_t bewe_t = (mux.len >= 5) ? buf[4] : 0xFF;
                static uint64_t data_fwd_count = 0;
                data_fwd_count++;
                if(bewe_t == 0x02 || data_fwd_count % 300 == 1)
                    printf("[CentralClient] mux DATA→sp: cid=%u bewe_type=0x%02x len=%u joins=%zu (#%llu)\n",
                           cid, bewe_t, mux.len, mux_joins_.size(), (unsigned long long)data_fwd_count);
            }
            std::lock_guard<std::mutex> lk(mux_joins_mtx_);
            for(auto& [id, jp] : mux_joins_){
                if(cid != 0xFFFF && id != cid) continue;
                if(jp->remote_fd >= 0){
                    uint8_t bewe_t = (mux.len >= 5) ? buf[4] : 0xFF;
                    ssize_t sr = send(jp->remote_fd, buf.data(), mux.len, MSG_NOSIGNAL | MSG_DONTWAIT);
                    if(sr != (ssize_t)mux.len)
                        printf("[CentralClient] mux DATA→sp PARTIAL: cid=%u bewe_type=0x%02x len=%u sent=%zd errno=%d(%s)\n",
                               cid, bewe_t, mux.len, sr, errno, strerror(errno));
                }
            }

        } else if(mux_type == CentralMuxType::CONN_CLOSE){
            std::lock_guard<std::mutex> lk(mux_joins_mtx_);
            auto it = mux_joins_.find(cid);
            if(it != mux_joins_.end()){
                auto& jp = it->second;
                // local_fd는 shutdown만 — close는 NetServer::drop_client가 담당
                // (이중 close 시 ALSA 등이 재사용한 fd를 닫아 abort 발생)
                if(jp->local_fd  >= 0){ shutdown(jp->local_fd,  SHUT_RDWR); jp->local_fd=-1;  }
                if(jp->remote_fd >= 0){ shutdown(jp->remote_fd, SHUT_RDWR); close(jp->remote_fd); jp->remote_fd=-1; }
                mux_joins_.erase(it);
                printf("[CentralClient] JOIN conn_id=%u disconnected\n", cid);
            }
        }
    }
    mux_running_.store(false);
    if(on_central_disconnect_) on_central_disconnect_();
}

// ── JOIN 모드: 룸 입장 ────────────────────────────────────────────────────
int CentralClient::join_room(const std::string& central_host, int central_port,
                            const std::string& station_id){
    auto cands = make_candidates(central_host);
    int fd = tcp_connect_any(cands, central_port);
    if(fd < 0){ printf("[CentralClient] join_room: connect failed\n"); return -1; }

    CentralJoinRoom jr{};
    strncpy(jr.station_id, station_id.c_str(), sizeof(jr.station_id)-1);
    if(!central_send_pkt(fd, CentralPktType::JOIN_ROOM, &jr, sizeof(jr))){
        close(fd); return -1;
    }

    // ERROR 체크 (룸 없으면 relay가 즉시 ERROR 전송)
    timeval tv{2,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    uint8_t peek[4] = {};
    ssize_t pn = recv(fd, peek, sizeof(peek), MSG_PEEK | MSG_DONTWAIT);
    if(pn >= 4 && memcmp(peek, CENTRAL_MAGIC, 4) == 0){
        CentralPktHdr ehdr{}; std::vector<uint8_t> ep;
        if(central_recv_pkt(fd, ehdr, ep, 256)){
            if(static_cast<CentralPktType>(ehdr.type) == CentralPktType::ERROR){
                printf("[CentralClient] join_room error: %s\n",
                       ep.size()>=sizeof(CentralError)
                       ? reinterpret_cast<const CentralError*>(ep.data())->msg : "unknown");
                close(fd); return -1;
            }
        }
    }
    timeval tv0{0,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));

    printf("[CentralClient] joined room '%s'\n", station_id.c_str());
    return fd;
}
