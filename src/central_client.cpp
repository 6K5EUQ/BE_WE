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

extern void bewe_log_push(int col, const char* fmt, ...);

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
            bewe_log_push(1,"[CentralClient] connected via %s:%d\n", h.c_str(), port);
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

// ── 목록 조회 (persistent TCP) ────────────────────────────────────────────
// 한 fd를 유지하며 LIST_REQ를 1초마다 반복 송신. WiFi에서 매 폴링마다 새 connect
// 하던 기존 방식의 round-trip 비용 (3-5s)을 제거 → 1초 주기 실효화.

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
        // 1) connect (실패 시 backoff 후 재시도)
        auto cands = make_candidates(host);
        int fd = tcp_connect_any(cands, port);
        if(fd < 0){
            for(int i = 0; i < 20 && poll_running_.load(); i++)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        timeval tv{5,0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        // 2) 같은 fd에서 LIST_REQ 반복 — socket 사망 시 outer loop가 reconnect
        while(poll_running_.load()){
            if(!central_send_pkt(fd, CentralPktType::LIST_REQ, nullptr, 0)) break;
            CentralPktHdr hdr{}; std::vector<uint8_t> payload;
            if(!central_recv_pkt(fd, hdr, payload, 256*1024)) break;
            if(static_cast<CentralPktType>(hdr.type) != CentralPktType::LIST_RESP) break;
            if(payload.size() < sizeof(CentralListResp)) break;

            auto* resp = reinterpret_cast<const CentralListResp*>(payload.data());

            // LAN IP 캐시 갱신
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
            std::vector<Station> stations;
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
                stations.push_back(std::move(s));
            }
            cb(stations);

            // 700ms sleep — cb 간격이 grace 1초보다 짧도록 보장 (RTT 여유 300ms)
            for(int i = 0; i < 7 && poll_running_.load(); i++)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        close(fd);
        // socket 끊김 → outer loop가 즉시 reconnect 시도
    }
}

// ── HOST: 룸 열기 ─────────────────────────────────────────────────────────
int CentralClient::open_room(const std::string& central_host, int central_port,
                            const std::string& station_id, const std::string& station_name,
                            float lat, float lon, uint8_t host_tier){
    stat_room_id = station_id;
    auto cands = make_candidates(central_host);
    int fd = tcp_connect_any(cands, central_port);
    if(fd < 0){ bewe_log_push(1,"[CentralClient] open_room: connect failed\n"); return -1; }

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
    bewe_log_push(1,"[CentralClient] room '%s' opened\n", station_id.c_str());
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
                                 const void* data, size_t data_len,
                                 bool no_drop){
    if(!central_sender_running_.load()) return;
    size_t total = hdr_len + data_len;
    std::lock_guard<std::mutex> lk(central_queue_mtx_);
    // 큐 오버플로: 드롭 가능한(no_drop=false) 항목만 제거
    if(!no_drop){
        while(central_queue_bytes_ + total > CENTRAL_QUEUE_MAX_BYTES && !central_send_queue_.empty()){
            // no_drop 패킷은 건너뛰고 드롭 가능한 패킷만 제거
            bool dropped = false;
            for(auto it = central_send_queue_.begin(); it != central_send_queue_.end(); ++it){
                if(!it->no_drop){
                    central_queue_bytes_ -= it->data.size();
                    central_send_queue_.erase(it);
                    dropped = true;
                    break;
                }
            }
            if(!dropped) break; // 드롭 가능한 패킷 없으면 중단
        }
    }
    std::vector<uint8_t> pkt(total);
    if(hdr_len && hdr)   memcpy(pkt.data(),           hdr,  hdr_len);
    if(data_len && data) memcpy(pkt.data() + hdr_len, data, data_len);
    // HEARTBEAT(0x14)는 큐 front에 priority push — backed up이어도 다음 batch 첫 번째로 송신됨.
    // BEWE 헤더 type 위치: hdr_len(CentralMuxHdr 7B) + 4 (BEWE magic 뒤).
    bool is_hb = (data_len >= 9 && ((const uint8_t*)data)[4] == 0x14);
    if(is_hb) central_send_queue_.push_front({std::move(pkt), no_drop});
    else      central_send_queue_.push_back({std::move(pkt), no_drop});
    central_queue_bytes_ += total;
    central_queue_cv_.notify_one();
}

// central_fd 단독 write 스레드: 배치 처리로 큐를 빠르게 드레인
void CentralClient::central_sender_loop(int central_fd){
    std::deque<std::vector<uint8_t>> batch;
    while(central_sender_running_.load()){
        batch.clear();
        {
            std::unique_lock<std::mutex> lk(central_queue_mtx_);
            central_queue_cv_.wait_for(lk, std::chrono::milliseconds(200),
                [this]{ return !central_send_queue_.empty() || !central_sender_running_.load(); });
            if(central_send_queue_.empty()) continue;
            // 한 번에 최대 64개 꺼내기
            int n = 0;
            while(!central_send_queue_.empty() && n++ < 64){
                central_queue_bytes_ -= central_send_queue_.front().data.size();
                batch.push_back(std::move(central_send_queue_.front().data));
                central_send_queue_.pop_front();
            }
        }
        for(auto& pkt : batch){
            if(!central_send_all(central_fd, pkt.data(), pkt.size())){
                int e = errno;
                bewe_log_push(1,"[CentralClient] relay_sender: send failed errno=%d(%s), closing central_fd\n",
                       e, strerror(e));
                shutdown(central_fd, SHUT_RDWR);
                central_sender_running_.store(false);
                return;
            }
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

    // central_fd 송신 타임아웃 해제 (relay_sender가 단독으로 블로킹 write)
    // tcp_connect에서 설정된 3초 SNDTIMEO를 제거해 장기 스트림에 맞게 설정
    timeval tv0{0, 0};
    setsockopt(central_fd, SOL_SOCKET, SO_SNDTIMEO, &tv0, sizeof(tv0));

    // relay_sender 먼저 시작 (pump/HB enqueue 직전)
    central_queue_bytes_ = 0;
    central_send_queue_.clear();
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
        central_queue_bytes_ = 0;
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
    auto last_state = last_hb;
    auto stats_start = last_hb;
    auto stats_last  = last_hb;
    uint64_t sp_tot=stat_tx_total_bytes.load(), sp_fft=stat_tx_fft_bytes.load();
    uint64_t sp_aud=stat_tx_audio_bytes.load(), sp_hb=stat_tx_hb_bytes.load();
    uint64_t sp_file=stat_tx_file_bytes.load();
    uint64_t sp_hist=stat_tx_hist_bytes.load();
    auto print_host_stats = [&](){
        auto now = std::chrono::steady_clock::now();
        double win_sec = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_last).count() / 1000.0;
        if(win_sec < 2.999) return;
        long long uptime = std::chrono::duration_cast<std::chrono::seconds>(now - stats_start).count();
        uint64_t t=stat_tx_total_bytes.load(), f=stat_tx_fft_bytes.load();
        uint64_t a=stat_tx_audio_bytes.load(), h=stat_tx_hb_bytes.load();
        uint64_t fl=stat_tx_file_bytes.load();
        uint64_t hi=stat_tx_hist_bytes.load();
        bewe_log_push(0,"[HOST] [STATS] room='%s' uptime=%llds | trans: %.1f KB/s | hb=%.1f KB/s fft=%.1f KB/s audio=%.1f KB/s File=%.1f KB/s hist=%.1f KB/s\n",
            stat_room_id.c_str(), uptime,
            (double)(t-sp_tot)/win_sec/1024.0,
            (double)(h-sp_hb)/win_sec/1024.0,
            (double)(f-sp_fft)/win_sec/1024.0,
            (double)(a-sp_aud)/win_sec/1024.0,
            (double)(fl-sp_file)/win_sec/1024.0,
            (double)(hi-sp_hist)/win_sec/1024.0);
        stats_last=now; sp_tot=t; sp_fft=f; sp_aud=a; sp_hb=h; sp_file=fl; sp_hist=hi;
    };

    // SO_RCVTIMEO 루프 밖에서 한 번만 설정 (HB 주기 3초)
    {
        timeval tv{3,0};
        setsockopt(central_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    while(mux_running_.load()){
        print_host_stats();
        CentralMuxHdr mux{};
        ssize_t r = recv(central_fd, &mux, CENTRAL_MUX_HDR_SIZE, MSG_WAITALL);
        if(r <= 0){
            if(r == 0){
                bewe_log_push(1,"[CentralClient] mux_loop: relay connection closed (EOF)\n");
                break;
            }
            if(errno != EAGAIN && errno != EWOULDBLOCK && errno != ETIMEDOUT){
                bewe_log_push(1,"[CentralClient] mux_loop: recv error errno=%d(%s)\n",
                       errno, strerror(errno));
                break;
            }
            // EAGAIN/ETIMEDOUT → heartbeat enqueue 후 계속
            // enqueue_central는 블로킹 없음 (central_sender_thr_가 실제 write)
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(now-last_hb).count() >= 3){
                last_hb = now;
                CentralMuxHdr hb{}; hb.type = 0x00; hb.len = sizeof(CentralHostHb);
                CentralHostHb hbp{}; hbp.user_count = count_fn ? count_fn() : 0;
                uint64_t hb_tot = CENTRAL_MUX_HDR_SIZE + sizeof(CentralHostHb);
                stat_tx_total_bytes.fetch_add(hb_tot, std::memory_order_relaxed);
                stat_tx_hb_bytes.fetch_add(hb_tot, std::memory_order_relaxed);
                enqueue_central(&hb, CENTRAL_MUX_HDR_SIZE, &hbp, sizeof(hbp));
            }
            // HOST_STATE every 5 s for status page v2 (skipped if no state_fn)
            if(state_fn_ &&
               std::chrono::duration_cast<std::chrono::seconds>(now-last_state).count() >= 5){
                last_state = now;
                CentralHostStateFull st{};
                state_fn_(st);
                CentralMuxHdr smh{}; smh.conn_id = 0xFFFF;
                smh.type = static_cast<uint8_t>(CentralMuxType::HOST_STATE);
                smh.len  = sizeof(st);
                uint64_t st_tot = CENTRAL_MUX_HDR_SIZE + sizeof(st);
                stat_tx_total_bytes.fetch_add(st_tot, std::memory_order_relaxed);
                stat_tx_hb_bytes.fetch_add(st_tot, std::memory_order_relaxed);
                enqueue_central(&smh, CENTRAL_MUX_HDR_SIZE, &st, sizeof(st));
            }
            continue;
        }
        if(r != CENTRAL_MUX_HDR_SIZE){
            bewe_log_push(1,"[CentralClient] mux_loop: short read %zd/%d\n", r, CENTRAL_MUX_HDR_SIZE);
            break;
        }

        auto mux_type = static_cast<CentralMuxType>(mux.type);
        uint16_t cid  = mux.conn_id;

        if(mux_type == CentralMuxType::CONN_OPEN){
            // 새 JOIN → socketpair 생성
            int sv[2];
            if(socketpair(AF_UNIX, SOCK_STREAM, 0, sv) < 0) continue;
            // socketpair 버퍼 확대: FFT 1프레임(131KB) burst 흡수
            int spbuf = 1 * 1024 * 1024;  // 1MB
            setsockopt(sv[0], SOL_SOCKET, SO_SNDBUF, &spbuf, sizeof(spbuf));
            setsockopt(sv[0], SOL_SOCKET, SO_RCVBUF, &spbuf, sizeof(spbuf));
            setsockopt(sv[1], SOL_SOCKET, SO_SNDBUF, &spbuf, sizeof(spbuf));
            setsockopt(sv[1], SOL_SOCKET, SO_RCVBUF, &spbuf, sizeof(spbuf));
            // sv[0]: NetServer가 accept 대신 직접 client_loop에 inject (local_fd)
            // sv[1]: relay로 데이터 보내는 쪽 (remote_fd)
            auto jp = std::make_shared<JoinPair>();
            jp->local_fd  = sv[0];
            jp->remote_fd = sv[1];
            {
                std::lock_guard<std::mutex> lk(mux_joins_mtx_);
                mux_joins_[cid] = jp;
            }

            // sv[1] → central_fd pump.
            // BEWE-frame-aware: 9byte header 읽고 len만큼 정확히 읽어 한 MUX 메시지로 전송.
            // (이전 64KB chunk 방식은 대용량 패킷이 여러 MUX 조각으로 분할되어 Central이
            //  각 조각을 독립 BEWE 패킷으로 잘못 해석 → 잘못된 queue로 분류 → out-of-order
            //  전달되어 JOIN에서 bad-magic 발생. 프레임 단위로 묶어 해결.)
            jp->thr = std::thread([jp, cid, this](){
                auto recv_all = [&](void* dst, size_t need) -> bool {
                    uint8_t* p = (uint8_t*)dst;
                    size_t got = 0;
                    while(got < need){
                        ssize_t r = recv(jp->remote_fd, p + got, need - got, 0);
                        if(r <= 0) return false;
                        got += (size_t)r;
                    }
                    return true;
                };
                std::vector<uint8_t> pkt;
                while(jp->local_fd >= 0 && mux_running_.load()){
                    uint8_t hdr[9];
                    if(!recv_all(hdr, 9)) break;
                    if(memcmp(hdr, "BEWE", 4) != 0){
                        // 비-BEWE 데이터(잘 안 발생). 안전하게 종료.
                        bewe_log_push(0,"[CentralClient] pump bad magic conn_id=%u\n", cid);
                        break;
                    }
                    uint32_t plen;
                    memcpy(&plen, hdr + 5, 4);
                    if(plen > 4*1024*1024){
                        bewe_log_push(0,"[CentralClient] pump oversize conn_id=%u plen=%u\n", cid, plen);
                        break;
                    }
                    pkt.resize(9 + plen);
                    memcpy(pkt.data(), hdr, 9);
                    if(plen > 0 && !recv_all(pkt.data() + 9, plen)) break;
                    bool no_drop = false;
                    uint8_t btype = hdr[4];
                    if(btype == 0x02 || btype == 0x06 || btype == 0x0F
                       || btype == 0x0D || btype == 0x0E)   // FILE_DATA + FILE_META
                        no_drop = true;  // AUTH_ACK, CMD_ACK, REGION_RESPONSE, FILE_*
                    CentralMuxHdr mh{}; mh.conn_id=cid;
                    mh.type = static_cast<uint8_t>(CentralMuxType::DATA);
                    mh.len  = (uint32_t)pkt.size();
                    enqueue_central(&mh, CENTRAL_MUX_HDR_SIZE,
                                    pkt.data(), pkt.size(), no_drop);
                }
                bewe_log_push(1,"[CentralClient] pump exit conn_id=%u\n", cid);
            });
            jp->thr.detach();

            // NetServer에 새 JOIN 알림 (sv[0]를 client 소켓으로 inject)
            on_new_join(sv[0]);
            bewe_log_push(1,"[CentralClient] new JOIN conn_id=%u, injected fd=%d\n", cid, sv[0]);

            // Host 쪽에서 BAND_PLAN_SYNC 등 초기 상태 푸시 트리거
            if(on_central_conn_open_) on_central_conn_open_(cid);

        } else if(mux_type == CentralMuxType::DATA){
            if(mux.len > (uint32_t)buf.size()) buf.resize(mux.len);
            if(mux.len > 0 && !central_recv_all(central_fd, buf.data(), mux.len)) break;

            // 릴레이→HOST 방향 broadcast (conn_id=0xFFFF) 처리
            if(cid == 0xFFFF && mux.len >= 9){
                uint8_t btype = buf[4];
                if(btype == 0x0A){  // CH_SYNC: audio_mask 갱신
                    if(on_central_ch_sync_)
                        on_central_ch_sync_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x09){  // OP_LIST: HOST UI 오퍼레이터 목록 갱신
                    if(on_central_op_list_)
                        on_central_op_list_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x26){  // DB_LIST: Central DB 파일 목록
                    if(on_central_db_list_)
                        on_central_db_list_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x22){  // REPORT_LIST: Central → HOST
                    if(on_central_report_list_)
                        on_central_report_list_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x28){  // DB_DOWNLOAD_DATA: Central → HOST
                    if(on_central_db_dl_data_)
                        on_central_db_dl_data_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x2D){  // DB_DOWNLOAD_INFO: Central → HOST .info delivery
                    if(on_central_db_dl_info_)
                        on_central_db_dl_info_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x30){  // SCHED_SYNC: Central이 저장해 둔 예약 리스트를 HOST에 복원
                    if(on_central_sched_sync_)
                        on_central_sched_sync_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x42){  // EMITTER_LIST: Central → HOST
                    if(on_central_emitter_list_)
                        on_central_emitter_list_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x46){  // SIGHTING_LIST: Central → HOST
                    if(on_central_sighting_list_)
                        on_central_sighting_list_(buf.data(), mux.len);
                    continue;
                }
                if(btype == 0x31){  // BAND_PLAN_SYNC: host owns it; ignore any incoming.
                    continue;
                }
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
            std::lock_guard<std::mutex> lk(mux_joins_mtx_);
            for(auto& [id, jp] : mux_joins_){
                if(cid != 0xFFFF && id != cid) continue;
                if(jp->remote_fd >= 0)
                    send(jp->remote_fd, buf.data(), mux.len, MSG_NOSIGNAL);
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
                bewe_log_push(1,"[CentralClient] JOIN conn_id=%u disconnected\n", cid);
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
    if(fd < 0){ bewe_log_push(1,"[CentralClient] join_room: connect failed\n"); return -1; }

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
                bewe_log_push(1,"[CentralClient] join_room error: %s\n",
                       ep.size()>=sizeof(CentralError)
                       ? reinterpret_cast<const CentralError*>(ep.data())->msg : "unknown");
                close(fd); return -1;
            }
        }
    }
    timeval tv0{0,0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv0, sizeof(tv0));

    bewe_log_push(1,"[CentralClient] joined room '%s'\n", station_id.c_str());
    return fd;
}
