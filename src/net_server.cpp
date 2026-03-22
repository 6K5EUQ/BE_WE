#include "net_server.hpp"
#include "udp_discovery.hpp"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <tuple>
#include <algorithm>
#include <chrono>
#include <thread>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <fcntl.h>

// IQ 파일 전송 속도: TCP send 실측 기반 적응형.
// send_all이 blocking이므로 네트워크 병목(HOST 업로드, JOIN 다운로드)에 자동 적응.
// FFT 스트림 보호를 위해 실측 속도의 90%만 사용.
static constexpr uint64_t FILE_RATE_FLOOR = 1 * 1024 * 1024;   // 최솟값 1 MB/s
static constexpr uint64_t FILE_RATE_INIT  = 5 * 1024 * 1024;   // 초기값 5 MB/s (보수적 시작, 빠르게 수렴)
static constexpr double   FILE_RATE_EWMA_ALPHA = 0.4;           // EWMA 빠른 수렴

// ── start / stop ──────────────────────────────────────────────────────────
bool NetServer::start(int port){
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd_ < 0){ perror("socket"); return false; }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)port);

    if(bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0){
        perror("bind"); close(server_fd_); server_fd_=-1; return false;
    }
    // port==0 이면 OS가 할당한 실제 포트를 가져옴
    {
        sockaddr_in bound{};
        socklen_t blen = sizeof(bound);
        if(getsockname(server_fd_, (sockaddr*)&bound, &blen) == 0)
            listen_port_ = ntohs(bound.sin_port);
        else
            listen_port_ = port;
    }
    if(listen(server_fd_, 8) < 0){
        perror("listen"); close(server_fd_); server_fd_=-1; return false;
    }

    running_.store(true);
    accept_thr_ = std::thread(&NetServer::accept_loop, this);
    printf("[NetServer] listening on port %d\n", listen_port_);
    return true;
}

void NetServer::stop(){
    running_.store(false);
    if(server_fd_ >= 0){ shutdown(server_fd_, SHUT_RDWR); close(server_fd_); server_fd_=-1; }
    if(accept_thr_.joinable()) accept_thr_.join();

    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        c->alive.store(false);
        c->stop_send_worker();
        if(c->fd >= 0){ shutdown(c->fd, SHUT_RDWR); close(c->fd); c->fd=-1; }
        if(c->thr.joinable()) c->thr.join();
    }
    clients_.clear();
}


int NetServer::client_count() const {
    std::lock_guard<std::mutex> lk(clients_mtx_);
    int n = 0;
    for(auto& c : clients_) if(c->authed && c->alive.load()) ++n;
    return n;
}

// ── Accept loop ───────────────────────────────────────────────────────────
void NetServer::accept_loop(){
    while(running_.load()){
        sockaddr_in caddr{}; socklen_t clen = sizeof(caddr);
        int cfd = accept(server_fd_, (sockaddr*)&caddr, &clen);
        if(cfd < 0){
            if(running_.load()) perror("accept");
            break;
        }
        // TCP_NODELAY: Nagle 알고리즘 비활성화 → FFT 스트림 지연 방지
        int nd=1; setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));
        // set TCP keepalive
        int ka=1; setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
        // SO_SNDTIMEO: 느린 클라이언트로 인한 send 블로킹 방지
        timeval stv{2, 0};
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

        auto conn = std::make_shared<ClientConn>();
        conn->fd = cfd;
        conn->alive.store(true);
        conn->start_send_worker();

        {
            std::lock_guard<std::mutex> lk(clients_mtx_);
            clients_.push_back(conn);
        }
        conn->thr = std::thread(&NetServer::client_loop, this, conn);
        conn->thr.detach();
    }
}

// ── Inject fd (relay MUX 모드) ────────────────────────────────────────────
void NetServer::inject_fd(int fd){
    auto conn = std::make_shared<ClientConn>();
    conn->fd = fd;
    conn->is_relay = true;
    conn->alive.store(true);
    conn->start_send_worker();
    relay_client_count_.fetch_add(1);
    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        clients_.push_back(conn);
    }
    conn->thr = std::thread(&NetServer::client_loop, this, conn);
    conn->thr.detach();
}

// ── Client loop ───────────────────────────────────────────────────────────
void NetServer::client_loop(std::shared_ptr<ClientConn> c){
    uint64_t pkt_count = 0;
    while(c->alive.load()){
        PktHdr hdr{};
        if(!recv_all(c->fd, &hdr, PKT_HDR_SIZE)){
            int e = errno;
            if(c->alive.load())  // 정상 stop이면 로그 생략
                printf("[NetServer] recv hdr failed op=%d('%s') fd=%d errno=%d(%s) pkts=%llu\n",
                       c->op_index, c->name, c->fd, e, strerror(e), (unsigned long long)pkt_count);
            break;
        }
        if(memcmp(hdr.magic, BEWE_MAGIC, 4) != 0){
            printf("[NetServer] bad magic from op=%d('%s') fd=%d (type=0x%02x)\n",
                   c->op_index, c->name, c->fd, hdr.type);
            break;
        }
        uint32_t len = hdr.len;
        if(len > 1024*1024){
            printf("[NetServer] oversized pkt op=%d type=0x%02x len=%u\n",
                   c->op_index, (uint8_t)hdr.type, len);
            break;
        }
        std::vector<uint8_t> payload(len);
        if(len > 0 && !recv_all(c->fd, payload.data(), len)){
            int e = errno;
            printf("[NetServer] recv payload failed op=%d type=0x%02x len=%u errno=%d(%s)\n",
                   c->op_index, (uint8_t)hdr.type, len, e, strerror(e));
            break;
        }
        pkt_count++;
        handle_packet(c, static_cast<PacketType>(hdr.type),
                      payload.data(), len);
    }
    drop_client(c);
}

// ── Packet handler ────────────────────────────────────────────────────────
void NetServer::handle_packet(std::shared_ptr<ClientConn> c,
                               PacketType type,
                               const uint8_t* payload, uint32_t len){
    switch(type){

    case PacketType::AUTH_REQ: {
        if(len < sizeof(PktAuthReq)) break;
        auto* req = reinterpret_cast<const PktAuthReq*>(payload);
        PktAuthAck ack{};
        uint8_t idx = 0;
        if(cb.on_auth && cb.on_auth(req->id, req->pw, req->tier, idx)){
            ack.ok        = 1;
            ack.op_index  = idx;
            c->op_index   = idx;
            c->tier       = req->tier;
            strncpy(c->name, req->id, 31);
            c->authed     = true;
            strncpy(ack.reason, "OK", sizeof(ack.reason));
            printf("[NetServer] op %d '%s' (Tier%d) connected\n",
                   idx, c->name, c->tier);
        } else {
            ack.ok = 0;
            strncpy(ack.reason, "Auth failed", sizeof(ack.reason));
        }
        send_to(*c, PacketType::AUTH_ACK, &ack, sizeof(ack));
        if(ack.ok){
            broadcast_operator_list();
            update_discovery_user_count();
        }
        break;
    }

    case PacketType::CMD: {
        if(!c->authed || len < sizeof(PktCmd)) break;
        auto* cmd = reinterpret_cast<const PktCmd*>(payload);
        switch(static_cast<CmdType>(cmd->cmd)){
            case CmdType::SET_FREQ:
                if(cb.on_set_freq) cb.on_set_freq(cmd->set_freq.cf_mhz);
                break;
            case CmdType::SET_GAIN:
                if(cb.on_set_gain) cb.on_set_gain(cmd->set_gain.db);
                break;
            case CmdType::CREATE_CH:
                if(cb.on_create_ch)
                    cb.on_create_ch(cmd->create_ch.idx, cmd->create_ch.s, cmd->create_ch.e, c->name);
                break;
            case CmdType::DELETE_CH:
                if(cb.on_delete_ch) cb.on_delete_ch(cmd->delete_ch.idx);
                break;
            case CmdType::SET_CH_MODE:
                if(cb.on_set_ch_mode)
                    cb.on_set_ch_mode(cmd->set_ch_mode.idx, cmd->set_ch_mode.mode);
                break;
            case CmdType::SET_CH_AUDIO:
                if(cb.on_set_ch_audio)
                    cb.on_set_ch_audio(cmd->set_ch_audio.idx, cmd->set_ch_audio.mask);
                break;
            case CmdType::START_REC:
                if(cb.on_start_rec) cb.on_start_rec(cmd->start_rec.ch_idx);
                break;
            case CmdType::STOP_REC:
                if(cb.on_stop_rec) cb.on_stop_rec();
                break;
            case CmdType::SET_CH_PAN:
                if(cb.on_set_ch_pan)
                    cb.on_set_ch_pan(cmd->set_ch_pan.idx, cmd->set_ch_pan.pan);
                break;
            case CmdType::SET_SQ_THRESH:
                if(cb.on_set_sq_thresh)
                    cb.on_set_sq_thresh(cmd->set_sq_thresh.idx, cmd->set_sq_thresh.thr);
                break;
            case CmdType::SET_AUTOSCALE:
                if(cb.on_set_autoscale) cb.on_set_autoscale();
                break;
            case CmdType::TOGGLE_RECV:
                if(cb.on_toggle_recv)
                    cb.on_toggle_recv(cmd->toggle_recv.idx, c->op_index,
                                      cmd->toggle_recv.enable != 0);
                break;
            case CmdType::UPDATE_CH_RANGE:
                if(cb.on_update_ch_range)
                    cb.on_update_ch_range(cmd->update_ch_range.idx,
                                          cmd->update_ch_range.s,
                                          cmd->update_ch_range.e);
                break;
            case CmdType::TOGGLE_TM_IQ:
                if(cb.on_toggle_tm_iq) cb.on_toggle_tm_iq();
                break;
            case CmdType::SET_CAPTURE_PAUSE:
                if(cb.on_set_capture_pause)
                    cb.on_set_capture_pause(cmd->set_capture_pause.pause != 0);
                break;
            case CmdType::SET_SPECTRUM_PAUSE:
                if(cb.on_set_spectrum_pause)
                    cb.on_set_spectrum_pause(cmd->set_spectrum_pause.pause != 0);
                break;
            case CmdType::REQUEST_REGION:
                if(cb.on_request_region)
                    cb.on_request_region(c->op_index, c->name,
                        cmd->request_region.fft_top, cmd->request_region.fft_bot,
                        cmd->request_region.freq_lo, cmd->request_region.freq_hi,
                        cmd->request_region.time_start, cmd->request_region.time_end);
                break;
            case CmdType::CHASSIS_RESET:
                if(cb.on_chassis_reset) cb.on_chassis_reset();
                break;
            case CmdType::NET_RESET:
                if(cb.on_net_reset) cb.on_net_reset();
                break;
            case CmdType::RX_STOP:
                if(cb.on_rx_stop) cb.on_rx_stop();
                break;
            case CmdType::RX_START:
                if(cb.on_rx_start) cb.on_rx_start();
                break;
            case CmdType::SET_FFT_SIZE:
                if(cb.on_set_fft_size) cb.on_set_fft_size(cmd->set_fft_size.size);
                break;
            case CmdType::SET_SR:
                if(cb.on_set_sr) cb.on_set_sr(cmd->set_sr.msps);
                break;
            default: break;
        }
        // ACK
        PktCmdAck ack{}; ack.ok=1; ack.cmd=cmd->cmd;
        strncpy(ack.msg,"OK",sizeof(ack.msg));
        send_to(*c, PacketType::CMD_ACK, &ack, sizeof(ack));
        break;
    }

    case PacketType::CHAT: {
        if(!c->authed || len < sizeof(PktChat)) break;
        auto* chat = reinterpret_cast<const PktChat*>(payload);
        // override 'from' with actual connected name
        PktChat out{}; strncpy(out.from, c->name, 31);
        strncpy(out.msg, chat->msg, sizeof(out.msg)-1);
        if(cb.on_chat) cb.on_chat(out.from, out.msg);
        // broadcast to all (including server UI)
        broadcast_chat(out.from, out.msg);
        break;
    }

    case PacketType::DISCONNECT:
        c->alive.store(false);
        break;

    case PacketType::SHARE_DOWNLOAD_REQ: {
        if(!c->authed || len < sizeof(PktShareDownloadReq)) break;
        auto* req = reinterpret_cast<const PktShareDownloadReq*>(payload);
        char fname[129]; strncpy(fname, req->filename, 128); fname[128]='\0';
        if(cb.on_share_download_req) cb.on_share_download_req(c->op_index, fname);
        break;
    }

    case PacketType::SHARE_UPLOAD_META: {
        if(!c->authed || len < sizeof(PktShareUploadMeta)) break;
        auto* m = reinterpret_cast<const PktShareUploadMeta*>(payload);
        // 기존 업로드 중인 파일 닫기
        if(c->upload.fp){ fclose(c->upload.fp); c->upload.fp=nullptr; }
        strncpy(c->upload.filename, m->filename, 127); c->upload.filename[127]='\0';
        c->upload.total_bytes  = m->total_bytes;
        c->upload.recv_bytes   = 0;
        c->upload.transfer_id  = m->transfer_id;
        c->upload.active       = true;
        // 임시 저장 경로: /tmp/bewe_up_<name>
        snprintf(c->upload.save_path, sizeof(c->upload.save_path),
                 "/tmp/bewe_up_%s", c->upload.filename);
        c->upload.fp = fopen(c->upload.save_path, "wb");
        if(!c->upload.fp){
            fprintf(stderr,"SHARE_UPLOAD_META: fopen failed %s\n", c->upload.save_path);
            c->upload.active=false;
        }
        break;
    }

    case PacketType::SHARE_UPLOAD_DATA: {
        if(!c->authed || !c->upload.active || !c->upload.fp) break;
        if(len < sizeof(PktShareUploadData)) break;
        auto* d = reinterpret_cast<const PktShareUploadData*>(payload);
        if(d->transfer_id != c->upload.transfer_id) break;
        uint32_t data_bytes = d->chunk_bytes;
        if((uint32_t)len < (uint32_t)sizeof(PktShareUploadData) + data_bytes) break;
        const uint8_t* data_ptr = payload + sizeof(PktShareUploadData);
        fwrite(data_ptr, 1, data_bytes, c->upload.fp);
        c->upload.recv_bytes += data_bytes;
        if(d->is_last){
            fclose(c->upload.fp); c->upload.fp=nullptr;
            c->upload.active=false;
            if(cb.on_share_upload_done)
                cb.on_share_upload_done(c->op_index, c->name, c->upload.save_path);
        }
        break;
    }

    case PacketType::PUB_DELETE_REQ: {
        if(!c->authed || len < sizeof(PktPubDeleteReq)) break;
        auto* req = reinterpret_cast<const PktPubDeleteReq*>(payload);
        char fname[129]; strncpy(fname, req->filename, 128); fname[128]='\0';
        if(cb.on_pub_delete_req) cb.on_pub_delete_req(c->name, fname);
        break;
    }

    default: break;
    }
}

// ── drop_client ───────────────────────────────────────────────────────────
void NetServer::drop_client(std::shared_ptr<ClientConn> c){
    if(c->is_relay) relay_client_count_.fetch_sub(1);
    bool was_authed = c->authed;
    uint8_t idx = c->op_index;
    char name[32]; strncpy(name, c->name, 31);

    c->alive.store(false);
    c->stop_send_worker();
    if(c->fd >= 0){ shutdown(c->fd, SHUT_RDWR); close(c->fd); c->fd=-1; }

    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        clients_.erase(std::remove_if(clients_.begin(), clients_.end(),
            [&](const std::shared_ptr<ClientConn>& x){ return x.get()==c.get(); }),
            clients_.end());
    }

    if(was_authed){
        printf("[NetServer] op %d '%s' disconnected\n", idx, name);
        broadcast_operator_list();
        update_discovery_user_count();
    }
}

// ── send_to ───────────────────────────────────────────────────────────────
void NetServer::send_to(ClientConn& c, PacketType type,
                         const void* payload, uint32_t len){
    if(!c.alive.load() || c.fd < 0) return;
    c.enqueue(make_packet(type, payload, len), false);
}

// ── Broadcast FFT ─────────────────────────────────────────────────────────
void NetServer::broadcast_fft(const float* data, int fft_size,
                               int64_t wall_time,
                               uint64_t center_hz, uint32_t sr,
                               float pmin, float pmax){
    if(bcast_pause_.load(std::memory_order_relaxed)) return;
    PktFftFrame hdr{};
    hdr.center_freq_hz = center_hz;
    hdr.sample_rate    = sr;
    hdr.fft_size       = (uint32_t)fft_size;
    hdr.power_min      = pmin;
    hdr.power_max      = pmax;
    hdr.wall_time      = wall_time;

    uint32_t data_bytes = (uint32_t)(fft_size * sizeof(float));
    uint32_t total = (uint32_t)(sizeof(PktFftFrame) + data_bytes);
    std::vector<uint8_t> payload(total);
    memcpy(payload.data(), &hdr, sizeof(PktFftFrame));
    memcpy(payload.data() + sizeof(PktFftFrame), data, data_bytes);

    auto pkt = make_packet(PacketType::FFT_FRAME, payload.data(), total);
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size());
    else {
        static int fft_no_relay_warn = 0;
        if(fft_no_relay_warn++ % 600 == 0)
            printf("[NetServer] broadcast_fft: on_relay_broadcast is NULL (fft #%d)\n", fft_no_relay_warn);
    }
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(c->is_relay || !c->authed || !c->alive.load()) continue;
        c->enqueue(pkt, true);
    }
}

// ── Send audio to specific operators (legacy, mask-based) ────────────────
void NetServer::send_audio(uint32_t op_mask, uint8_t ch_idx, int8_t pan,
                            const float* pcm, uint32_t n_samples){
    if(!op_mask || !n_samples) return;
    if(bcast_pause_.load(std::memory_order_relaxed)) return;

    uint32_t payload_size = (uint32_t)(sizeof(PktAudioFrame) + n_samples*sizeof(float));
    std::vector<uint8_t> payload(payload_size);
    auto* ah = reinterpret_cast<PktAudioFrame*>(payload.data());
    ah->ch_idx    = ch_idx;
    ah->pan       = (uint8_t)(int8_t)pan;
    ah->n_samples = n_samples;
    memcpy(payload.data() + sizeof(PktAudioFrame), pcm, n_samples*sizeof(float));

    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(c->is_relay || !c->authed || !c->alive.load()) continue;
        if(!(op_mask & (1u << c->op_index))) continue;
        c->enqueue(make_packet(PacketType::AUDIO_FRAME, payload.data(), payload_size), false, true);
    }
}

void NetServer::broadcast_audio_all(uint8_t ch_idx, int8_t pan,
                                     const float* pcm, uint32_t n_samples){
    if(!n_samples) return;
    if(bcast_pause_.load(std::memory_order_relaxed)) return;

    uint32_t payload_size = (uint32_t)(sizeof(PktAudioFrame) + n_samples*sizeof(float));
    std::vector<uint8_t> payload(payload_size);
    auto* ah = reinterpret_cast<PktAudioFrame*>(payload.data());
    ah->ch_idx    = ch_idx;
    ah->pan       = (uint8_t)(int8_t)pan;
    ah->n_samples = n_samples;
    memcpy(payload.data() + sizeof(PktAudioFrame), pcm, n_samples*sizeof(float));

    auto pkt = make_packet(PacketType::AUDIO_FRAME, payload.data(), payload_size);
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size());
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(c->is_relay || !c->authed || !c->alive.load()) continue;
        c->enqueue(pkt, false, true);
    }
}


// ── Broadcast channel sync ────────────────────────────────────────────────
void NetServer::broadcast_channel_sync(const Channel* chs, int n){
    PktChannelSync sync{};
    for(int i=0; i<n && i<MAX_CHANNELS; i++){
        sync.ch[i].idx        = (uint8_t)i;
        sync.ch[i].active     = chs[i].filter_active ? 1 : 0;
        sync.ch[i].s          = chs[i].s;
        sync.ch[i].e          = chs[i].e;
        sync.ch[i].mode       = (uint8_t)chs[i].mode;
        sync.ch[i].pan        = (int8_t)chs[i].pan;
        sync.ch[i].audio_mask    = chs[i].audio_mask.load();
        sync.ch[i].sq_threshold  = chs[i].sq_threshold.load(std::memory_order_relaxed);
        sync.ch[i].sq_sig        = chs[i].sq_sig.load(std::memory_order_relaxed);
        sync.ch[i].sq_gate       = chs[i].sq_gate.load(std::memory_order_relaxed) ? 1 : 0;
        strncpy(sync.ch[i].owner_name, chs[i].owner, 31);
    }
    auto pkt = make_packet(PacketType::CHANNEL_SYNC, &sync, sizeof(sync));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size());
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(c->is_relay || !c->authed || !c->alive.load()) continue;
        c->enqueue(pkt, false);
    }
}

// ── Broadcast chat ────────────────────────────────────────────────────────
void NetServer::broadcast_chat(const char* from, const char* msg){
    PktChat chat{};
    strncpy(chat.from, from, 31);
    strncpy(chat.msg,  msg,  sizeof(chat.msg)-1);
    auto pkt = make_packet(PacketType::CHAT, &chat, sizeof(chat));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size());
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(c->is_relay || !c->authed || !c->alive.load()) continue;
        c->enqueue(pkt, false);
    }
}

// ── Broadcast heartbeat ───────────────────────────────────────────────────
void NetServer::broadcast_heartbeat(uint8_t host_state, uint8_t sdr_temp_c, uint8_t sdr_state, uint8_t iq_on){
    PktHeartbeat hb{}; hb.host_state = host_state; hb.sdr_temp_c = sdr_temp_c; hb.sdr_state = sdr_state; hb.iq_on = iq_on;
    auto pkt = make_packet(PacketType::HEARTBEAT, &hb, sizeof(hb));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size());
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(c->is_relay || !c->authed || !c->alive.load()) continue;
        c->enqueue(pkt, false);
    }
}

// ── Broadcast status ──────────────────────────────────────────────────────
void NetServer::broadcast_status(float cf_mhz, float gain_db,
                                  uint32_t sr, uint8_t hw_type){
    PktStatus s{}; s.cf_mhz=cf_mhz; s.gain_db=gain_db;
    s.sample_rate=sr; s.hw_type=hw_type;
    auto pkt = make_packet(PacketType::STATUS, &s, sizeof(s));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size());
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(c->is_relay || !c->authed || !c->alive.load()) continue;
        c->enqueue(pkt, false);
    }
}

// ── Broadcast operator list ───────────────────────────────────────────────
void NetServer::broadcast_operator_list(){
    PktOperatorList ol{};
    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        int cnt = 0;
        // index=0: HOST 본인
        if(cnt < MAX_OPERATORS){
            ol.ops[cnt].index = 0;
            ol.ops[cnt].tier  = host_tier_;
            strncpy(ol.ops[cnt].name, host_name_, 31);
            ++cnt;
        }
        for(auto& c : clients_){
            if(c->is_relay || !c->authed || !c->alive.load()) continue;
            if(cnt >= MAX_OPERATORS) break;
            ol.ops[cnt].index = c->op_index;
            ol.ops[cnt].tier  = c->tier;
            strncpy(ol.ops[cnt].name, c->name, 31);
            ++cnt;
        }
        ol.count = (uint8_t)cnt;
        // relay 클라이언트는 제외 (중앙서버가 op_list 관리)
        for(auto& c : clients_){
            if(c->is_relay || !c->authed || !c->alive.load()) continue;
            send_to(*c, PacketType::OPERATOR_LIST, &ol, sizeof(ol));
        }
    }
}

// ── Get operators (for UI) ────────────────────────────────────────────────
void NetServer::broadcast_wf_event(int32_t fft_offset, int64_t wall_time,
                                    uint8_t type, const char* label){
    PktWfEvent ev{};
    ev.fft_idx_offset = fft_offset;
    ev.wall_time      = wall_time;
    ev.type           = type;
    strncpy(ev.label, label, 31);
    auto pkt = make_packet(PacketType::WF_EVENT, &ev, sizeof(ev));
    if(cb.on_relay_broadcast)
        cb.on_relay_broadcast(pkt.data(), pkt.size());
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& cli : clients_){
        if(cli->is_relay || !cli->authed || !cli->alive.load()) continue;
        cli->enqueue(pkt, false);
    }
}

void NetServer::send_region_response(int op_index, bool allowed){
    PktRegionResponse resp{}; resp.allowed = allowed ? 1 : 0;
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& cli : clients_){
        if(cli->authed && cli->alive.load() && cli->op_index==(uint8_t)op_index){
            cli->enqueue(make_packet(PacketType::REGION_RESPONSE, &resp, sizeof(resp)), false);
            break;
        }
    }
}

void NetServer::send_file_to(int op_index, const char* path, uint8_t transfer_id,
                              std::function<void(uint64_t,uint64_t)> progress_cb){
    FILE* fp = fopen(path, "rb");
    if(!fp){ fprintf(stderr,"send_file_to: open failed %s\n",path); return; }
    fseek(fp,0,SEEK_END); uint64_t total=(uint64_t)ftell(fp); fseek(fp,0,SEEK_SET);

    // find target client
    std::shared_ptr<ClientConn> target;
    {
        std::lock_guard<std::mutex> lk(clients_mtx_);
        for(auto& cli : clients_)
            if(cli->authed && cli->alive.load() && cli->op_index==(uint8_t)op_index)
                { target=cli; break; }
    }
    if(!target){ fclose(fp); return; }

    // send meta
    PktFileMeta meta{};
    const char* fn = strrchr(path,'/'); fn = fn ? fn+1 : path;
    strncpy(meta.filename, fn, 127);
    meta.total_bytes  = total;
    meta.transfer_id  = transfer_id;
    {
        auto pkt = make_packet(PacketType::FILE_META, &meta, sizeof(meta));
        std::lock_guard<std::mutex> slk(target->fd_write_mtx);
        send_all(target->fd, pkt.data(), pkt.size());
    }

    // send chunks: 64KB 청크로 TCP 효율 극대화
    // send_file_to는 detach 스레드에서 실행되므로 캡처·오디오 스레드 차단 없음
    // 속도: 측정 속도의 80% 사용 → FFT 스트림 보호하되 전송 속도 확보
    const uint32_t CHUNK = 256 * 1024;  // 256KB 청크
    std::vector<uint8_t> buf(sizeof(PktFileData)+CHUNK);
    uint64_t offset=0;
    // EWMA로 측정한 실제 TCP send 속도 (bytes/sec)
    double measured_bps = (double)FILE_RATE_INIT;
    auto rate_epoch = std::chrono::steady_clock::now();
    uint64_t rate_sent = 0;
    while(true){
        size_t n = fread(buf.data()+sizeof(PktFileData), 1, CHUNK, fp);
        if(n==0) break;
        PktFileData* d = reinterpret_cast<PktFileData*>(buf.data());
        d->transfer_id  = transfer_id;
        d->is_last      = feof(fp) ? 1 : 0;
        d->chunk_bytes  = (uint32_t)n;
        d->offset       = offset;
        offset += n;
        uint32_t total_payload = (uint32_t)(sizeof(PktFileData)+n);
        auto pkt = make_packet(PacketType::FILE_DATA, buf.data(), total_payload);
        // send 에 걸리는 시간 측정 → 실 TCP throughput
        auto t0 = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> slk(target->fd_write_mtx);
            send_all(target->fd, pkt.data(), pkt.size());
        }
        double send_us = (double)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if(send_us > 0.0){
            double chunk_bps = (double)pkt.size() / (send_us * 1e-6);
            measured_bps = measured_bps * (1.0 - FILE_RATE_EWMA_ALPHA)
                         + chunk_bps    *        FILE_RATE_EWMA_ALPHA;
        }
        if(progress_cb) progress_cb(offset, total);
        // 목표: 측정 속도의 90% 사용 → 나머지 10%를 FFT 스트림에 양보
        uint64_t target_bps = (uint64_t)(measured_bps * 0.90);
        if(target_bps < FILE_RATE_FLOOR) target_bps = FILE_RATE_FLOOR;
        // 누적 기준으로 sleep (drift 방지)
        rate_sent += n;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - rate_epoch).count();
        int64_t want_us = (int64_t)(rate_sent * 1000000ULL / target_bps);
        if(want_us > elapsed_us)
            std::this_thread::sleep_for(std::chrono::microseconds(want_us - elapsed_us));
    }
    fclose(fp);
}

void NetServer::send_share_list(int op_index,
                                 const std::vector<std::tuple<std::string,uint64_t,std::string>>& files){
    uint16_t cnt = (uint16_t)std::min((size_t)files.size(), (size_t)UINT16_MAX);
    size_t payload_size = sizeof(PktShareList) + cnt * sizeof(ShareFileEntry);
    std::vector<uint8_t> payload(payload_size, 0);
    auto* hdr = reinterpret_cast<PktShareList*>(payload.data());
    hdr->count = cnt;
    ShareFileEntry* entries = reinterpret_cast<ShareFileEntry*>(payload.data() + sizeof(PktShareList));
    for(uint16_t i = 0; i < cnt; i++){
        strncpy(entries[i].filename, std::get<0>(files[i]).c_str(), 127);
        entries[i].size_bytes = std::get<1>(files[i]);
        strncpy(entries[i].uploader, std::get<2>(files[i]).c_str(), 31);
    }
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load()) continue;
        if(op_index >= 0 && c->op_index != (uint8_t)op_index) continue;
        auto pkt = make_packet(PacketType::SHARE_LIST, payload.data(), (uint32_t)payload_size);
        std::lock_guard<std::mutex> slk(c->send_mtx);
        send_all(c->fd, pkt.data(), pkt.size());
    }
}

std::vector<OpEntry> NetServer::get_operators() const {
    std::vector<OpEntry> ops;
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load()) continue;
        OpEntry e{}; e.index=c->op_index; e.tier=c->tier;
        strncpy(e.name, c->name, 31);
        ops.push_back(e);
    }
    return ops;
}

// ── UDP Discovery Broadcast ───────────────────────────────────────────────
void NetServer::start_discovery_broadcast(const char* station_name,
                                           float lat, float lon,
                                           uint16_t tcp_port,
                                           const char* host_ip,
                                           uint8_t host_tier) {
    stop_discovery_broadcast();
    auto* b = new DiscoveryBroadcaster();
    b->set_info(station_name, lat, lon, tcp_port, host_ip, host_tier);
    b->set_user_count((uint8_t)client_count());
    if (b->start())
        discovery_bcast_ = b;
    else
        delete b;
}

void NetServer::stop_discovery_broadcast() {
    if (discovery_bcast_) {
        discovery_bcast_->stop();
        delete discovery_bcast_;
        discovery_bcast_ = nullptr;
    }
}

void NetServer::update_discovery_user_count() {
    if (discovery_bcast_)
        discovery_bcast_->set_user_count((uint8_t)client_count());
}