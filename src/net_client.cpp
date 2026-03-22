#include "net_client.hpp"
#include "udp_discovery.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>

// ── connect ───────────────────────────────────────────────────────────────
bool NetClient::connect(const char* host, int port,
                         const char* id, const char* pw, uint8_t tier){
    // Resolve host
    addrinfo hints{}, *res = nullptr;
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    char port_str[16]; snprintf(port_str, sizeof(port_str), "%d", port);
    if(getaddrinfo(host, port_str, &hints, &res) != 0){
        printf("[NetClient] getaddrinfo failed for %s\n", host);
        return false;
    }

    fd_ = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(fd_ < 0){ freeaddrinfo(res); return false; }

    // non-blocking connect → poll 3초 타임아웃
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    int cr = ::connect(fd_, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if(cr < 0 && errno != EINPROGRESS){
        close(fd_); fd_=-1; return false;
    }
    if(cr < 0){
        pollfd pfd{fd_, POLLOUT, 0};
        int pr = poll(&pfd, 1, 3000);
        if(pr <= 0){ close(fd_); fd_=-1; return false; }
        int so_err = 0; socklen_t sl = sizeof(so_err);
        getsockopt(fd_, SOL_SOCKET, SO_ERROR, &so_err, &sl);
        if(so_err != 0){ close(fd_); fd_=-1; return false; }
    }
    fcntl(fd_, F_SETFL, flags);  // blocking 복원

    // TCP_NODELAY: Nagle 비활성화 → 실시간 스트림 지연 방지
    int nd=1; setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &nd, sizeof(nd));

    // SO_RCVTIMEO: recv() 3초 타임아웃 → Host 무응답 시 UI 멈춤 방지
    timeval tv{3, 0};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // SO_SNDTIMEO: send() 2초 타임아웃 → 네트워크 장애 시 채팅/명령 전송 블로킹 방지
    timeval stv{2, 0};
    setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

    // Send AUTH_REQ
    PktAuthReq req{};
    strncpy(req.id, id, 31);
    strncpy(req.pw, pw, 63);
    req.tier = tier;
    if(!raw_send(PacketType::AUTH_REQ, &req, sizeof(req))){
        close(fd_); fd_=-1; return false;
    }

    // Wait for AUTH_ACK
    PktHdr hdr{};
    if(!recv_all(fd_, &hdr, PKT_HDR_SIZE) ||
       static_cast<PacketType>(hdr.type) != PacketType::AUTH_ACK){
        printf("[NetClient] no AUTH_ACK\n");
        close(fd_); fd_=-1; return false;
    }
    std::vector<uint8_t> payload(hdr.len);
    if(hdr.len > 0 && !recv_all(fd_, payload.data(), hdr.len)){
        close(fd_); fd_=-1; return false;
    }
    if(hdr.len < sizeof(PktAuthAck)){
        close(fd_); fd_=-1; return false;
    }
    auto* ack = reinterpret_cast<PktAuthAck*>(payload.data());
    if(!ack->ok){
        printf("[NetClient] auth failed: %s\n", ack->reason);
        close(fd_); fd_=-1; return false;
    }

    my_op_index = ack->op_index;
    my_tier     = tier;
    strncpy(my_name, id, 31);
    connected_.store(true);

    printf("[NetClient] connected as op %d '%s' (Tier%d)\n",
           my_op_index, my_name, my_tier);

    if(recv_thr_.joinable()) recv_thr_.join();
    recv_thr_ = std::thread(&NetClient::recv_loop, this);
    return true;
}

// ── relay 모드: 이미 연결된 fd로 AUTH만 수행 ─────────────────────────────
bool NetClient::connect_fd(int fd, const char* id, const char* pw, uint8_t tier){
    fd_ = fd;

    // SO_RCVTIMEO: recv() 3초 타임아웃 (relay 모드도 동일)
    timeval tv{3, 0};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // SO_SNDTIMEO: send() 2초 타임아웃 → 채팅/명령 전송 블로킹 방지
    timeval stv{2, 0};
    setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

    PktAuthReq req{};
    strncpy(req.id, id, 31);
    strncpy(req.pw, pw, 63);
    req.tier = tier;
    if(!raw_send(PacketType::AUTH_REQ, &req, sizeof(req))){
        fd_=-1; return false;
    }

    printf("[NetClient] connect_fd: sent AUTH_REQ id='%s' tier=%u fd=%d\n", id, tier, fd);

    PktHdr hdr{};
    printf("[NetClient] connect_fd: waiting for AUTH_ACK...\n");
    ssize_t hr = recv(fd_, &hdr, PKT_HDR_SIZE, MSG_WAITALL);
    if(hr != PKT_HDR_SIZE){
        printf("[NetClient] connect_fd: recv AUTH_ACK hdr failed hr=%zd errno=%d(%s)\n",
               hr, errno, strerror(errno));
        fd_=-1; return false;
    }
    printf("[NetClient] connect_fd: got hdr magic=%02x%02x%02x%02x type=0x%02x len=%u\n",
           hdr.magic[0], hdr.magic[1], hdr.magic[2], hdr.magic[3], hdr.type, hdr.len);
    if(static_cast<PacketType>(hdr.type) != PacketType::AUTH_ACK){
        printf("[NetClient] connect_fd: expected AUTH_ACK got type=0x%02x\n", hdr.type);
        fd_=-1; return false;
    }
    std::vector<uint8_t> payload(hdr.len);
    if(hdr.len > 0){
        ssize_t pr = recv(fd_, payload.data(), hdr.len, MSG_WAITALL);
        if(pr != (ssize_t)hdr.len){
            printf("[NetClient] connect_fd: recv AUTH_ACK payload failed pr=%zd/%u errno=%d(%s)\n",
                   pr, hdr.len, errno, strerror(errno));
            fd_=-1; return false;
        }
    }
    if(hdr.len < sizeof(PktAuthAck)){
        printf("[NetClient] connect_fd: AUTH_ACK payload too short %u\n", hdr.len);
        fd_=-1; return false;
    }
    auto* ack = reinterpret_cast<PktAuthAck*>(payload.data());
    if(!ack->ok){
        printf("[NetClient] relay auth failed: %s\n", ack->reason);
        fd_=-1; return false;
    }

    my_op_index = ack->op_index;
    my_tier     = tier;
    strncpy(my_name, id, 31);
    connected_.store(true);

    printf("[NetClient] relay connected as op %d '%s' (Tier%d) fd=%d\n",
           my_op_index, my_name, my_tier, fd_);

    if(recv_thr_.joinable()) recv_thr_.join();
    recv_thr_ = std::thread(&NetClient::recv_loop, this);
    printf("[NetClient] connect_fd: recv_thr started\n");
    return true;
}

// ── FFT buffer pop (1s delay) ─────────────────────────────────────────────
bool NetClient::pop_fft_frame(FftFrame& out){
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lk(fft_queue_mtx_);
    if(fft_queue_.empty()) return false;
    // 앞에서부터 충분히 오래된 프레임만 꺼냄
    if(now_us - fft_queue_.front().recv_us < DISPLAY_DELAY_US) return false;
    out = std::move(fft_queue_.front());
    fft_queue_.erase(fft_queue_.begin());
    return true;
}

// ── disconnect ────────────────────────────────────────────────────────────
void NetClient::disconnect(){
    stop_discovery_listen();
    connected_.store(false);
    if(fd_ >= 0){
        send_packet(fd_, PacketType::DISCONNECT, nullptr, 0);
        shutdown(fd_, SHUT_RDWR);
        close(fd_); fd_=-1;
    }
    if(recv_thr_.joinable()) recv_thr_.join();
}

// ── UDP Discovery Listener ────────────────────────────────────────────────
bool NetClient::start_discovery_listen(
        std::function<void(const DiscoveryAnnounce&)> callback) {
    stop_discovery_listen();
    auto* l = new DiscoveryListener();
    l->on_station_found = std::move(callback);
    if (!l->start()) {
        delete l;
        return false;
    }
    discovery_listener_ = l;
    return true;
}

void NetClient::stop_discovery_listen() {
    if (discovery_listener_) {
        discovery_listener_->stop();
        delete discovery_listener_;
        discovery_listener_ = nullptr;
    }
}

// ── recv loop ─────────────────────────────────────────────────────────────
void NetClient::recv_loop(){
    uint64_t pkt_count = 0;
    const char* disc_reason = "unknown";
    printf("[NetClient] recv_loop started fd=%d\n", fd_);
    while(connected_.load()){
        PktHdr hdr{};
        int rc = recv_all_ex(fd_, &hdr, PKT_HDR_SIZE, connected_);
        if(rc == 0) continue;   // timeout → Host 무응답, 소켓 유지하고 재시도
        if(rc < 0){
            int e = errno;
            if(connected_.load()){
                printf("[NetClient] recv hdr failed: errno=%d(%s) pkts=%llu\n",
                       e, strerror(e), (unsigned long long)pkt_count);
                disc_reason = "recv_hdr_fail";
            } else {
                disc_reason = "disconnect_called";
            }
            break;
        }
        if(memcmp(hdr.magic, BEWE_MAGIC, 4) != 0){
            printf("[NetClient] bad magic: %02x%02x%02x%02x type=0x%02x len=%u pkts=%llu\n",
                   hdr.magic[0], hdr.magic[1], hdr.magic[2], hdr.magic[3],
                   hdr.type, hdr.len, (unsigned long long)pkt_count);
            disc_reason = "bad_magic";
            break;
        }
        uint32_t len = hdr.len;
        if(len > 4*1024*1024){
            printf("[NetClient] oversized pkt: type=0x%02x len=%u\n", hdr.type, len);
            disc_reason = "oversized";
            break;
        }
        printf("[NetClient] recv pkt#%llu type=0x%02x len=%u\n",
               (unsigned long long)pkt_count, hdr.type, len);
        std::vector<uint8_t> payload(len);
        if(len > 0){
            int rc2 = recv_all_ex(fd_, payload.data(), len, connected_);
            if(rc2 == 0) continue;  // timeout mid-packet → retry
            if(rc2 < 0){
                int e = errno;
                printf("[NetClient] recv payload failed: type=0x%02x len=%u errno=%d(%s)\n",
                       hdr.type, len, e, strerror(e));
                disc_reason = "recv_payload_fail";
                break;
            }
        }
        pkt_count++;
        handle_packet(static_cast<PacketType>(hdr.type), payload.data(), len);
    }
    connected_.store(false);
    printf("[NetClient] disconnected: reason=%s total_pkts=%llu fd=%d\n",
           disc_reason, (unsigned long long)pkt_count, fd_);
}

// ── handle_packet ─────────────────────────────────────────────────────────
void NetClient::handle_packet(PacketType type,
                               const uint8_t* payload, uint32_t len){
    switch(type){

    case PacketType::FFT_FRAME: {
        if(len < sizeof(PktFftFrame)) break;
        // 수직바가 왼쪽 끝으로 밀려있으면 FFT 패킷 드롭 (큐에 쌓지 않음)
        if(!fft_recv_enabled.load(std::memory_order_relaxed)) break;
        auto* fh = reinterpret_cast<const PktFftFrame*>(payload);
        uint32_t data_bytes = len - (uint32_t)sizeof(PktFftFrame);
        if(data_bytes != fh->fft_size * sizeof(float)) break;

        // 수신 시각 (steady_clock μs)
        auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        const float* fft_floats = reinterpret_cast<const float*>(
                                      payload + sizeof(PktFftFrame));
        FftFrame frm;
        frm.data.assign(fft_floats, fft_floats + fh->fft_size);
        frm.cf_hz     = fh->center_freq_hz;
        frm.sr        = fh->sample_rate;
        frm.fft_sz    = fh->fft_size;
        frm.pmin      = fh->power_min;
        frm.pmax      = fh->power_max;
        frm.wall_time = fh->wall_time;
        frm.recv_us   = now_us;

        {
            std::lock_guard<std::mutex> qlk(fft_queue_mtx_);
            // 큐가 너무 크면 오래된 것부터 drop (백로그 방지)
            if(fft_queue_.size() >= FFT_QUEUE_MAX)
                fft_queue_.erase(fft_queue_.begin());
            fft_queue_.push_back(std::move(frm));
        }
        // 단일 프레임도 업데이트 (legacy 호환)
        {
            std::lock_guard<std::mutex> lk(fft_mtx);
            cf_hz        = fh->center_freq_hz;
            sr           = fh->sample_rate;
            fft_sz       = fh->fft_size;
            pmin         = fh->power_min;
            pmax         = fh->power_max;
            fft_wall_time= fh->wall_time;
        }
        fft_seq.fetch_add(1, std::memory_order_release);
        break;
    }

    case PacketType::AUDIO_FRAME: {
        if(len < sizeof(PktAudioFrame)) break;
        auto* ah = reinterpret_cast<const PktAudioFrame*>(payload);
        if(ah->ch_idx >= MAX_CHANNELS) break;
        uint32_t n = ah->n_samples;
        if(len < sizeof(PktAudioFrame) + n*sizeof(float)) break;
        const float* pcm = reinterpret_cast<const float*>(
                               payload + sizeof(PktAudioFrame));
        int8_t pan = (int8_t)ah->pan;
        auto& ring = audio[ah->ch_idx];
        for(uint32_t i=0; i<n; i++) ring.push(pcm[i], pan);
        break;
    }

    case PacketType::CHANNEL_SYNC: {
        if(len < sizeof(PktChannelSync)) break;
        auto* sync = reinterpret_cast<const PktChannelSync*>(payload);
        if(on_channel_sync) on_channel_sync(*sync);
        break;
    }

    case PacketType::WF_EVENT: {
        if(len < sizeof(PktWfEvent)) break;
        auto* ev = reinterpret_cast<const PktWfEvent*>(payload);
        if(on_wf_event) on_wf_event(*ev);
        break;
    }

    case PacketType::FILE_META: {
        if(len < sizeof(PktFileMeta)) break;
        auto* meta = reinterpret_cast<const PktFileMeta*>(payload);
        std::lock_guard<std::mutex> lk(file_recv_mtx);
        FileRecv fr{};
        fr.filename    = meta->filename;
        fr.total_bytes = meta->total_bytes;
        fr.transfer_id = meta->transfer_id;
        fr.done        = false;
        fr.recv_bytes  = 0;
        // determine save directory (callback override or ~/recordings)
        std::string dir;
        if(on_get_save_dir) dir = on_get_save_dir(fr.filename);
        if(dir.empty()){
            const char* home = getenv("HOME");
            dir = home ? std::string(home)+"/Downloads" : "/tmp";
        }
        fr.save_path = dir + "/" + fr.filename;
        fr.fp = fopen(fr.save_path.c_str(), "wb");
        std::string fn2 = fr.filename;
        uint64_t tb2 = fr.total_bytes;
        file_recv_list.push_back(std::move(fr));
        if(on_file_meta) on_file_meta(fn2, tb2);
        break;
    }

    case PacketType::FILE_DATA: {
        if(len < sizeof(PktFileData)) break;
        auto* d = reinterpret_cast<const PktFileData*>(payload);
        uint32_t data_bytes = d->chunk_bytes;
        if(len < sizeof(PktFileData)+data_bytes) break;
        const uint8_t* data_ptr = payload + sizeof(PktFileData);
        std::lock_guard<std::mutex> lk(file_recv_mtx);
        for(auto& fr : file_recv_list){
            if(fr.transfer_id == d->transfer_id && !fr.done && fr.fp){
                fwrite(data_ptr, 1, data_bytes, fr.fp);
                fr.recv_bytes += data_bytes;
                if(on_file_progress) on_file_progress(fr.filename, fr.recv_bytes, fr.total_bytes);
                if(d->is_last){
                    fclose(fr.fp); fr.fp=nullptr; fr.done=true;
                    if(on_file_done) on_file_done(fr.save_path, fr.filename);
                }
                break;
            }
        }
        break;
    }

    case PacketType::STATUS: {
        if(len < sizeof(PktStatus)) break;
        auto* s = reinterpret_cast<const PktStatus*>(payload);
        remote_cf_mhz.store(s->cf_mhz);
        remote_gain_db.store(s->gain_db);
        remote_sr.store(s->sample_rate);
        remote_hw.store(s->hw_type);
        break;
    }

    case PacketType::OPERATOR_LIST: {
        if(len < sizeof(PktOperatorList)) break;
        std::lock_guard<std::mutex> lk(op_mtx);
        op_list = *reinterpret_cast<const PktOperatorList*>(payload);
        op_list_updated.store(true);
        break;
    }

    case PacketType::CHAT: {
        if(len < sizeof(PktChat)) break;
        auto* c = reinterpret_cast<const PktChat*>(payload);
        std::lock_guard<std::mutex> lk(chat_mtx);
        if((int)chat_log.size() >= CHAT_LOG_MAX) chat_log.erase(chat_log.begin());
        ChatMsg m{}; strncpy(m.from, c->from, 31); strncpy(m.msg, c->msg, 255);
        chat_log.push_back(m);
        chat_updated.store(true);
        break;
    }

    case PacketType::DISCONNECT:
        connected_.store(false);
        break;

    case PacketType::HEARTBEAT: {
        if(len < sizeof(PktHeartbeat)) break;
        auto* hb = reinterpret_cast<const PktHeartbeat*>(payload);
        host_state.store((int)hb->host_state);
        remote_sdr_temp_c.store(hb->sdr_temp_c);
        remote_sdr_state.store(hb->sdr_state);
        remote_iq_on.store(hb->iq_on);
        // Use wall-clock seconds (monotonic substitute via steady_clock)
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        last_heartbeat_time.store(
            std::chrono::duration<double>(now).count());
        break;
    }

    case PacketType::REGION_RESPONSE: {
        if(len < sizeof(PktRegionResponse)) break;
        auto* r = reinterpret_cast<const PktRegionResponse*>(payload);
        if(on_region_response) on_region_response(r->allowed != 0);
        break;
    }

    case PacketType::SHARE_LIST: {
        if(len < sizeof(PktShareList)) break;
        auto* hdr = reinterpret_cast<const PktShareList*>(payload);
        uint16_t cnt = hdr->count;
        size_t expected = sizeof(PktShareList) + cnt * sizeof(ShareFileEntry);
        if(len < expected) break;
        const ShareFileEntry* entries = reinterpret_cast<const ShareFileEntry*>(
            payload + sizeof(PktShareList));
        std::vector<std::tuple<std::string,uint64_t,std::string>> files;
        files.reserve(cnt);
        for(uint16_t i = 0; i < cnt; i++){
            char fn[129]; strncpy(fn, entries[i].filename, 128); fn[128]='\0';
            char up[33]; strncpy(up, entries[i].uploader, 32); up[32]='\0';
            files.push_back({fn, entries[i].size_bytes, up});
        }
        if(on_share_list) on_share_list(files);
        break;
    }

    case PacketType::IQ_PROGRESS: {
        if(len < sizeof(PktIqProgress)) break;
        auto* p = reinterpret_cast<const PktIqProgress*>(payload);
        if(on_iq_progress) on_iq_progress(*p);
        break;
    }

    case PacketType::IQ_PIPE_READY: {
        if(len < sizeof(PktIqPipeReady)) break;
        auto* p = reinterpret_cast<const PktIqPipeReady*>(payload);
        if(on_iq_pipe_ready) on_iq_pipe_ready(p->req_id, p->filename, p->filesize);
        break;
    }

    default: break;
    }
}

// ── raw_send ──────────────────────────────────────────────────────────────
bool NetClient::raw_send(PacketType type, const void* payload, uint32_t len){
    std::lock_guard<std::mutex> lk(send_mtx_);
    return send_packet(fd_, type, payload, len);
}

// ── send_cmd ──────────────────────────────────────────────────────────────
bool NetClient::send_cmd(const PktCmd& cmd){
    return raw_send(PacketType::CMD, &cmd, sizeof(cmd));
}

bool NetClient::send_chat(const char* msg){
    PktChat c{};
    strncpy(c.from, my_name, 31);
    strncpy(c.msg,  msg, sizeof(c.msg)-1);
    return raw_send(PacketType::CHAT, &c, sizeof(c));
}

// ── Convenience helpers ───────────────────────────────────────────────────
bool NetClient::cmd_set_freq(float cf_mhz){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_FREQ; c.set_freq.cf_mhz=cf_mhz;
    return send_cmd(c);
}
bool NetClient::cmd_set_gain(float db){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_GAIN; c.set_gain.db=db;
    return send_cmd(c);
}
bool NetClient::cmd_create_ch(int idx, float s, float e){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::CREATE_CH;
    c.create_ch.idx=(uint8_t)idx; c.create_ch.s=s; c.create_ch.e=e;
    return send_cmd(c);
}
bool NetClient::cmd_delete_ch(int idx){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::DELETE_CH; c.delete_ch.idx=(uint8_t)idx;
    return send_cmd(c);
}
bool NetClient::cmd_set_ch_mode(int idx, int mode){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_CH_MODE;
    c.set_ch_mode.idx=(uint8_t)idx; c.set_ch_mode.mode=(uint8_t)mode;
    return send_cmd(c);
}
bool NetClient::cmd_set_ch_audio(int idx, uint32_t mask){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_CH_AUDIO;
    c.set_ch_audio.idx=(uint8_t)idx; c.set_ch_audio.mask=mask;
    return send_cmd(c);
}
bool NetClient::cmd_set_ch_pan(int idx, int pan){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_CH_PAN;
    c.set_ch_pan.idx=(uint8_t)idx; c.set_ch_pan.pan=(int8_t)pan;
    return send_cmd(c);
}
bool NetClient::cmd_set_sq_thresh(int idx, float thr){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_SQ_THRESH;
    c.set_sq_thresh.idx=(uint8_t)idx; c.set_sq_thresh.thr=thr;
    return send_cmd(c);
}
bool NetClient::cmd_set_autoscale(){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_AUTOSCALE;
    return send_cmd(c);
}
bool NetClient::cmd_toggle_tm_iq(){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::TOGGLE_TM_IQ;
    return send_cmd(c);
}
bool NetClient::cmd_set_capture_pause(bool pause){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_CAPTURE_PAUSE;
    c.set_capture_pause.pause=pause?1:0;
    return send_cmd(c);
}
bool NetClient::cmd_set_spectrum_pause(bool pause){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_SPECTRUM_PAUSE;
    c.set_spectrum_pause.pause=pause?1:0;
    return send_cmd(c);
}
bool NetClient::cmd_chassis_reset(){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::CHASSIS_RESET;
    return send_cmd(c);
}
bool NetClient::cmd_net_reset(){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::NET_RESET;
    return send_cmd(c);
}
bool NetClient::cmd_rx_stop(){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::RX_STOP;
    return send_cmd(c);
}
bool NetClient::cmd_rx_start(){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::RX_START;
    return send_cmd(c);
}
bool NetClient::cmd_set_fft_size(uint32_t size){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_FFT_SIZE;
    c.set_fft_size.size=size;
    return send_cmd(c);
}
bool NetClient::cmd_set_sr(float msps){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::SET_SR;
    c.set_sr.msps=msps;
    return send_cmd(c);
}
bool NetClient::cmd_delete_pub_file(const char* filename){
    PktPubDeleteReq req{};
    strncpy(req.filename, filename, sizeof(req.filename)-1);
    return send_packet(fd_, PacketType::PUB_DELETE_REQ, &req, sizeof(req));
}
bool NetClient::cmd_request_region(int32_t fft_top, int32_t fft_bot,
                                    float freq_lo, float freq_hi,
                                    int32_t time_start, int32_t time_end){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::REQUEST_REGION;
    c.request_region.fft_top=fft_top; c.request_region.fft_bot=fft_bot;
    c.request_region.freq_lo=freq_lo; c.request_region.freq_hi=freq_hi;
    c.request_region.time_start=time_start; c.request_region.time_end=time_end;
    return send_cmd(c);
}
bool NetClient::cmd_toggle_recv(int ch_idx, bool enable){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::TOGGLE_RECV;
    c.toggle_recv.idx=(uint8_t)ch_idx;
    c.toggle_recv.enable=enable?1:0;
    return send_cmd(c);
}
bool NetClient::cmd_update_ch_range(int idx, float s, float e){
    PktCmd c{}; c.cmd=(uint8_t)CmdType::UPDATE_CH_RANGE;
    c.update_ch_range.idx=(uint8_t)idx;
    c.update_ch_range.s=s; c.update_ch_range.e=e;
    return send_cmd(c);
}
bool NetClient::cmd_request_share_download(const char* filename){
    PktShareDownloadReq req{};
    strncpy(req.filename, filename, 127);
    return raw_send(PacketType::SHARE_DOWNLOAD_REQ, &req, sizeof(req));
}

bool NetClient::cmd_share_upload(const char* filepath, uint8_t transfer_id){
    FILE* fp = fopen(filepath, "rb");
    if(!fp){ fprintf(stderr,"cmd_share_upload: open failed %s\n", filepath); return false; }
    fseek(fp, 0, SEEK_END); uint64_t total=(uint64_t)ftell(fp); fseek(fp, 0, SEEK_SET);

    const char* fn = strrchr(filepath, '/'); fn = fn ? fn+1 : filepath;

    // META 전송
    PktShareUploadMeta meta{};
    strncpy(meta.filename, fn, 127);
    meta.total_bytes = total;
    meta.transfer_id = transfer_id;
    if(!raw_send(PacketType::SHARE_UPLOAD_META, &meta, sizeof(meta))){ fclose(fp); return false; }

    // DATA 청크 전송 (유동 속도 상한: 실측 TCP 속도의 50%, FFT·오디오 스트림 보호)
    // 청크 크기를 4KB로 줄여 send_mtx 점유 시간 최소화
    static constexpr uint64_t UPL_RATE_FLOOR = 256 * 1024;      // 최솟값 256 KB/s
    static constexpr uint64_t UPL_RATE_INIT  = 1 * 1024 * 1024; // 초기값 1 MB/s
    const uint32_t CHUNK = 4096;
    std::vector<uint8_t> buf(sizeof(PktShareUploadData) + CHUNK);
    uint64_t offset = 0;
    double measured_bps = (double)UPL_RATE_INIT;
    auto rate_epoch = std::chrono::steady_clock::now();
    uint64_t rate_sent = 0;
    while(true){
        size_t n = fread(buf.data() + sizeof(PktShareUploadData), 1, CHUNK, fp);
        if(n == 0) break;
        PktShareUploadData* d = reinterpret_cast<PktShareUploadData*>(buf.data());
        d->transfer_id = transfer_id;
        d->is_last     = feof(fp) ? 1 : 0;
        d->chunk_bytes = (uint32_t)n;
        d->offset      = offset;
        offset += n;
        uint32_t send_len = (uint32_t)(sizeof(PktShareUploadData)+n);
        auto t0 = std::chrono::steady_clock::now();
        if(!raw_send(PacketType::SHARE_UPLOAD_DATA, buf.data(), send_len)){
            fclose(fp); return false;
        }
        double send_us = (double)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if(send_us > 0.0){
            double chunk_bps = (double)(send_len + PKT_HDR_SIZE) / (send_us * 1e-6);
            measured_bps = measured_bps * 0.75 + chunk_bps * 0.25;
        }
        uint64_t target_bps = (uint64_t)(measured_bps * 0.50);
        if(target_bps < UPL_RATE_FLOOR) target_bps = UPL_RATE_FLOOR;
        rate_sent += n;
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - rate_epoch).count();
        int64_t want_us = (int64_t)(rate_sent * 1000000ULL / target_bps);
        if(want_us > elapsed_us)
            std::this_thread::sleep_for(std::chrono::microseconds(want_us - elapsed_us));
    }
    fclose(fp);
    return true;
}