#include "net_server.hpp"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>

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
    if(listen(server_fd_, 8) < 0){
        perror("listen"); close(server_fd_); server_fd_=-1; return false;
    }

    running_.store(true);
    accept_thr_ = std::thread(&NetServer::accept_loop, this);
    printf("[NetServer] listening on port %d\n", port);
    return true;
}

void NetServer::stop(){
    running_.store(false);
    if(server_fd_ >= 0){ shutdown(server_fd_, SHUT_RDWR); close(server_fd_); server_fd_=-1; }
    if(accept_thr_.joinable()) accept_thr_.join();

    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        c->alive.store(false);
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
        // set TCP keepalive
        int ka=1; setsockopt(cfd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));

        auto conn = std::make_shared<ClientConn>();
        conn->fd = cfd;
        conn->alive.store(true);

        {
            std::lock_guard<std::mutex> lk(clients_mtx_);
            clients_.push_back(conn);
        }
        conn->thr = std::thread(&NetServer::client_loop, this, conn);
        conn->thr.detach();
    }
}

// ── Client loop ───────────────────────────────────────────────────────────
void NetServer::client_loop(std::shared_ptr<ClientConn> c){
    while(c->alive.load()){
        PktHdr hdr{};
        if(!recv_all(c->fd, &hdr, PKT_HDR_SIZE)){
            break;
        }
        // validate magic
        if(memcmp(hdr.magic, BEWE_MAGIC, 4) != 0){
            printf("[NetServer] bad magic from op %d\n", c->op_index);
            break;
        }
        uint32_t len = hdr.len;
        if(len > 1024*1024){ // 1MB sanity limit
            printf("[NetServer] oversized packet %u\n", len);
            break;
        }
        std::vector<uint8_t> payload(len);
        if(len > 0 && !recv_all(c->fd, payload.data(), len)) break;

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
                        cmd->request_region.freq_lo, cmd->request_region.freq_hi);
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

    default: break;
    }
}

// ── drop_client ───────────────────────────────────────────────────────────
void NetServer::drop_client(std::shared_ptr<ClientConn> c){
    bool was_authed = c->authed;
    uint8_t idx = c->op_index;
    char name[32]; strncpy(name, c->name, 31);

    c->alive.store(false);
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
    }
}

// ── send_to ───────────────────────────────────────────────────────────────
void NetServer::send_to(ClientConn& c, PacketType type,
                         const void* payload, uint32_t len){
    if(!c.alive.load() || c.fd < 0) return;
    auto pkt = make_packet(type, payload, len);
    std::lock_guard<std::mutex> lk(c.send_mtx);
    if(!send_all(c.fd, pkt.data(), pkt.size())){
        c.alive.store(false);
    }
}

// ── Broadcast FFT ─────────────────────────────────────────────────────────
void NetServer::broadcast_fft(const int8_t* data, int fft_size,
                               int64_t wall_time,
                               uint64_t center_hz, uint32_t sr,
                               float pmin, float pmax){
    PktFftFrame hdr{};
    hdr.center_freq_hz = center_hz;
    hdr.sample_rate    = sr;
    hdr.fft_size       = (uint16_t)fft_size;
    hdr.power_min      = pmin;
    hdr.power_max      = pmax;
    hdr.wall_time      = wall_time;

    uint32_t total = (uint32_t)(sizeof(PktFftFrame) + fft_size);
    std::vector<uint8_t> payload(total);
    memcpy(payload.data(), &hdr, sizeof(PktFftFrame));
    memcpy(payload.data() + sizeof(PktFftFrame), data, fft_size);

    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load()) continue;
        auto pkt = make_packet(PacketType::FFT_FRAME, payload.data(), total);
        std::lock_guard<std::mutex> slk(c->send_mtx);
        send_all(c->fd, pkt.data(), pkt.size());
    }
}

// ── Send audio to specific operators ─────────────────────────────────────
void NetServer::send_audio(uint32_t op_mask, uint8_t ch_idx, int8_t pan,
                            const float* pcm, uint32_t n_samples){
    if(!op_mask || !n_samples) return;

    uint32_t payload_size = (uint32_t)(sizeof(PktAudioFrame) + n_samples*sizeof(float));
    std::vector<uint8_t> payload(payload_size);
    auto* ah = reinterpret_cast<PktAudioFrame*>(payload.data());
    ah->ch_idx    = ch_idx;
    ah->pan       = (uint8_t)(int8_t)pan;
    ah->n_samples = n_samples;
    memcpy(payload.data() + sizeof(PktAudioFrame), pcm, n_samples*sizeof(float));

    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load()) continue;
        // bit (op_index) set → send to this operator
        if(!(op_mask & (1u << c->op_index))) continue;
        auto pkt = make_packet(PacketType::AUDIO_FRAME, payload.data(), payload_size);
        std::lock_guard<std::mutex> slk(c->send_mtx);
        send_all(c->fd, pkt.data(), pkt.size());
    }
}

// ── Broadcast channel sync ────────────────────────────────────────────────
void NetServer::broadcast_channel_sync(const Channel* chs, int n){
    PktChannelSync sync{};
    for(int i=0; i<n && i<5; i++){
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
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load()) continue;
        send_to(*c, PacketType::CHANNEL_SYNC, &sync, sizeof(sync));
    }
}

// ── Broadcast chat ────────────────────────────────────────────────────────
void NetServer::broadcast_chat(const char* from, const char* msg){
    PktChat chat{};
    strncpy(chat.from, from, 31);
    strncpy(chat.msg,  msg,  sizeof(chat.msg)-1);
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load()) continue;
        send_to(*c, PacketType::CHAT, &chat, sizeof(chat));
    }
}

// ── Broadcast status ──────────────────────────────────────────────────────
void NetServer::broadcast_status(float cf_mhz, float gain_db,
                                  uint32_t sr, uint8_t hw_type){
    PktStatus s{}; s.cf_mhz=cf_mhz; s.gain_db=gain_db;
    s.sample_rate=sr; s.hw_type=hw_type;
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& c : clients_){
        if(!c->authed || !c->alive.load()) continue;
        send_to(*c, PacketType::STATUS, &s, sizeof(s));
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
            if(!c->authed || !c->alive.load()) continue;
            if(cnt >= MAX_OPERATORS) break;
            ol.ops[cnt].index = c->op_index;
            ol.ops[cnt].tier  = c->tier;
            strncpy(ol.ops[cnt].name, c->name, 31);
            ++cnt;
        }
        ol.count = (uint8_t)cnt;
        for(auto& c : clients_){
            if(!c->authed || !c->alive.load()) continue;
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
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for(auto& cli : clients_){
        if(!cli->authed || !cli->alive.load()) continue;
        auto pkt = make_packet(PacketType::WF_EVENT, &ev, sizeof(ev));
        std::lock_guard<std::mutex> slk(cli->send_mtx);
        send_all(cli->fd, pkt.data(), pkt.size());
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
        std::lock_guard<std::mutex> slk(target->send_mtx);
        send_all(target->fd, pkt.data(), pkt.size());
    }

    // send chunks
    const uint32_t CHUNK = 65536;
    std::vector<uint8_t> buf(sizeof(PktFileData)+CHUNK);
    uint64_t offset=0;
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
        {
            std::lock_guard<std::mutex> slk(target->send_mtx);
            send_all(target->fd, pkt.data(), pkt.size());
        }
        if(progress_cb) progress_cb(offset, total);
    }
    fclose(fp);
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