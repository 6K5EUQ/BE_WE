#include "net_client.hpp"
#include "udp_discovery.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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

    if(::connect(fd_, res->ai_addr, res->ai_addrlen) < 0){
        perror("[NetClient] connect");
        close(fd_); fd_=-1; freeaddrinfo(res); return false;
    }
    freeaddrinfo(res);

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
    while(connected_.load()){
        PktHdr hdr{};
        if(!recv_all(fd_, &hdr, PKT_HDR_SIZE)) break;
        if(memcmp(hdr.magic, BEWE_MAGIC, 4) != 0) break;

        uint32_t len = hdr.len;
        if(len > 4*1024*1024) break;

        std::vector<uint8_t> payload(len);
        if(len > 0 && !recv_all(fd_, payload.data(), len)) break;

        handle_packet(static_cast<PacketType>(hdr.type), payload.data(), len);
    }
    connected_.store(false);
    printf("[NetClient] disconnected\n");
}

// ── handle_packet ─────────────────────────────────────────────────────────
void NetClient::handle_packet(PacketType type,
                               const uint8_t* payload, uint32_t len){
    switch(type){

    case PacketType::FFT_FRAME: {
        if(len < sizeof(PktFftFrame)) break;
        auto* fh = reinterpret_cast<const PktFftFrame*>(payload);
        uint32_t data_len = len - (uint32_t)sizeof(PktFftFrame);
        if(data_len != fh->fft_size) break;

        std::lock_guard<std::mutex> lk(fft_mtx);
        cf_hz   = fh->center_freq_hz;
        sr      = fh->sample_rate;
        fft_sz  = fh->fft_size;
        pmin         = fh->power_min;
        pmax         = fh->power_max;
        fft_wall_time= fh->wall_time;
        fft_data.assign(payload + sizeof(PktFftFrame),
                        payload + sizeof(PktFftFrame) + data_len);
        fft_seq.fetch_add(1, std::memory_order_release);
        break;
    }

    case PacketType::AUDIO_FRAME: {
        if(len < sizeof(PktAudioFrame)) break;
        auto* ah = reinterpret_cast<const PktAudioFrame*>(payload);
        if(ah->ch_idx >= 5) break;
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

    // DATA 청크 전송
    const uint32_t CHUNK = 65536;
    std::vector<uint8_t> buf(sizeof(PktShareUploadData) + CHUNK);
    uint64_t offset = 0;
    while(true){
        size_t n = fread(buf.data() + sizeof(PktShareUploadData), 1, CHUNK, fp);
        if(n == 0) break;
        PktShareUploadData* d = reinterpret_cast<PktShareUploadData*>(buf.data());
        d->transfer_id = transfer_id;
        d->is_last     = feof(fp) ? 1 : 0;
        d->chunk_bytes = (uint32_t)n;
        d->offset      = offset;
        offset += n;
        if(!raw_send(PacketType::SHARE_UPLOAD_DATA, buf.data(), (uint32_t)(sizeof(PktShareUploadData)+n))){
            fclose(fp); return false;
        }
    }
    fclose(fp);
    return true;
}