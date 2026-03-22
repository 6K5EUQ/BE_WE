#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <atomic>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>

// ── Magic & version ───────────────────────────────────────────────────────
static constexpr uint8_t BEWE_MAGIC[4] = {'B','E','W','E'};
static constexpr uint16_t BEWE_PROTO_VER = 1;
static constexpr int      BEWE_DEFAULT_PORT = 7701;

// ── Packet types ──────────────────────────────────────────────────────────
enum class PacketType : uint8_t {
    AUTH_REQ       = 0x01,  // client → server
    AUTH_ACK       = 0x02,  // server → client
    FFT_FRAME      = 0x03,  // server → all clients (broadcast)
    AUDIO_FRAME    = 0x04,  // server → specific operator(s)
    CMD            = 0x05,  // client → server
    CMD_ACK        = 0x06,  // server → client
    CHAT           = 0x07,  // bidirectional
    STATUS         = 0x08,  // server → all clients
    OPERATOR_LIST  = 0x09,  // server → all clients
    CHANNEL_SYNC   = 0x0A,  // server → all clients
    DISCONNECT     = 0x0B,
    WF_EVENT       = 0x0C,  // server → all: waterfall time/event tag
    FILE_DATA        = 0x0D,  // server → client: region file transfer chunk
    FILE_META        = 0x0E,  // server → client: file transfer start info
    REGION_RESPONSE  = 0x0F,  // server → client: region request allow/deny
    SHARE_LIST         = 0x10,  // server → all clients: list of shared files
    SHARE_DOWNLOAD_REQ = 0x11,  // client → server: request download of shared file
    SHARE_UPLOAD_META  = 0x12,  // client → server: upload file meta (start)
    SHARE_UPLOAD_DATA  = 0x13,  // client → server: upload file chunk
    HEARTBEAT          = 0x14,  // server → all clients: 3s keepalive + state
    PUB_DELETE_REQ     = 0x15,  // client → server: request delete of public file (owner only)
    IQ_PROGRESS        = 0x16,  // server → all: IQ 파일 전송 진행상황 (REC/Transferring/Done)
    IQ_PIPE_READY      = 0x20,  // 중앙서버 → JOIN: 파이프 연결 준비 완료, req_id 전달
};

// ── Packet header (9 bytes, packed) ──────────────────────────────────────
struct __attribute__((packed)) PktHdr {
    uint8_t  magic[4];
    uint8_t  type;      // PacketType
    uint32_t len;       // payload length (LE)
};
static constexpr int PKT_HDR_SIZE = sizeof(PktHdr);

// ── AUTH_REQ ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktAuthReq {
    char    id[32];
    char    pw[64];
    uint8_t tier;
};

// ── AUTH_ACK ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktAuthAck {
    uint8_t ok;          // 1=success
    uint8_t op_index;    // 1-based operator index (0 if fail)
    char    reason[48];  // fail reason or "OK"
};

// ── FFT_FRAME ─────────────────────────────────────────────────────────────
// header followed by float[fft_size] (dB values)
struct __attribute__((packed)) PktFftFrame {
    uint64_t center_freq_hz;
    uint32_t sample_rate;
    uint32_t fft_size;
    float    power_min;
    float    power_max;
    int64_t  wall_time;  // unix timestamp (time_t), 0=none
    // float data[fft_size] follows (raw dB power per bin)
};

// ── AUDIO_FRAME ───────────────────────────────────────────────────────────
// header followed by float[n_samples] PCM mono
struct __attribute__((packed)) PktAudioFrame {
    uint8_t  ch_idx;
    uint8_t  pan;        // -1(L) 0(both) 1(R) cast as int8
    uint32_t n_samples;
    // float[n_samples] follows
};

// ── CMD ───────────────────────────────────────────────────────────────────
enum class CmdType : uint8_t {
    SET_FREQ     = 0x01,
    SET_GAIN     = 0x02,
    CREATE_CH    = 0x03,
    DELETE_CH    = 0x04,
    SET_CH_MODE  = 0x05,
    SET_CH_AUDIO = 0x06,  // set audio_mask for channel
    START_REC    = 0x07,
    STOP_REC     = 0x08,
    SET_CH_PAN      = 0x09,
    SET_SQ_THRESH   = 0x0A,  // set squelch threshold for channel
    SET_AUTOSCALE   = 0x0B,  // trigger autoscale
    TOGGLE_RECV     = 0x0C,  // JOIN: enable/disable receiving audio for channel
    UPDATE_CH_RANGE = 0x0D,  // update channel s/e (drag move/resize)
    TOGGLE_TM_IQ    = 0x0E,  // toggle TM IQ recording
    SET_CAPTURE_PAUSE  = 0x0F,
    SET_SPECTRUM_PAUSE = 0x10,
    REQUEST_REGION  = 0x11,  // JOIN: request region file transfer
    CHASSIS_RESET   = 0x12,  // JOIN → server: trigger chassis 1 reset on HOST
    DELETE_PUB_FILE = 0x13,  // JOIN → server: delete a public file (owner only)
    SET_FFT_SIZE    = 0x14,  // bidirectional: change FFT size
    SET_SR          = 0x15,  // bidirectional: change sample rate
    NET_RESET       = 0x16,  // JOIN → server: trigger chassis 2 (net-only) reset
    RX_STOP         = 0x17,  // JOIN → server: /rx stop
    RX_START        = 0x18,  // JOIN → server: /rx start
};

struct __attribute__((packed)) PktCmd {
    uint8_t cmd;   // CmdType
    uint8_t pad[3];
    union {
        struct { float cf_mhz; }                           set_freq;
        struct { float db; }                               set_gain;
        struct { uint8_t idx; float s; float e; }          create_ch;
        struct { uint8_t idx; }                            delete_ch;
        struct { uint8_t idx; uint8_t mode; }              set_ch_mode;
        struct { uint8_t idx; uint32_t mask; }             set_ch_audio;
        struct { uint8_t ch_idx; }                         start_rec;
        struct { uint8_t idx; int8_t pan; }                set_ch_pan;
        struct { uint8_t idx; float thr; }                set_sq_thresh;
        struct { uint8_t dummy; }                          set_autoscale;
        struct { uint8_t idx; uint8_t enable; }            toggle_recv;
        struct { uint8_t idx; float s; float e; }          update_ch_range;
        struct { uint8_t dummy; }                          toggle_tm_iq;
        struct { uint8_t pause; }                          set_capture_pause;
        struct { uint8_t pause; }                          set_spectrum_pause;
        struct { int32_t fft_top; int32_t fft_bot;
                 float freq_lo; float freq_hi;
                 int32_t time_start; int32_t time_end; }  request_region;
        struct { uint32_t size; }                          set_fft_size;
        struct { float msps; }                             set_sr;
        uint8_t raw[32];
    };
};

// ── CMD_ACK ───────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktCmdAck {
    uint8_t ok;
    uint8_t cmd;
    char    msg[32];
};

// ── CHAT ──────────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktChat {
    char from[32];
    char msg[256];
};

// ── STATUS ────────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktStatus {
    float    cf_mhz;
    float    gain_db;
    uint32_t sample_rate;
    uint8_t  hw_type;  // 0=BladeRF, 1=RTL-SDR
    uint8_t  pad[3];
};

// ── Operator entry ────────────────────────────────────────────────────────
struct __attribute__((packed)) OpEntry {
    uint8_t index;   // 1-based
    uint8_t tier;
    char    name[32];
};
static constexpr int MAX_OPERATORS = 16;

// ── OPERATOR_LIST ─────────────────────────────────────────────────────────
struct __attribute__((packed)) PktOperatorList {
    uint8_t  count;
    OpEntry  ops[MAX_OPERATORS];
};

// ── CHANNEL_SYNC entry ────────────────────────────────────────────────────
struct __attribute__((packed)) ChSyncEntry {
    uint8_t  idx;
    uint8_t  active;    // filter_active
    float    s, e;      // freq range MHz
    uint8_t  mode;      // DemodMode
    int8_t   pan;
    uint32_t audio_mask; // bitmask: bit i → operator (i+1) gets audio; bit0=host
    float    sq_threshold; // squelch threshold dB
    float    sq_sig;       // current signal level dB (for sq meter UI)
    uint8_t  sq_gate;      // squelch gate open (1=open)
    uint8_t  _pad2[3];
    char     owner_name[32]; // channel creator ID
};

struct __attribute__((packed)) PktChannelSync {
    ChSyncEntry ch[10]; // MAX_CHANNELS
};

// ── WF_EVENT ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktWfEvent {
    int32_t  fft_idx_offset; // relative to current_fft_idx at time of send
    int64_t  wall_time;
    uint8_t  type;           // 0=time-tag, 1=IQ Start, 2=IQ Stop
    char     label[32];
};

// ── REGION_RESPONSE ────────────────────────────────────────────────────────
struct __attribute__((packed)) PktRegionResponse {
    uint8_t allowed;   // 1=allow, 0=deny
    uint8_t pad[3];
};

// ── FILE_META ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktFileMeta {
    char     filename[128];
    uint64_t total_bytes;
    uint8_t  transfer_id;
};

// ── FILE_DATA ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktFileData {
    uint8_t  transfer_id;
    uint8_t  is_last;        // 1 = final chunk
    uint32_t chunk_bytes;
    uint64_t offset;
    // uint8_t data[chunk_bytes] follows
};

// ── SHARE_LIST ─────────────────────────────────────────────────────────────
// Variable-length payload: count (uint16_t) followed by count entries
struct __attribute__((packed)) ShareFileEntry {
    char     filename[128];
    uint64_t size_bytes;
    char     uploader[32];  // 업로드한 사람 이름 (HOST 이름 또는 JOIN 이름)
};
struct __attribute__((packed)) PktShareList {
    uint16_t count;
    // ShareFileEntry entries[count] follow
};

// ── SHARE_DOWNLOAD_REQ ─────────────────────────────────────────────────────
struct __attribute__((packed)) PktShareDownloadReq {
    char filename[128];
};

// ── SHARE_UPLOAD_META (client → server) ────────────────────────────────────
struct __attribute__((packed)) PktShareUploadMeta {
    char     filename[128];
    uint64_t total_bytes;
    uint8_t  transfer_id;
};

// ── SHARE_UPLOAD_DATA (client → server) ────────────────────────────────────
struct __attribute__((packed)) PktShareUploadData {
    uint8_t  transfer_id;
    uint8_t  is_last;
    uint32_t chunk_bytes;
    uint64_t offset;
    // uint8_t data[chunk_bytes] follows
};

// ── PUB_DELETE_REQ (client → server) ─────────────────────────────────────
// JOIN sends this to request deletion of a public file they own.
struct __attribute__((packed)) PktPubDeleteReq {
    char filename[128];  // null-terminated filename (no path)
};

// ── HEARTBEAT (server → all clients) ─────────────────────────────────────
// Sent every 3 seconds.
// host_state: 0=OK, 1=CHASSIS_RESETTING, 2=SPECTRUM_PAUSED
// sdr_state:  0=OK, 1=ERROR (stall/buffer problem)
struct __attribute__((packed)) PktHeartbeat {
    uint8_t host_state;  // 0=OK, 1=CHASSIS_RESETTING, 2=SPECTRUM_PAUSED
    uint8_t sdr_temp_c;  // SDR 온도 (°C 정수, 0=미지원/미측정)
    uint8_t sdr_state;   // 0=streaming OK, 1=stream error
    uint8_t iq_on;       // 0=IQ 롤링 off, 1=on
};

// ── IQ_PIPE_READY ─────────────────────────────────────────────────────────
// 중앙서버 → JOIN (MUX 경유): 파이프 포트(7702)에 접속하라는 신호
struct __attribute__((packed)) PktIqPipeReady {
    uint32_t req_id;
    char     filename[128];
    uint64_t filesize;
};

// ── IQ_PROGRESS ───────────────────────────────────────────────────────────
// server → all: IQ 파일 전송 진행상황
// phase: 0=REC중, 1=Transferring, 2=Done
struct __attribute__((packed)) PktIqProgress {
    uint32_t req_id;
    char     filename[128];
    uint64_t done;
    uint64_t total;
    uint8_t  phase;  // 0=REC, 1=Transferring, 2=Done
};

// ── Wire helpers ──────────────────────────────────────────────────────────
inline std::vector<uint8_t> make_packet(PacketType type,
                                         const void* payload, uint32_t len){
    std::vector<uint8_t> pkt(PKT_HDR_SIZE + len);
    PktHdr* h = reinterpret_cast<PktHdr*>(pkt.data());
    memcpy(h->magic, BEWE_MAGIC, 4);
    h->type = static_cast<uint8_t>(type);
    h->len  = len;
    if(len && payload)
        memcpy(pkt.data() + PKT_HDR_SIZE, payload, len);
    return pkt;
}

inline bool send_all(int fd, const void* buf, size_t len){
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    while(len > 0){
        ssize_t r = send(fd, p, len, MSG_NOSIGNAL);
        if(r <= 0) return false;
        p += r; len -= r;
    }
    return true;
}

// Returns:  1 = OK,  0 = timeout (EAGAIN),  -1 = error/disconnect
inline int recv_all_ex(int fd, void* buf, size_t len, const std::atomic<bool>& alive){
    uint8_t* p = static_cast<uint8_t*>(buf);
    size_t remaining = len;
    while(remaining > 0){
        if(!alive.load(std::memory_order_relaxed)) return -1;
        ssize_t r = recv(fd, p, remaining, 0);
        if(r > 0){ p += r; remaining -= r; continue; }
        if(r == 0) return -1;                         // peer closed
        if(errno == EAGAIN || errno == EWOULDBLOCK){
            if(remaining == len) return 0;             // nothing read yet → timeout
            continue;                                  // partial read → retry
        }
        return -1;                                     // real error
    }
    return 1;
}

// Legacy wrapper (blocking, used during AUTH handshake before timeout is set)
inline bool recv_all(int fd, void* buf, size_t len){
    uint8_t* p = static_cast<uint8_t*>(buf);
    while(len > 0){
        ssize_t r = recv(fd, p, len, 0);
        if(r <= 0) return false;
        p += r; len -= r;
    }
    return true;
}

inline bool send_packet(int fd, PacketType type, const void* payload, uint32_t len){
    auto pkt = make_packet(type, payload, len);
    return send_all(fd, pkt.data(), pkt.size());
}

// ── UDP Discovery (port 7701, independent of TCP protocol) ───────────────
static constexpr int BEWE_DISCOVERY_PORT = 7701;

struct __attribute__((packed)) DiscoveryAnnounce {
    char     magic[4];          // 'B','E','W','G'
    char     station_name[64];  // null-terminated UTF-8
    float    lat;               // degrees [-90, +90]
    float    lon;               // degrees [-180, +180]
    uint16_t tcp_port;          // TCP listen port for connections
    uint8_t  user_count;        // currently connected operator count
    uint8_t  host_tier;         // host operator tier (1=Tier1, 2=Tier2)
    char     host_ip[16];       // IPv4 dotted-decimal, null-terminated
    uint8_t  _pad[8];           // reserved
};                              // total: 104 bytes