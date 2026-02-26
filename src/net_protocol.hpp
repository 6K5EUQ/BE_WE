#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <sys/socket.h>
#include <unistd.h>

// ── Magic & version ───────────────────────────────────────────────────────
static constexpr uint8_t BEWE_MAGIC[4] = {'B','E','W','E'};
static constexpr uint16_t BEWE_PROTO_VER = 1;
static constexpr int      BEWE_DEFAULT_PORT = 7700;

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
    FILE_DATA      = 0x0D,  // server → client: region file transfer chunk
    FILE_META      = 0x0E,  // server → client: file transfer start info
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
// header followed by int8_t[fft_size]
struct __attribute__((packed)) PktFftFrame {
    uint64_t center_freq_hz;
    uint32_t sample_rate;
    uint16_t fft_size;
    float    power_min;
    float    power_max;
    int64_t  wall_time;  // unix timestamp (time_t), 0=none
    // int8_t data[fft_size] follows
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
                 float freq_lo; float freq_hi; }           request_region;
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
};

struct __attribute__((packed)) PktChannelSync {
    ChSyncEntry ch[5];  // MAX_CHANNELS
};

// ── WF_EVENT ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktWfEvent {
    int32_t  fft_idx_offset; // relative to current_fft_idx at time of send
    int64_t  wall_time;
    uint8_t  type;           // 0=time-tag, 1=IQ Start, 2=IQ Stop
    char     label[32];
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