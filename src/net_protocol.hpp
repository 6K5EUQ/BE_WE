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
    IQ_CHUNK           = 0x20,  // HOST → JOIN (MUX): IQ 파일 청크 전송
    REPORT_LIST        = 0x22,  // server → all: reported files list
    REPORT_ADD         = 0x23,  // client → server: new report notification
    DB_SAVE_META       = 0x24,  // client → server: save file+info to DB
    DB_SAVE_DATA       = 0x25,  // client → server: DB file data chunk
    DB_LIST            = 0x26,  // central → all: database file list
    DB_DOWNLOAD_REQ    = 0x27,  // client → central: request DB file download
    DB_DOWNLOAD_DATA   = 0x28,  // central → client: DB file data chunk
    DB_DELETE_REQ      = 0x29,  // client → central: delete DB file
    REPORT_DELETE      = 0x2A,  // client → central: delete report
    REPORT_UPDATE      = 0x2B,  // client → central: update report .info
    DIGI_LOG           = 0x2C,  // server → clients: digital decode text result
    DB_DOWNLOAD_INFO   = 0x2D,  // central → client: DB file .info contents (sent before DATA)
    DB_LIST_REQ        = 0x2E,  // client → central: request DB list refresh
    REPORT_LIST_REQ    = 0x2F,  // client → central: request Report list refresh
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
    int64_t  iq_write_sample;   // HOST tm_iq_write_sample at broadcast (0=N/A)
    int64_t  iq_total_samples;  // HOST tm_iq_total_samples (rolling buffer capacity)
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
    START_IQ_REC    = 0x19,  // JOIN → server: start per-ch IQ recording
    STOP_IQ_REC     = 0x1A,  // JOIN → server: stop per-ch IQ recording + transfer
    START_DIGI      = 0x1B,  // JOIN → server: start digital demod
    STOP_DIGI       = 0x1C,  // JOIN → server: stop digital demod
    SET_ANTENNA     = 0x1D,  // bidirectional: set HOST antenna free text (char[32])
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
                 int64_t time_start_ms; int64_t time_end_ms;
                 int64_t samp_start; int64_t samp_end; }  request_region;
        struct { uint32_t size; }                          set_fft_size;
        struct { float msps; }                             set_sr;
        struct { uint8_t idx; }                            start_iq_rec;
        struct { uint8_t idx; }                            stop_iq_rec;
        struct { uint8_t idx; uint8_t mode; uint8_t demod_type; uint8_t pad; float baud_rate; } start_digi;
        struct { uint8_t idx; }                            stop_digi;
        struct { char    antenna[32]; }                    set_antenna;
        uint8_t raw[48];
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
    uint8_t  dem_paused;   // 1=Holding (out-of-range, demod paused)
    uint8_t  _pad2[2];
    char     owner_name[32]; // channel creator ID
    // ── 녹음 상태 (HOST → JOIN 동기화) ──
    uint32_t iq_rec_secs;     // IQ 녹음 경과 시간 (초)
    uint32_t audio_rec_secs;  // 오디오 녹음 경과 시간 (초)
    uint32_t sq_active_secs;  // 스컬치 넘긴 시간 (초)
    uint32_t sq_total_secs;   // 전체 경과 시간 (초)
    uint8_t  iq_rec_on;       // IQ 녹음 활성 (0/1)
    uint8_t  audio_rec_on;    // 오디오 녹음 활성 (0/1)
    uint8_t  _pad3[2];
    // ── 디지털 복조 상태 ──
    uint8_t  digital_mode;     // Channel::DigitalMode (0-4)
    uint8_t  digi_run;         // 1=running
    uint8_t  digi_demod_type;  // 0=ASK, 1=FSK, 2=BPSK
    uint8_t  _pad_digi;
    float    digi_baud_rate;
    // Auto-ID 결과
    uint8_t  auto_id_state;    // 0=IDLE ~ 4=DECODING
    uint8_t  auto_id_mod;      // ModType
    uint8_t  _pad_auto[2];
    float    auto_id_baud;
    float    auto_id_conf;
    float    auto_id_snr;
    char     auto_id_proto[32];
};

struct __attribute__((packed)) PktChannelSync {
    ChSyncEntry ch[10]; // MAX_CHANNELS
};

// 중앙 릴레이(central_proto.hpp)의 CH_SYNC_ENTRY_SIZE와 반드시 일치해야 함.
// 이 값이 바뀌면 central도 같이 고쳐야 함.
static_assert(sizeof(ChSyncEntry) == 136, "ChSyncEntry size must match central/central_proto.hpp CH_SYNC_ENTRY_SIZE");

// ── DIGI_LOG ─────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktDigiLog {
    uint8_t  tab;       // 0=AIS, 1=ADS-B, 2=UAV, 3=DEMOD
    uint8_t  ch_idx;
    uint16_t msg_len;   // strlen of message
    // char msg[msg_len] follows (variable length)
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
    uint8_t host_state;      // 0=OK, 1=CHASSIS_RESETTING, 2=SPECTRUM_PAUSED
    uint8_t sdr_temp_c;      // SDR 온도 (°C 정수, 0=미지원/미측정)
    uint8_t sdr_state;       // 0=streaming OK, 1=stream error
    uint8_t iq_on;           // 0=IQ 롤링 off, 1=on
    uint8_t host_cpu_pct;    // HOST CPU % (0-100)
    uint8_t host_ram_pct;    // HOST RAM % (0-100)
    uint8_t host_cpu_temp_c; // HOST CPU 온도 °C
    uint8_t pad;
    char    antenna[32];     // HOST 안테나 자유텍스트 (JOIN/HOST 모두에 표시)
};

// ── IQ_CHUNK ──────────────────────────────────────────────────────────────
// HOST → JOIN (central MUX 경유): IQ 파일 청크 스트리밍
// seq==0: 전송 시작 (filename/filesize 유효), data_len>0: 데이터, data_len==0 && seq==0xFFFFFFFF: 완료
struct __attribute__((packed)) PktIqChunkHdr {
    uint32_t req_id;
    uint32_t seq;        // 0=START, 0xFFFFFFFF=END, 그 외=데이터 순번
    char     filename[128];
    uint64_t filesize;   // START 패킷에서만 유효
    uint32_t data_len;   // 뒤따르는 데이터 바이트 수 (END 패킷은 0)
    // uint8_t data[data_len] follows
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

// ── REPORT_LIST / REPORT_ADD ──────────────────────────────────────────────
struct __attribute__((packed)) ReportFileEntry {
    char     filename[128];
    uint64_t size_bytes;
    char     reporter[32];
    char     info_summary[256]; // key .info fields (Freq, Protocol, Target...)
};
struct __attribute__((packed)) PktReportList {
    uint16_t count;
    // ReportFileEntry[count] follows
};
struct __attribute__((packed)) PktReportAdd {
    char     filename[128];
    char     reporter[32];
    char     info_summary[256];
};

// ── DB_SAVE ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktDbSaveMeta {
    char     filename[128];
    uint64_t total_bytes;
    uint8_t  transfer_id;
    char     operator_name[32];
    char     info_data[512];    // full .info contents
};
struct __attribute__((packed)) PktDbSaveData {
    uint8_t  transfer_id;
    uint8_t  is_last;
    uint32_t chunk_bytes;
    // uint8_t data[chunk_bytes] follows
};

// ── DB_LIST ──────────────────────────────────────────────────────────────
struct __attribute__((packed)) DbFileEntry {
    char     filename[128];
    uint64_t size_bytes;
    char     operator_name[32];
    char     info_data[512];
};
struct __attribute__((packed)) PktDbList {
    uint16_t count;
    // DbFileEntry[count] follows
};

// ── DB_DOWNLOAD ──────────────────────────────────────────────────────────
struct __attribute__((packed)) PktDbDownloadReq {
    char     filename[128];
    char     operator_name[32];
};
struct __attribute__((packed)) PktDbDownloadData {
    char     filename[128];
    uint64_t total_bytes;   // valid in first chunk
    uint32_t chunk_bytes;
    uint8_t  is_first;
    uint8_t  is_last;
    // uint8_t data[chunk_bytes] follows
};
// .info contents delivered alongside DB download (sent before any DATA chunks)
struct __attribute__((packed)) PktDbDownloadInfo {
    char     filename[128];
    char     info_data[512];
};

// ── REPORT_DELETE / REPORT_UPDATE ─────────────────────────────────────────
struct __attribute__((packed)) PktReportDelete {
    char     filename[128];
};
struct __attribute__((packed)) PktReportUpdate {
    char     filename[128];
    char     info_data[512];   // full .info contents
};

// ── DB_DELETE ─────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktDbDeleteReq {
    char     filename[128];
    char     operator_name[32];
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

