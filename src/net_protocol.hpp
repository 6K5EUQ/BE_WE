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
    REPORT_ADD         = 0x23,  // client → central: ingest report into emitter DB
    DB_SAVE_META       = 0x24,  // client → server: save file+info to DB
    DB_SAVE_DATA       = 0x25,  // client → server: DB file data chunk
    DB_LIST            = 0x26,  // central → all: database file list
    DB_DOWNLOAD_REQ    = 0x27,  // client → central: request DB file download
    DB_DOWNLOAD_DATA   = 0x28,  // central → client: DB file data chunk
    DB_DELETE_REQ      = 0x29,  // client → central: delete DB file
    DB_DOWNLOAD_INFO   = 0x2D,  // central → client: DB file .info contents (sent before DATA)
    DB_LIST_REQ        = 0x2E,  // client → central: request DB list refresh
    SCHED_SYNC         = 0x30,  // server → all clients: scheduled recording list snapshot
    BAND_PLAN_SYNC     = 0x31,  // host → all clients: band plan (frequency allocation overlay)
    BAND_ADD           = 0x32,  // any → host: add band segment
    BAND_REMOVE        = 0x33,  // any → host: remove band segment (by freq_lo+hi)
    BAND_UPDATE        = 0x34,  // any → host: update band segment
    BAND_CAT_SYNC      = 0x35,  // host → all clients: band category list (id/name/color)
    BAND_CAT_UPSERT    = 0x36,  // any → host: insert/update one band category
    BAND_CAT_DELETE    = 0x37,  // any → host: delete one band category by id
    LWF_LIST_REQ       = 0x38,  // any → host: request long-waterfall file list
    LWF_LIST           = 0x39,  // host → client: long-waterfall file list
    LWF_DL_REQ         = 0x3A,  // any → host: download long-waterfall file by name
    LWF_DL_DATA        = 0x3B,  // host → client: (deprecated; reuses FILE_DATA)
    LWF_LIVE_START     = 0x3C,  // host → joins: start streaming current LIVE file
    LWF_LIVE_ROW       = 0x3D,  // host → joins: one row append to LIVE file
    LWF_LIVE_STOP      = 0x3E,  // host → joins: LIVE file rotated/closed
    LWF_LIVE_REQ       = 0x3F,  // any → host: opt-in request to start LIVE stream
    LWF_DELETE_REQ     = 0x40,  // any → host: delete a long-waterfall file by name

    // ── Signal Library / Emitter DB (central-managed) ─────────────────────
    EMITTER_LIST_REQ   = 0x41,  // client → central: 전체 emitter 목록 요청 (off/lim)
    EMITTER_LIST       = 0x42,  // central → all: emitter 목록 broadcast (페이지네이션)
    EMITTER_UPSERT     = 0x43,  // client → central: emitter create/update (이름·메모 등)
    EMITTER_DELETE     = 0x44,  // client → central: emitter 삭제 (자식 sighting은 unlink)
    SIGHTING_LIST_REQ  = 0x45,  // client → central: sighting 목록 (특정 emitter 또는 전체)
    SIGHTING_LIST      = 0x46,  // central → caller: sighting 목록 응답
    SIGHTING_LINK      = 0x47,  // client → central: confirm/reject/move/split sighting
    // ── SIGINT Mission System ────────────────────────────────────────────
    MISSION_SYNC       = 0x48,  // host → central → all: 미션 전체 스냅샷 (active + history)
    MISSION_START      = 0x49,  // any → host: 미션 시작 요청 (relay via central)
    MISSION_END        = 0x4A,  // any → host: 활성 미션 종료 요청
    MISSION_UPDATE     = 0x4B,  // any → host: 미션 메타데이터 업데이트 (notes 등)
    MISSION_LIST_REQ   = 0x4C,  // any → central: MISSION_SYNC 재발송 요청
    MISSION_DELETE     = 0x4D,  // any → host: 특정 미션 디렉토리/메타 삭제
    // ── Mission File Archive (Central-centric, station-keyed) ────────────
    MISSION_FILE_PUSH_META = 0x4E,  // host → central: 파일 업로드 시작 (transfer 선언)
    MISSION_FILE_PUSH_DATA = 0x4F,  // host → central: 파일 청크 (offset append)
    MISSION_FILE_LIST_REQ  = 0x50,  // any → central: station/year/code 파일 목록 요청
    MISSION_FILE_LIST      = 0x51,  // central → caller: 파일 목록 응답
    MISSION_FILE_DL_REQ    = 0x52,  // any → central: 파일 다운로드 요청
    MISSION_FILE_DL_DATA   = 0x53,  // central → caller: 다운로드 청크 (첫 청크에 .info)
    MISSION_FILE_DELETE    = 0x54,  // any → central: Central archive 파일 삭제
    MISSION_FILE_RENAME    = 0x55,  // any → central: Central archive 파일 이름 변경
    MISSION_FILE_PUSH_ACK  = 0x56,  // central → host: PUSH 전송 종료 ACK (HOST가 로컬 unlink 트리거)
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
    SET_ANTENNA     = 0x1D,  // bidirectional: set HOST antenna free text (char[32])
    ADD_SCHED       = 0x1E,  // JOIN → server: add scheduled IQ recording
    REMOVE_SCHED    = 0x1F,  // JOIN → server: remove own scheduled entry
    SET_HW          = 0x21,  // JOIN → server: switch HOST SDR runtime ("bladerf"/"pluto"/"rtlsdr")
    TOGGLE_FFT_RECV = 0x22,  // JOIN → central: enable/disable FFT stream (audio/HB unaffected)
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
        struct { char    antenna[32]; }                    set_antenna;
        struct { int64_t start_time; float duration_sec; float freq_mhz; float bw_khz;
                 char target[32]; }                                add_sched;
        struct { int64_t start_time; float freq_mhz; }             remove_sched;
        struct { char    name[16]; }                       set_hw;
        struct { uint8_t enable; }                         toggle_fft_recv;
        uint8_t raw[64];
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
};

struct __attribute__((packed)) PktChannelSync {
    ChSyncEntry ch[10]; // MAX_CHANNELS
};

// 중앙 릴레이(central_proto.hpp)의 CH_SYNC_ENTRY_SIZE와 반드시 일치해야 함.
// 이 값이 바뀌면 central도 같이 고쳐야 함.
static_assert(sizeof(ChSyncEntry) == 80, "ChSyncEntry size must match central/central_proto.hpp CH_SYNC_ENTRY_SIZE");

// ── SCHED_SYNC: 예약 녹음 리스트 전체 스냅샷 (server → all) ───────────────
static constexpr int MAX_SCHED_ENTRIES = 32;
struct __attribute__((packed)) SchedSyncEntry {
    uint8_t  valid;            // 1=유효, 0=빈 슬롯
    uint8_t  status;           // SchedEntry::Status
    uint8_t  op_index;         // 0=HOST, 1..N=JOIN op_index
    uint8_t  _pad;
    int64_t  start_time;       // Unix epoch seconds
    float    duration_sec;
    float    freq_mhz;
    float    bw_khz;
    char     operator_name[32];
    char     target[32];       // free-form 식별 라벨
    // v4.0: mission context — set by HOST at schedule-add time so JOINs
    // can filter the list per mission. Empty for entries added outside a mission.
    uint16_t mission_year;
    char     mission_code[8];
    uint8_t  _pad2[6];
}; // 104 bytes
static_assert(sizeof(SchedSyncEntry) == 104, "SchedSyncEntry size");
struct __attribute__((packed)) PktSchedSync {
    uint8_t        count;
    uint8_t        _pad[3];
    SchedSyncEntry entries[MAX_SCHED_ENTRIES];
};

// ── BAND_PLAN: 주파수 할당 오버레이 (central → all) ──────────────────────
static constexpr int MAX_BAND_SEGMENTS = 128;
struct __attribute__((packed)) PktBandEntry {
    uint8_t  valid;            // 1=유효, 0=빈
    uint8_t  category;         // 0..10 (FFTViewer::BandSegment 카테고리)
    uint8_t  _pad[2];
    float    freq_lo_mhz;
    float    freq_hi_mhz;
    char     label[24];
    char     description[128];
}; // 164 bytes
static_assert(sizeof(PktBandEntry) == 164, "PktBandEntry size");
struct __attribute__((packed)) PktBandPlan {
    uint16_t     count;
    uint8_t      _pad[2];
    PktBandEntry entries[MAX_BAND_SEGMENTS];
};
struct __attribute__((packed)) PktBandRemove {
    float freq_lo_mhz;
    float freq_hi_mhz;
};

// ── BAND_CAT: 카테고리 목록 (host → all). 가변 크기. ───────────────────
// id 0~10 builtin, 11~255 user-defined. 빈 슬롯은 valid=0.
static constexpr int MAX_BAND_CATEGORIES = 64;
struct __attribute__((packed)) PktBandCategory {
    uint8_t  id;
    uint8_t  valid;
    uint8_t  r, g, b;        // RGB color
    uint8_t  _pad[3];
    char     name[24];
}; // 32 bytes
static_assert(sizeof(PktBandCategory) == 32, "PktBandCategory size");
struct __attribute__((packed)) PktBandCatSync {
    uint16_t        count;        // entries[0..count-1] valid
    uint8_t         _pad[2];
    PktBandCategory entries[MAX_BAND_CATEGORIES];
};
struct __attribute__((packed)) PktBandCatDelete {
    uint8_t id;
    uint8_t _pad[3];
};

// ── LWF: long-waterfall (host-owned image files) ─────────────────────────
static constexpr int MAX_LWF_FILES = 64;
struct __attribute__((packed)) LwfFileEntry {
    char     filename[64];     // basename only
    uint64_t size_bytes;
    uint64_t start_utc;        // header.start_utc_unix
    uint64_t center_freq_hz;
    uint64_t sample_rate_hz;
    uint32_t fft_size;
    uint32_t num_rows;         // (size_bytes - sizeof(FileHeader)) / fft_size
    char     station_name[32]; // v3: host station name
    float    station_lat;      // v3
    float    station_lon;      // v3
};
static_assert(sizeof(LwfFileEntry) == 144, "LwfFileEntry size");
struct __attribute__((packed)) PktLwfList {
    uint16_t     count;
    uint8_t      _pad[2];
    LwfFileEntry entries[MAX_LWF_FILES];
};
struct __attribute__((packed)) PktLwfDlReq {
    char filename[64];
};
struct __attribute__((packed)) PktLwfDlData {
    char     filename[64];
    uint64_t total_bytes;
    uint64_t offset;
    uint32_t chunk_bytes;
    uint8_t  is_first;
    uint8_t  is_last;
    uint8_t  _pad[6];
    // raw chunk bytes follow (chunk_bytes)
};

// ── LWF_LIVE_*: host의 누적 행을 JOIN에 실시간 push ─────────────────────
struct __attribute__((packed)) PktLwfLiveStart {
    char     filename[64];
    uint32_t fft_size;            // padded (디스크 row 폭)
    uint32_t fft_input_size;      // 사용자 설정 FFT
    uint64_t sample_rate_hz;
    uint64_t center_freq_hz;
    float    row_rate_hz;
    float    db_min;
    float    db_max;
    uint64_t start_utc_unix;
    float    station_lon;
    int32_t  utc_offset_hours;
    char     station_name[32];    // v3: host station name
    float    station_lat;         // v3
};
struct __attribute__((packed)) PktLwfLiveRowHdr {
    char     filename[64];
    uint32_t row_index;
    // raw row bytes (fft_size 길이) follow
};
struct __attribute__((packed)) PktLwfLiveStop {
    char     filename[64];
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

// ── REPORT_ADD ────────────────────────────────────────────────────────────
// 운용사가 파일을 report → Central이 info_data를 파싱해 Signal Library에 ingest.
// 디스크에 .info 저장은 하지 않음 (Library가 단일 진실 원천).
struct __attribute__((packed)) PktReportAdd {
    char     filename[128];
    char     reporter[32];
    char     info_data[512];    // full .info contents (parsed into Sighting fields)
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

// ── DB_DELETE ─────────────────────────────────────────────────────────────
struct __attribute__((packed)) PktDbDeleteReq {
    char     filename[128];
    char     operator_name[32];
};

// ── Signal Library / Emitter DB (0x41–0x47) ──────────────────────────────
// 운영자가 .info에 직접 적은 값 + 녹음 시점 자동 기재값만 매칭에 사용.
// 자동 디코더 결과(AIS/ADS-B/PRI)는 .info 자체에 자동 stamp되지 않으므로
// 본 패킷 페이로드에도 들어오지 않음.
constexpr int EMITTER_UID_LEN     = 16;     // "e_<8hex>"
constexpr int EMITTER_NAME_LEN    = 64;
constexpr int EMITTER_NOTES_LEN   = 512;
constexpr int EMITTER_MOD_LEN     = 16;
constexpr int EMITTER_PROTO_LEN   = 16;
constexpr int EMITTER_TAGS_LEN    = 32;
constexpr int STATIONS_LIST_LEN   = 128;    // 콤마 구분 contributing stations
constexpr int SIGHTING_ID_LEN     = 16;
constexpr int SIGHTING_FILE_LEN   = 128;
constexpr int SIGHTING_REPORTER_LEN = 32;
constexpr int SIGHTING_STATION_LEN  = 32;
constexpr int MAX_EMITTERS_PER_PKT  = 200;  // 페이지네이션 단위
constexpr int MAX_SIGHTINGS_PER_PKT = 200;

struct __attribute__((packed)) PktEmitterEntry {
    char     emitter_uid[EMITTER_UID_LEN];
    char     display_name[EMITTER_NAME_LEN];
    float    freq_center_mhz;
    float    freq_tolerance_khz;
    float    bw_khz;
    char     modulation[EMITTER_MOD_LEN];
    char     protocol[EMITTER_PROTO_LEN];
    char     tags_id[EMITTER_TAGS_LEN];
    int64_t  first_seen_utc;
    int64_t  last_seen_utc;
    uint32_t sighting_count;
    char     contributing_stations[STATIONS_LIST_LEN]; // 콤마 구분
    char     operator_notes[EMITTER_NOTES_LEN];
};

struct __attribute__((packed)) PktEmitterListReq {
    uint16_t offset;
    uint16_t limit;
};

struct __attribute__((packed)) PktEmitterList {
    uint16_t total_count;
    uint16_t offset;
    uint16_t count;
    uint8_t  _pad[2];
    // PktEmitterEntry[count] follows
};

// 신규(emitter_uid 빈) 또는 기존 갱신.
struct __attribute__((packed)) PktEmitterUpsert {
    char     emitter_uid[EMITTER_UID_LEN];
    char     display_name[EMITTER_NAME_LEN];
    float    freq_center_mhz;
    float    freq_tolerance_khz;
    float    bw_khz;
    char     modulation[EMITTER_MOD_LEN];
    char     protocol[EMITTER_PROTO_LEN];
    char     tags_id[EMITTER_TAGS_LEN];
    char     operator_notes[EMITTER_NOTES_LEN];
    char     editor[SIGHTING_REPORTER_LEN]; // 변경 stamp용
};

struct __attribute__((packed)) PktEmitterDelete {
    char emitter_uid[EMITTER_UID_LEN];
};

struct __attribute__((packed)) PktSightingEntry {
    char     sighting_id[SIGHTING_ID_LEN];
    char     filename[SIGHTING_FILE_LEN];
    char     reporter[SIGHTING_REPORTER_LEN];
    char     station[SIGHTING_STATION_LEN];
    float    freq_mhz;
    float    bw_khz;
    char     modulation[EMITTER_MOD_LEN];
    char     protocol[EMITTER_PROTO_LEN];
    int64_t  start_utc;
    uint32_t duration_s;
    char     emitter_uid[EMITTER_UID_LEN];
    uint8_t  match_status;          // MS_AUTO_HIGH..MS_MANUAL (emitter_db.hpp)
    uint8_t  _pad[3];
};

struct __attribute__((packed)) PktSightingListReq {
    char     emitter_uid[EMITTER_UID_LEN]; // 빈 = 전체
    uint16_t offset;
    uint16_t limit;
};

struct __attribute__((packed)) PktSightingList {
    char     emitter_uid_filter[EMITTER_UID_LEN];
    uint16_t total_count;
    uint16_t offset;
    uint16_t count;
    uint8_t  _pad[2];
    // PktSightingEntry[count] follows
};

struct __attribute__((packed)) PktSightingLink {
    char     sighting_id[SIGHTING_ID_LEN];
    char     emitter_uid[EMITTER_UID_LEN]; // move/split의 target (confirm/reject 시 빈)
    uint8_t  action;                       // 0=confirm 1=reject 2=move 3=split_to_new
    uint8_t  _pad[3];
    char     editor[SIGHTING_REPORTER_LEN];
};

// ── SIGINT Mission System ────────────────────────────────────────────────
static constexpr int MAX_MISSION_HISTORY_PER_PKT = 32;

// 한 미션의 wire-form representation (fixed POD)
// v4.0: removed name/purpose/target/notes. Added station/host/lat/lon/sdr/antenna
// captured automatically at mission start (no user-entered fields).
struct __attribute__((packed)) MissionSyncEntry {
    uint8_t  valid;          // 1=유효, 0=빈 슬롯
    uint8_t  state;          // Mission::State (0=IDLE,1=ACTIVE,2=CLOSING)
    uint8_t  op_index;       // 0=HOST, 1..N=JOIN
    uint8_t  rollover;       // 1=UTC0 자동 시작
    uint16_t year;
    uint16_t _pad;
    char     code[8];        // "A03"
    int64_t  start_utc;
    int64_t  end_utc;        // 0 = open
    char     started_by[32];
    // Auto-captured metadata
    char     station_name[64];
    char     host_name[32];
    float    lat;
    float    lon;
    char     sdr_kind[24];   // "BladeRF" / "RTL-SDR" / "Pluto"
    char     antenna[64];
};

// Central → all clients (handshake replay + broadcast)
struct __attribute__((packed)) PktMissionSync {
    uint8_t          active_valid;       // 1 = active entry below is meaningful
    uint8_t          _pad[3];
    uint16_t         history_count;      // entries[]의 유효 개수
    uint16_t         _pad2;
    MissionSyncEntry active;             // 현재 ACTIVE 미션 (없으면 valid=0)
    MissionSyncEntry entries[MAX_MISSION_HISTORY_PER_PKT];
};

// JOIN/HOST → Central → HOST (relay) — v4.0: payload reduced to trigger only.
// started_by is filled by HOST from op_name; op_index is filled by central.
struct __attribute__((packed)) PktMissionStart {
    uint8_t op_index;         // central이 채움 (JOIN op_index)
    uint8_t _pad[3];
};

struct __attribute__((packed)) PktMissionEnd {
    uint8_t op_index;
    uint8_t _pad[3];
};

struct __attribute__((packed)) PktMissionUpdate {
    char    code[8];
    uint16_t year;
    uint8_t  _pad[2];
    char    name[64];
    char    purpose[128];
    char    target[64];
    char    notes[256];
    uint8_t op_index;
    uint8_t _pad2[3];
};

struct __attribute__((packed)) PktMissionListReq {
    uint8_t _pad[4];
};

struct __attribute__((packed)) PktMissionDelete {
    uint16_t year;          // e.g. 2026
    uint8_t  _pad[2];
    char     code[8];       // "A03"
    uint8_t  op_index;      // 누가 요청 (central이 채움)
    uint8_t  _pad2[3];
};

// ── Mission File Archive (Central-centric, station-keyed) ─────────────────
// Subdir codes (1 byte): 1=iq, 2=audio, 3=hist
static constexpr uint8_t MFS_IQ    = 1;
static constexpr uint8_t MFS_AUDIO = 2;
static constexpr uint8_t MFS_HIST  = 3;

// Mission station + dir + filename key. station[64] matches mission_station_name.
struct __attribute__((packed)) MissionFileKey {
    char     station[64];   // "DGS-1" 등 (mission_station_name과 동일 폭)
    uint16_t year;
    uint8_t  subdir;        // MFS_*
    uint8_t  _pad;
    char     code[8];       // "A03"
    char     filename[128]; // basename only (경로 분리자 금지)
};

// HOST → Central: 파일 업로드 시작
// total_bytes==0 이면 활성 미션 중 real-time append (close 시점 미정)
struct __attribute__((packed)) PktMissionFilePushMeta {
    MissionFileKey key;
    uint64_t       total_bytes;     // 0 = unknown / streaming
    uint8_t        transfer_id;     // HOST가 부여 (room 내 unique)
    uint8_t        mode;            // 0=replace (truncate), 1=append (offset 따름)
    uint8_t        _pad[2];
    char           info_data[512];  // .info 사이드카 (없으면 빈)
};

// HOST → Central: 청크 (offset append 지원)
struct __attribute__((packed)) PktMissionFilePushData {
    uint8_t  transfer_id;
    uint8_t  is_last;       // 1: 마지막 청크 → Central은 파일 close 후 ack
    uint8_t  _pad[2];
    uint64_t offset;        // 목적 파일 내 쓰기 시작 위치
    uint32_t chunk_bytes;
    uint8_t  _pad2[4];      // 8-byte align before raw bytes
    // 뒤에 raw bytes [chunk_bytes]
};

// any → Central: 파일 목록 (station/year/code 필터)
struct __attribute__((packed)) PktMissionFileListReq {
    char     station[64];   // 빈 = 모든 station
    uint16_t year;          // 0 = 모든 year
    uint8_t  subdir;        // 0 = 모두 (iq+audio+hist)
    uint8_t  _pad;
    char     code[8];       // 빈 = 모든 code
};

static constexpr int MAX_MISSION_FILES_PER_PKT = 32;
struct __attribute__((packed)) MissionFileEntry {
    char     station[64];
    uint16_t year;
    uint8_t  subdir;
    uint8_t  _pad;
    char     code[8];
    char     filename[128];
    uint64_t size_bytes;
    int64_t  mtime_unix;
};

// Central → caller: 파일 목록 응답 (count > MAX → 다중 패킷)
struct __attribute__((packed)) PktMissionFileList {
    uint16_t           count;              // entries[] 유효 개수 (≤ MAX_*)
    uint8_t            is_last_page;       // 1: 더 없음
    uint8_t            _pad;
    MissionFileEntry   entries[MAX_MISSION_FILES_PER_PKT];
};

// any → Central: 다운로드 요청
// start_offset: 0 = 처음부터, n = n byte 이후부터 stream (resume 다운로드).
// LIVE HIST 같이 자라는 파일 이어받기.
struct __attribute__((packed)) PktMissionFileDlReq {
    MissionFileKey key;
    uint64_t       start_offset;
};

// Central → caller: 다운로드 청크 (첫 청크에 .info 동봉)
struct __attribute__((packed)) PktMissionFileDlData {
    MissionFileKey key;
    uint64_t       total_bytes;
    uint64_t       offset;
    uint32_t       chunk_bytes;
    uint8_t        is_first;        // 1: info_data 유효
    uint8_t        is_last;
    uint8_t        _pad[2];
    char           info_data[512];  // is_first 일 때만
    // 뒤에 raw bytes [chunk_bytes]
};

// any → Central: 파일 삭제
struct __attribute__((packed)) PktMissionFileDelete {
    MissionFileKey key;
};

// any → Central: 파일 이름 변경 (subdir 내에서만)
struct __attribute__((packed)) PktMissionFileRename {
    MissionFileKey key;
    char           new_filename[128];
};

// Central → HOST: PUSH 완료 (or 실패) ACK
//   status: 0=ok (file fully committed), 1=io error, 2=protocol error, 3=key invalid
struct __attribute__((packed)) PktMissionFilePushAck {
    MissionFileKey key;
    uint8_t        transfer_id;
    uint8_t        status;
    uint8_t        _pad[2];
    uint64_t       total_bytes;     // 실제 디스크에 쓰인 바이트
    char           error_msg[64];
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
// On -1, errno is set: real recv() errno on syscall failure, or ECONNRESET on peer-close
// (r==0). Without this, callers would see stale errno from earlier syscalls (e.g. mkdir
// EEXIST=17), producing misleading "(File exists)" diagnostics.
inline int recv_all_ex(int fd, void* buf, size_t len, const std::atomic<bool>& alive){
    uint8_t* p = static_cast<uint8_t*>(buf);
    size_t remaining = len;
    while(remaining > 0){
        if(!alive.load(std::memory_order_relaxed)){ errno = ECONNRESET; return -1; }
        ssize_t r = recv(fd, p, remaining, 0);
        if(r > 0){ p += r; remaining -= r; continue; }
        if(r == 0){ errno = ECONNRESET; return -1; }  // peer closed
        if(errno == EAGAIN || errno == EWOULDBLOCK){
            if(remaining == len) return 0;             // nothing read yet → timeout
            continue;                                  // partial read → retry
        }
        return -1;                                     // real error (errno already set)
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

