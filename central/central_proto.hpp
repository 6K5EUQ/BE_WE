#pragma once
#include <cstdint>
#include <cstring>

// ── BEWE Relay Protocol v3 ─────────────────────────────────────────────────
//
// 단일 포트(7700). 첫 패킷으로 역할 구분:
//   HOST → relay : HOST_OPEN  (스테이션 등록)
//   JOIN → relay : JOIN_ROOM  (룸 입장)
//   any  → relay : LIST_REQ   (목록 조회 후 연결 종료)
//
// HOST_OPEN 이후: relay ↔ HOST 간 multiplexed 스트림
//   relay→HOST : MUX 헤더 + BEWE 패킷  (JOIN에서 온 데이터)
//   HOST→relay : MUX 헤더 + BEWE 패킷  (JOIN에게 보낼 데이터, conn_id=0xFFFF이면 broadcast)
//
// JOIN_ROOM 이후: relay ↔ JOIN 간 투명 BEWE 스트림 (MUX 없음)
//   JOIN→relay  : BEWE 패킷  → relay가 MUX 붙여서 HOST로 forward
//   relay→JOIN  : BEWE 패킷  (HOST가 보낸 데이터에서 MUX 제거)

static constexpr uint8_t CENTRAL_MAGIC[4] = {'B','R','L','Y'};
static constexpr int      CENTRAL_PORT    = 7700;
static constexpr int      CENTRAL_PIPE_PORT = 7702; // IQ 파일 전송 전용 포트

// ── 파이프 핸드셰이크 (7701 포트) ─────────────────────────────────────────
// HOST가 먼저 연결 후 PIPE_HOST 전송, JOIN이 나중에 PIPE_JOIN 전송
// 중앙서버가 같은 req_id를 가진 HOST+JOIN 쌍을 연결

enum class PipePktType : uint8_t {
    PIPE_HOST  = 0x01,  // HOST → 중앙서버: 파일 전송 준비 완료
    PIPE_JOIN  = 0x02,  // JOIN → 중앙서버: 파일 수신 준비
    PIPE_READY = 0x03,  // 중앙서버 → JOIN(MUX): 파이프 연결됨, 접속하라
    PIPE_ERROR = 0xFF,
};

struct __attribute__((packed)) PipePktHdr {
    uint8_t  magic[4];   // 'P','I','P','E'
    uint8_t  type;       // PipePktType
    uint32_t len;
};
static constexpr uint8_t PIPE_MAGIC[4] = {'P','I','P','E'};
static constexpr int PIPE_HDR_SIZE = sizeof(PipePktHdr);

// PIPE_HOST payload: HOST가 전송할 파일 정보를 중앙서버에 알림
struct __attribute__((packed)) PipePktHost {
    char     station_id[32];
    uint32_t req_id;       // 고유 요청 ID (HOST가 생성)
    uint16_t target_conn_id; // 파일 받을 JOIN의 conn_id
    char     filename[128];
    uint64_t filesize;
};

// PIPE_JOIN payload: JOIN이 파이프에 연결 요청
struct __attribute__((packed)) PipePktJoin {
    uint32_t req_id;     // HOST가 알려준 req_id
};

// MUX DATA로 JOIN에게 전달: IQ_CHUNK (net_protocol.hpp의 PktIqChunkHdr 사용)
static constexpr uint8_t BEWE_TYPE_IQ_CHUNK = 0x20;

// BEWE_TYPE_IQ_PROGRESS: 전송 진행 브로드캐스트 (net_protocol.hpp의 PktIqProgress 사용)
static constexpr uint8_t BEWE_TYPE_IQ_PROGRESS = 0x21;

// ── 핸드셰이크 패킷 헤더 ──────────────────────────────────────────────────
struct __attribute__((packed)) CentralPktHdr {
    uint8_t  magic[4];
    uint8_t  type;
    uint32_t len;
};
static constexpr int CENTRAL_HDR_SIZE = sizeof(CentralPktHdr);

enum class CentralPktType : uint8_t {
    HOST_OPEN = 0x01,
    HOST_HB   = 0x02,
    JOIN_ROOM = 0x10,
    LIST_REQ  = 0x20,
    LIST_RESP = 0x21,
    ERROR     = 0xFF,
};

// ── HOST_OPEN payload ──────────────────────────────────────────────────────
struct __attribute__((packed)) CentralHostOpen {
    char    station_id[32];
    char    station_name[64];
    float   lat, lon;
    uint8_t host_tier;
    uint8_t user_count;
    uint8_t _pad[2];
};

// ── HOST_HB payload ────────────────────────────────────────────────────────
struct __attribute__((packed)) CentralHostHb {
    uint8_t user_count;
};

// ── JOIN_ROOM payload ──────────────────────────────────────────────────────
struct __attribute__((packed)) CentralJoinRoom {
    char station_id[32];
};

// ── MUX 헤더 (HOST_OPEN 핸드셰이크 이후 relay↔HOST 간 모든 패킷 앞에 붙음) ─
// relay→HOST: JOIN에서 받은 BEWE 패킷 앞에 conn_id 붙여서 전달
// HOST→relay: 보낼 BEWE 패킷 앞에 conn_id(0xFFFF=broadcast) 붙여서 전달
struct __attribute__((packed)) CentralMuxHdr {
    uint16_t conn_id;    // JOIN 식별자 (relay가 할당, 0xFFFF=broadcast)
    uint8_t  type;       // CentralMuxType
    uint32_t len;        // 뒤따르는 BEWE 패킷 길이 (0이면 데이터 없음)
};
static constexpr int CENTRAL_MUX_HDR_SIZE = sizeof(CentralMuxHdr);

enum class CentralMuxType : uint8_t {
    DATA       = 0x01,  // BEWE 패킷 데이터
    CONN_OPEN  = 0x02,  // 새 JOIN 연결됨 (len=0)
    CONN_CLOSE = 0x03,  // JOIN 연결 끊김 (len=0)
    NET_RESET  = 0x04,  // 네트워크 리셋 (len=1: 0=reset, 1=open)
};

// ── BEWE 패킷 타입 상수 (릴레이가 내부 파싱에 사용) ─────────────────────
// BEWE 패킷 형식: magic[4] + type[1] + len[4] + payload
static constexpr int    BEWE_HDR_SIZE       = 9;
static constexpr uint8_t BEWE_TYPE_AUTH_REQ  = 0x01;
static constexpr uint8_t BEWE_TYPE_AUTH_ACK  = 0x02;
static constexpr uint8_t BEWE_TYPE_FFT      = 0x03;
static constexpr uint8_t BEWE_TYPE_AUDIO    = 0x04;
static constexpr uint8_t BEWE_TYPE_CMD      = 0x05;
static constexpr uint8_t BEWE_TYPE_STATUS   = 0x08;
static constexpr uint8_t BEWE_TYPE_OP_LIST  = 0x09;
static constexpr uint8_t BEWE_TYPE_CH_SYNC  = 0x0A;
static constexpr uint8_t BEWE_TYPE_HEARTBEAT = 0x14;

static constexpr uint8_t BEWE_TYPE_CHAT     = 0x07;

// AUTH_REQ payload: id[32] + pw[64] + tier[1] = 97 bytes
static constexpr int BEWE_AUTH_REQ_SIZE = 97;
// AUTH_ACK payload: ok[1] + op_index[1] + reason[48] = 50 bytes
static constexpr int BEWE_AUTH_ACK_SIZE = 50;

// OPERATOR_LIST: count[1] + OpEntry[16]
// OpEntry: index[1] + tier[1] + name[32] = 34 bytes
static constexpr int BEWE_OP_ENTRY_SIZE = 34;
static constexpr int BEWE_MAX_OPERATORS = 16;

// CMD subtypes (BEWE payload offset 0 = cmd byte)
static constexpr uint8_t BEWE_CMD_TOGGLE_RECV = 0x0C;

// AUDIO_FRAME header: ch_idx[1] + pan[1] + n_samples[4]
// → ch_idx is at BEWE payload offset 0
static constexpr int BEWE_AUDIO_HDR_SIZE = 6;

// CHANNEL_SYNC: ChSyncEntry[10], each 72 bytes
// ChSyncEntry layout: idx[1] active[1] s[4] e[4] mode[1] pan[1] audio_mask[4] ...
// + iq_rec_secs[4] audio_rec_secs[4] iq_rec_on[1] audio_rec_on[1] pad[2]
static constexpr int CH_SYNC_ENTRY_SIZE = 72;
static constexpr int CH_SYNC_MASK_OFFSET = 12;  // audio_mask offset within ChSyncEntry
static constexpr int CH_SYNC_OWNER_OFFSET = 28; // owner_name offset within ChSyncEntry
static constexpr int MAX_CHANNELS_RELAY = 10;

// ── 스테이션 목록 ─────────────────────────────────────────────────────────
struct __attribute__((packed)) CentralStation {
    char    station_id[32];
    char    station_name[64];
    float   lat, lon;
    uint8_t host_tier;
    uint8_t user_count;
    uint8_t _pad[2];
};

// relay 서버의 LAN IP 목록 (LIST_RESP에 첨부)
// 같은 망 클라이언트가 LAN IP로 직접 접속할 수 있도록
static constexpr int CENTRAL_MAX_LAN_IPS = 8;
struct __attribute__((packed)) CentralListResp {
    uint16_t count;
    uint8_t  lan_ip_count;                          // LAN IP 개수 (0~CENTRAL_MAX_LAN_IPS)
    char     lan_ips[CENTRAL_MAX_LAN_IPS][16];        // IPv4 문자열 (최대 "255.255.255.255\0")
    // CentralStation[count] follows
};

struct __attribute__((packed)) CentralError {
    char msg[64];
};

// ── Wire helpers ───────────────────────────────────────────────────────────
#include <vector>
#include <cerrno>
#include <sys/socket.h>
#include <unistd.h>

inline bool central_send_all(int fd, const void* buf, size_t len){
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    while(len > 0){
        ssize_t r = send(fd, p, len, MSG_NOSIGNAL);
        if(r < 0){
            if(errno == EINTR) continue;
            return false;  // EAGAIN(timeout) 등 → 호출자가 판단
        }
        if(r == 0) return false;
        p += r; len -= r;
    }
    return true;
}

inline bool central_recv_all(int fd, void* buf, size_t len){
    uint8_t* p = static_cast<uint8_t*>(buf);
    while(len > 0){
        ssize_t r = recv(fd, p, len, 0);
        if(r <= 0) return false;
        p += r; len -= r;
    }
    return true;
}

inline bool central_send_pkt(int fd, CentralPktType type,
                            const void* payload, uint32_t plen){
    CentralPktHdr hdr{};
    memcpy(hdr.magic, CENTRAL_MAGIC, 4);
    hdr.type = static_cast<uint8_t>(type);
    hdr.len  = plen;
    if(!central_send_all(fd, &hdr, sizeof(hdr))) return false;
    if(plen && payload) return central_send_all(fd, payload, plen);
    return true;
}

inline bool central_recv_pkt(int fd, CentralPktHdr& hdr, std::vector<uint8_t>& payload,
                            uint32_t max_payload = 65536){
    if(!central_recv_all(fd, &hdr, CENTRAL_HDR_SIZE)) return false;
    if(memcmp(hdr.magic, CENTRAL_MAGIC, 4) != 0) return false;
    if(hdr.len > max_payload) return false;
    payload.resize(hdr.len);
    if(hdr.len > 0 && !central_recv_all(fd, payload.data(), hdr.len)) return false;
    return true;
}

// MUX 헤더 전송
inline bool central_send_mux(int fd, uint16_t conn_id, CentralMuxType type,
                            const void* data, uint32_t len){
    CentralMuxHdr mux{};
    mux.conn_id = conn_id;
    mux.type    = static_cast<uint8_t>(type);
    mux.len     = len;
    if(!central_send_all(fd, &mux, CENTRAL_MUX_HDR_SIZE)) return false;
    if(len && data) return central_send_all(fd, data, len);
    return true;
}

// ── 파이프 Wire helpers (central_send_all/recv_all 이후에 정의) ───────────
inline bool pipe_send_pkt(int fd, PipePktType type, const void* payload, uint32_t plen){
    PipePktHdr hdr{};
    memcpy(hdr.magic, PIPE_MAGIC, 4);
    hdr.type = static_cast<uint8_t>(type);
    hdr.len  = plen;
    if(!central_send_all(fd, &hdr, sizeof(hdr))) return false;
    if(plen && payload) return central_send_all(fd, payload, plen);
    return true;
}

inline bool pipe_recv_pkt(int fd, PipePktHdr& hdr, std::vector<uint8_t>& payload,
                          uint32_t max_payload = 65536){
    if(!central_recv_all(fd, &hdr, PIPE_HDR_SIZE)) return false;
    if(memcmp(hdr.magic, PIPE_MAGIC, 4) != 0) return false;
    if(hdr.len > max_payload) return false;
    payload.resize(hdr.len);
    if(hdr.len > 0 && !central_recv_all(fd, payload.data(), hdr.len)) return false;
    return true;
}
