#pragma once
#include "central_proto.hpp"
#include "../src/net_protocol.hpp"  // PktBandEntry/PktBandPlan/PktBandRemove
#include "emitter_db.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <deque>
#include <string>
#include <memory>
#include <chrono>
#include <condition_variable>
#include <unordered_map>
#include <cmath>
#include <cstdio>
#include <cstring>

// ── Mission File Archive (Phase 1) ─────────────────────────────────────────
// In-flight HOST → Central file transfer state (per transfer_id).
struct MissionFileTransfer {
    MissionFileKey key{};
    FILE*       fp = nullptr;
    uint64_t    expected_bytes = 0;  // 0 = streaming (size unknown)
    uint64_t    written_bytes  = 0;  // monotonically increasing
    uint64_t    high_water     = 0;  // max(offset + chunk) seen
    std::string archive_path;        // absolute path on Central disk
    std::string info_data;           // captured at PUSH_META, written as .info sidecar at close
};

// In-flight Central → JOIN download (server-side, just temporary streaming state).
// (Most state lives on the JOIN side; Central just sends chunks.)

// Live HIST stream tap (one per active LWF file per HOST room).
struct MissionHistStream {
    std::string archive_path;
    FILE*       fp = nullptr;
    // captured at LWF_LIVE_START
    char        station[64] = {};
    uint16_t    year = 0;
    char        code[8] = {};
    uint32_t    fft_size = 0;       // row size in bytes (each row = fft_size float? NO — row is uint8 per col)
                                    // 실제는 행 크기 = fft_size (8-bit packed dB). open_new_file에서 row_bytes = fft_size.
    uint32_t    rows_written = 0;
};

struct JoinEntry {
    uint16_t          conn_id = 0;
    int               fd      = -1;
    std::atomic<bool> alive{true};
    std::thread       thr;      // JOIN→HOST recv 스레드

    // ── per-JOIN 상태 ──────────────────────────────────────────────────
    // 뮤트 테이블: true = 이 JOIN이 해당 채널 오디오를 수신함
    // 초기값 true: JOIN이 명시적으로 뮤트(false)할 때만 끔
    bool     recv_audio[MAX_CHANNELS_RELAY] = {true,true,true,true,true,true,true,true,true,true};
    // 인증 정보 (릴레이가 관리)
    char     name[32]  = {};
    uint8_t  tier      = 0;
    uint8_t  op_index  = 0;   // 1-based (릴레이가 할당)
    bool     authed    = false;

    // ── DB save 수신 상태 ─────────────────────────────────────────────────
    FILE*       db_fp   = nullptr;
    std::string db_path;

    // 진행 중인 file 전송 수 (FILE_META에서 ++, FILE_DATA(is_last=1)에서 --).
    // > 0 이면 dispatch_to_joins가 FFT_FRAME을 이 JOIN에 보내지 않음 (HB는 계속 보냄).
    std::atomic<int> active_file_transfers{0};

    // P 키로 JOIN이 FFT 수신 일시정지 요청. dispatch_to_joins가 FFT_FRAME만 스킵.
    // audio/HB/CMD/STATUS 등 다른 트래픽은 그대로.
    std::atomic<bool> fft_paused{false};

    // ── 독립 송신 큐 ──────────────────────────────────────────────────────
    // 우선순위: ctrl_queue > file_queue > send_queue(FFT) > audio_queue
    // 단일 send 스레드가 우선순위 순서로 큐에서 꺼내 전송
    static constexpr size_t SEND_QUEUE_MAX_BYTES  = 2 * 1024 * 1024; // FFT 2MB (~0.5초)
    static constexpr size_t AUDIO_QUEUE_MAX_BYTES = 512 * 1024;      // 오디오 512KB (~0.5초)
    // FILE 큐: 한도 초과 시 enqueue가 BLOCK 되어 host_mux_loop을 통해 HOST까지 backpressure 전파.
    // 16MB로 키워 cascade backpressure 빈도 줄이고 다운로드 속도 향상.
    static constexpr size_t FILE_QUEUE_MAX_BYTES  = 16 * 1024 * 1024; // 16MB (~64 chunk × 256KB)

    // 제어 큐 (AUTH_ACK, CMD_ACK, STATUS, OP_LIST, CH_SYNC 등) — 드롭 없음
    std::deque<std::vector<uint8_t>> ctrl_queue;
    // FILE 큐 (FILE_META 0x0E, FILE_DATA 0x0D) — 드롭 없음, 한도 초과 시 enqueue BLOCK
    std::deque<std::vector<uint8_t>> file_queue;
    size_t                  file_queue_bytes = 0;
    // FFT 큐
    std::deque<std::vector<uint8_t>> send_queue;
    size_t                  send_queue_bytes = 0;
    // 오디오 큐
    std::deque<std::vector<uint8_t>> audio_queue;
    size_t                  audio_queue_bytes = 0;

    std::mutex              send_mtx;   // 모든 큐 공유 lock
    std::condition_variable send_cv;
    std::condition_variable file_drain_cv; // file_queue 소진 시 enqueue 깨움
    std::thread             send_thr;

    std::atomic<bool>       send_stop{false};
    std::mutex              fd_write_mtx;  // fd write 직렬화

    void send_raw(const std::vector<uint8_t>& pkt){
        if(fd < 0 || !alive.load()) return;
        std::lock_guard<std::mutex> wlk(fd_write_mtx);
        const uint8_t* p = pkt.data();
        size_t rem = pkt.size();
        uint8_t bewe_t = (pkt.size() >= 5) ? pkt[4] : 0xFF;
        while(rem > 0){
            ssize_t r = ::send(fd, p, rem, MSG_NOSIGNAL);
            if(r <= 0){
                printf("[JoinEntry] send_raw FAIL conn_id=%u bewe_type=0x%02x total=%zu r=%zd errno=%d(%s)\n",
                       conn_id, bewe_t, pkt.size(), r, errno, strerror(errno));
                alive.store(false);
                file_drain_cv.notify_all(); // blocked enqueue_file 깨움
                break;
            }
            p += r; rem -= r;
        }
        if(bewe_t == 0x02)
            printf("[JoinEntry] send_raw AUTH_ACK conn_id=%u size=%zu OK\n", conn_id, pkt.size());
    }

    void start_send_worker(){
        // 단일 스레드: ctrl → FFT → 오디오 우선순위 순서로 배치 전송
        send_thr = std::thread([this](){
            while(true){
                std::vector<std::vector<uint8_t>> batch;
                batch.reserve(32);
                {
                    std::unique_lock<std::mutex> lk(send_mtx);
                    send_cv.wait(lk, [this]{
                        return !ctrl_queue.empty() || !file_queue.empty() ||
                               !send_queue.empty() || !audio_queue.empty() ||
                               send_stop.load();
                    });
                    if(send_stop.load() && ctrl_queue.empty() && file_queue.empty() &&
                       send_queue.empty() && audio_queue.empty()) break;
                    // 제어 패킷이 있으면 제어만 먼저 전송 (FFT/오디오와 절대 혼합 금지)
                    // → AUTH_ACK가 FFT보다 항상 먼저 JOIN에 도달 보장
                    if(!ctrl_queue.empty()){
                        while(!ctrl_queue.empty()){
                            batch.push_back(std::move(ctrl_queue.front()));
                            ctrl_queue.pop_front();
                        }
                    } else {
                        // FILE 청크 최대 4개/라운드 (1MB) — drain 속도 ↑, drain 시 enqueue 깨움.
                        // FFT는 v1.5.15부터 다운로드 중 JOIN에 안 보내므로 파일에 더 양보 가능.
                        int nf = 0;
                        while(!file_queue.empty() && nf++ < 4){
                            size_t sz = file_queue.front().size();
                            batch.push_back(std::move(file_queue.front()));
                            file_queue.pop_front();
                            if(file_queue_bytes >= sz) file_queue_bytes -= sz;
                            else file_queue_bytes = 0;
                        }
                        if(nf > 0) file_drain_cv.notify_one();
                        // FFT 최대 4개 (burst 완화)
                        int n = 0;
                        while(!send_queue.empty() && n++ < 4){
                            size_t sz = send_queue.front().size();
                            batch.push_back(std::move(send_queue.front()));
                            send_queue.pop_front();
                            if(send_queue_bytes >= sz) send_queue_bytes -= sz;
                            else send_queue_bytes = 0;
                        }
                        // 오디오 최대 8개 (burst 완화)
                        n = 0;
                        while(!audio_queue.empty() && n++ < 8){
                            size_t sz = audio_queue.front().size();
                            batch.push_back(std::move(audio_queue.front()));
                            audio_queue.pop_front();
                            if(audio_queue_bytes >= sz) audio_queue_bytes -= sz;
                            else audio_queue_bytes = 0;
                        }
                    }
                }
                for(auto& pkt : batch) send_raw(pkt);
            }
        });
    }

    void stop_send_worker(){
        send_stop.store(true);
        send_cv.notify_all();
        file_drain_cv.notify_all(); // blocked enqueue_file 깨움
        if(send_thr.joinable()) send_thr.join();
    }

    // 제어 패킷 큐에 push (AUTH_ACK, CMD_ACK, STATUS, OP_LIST, CH_SYNC 등)
    // 드롭 없음, FFT보다 항상 먼저 전송
    void enqueue_ctrl(const uint8_t* data, size_t len){
        std::lock_guard<std::mutex> lk(send_mtx);
        if(len >= 5 && data[4] == 0x02)
            printf("[JoinEntry] enqueue_ctrl AUTH_ACK conn_id=%u\n", conn_id);
        if(len >= 5 && data[4] == 0x20)
            printf("[JoinEntry] enqueue_ctrl IQ_CHUNK conn_id=%u len=%zu\n", conn_id, len);
        ctrl_queue.emplace_back(data, data + len);
        send_cv.notify_one();
    }

    // FFT 큐에 push (바이트 기준 32MB 제한)
    void enqueue_data(const uint8_t* data, size_t len){
        std::lock_guard<std::mutex> lk(send_mtx);
        // 제어 패킷은 enqueue_ctrl로 보내야 함 — 여기선 FFT만
        while(send_queue_bytes + len > SEND_QUEUE_MAX_BYTES && !send_queue.empty()){
            send_queue_bytes -= send_queue.front().size();
            send_queue.pop_front();
        }
        send_queue.emplace_back(data, data + len);
        send_queue_bytes += len;
        send_cv.notify_one();
    }

    // 오디오 큐에 push (바이트 기준 16MB 제한)
    void enqueue_audio(const uint8_t* data, size_t len){
        std::lock_guard<std::mutex> lk(send_mtx);
        while(audio_queue_bytes + len > AUDIO_QUEUE_MAX_BYTES && !audio_queue.empty()){
            audio_queue_bytes -= audio_queue.front().size();
            audio_queue.pop_front();
        }
        audio_queue.emplace_back(data, data + len);
        audio_queue_bytes += len;
        send_cv.notify_one();
    }

    // FILE 큐에 push (드롭 없음, 한도 초과 시 BLOCK).
    // 호출자: dispatch_to_joins. 블로킹이 host_mux_loop을 막아 HOST send까지 backpressure 전파.
    // joins_mtx를 들고 있는 동안 호출하면 안 됨 — 호출 측에서 snapshot 패턴으로 lock 해제 후 호출.
    void enqueue_file(const uint8_t* data, size_t len){
        std::unique_lock<std::mutex> lk(send_mtx);
        // 빈 큐일 땐 한도 무관 통과 (단일 chunk가 한도보다 커도 보낼 수 있도록)
        file_drain_cv.wait(lk, [this, len]{
            return !alive.load() || send_stop.load() ||
                   file_queue.empty() ||
                   file_queue_bytes + len <= FILE_QUEUE_MAX_BYTES;
        });
        if(!alive.load() || send_stop.load()) return;
        file_queue.emplace_back(data, data + len);
        file_queue_bytes += len;
        send_cv.notify_one();
    }
};

struct HostRoom {
    std::string               station_id;
    CentralStation              info;
    int                       fd = -1;       // HOST ctrl+mux 소켓
    std::atomic<bool>         alive{true};
    std::atomic<bool>         resetting{false};  // chassis 2 reset: LIST에서 제외
    std::chrono::steady_clock::time_point last_hb;

    mutable std::mutex                    host_send_mtx; // HOST fd write 직렬화
    // HOST fd 송신 큐: join_loop 등이 HOST에 보낼 데이터를 여기에 넣고,
    // host_mux_loop이 recv 루프에서 매번 flush → blocking send 없이 안전
    std::deque<std::vector<uint8_t>>      host_send_queue;

    // ── HOST→Central DB 업로드 수신 상태 (룸당 단일 mux_loop 스레드, mutex 불필요) ─
    FILE*       db_fp   = nullptr;
    std::string db_path;

    mutable std::mutex                    joins_mtx;
    std::vector<std::shared_ptr<JoinEntry>> joins;
    uint16_t                              next_conn_id = 1;

    // ── 중앙 상태 캐시 (새 JOIN 접속 시 즉시 전송) ────────────────────
    uint32_t ch_audio_mask[MAX_CHANNELS_RELAY] = {};  // 서버 기준 audio_mask

    // 최신 패킷 캐시 (HOST가 보낸 원본 BEWE 패킷)
    std::mutex                cache_mtx;
    std::vector<uint8_t>      cached_heartbeat;
    std::vector<uint8_t>      cached_status;
    std::vector<uint8_t>      cached_ch_sync;
    std::vector<uint8_t>      cached_op_list;     // 릴레이가 빌드
    std::vector<uint8_t>      cached_sched_sync;  // 예약 리스트 (JSON에 영속화)
    std::vector<uint8_t>      cached_mission_sync; // 미션 스냅샷 (JSON에 영속화)

    // Status page v2 — host periodic state cache (CentralHostStateFull).
    // Updated whenever a HOST_STATE MUX message arrives. has_state_ false
    // until first HOST_STATE seen → LIST_RESP_V2 fields default to empty.
    std::mutex                state_mtx;
    bool                      has_state = false;
    CentralHostStateFull      state{};
    // Optional HIST live-recording snapshot carried as trailer after the
    // CentralHostStateFull in HOST_STATE. has_hist_info=false → no LIVE.
    bool                      has_hist_info = false;
    CentralHostHistInfo       hist_info{};

    // 인증/오퍼레이터 관리 (릴레이가 중앙 관리)
    uint8_t                   next_op_idx = 1;
    char                      host_name[32] = {};
    uint8_t                   host_tier = 1;

    // ── Mission File Archive (Phase 1, v3.8.0) ──────────────────────────────
    // HOST→Central 파일 push 진행 중인 transfer 들 (transfer_id → state).
    // host_mux_loop 단일 스레드만 접근 → mutex 불필요.
    std::unordered_map<uint8_t, MissionFileTransfer> mission_xfers;

    // LWF live stream tap → HIST archive 파일.
    // host_mux_loop가 LWF_LIVE_START에서 open, LWF_LIVE_ROW에서 append, LWF_LIVE_STOP에서 close.
    // key = filename (PktLwfLiveStart.filename) — basename only.
    std::unordered_map<std::string, MissionHistStream> hist_streams;

    // Active mission shadow (from MISSION_SYNC.active). LWF tap에서 archive 경로 결정용.
    bool     active_mission_valid = false;
    uint16_t active_mission_year  = 0;
    char     active_mission_code[8]    = {};
    char     active_mission_station[64] = {};
};

// HOST fd에 MUX 패킷 enqueue (non-blocking; host_mux_loop이 flush).
// inline: central_server.cpp + central_mission_archive.cpp 양쪽 TU에서 사용.
inline void enqueue_host_send(std::shared_ptr<HostRoom>& room, uint16_t conn_id,
                              CentralMuxType type, const void* data, uint32_t len){
    CentralMuxHdr mh{};
    mh.conn_id = conn_id;
    mh.type = static_cast<uint8_t>(type);
    mh.len = len;
    std::vector<uint8_t> pkt(CENTRAL_MUX_HDR_SIZE + len);
    memcpy(pkt.data(), &mh, CENTRAL_MUX_HDR_SIZE);
    if(len > 0 && data) memcpy(pkt.data() + CENTRAL_MUX_HDR_SIZE, data, len);
    std::lock_guard<std::mutex> lk(room->host_send_mtx);
    room->host_send_queue.push_back(std::move(pkt));
}

class CentralServer {
public:
    bool start(int port = CENTRAL_PORT);
    void run();
    void stop();

private:
    int               listen_fd_ = -1;
    std::atomic<bool> running_{false};

    mutable std::mutex                     rooms_mtx_;
    std::vector<std::shared_ptr<HostRoom>> rooms_;

    std::thread accept_thr_;
    std::thread watchdog_thr_;

    int  make_listen_sock(int port);
    void accept_loop();
    void watchdog_loop();
    void handshake(int fd);

    void host_mux_loop(std::shared_ptr<HostRoom> room);
    void join_loop(std::shared_ptr<JoinEntry> je, std::shared_ptr<HostRoom> room);

    void handle_list_req(int fd);
    void handle_list_req_v2(int fd);                 // status page v2
    void handle_station_detail_req(int fd, const CentralStationDetailReq& req);
    void list_poller_loop(int fd);  // persistent LIST_REQ polling (fd 닫지 않고 반복 응답)
    std::shared_ptr<HostRoom> find_room(const std::string& id) const;

    // ── BEWE 패킷 인터셉트 (중앙 제어) ──────────────────────────────────
    // HOST→JOIN 방향: 오디오 필터링, 상태 캐시, CHANNEL_SYNC 인터셉트
    void dispatch_to_joins(std::shared_ptr<HostRoom> room,
                           uint16_t conn_id,
                           const uint8_t* bewe_pkt, size_t bewe_len);

    // JOIN→HOST 방향: 릴레이가 처리할 패킷 인터셉트
    // 반환값: true=릴레이가 소비 (HOST에 포워드 안 함)
    bool intercept_join_cmd(std::shared_ptr<JoinEntry> je,
                            std::shared_ptr<HostRoom> room,
                            const uint8_t* bewe_pkt, size_t bewe_len);

    // 릴레이가 OPERATOR_LIST 빌드 + broadcast
    void build_and_broadcast_op_list(std::shared_ptr<HostRoom> room);

    // 릴레이가 CHANNEL_SYNC의 audio_mask를 재작성해서 broadcast
    // (recv_audio[] 테이블 기반으로 listener 정보 반영)
    // send_to_host: true=HOST에도 재작성 CH_SYNC 전송, false=JOIN에게만
    // host_mux_loop 경유 시 false (HOST fd에 blocking write → 데드락 방지)
    void rebuild_and_broadcast_ch_sync(std::shared_ptr<HostRoom> room, bool send_to_host = true);

    // DB 파일 목록 스캔 → 모든 JOIN + HOST에 브로드캐스트
    void broadcast_db_list(std::shared_ptr<HostRoom> room);

    // BEWE 패킷 빌드 헬퍼 (magic + type + len + payload)
    static std::vector<uint8_t> make_bewe_packet(uint8_t type, const void* payload, uint32_t plen);

    // ── Signal Library / Emitter DB ─────────────────────────────────────
    BeweCentral::EmitterDb emitter_db_;
    // 새 sighting을 emitter_db에 ingest (info_data 파싱 → Sighting → ingest_sighting).
    // 클라이언트는 EMITTER_LIST_REQ / SIGHTING_LIST_REQ로 Refresh 시점에 데이터 가져감.
    void ingest_report_to_emitter_db(const char* filename,
                                     const char* reporter,
                                     const char* info_data);
    // 한 JOIN 또는 HOST에게 응답 형태로 페이지 송신.
    void send_emitter_list_page(std::shared_ptr<JoinEntry> je,
                                std::shared_ptr<HostRoom> room,
                                uint16_t conn_id, uint16_t off, uint16_t lim);
    void send_sighting_list_page(std::shared_ptr<JoinEntry> je,
                                 std::shared_ptr<HostRoom> room,
                                 uint16_t conn_id,
                                 const std::string& euid_filter,
                                 uint16_t off, uint16_t lim);

    // ── Scheduled recording persistence ─────────────────────────────────
    // ~/BE_WE/DataBase/schedules.json 에 station_id 별 SCHED_SYNC 스냅샷 저장
    std::mutex              sched_json_mtx_;
    std::string             schedules_json_path_;
    // station_id → cached SCHED_SYNC payload (BEWE 헤더 포함)
    std::unordered_map<std::string, std::vector<uint8_t>> sched_by_station_;
    void load_schedules_from_json();
    void save_schedules_to_json();   // 모든 station 스냅샷을 JSON으로 덤프

    // ── Mission persistence (station_id → BEWE 패킷 캐시) ────────────────
    std::mutex                missions_json_mtx_;
    std::string               missions_json_path_;
    std::unordered_map<std::string, std::vector<uint8_t>> missions_by_station_;
    void load_missions_from_json();
    void save_missions_to_json();

    // (band plan: now host-owned. Central just relays BAND_PLAN_SYNC and BAND_ADD/UPDATE/REMOVE.)

    // 전역 채팅: 중앙서버에 접속한 모든 JOIN + 다른 방의 HOST에게 CHAT BEWE 패킷 전달
    // skip_host_room: 소스 방의 HOST는 제외 (이미 알고 있음)
    void broadcast_global_chat(const uint8_t* bewe_pkt, size_t bewe_len,
                               HostRoom* skip_host_room = nullptr);

    // ── Mission File Archive (Phase 1, v3.8.0) ──────────────────────────────
    // Archive root: ~/BE_WE/DataBase/missions/  (Central 머신의 $HOME/BE_WE/...)
    std::string archive_root() const;
    std::string archive_dir(const char* station, uint16_t year,
                            const char* code, uint8_t subdir) const;
    // station/year/code 디렉토리 통째 삭제 (MISSION_DELETE intercept에서 호출).
    void archive_wipe_mission(const char* station, uint16_t year, const char* code);

    // HOST→Central PUSH 처리 (host_mux_loop에서 호출)
    void handle_mission_file_push_meta(std::shared_ptr<HostRoom> room,
                                       const uint8_t* payload, size_t plen);
    void handle_mission_file_push_data(std::shared_ptr<HostRoom> room,
                                       const uint8_t* payload, size_t plen);
    // PUSH 완료 ACK 전송 (HOST에게)
    void send_push_ack(std::shared_ptr<HostRoom> room,
                       const MissionFileKey& key, uint8_t transfer_id,
                       uint8_t status, uint64_t total_bytes, const char* err);

    // any → Central: LIST/DL/DELETE/RENAME (intercept_join_cmd에서 호출 OR host_mux_loop)
    // requester == nullptr 이면 HOST가 요청한 것 (응답은 host_send_queue로).
    void handle_mission_file_list_req(std::shared_ptr<HostRoom> room,
                                      std::shared_ptr<JoinEntry> requester,
                                      const uint8_t* payload, size_t plen);
    void handle_mission_file_dl_req(std::shared_ptr<HostRoom> room,
                                    std::shared_ptr<JoinEntry> requester,
                                    const uint8_t* payload, size_t plen);
    void handle_mission_file_delete(std::shared_ptr<HostRoom> room,
                                    std::shared_ptr<JoinEntry> requester,
                                    const uint8_t* payload, size_t plen);
    void handle_mission_file_rename(std::shared_ptr<HostRoom> room,
                                    std::shared_ptr<JoinEntry> requester,
                                    const uint8_t* payload, size_t plen);

    // LWF live stream → HIST archive 탭 (host_mux_loop의 LWF_LIVE_* 분기에서 호출)
    void archive_hist_on_live_start(std::shared_ptr<HostRoom> room,
                                    const PktLwfLiveStart& ls);
    void archive_hist_on_live_row  (std::shared_ptr<HostRoom> room,
                                    const PktLwfLiveRowHdr& hdr,
                                    const uint8_t* row, uint32_t row_bytes);
    void archive_hist_on_live_stop (std::shared_ptr<HostRoom> room,
                                    const PktLwfLiveStop& stop);
    // MISSION_SYNC.active 변동 시 HostRoom shadow 갱신 (cached_mission_sync 업데이트 직후 호출)
    void update_active_mission_shadow(std::shared_ptr<HostRoom> room,
                                      const uint8_t* bewe_pkt, size_t bewe_len);
};
