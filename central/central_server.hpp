#pragma once
#include "central_proto.hpp"
#include "../src/net_protocol.hpp"  // PktBandEntry/PktBandPlan/PktBandRemove
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

    // ── 독립 송신 큐 ──────────────────────────────────────────────────────
    // 우선순위: ctrl_queue(제어) > send_queue(FFT) > audio_queue(오디오)
    // 단일 send 스레드가 우선순위 순서로 큐에서 꺼내 전송
    static constexpr size_t SEND_QUEUE_MAX_BYTES  = 2 * 1024 * 1024; // FFT 2MB (~0.5초)
    static constexpr size_t AUDIO_QUEUE_MAX_BYTES = 512 * 1024;      // 오디오 512KB (~0.5초)

    // 제어 큐 (AUTH_ACK, CMD_ACK, STATUS, OP_LIST, CH_SYNC 등) — 드롭 없음
    std::deque<std::vector<uint8_t>> ctrl_queue;
    // FFT 큐
    std::deque<std::vector<uint8_t>> send_queue;
    size_t                  send_queue_bytes = 0;
    // 오디오 큐
    std::deque<std::vector<uint8_t>> audio_queue;
    size_t                  audio_queue_bytes = 0;

    std::mutex              send_mtx;   // 모든 큐 공유 lock
    std::condition_variable send_cv;
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
                alive.store(false); break;
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
                        return !ctrl_queue.empty() || !send_queue.empty() ||
                               !audio_queue.empty() || send_stop.load();
                    });
                    if(send_stop.load() && ctrl_queue.empty() &&
                       send_queue.empty() && audio_queue.empty()) break;
                    // 제어 패킷이 있으면 제어만 먼저 전송 (FFT/오디오와 절대 혼합 금지)
                    // → AUTH_ACK가 FFT보다 항상 먼저 JOIN에 도달 보장
                    if(!ctrl_queue.empty()){
                        while(!ctrl_queue.empty()){
                            batch.push_back(std::move(ctrl_queue.front()));
                            ctrl_queue.pop_front();
                        }
                    } else {
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

    // 인증/오퍼레이터 관리 (릴레이가 중앙 관리)
    uint8_t                   next_op_idx = 1;
    char                      host_name[32] = {};
    uint8_t                   host_tier = 1;
};

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
    void broadcast_report_list_central(std::shared_ptr<HostRoom> room);

    // BEWE 패킷 빌드 헬퍼 (magic + type + len + payload)
    static std::vector<uint8_t> make_bewe_packet(uint8_t type, const void* payload, uint32_t plen);

    // ── Scheduled recording persistence ─────────────────────────────────
    // ~/BE_WE/DataBase/schedules.json 에 station_id 별 SCHED_SYNC 스냅샷 저장
    std::mutex              sched_json_mtx_;
    std::string             schedules_json_path_;
    // station_id → cached SCHED_SYNC payload (BEWE 헤더 포함)
    std::unordered_map<std::string, std::vector<uint8_t>> sched_by_station_;
    void load_schedules_from_json();
    void save_schedules_to_json();   // 모든 station 스냅샷을 JSON으로 덤프

    // (band plan: now host-owned. Central just relays BAND_PLAN_SYNC and BAND_ADD/UPDATE/REMOVE.)

    // 전역 채팅: 중앙서버에 접속한 모든 JOIN + 다른 방의 HOST에게 CHAT BEWE 패킷 전달
    // skip_host_room: 소스 방의 HOST는 제외 (이미 알고 있음)
    void broadcast_global_chat(const uint8_t* bewe_pkt, size_t bewe_len,
                               HostRoom* skip_host_room = nullptr);
};
