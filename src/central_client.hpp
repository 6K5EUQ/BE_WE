#pragma once
#include "../central/central_proto.hpp"
#include "net_protocol.hpp"
#include <string>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <memory>
#include <unordered_map>

// ── CentralClient ────────────────────────────────────────────────────────────
// HOST 모드:
//   open_room() → relay에 연결, HOST_OPEN 전송 → central_fd 반환
//   이후 relay_fd는 MUX 스트림: JOIN패킷이 [MuxHdr+BEWE]로 들어오고
//   HOST→JOIN 전송도 [MuxHdr+BEWE]로 내보냄
//   NetServer 대신 RelayNetAdapter가 이 fd를 읽어 각 JOIN 가상소켓에 라우팅
//
// JOIN 모드:
//   join_room() → relay에 연결, JOIN_ROOM 전송 → central_fd 반환
//   이후 relay_fd는 투명 BEWE 스트림 → NetClient::connect_fd()에 전달
//
// 목록 조회:
//   fetch_stations() / start_polling()

class CentralClient {
public:
    ~CentralClient(){
        stop_polling();
        stop_mux_adapter();
    }

    struct Station {
        std::string station_id;
        std::string name;
        float       lat = 0, lon = 0;
        uint8_t     host_tier  = 1;
        uint8_t     user_count = 0;
    };

    // relay 서버의 LAN IP 캐시 (LIST_RESP에서 수신)
    // 같은 망에 있으면 LAN IP로 직접 접속 가능
    std::vector<std::string> central_lan_ips;
    std::mutex               central_lan_ips_mtx;

    // ── 목록 조회 ─────────────────────────────────────────────────────────
    std::vector<Station> fetch_stations(const std::string& host, int port);
    void start_polling(const std::string& host, int port,
                       std::function<void(const std::vector<Station>&)> cb);
    void stop_polling();

    // ── HOST 모드 ─────────────────────────────────────────────────────────
    // 성공 시 central_fd(MUX 스트림) 반환, 실패 시 -1
    int open_room(const std::string& central_host, int central_port,
                  const std::string& station_id, const std::string& station_name,
                  float lat, float lon, uint8_t host_tier);

    // MUX 스트림을 읽어 각 JOIN의 가상 socketpair에 라우팅하는 어댑터 시작
    // NetServer는 accept_fd(je->local_fd)로 JOIN 연결을 수락함
    // on_new_join(local_fd): 새 JOIN 연결 시 호출 → NetServer에 inject
    void start_mux_adapter(int central_fd,
                           std::function<void(int local_fd)> on_new_join,
                           std::function<uint8_t()> user_count_fn,
                           std::function<void()> on_disconnect = nullptr);
    void stop_mux_adapter();
    bool is_central_connected() const { return mux_running_.load(); }
    size_t queue_bytes() const { return central_queue_bytes_; }

    // HOST→중앙서버 broadcast (conn_id=0xFFFF, 1회 전송 → 중앙서버가 N명에게 fan-out)
    // N× 대역폭 문제 해결: 기존 per-JOIN socketpair 경유 방식 대체
    // no_drop=true: IQ_CHUNK 등 드롭하면 안 되는 패킷
    void enqueue_relay_broadcast(const uint8_t* bewe_pkt, size_t bewe_len,
                                 bool no_drop = false){
        if(!central_sender_running_.load()) return;
        CentralMuxHdr mh{}; mh.conn_id = 0xFFFF;
        mh.type = static_cast<uint8_t>(CentralMuxType::DATA);
        mh.len  = (uint32_t)bewe_len;
        enqueue_central(&mh, CENTRAL_MUX_HDR_SIZE, bewe_pkt, bewe_len, no_drop);
    }

    // 중앙서버→HOST 방향 전역 채팅 수신 콜백 설정
    void set_on_central_chat(std::function<void(const char* from, const char* msg)> cb){
        on_central_chat_ = std::move(cb);
    }

    // relay에 NET_RESET 신호 전송 (0=reset start, 1=open)
    void send_net_reset(uint8_t flag){
        if(!central_sender_running_.load()) return;
        CentralMuxHdr mh{}; mh.conn_id = 0xFFFF;
        mh.type = static_cast<uint8_t>(CentralMuxType::NET_RESET);
        mh.len = 1;
        enqueue_central(&mh, CENTRAL_MUX_HDR_SIZE, &flag, 1);
    }

    // ── JOIN 모드 ─────────────────────────────────────────────────────────
    // 성공 시 central_fd 반환 (투명 BEWE 스트림), 실패 시 -1
    int join_room(const std::string& central_host, int central_port,
                  const std::string& station_id);

    static int tcp_connect(const std::string& host, int port);
    // 여러 후보 IP 중 첫 번째 성공한 연결 반환 (LAN 우선)
    int tcp_connect_any(const std::vector<std::string>& candidates, int port);
    // primary_host + 캐시된 LAN IP를 합친 후보 목록 생성
    std::vector<std::string> make_candidates(const std::string& primary_host);

private:
    // 폴링
    std::thread       poll_thr_;
    std::atomic<bool> poll_running_{false};
    void poll_loop(std::string host, int port,
                   std::function<void(const std::vector<Station>&)> cb);

    // MUX 어댑터
    std::thread       mux_thr_;
    std::atomic<bool> mux_running_{false};
    int               mux_central_fd_ = -1;

    // central_fd 전용 송신 큐 + 스레드
    // 펌프/HB 모두 여기 enqueue → central_sender_thr_ 가 직렬로 write
    // → 펌프가 central_fd 블로킹으로 mutex 장시간 점유해 HB를 막는 문제 해결
    std::thread              central_sender_thr_;
    std::atomic<bool>        central_sender_running_{false};
    std::mutex               central_queue_mtx_;
    std::condition_variable  central_queue_cv_;
    struct QueueEntry { std::vector<uint8_t> data; bool no_drop; };
    std::deque<QueueEntry> central_send_queue_;
    size_t                   central_queue_bytes_ = 0;
    static constexpr size_t  CENTRAL_QUEUE_MAX_BYTES = 4 * 1024 * 1024; // 4MB (~1초)

    // (hdr, hdr_len) + (data, data_len) 을 합쳐 하나의 청크로 enqueue
    // queue가 CENTRAL_QUEUE_MAX_BYTES 를 초과하면 가장 오래된 항목부터 삭제
    // no_drop=true: IQ_CHUNK 등 드롭 불가 패킷 — 큐 제한 무시하고 항상 추가
    void enqueue_central(const void* hdr, size_t hdr_len,
                       const void* data, size_t data_len,
                       bool no_drop = false);
    void central_sender_loop(int central_fd);

    struct JoinPair {
        int local_fd  = -1;  // NetServer 쪽 (accept로 받음)
        int remote_fd = -1;  // relay로 보내는 쪽 (socketpair)
        std::thread thr;     // local→relay pump
    };
    std::mutex mux_joins_mtx_;
    std::unordered_map<uint16_t, std::shared_ptr<JoinPair>> mux_joins_;

    std::function<void()> on_central_disconnect_;  // mux_loop 종료 시 호출
    std::function<void(const uint8_t*, size_t)> on_central_ch_sync_;  // 릴레이 재작성 CHANNEL_SYNC
    std::function<void(const char*, const char*)> on_central_chat_;   // 중앙서버 전역 채팅 수신
    std::function<void(const uint8_t*, size_t)> on_central_op_list_;  // 릴레이 OP_LIST → HOST UI 갱신
    std::function<void(const uint8_t*, size_t)> on_central_db_list_; // Central DB 파일 목록
    std::function<void(const uint8_t*, size_t)> on_central_db_dl_data_; // Central DB 다운로드 데이터
    std::function<void(const uint8_t*, size_t)> on_central_db_dl_info_; // Central DB 다운로드 .info
    std::function<void(const uint8_t*, size_t)> on_central_report_list_; // Central Report 목록

    void mux_loop(int central_fd,
                  std::function<void(int)> on_new_join,
                  std::function<uint8_t()> count_fn);
public:
    // 릴레이가 보내는 CHANNEL_SYNC(audio_mask 재작성본)를 HOST에서 수신하는 콜백
    void set_on_central_ch_sync(std::function<void(const uint8_t*, size_t)> cb){
        on_central_ch_sync_ = std::move(cb);
    }
    // 릴레이가 보내는 OP_LIST를 HOST에서 수신하는 콜백 (JOIN 접속/해제 시 갱신)
    void set_on_central_op_list(std::function<void(const uint8_t*, size_t)> cb){
        on_central_op_list_ = std::move(cb);
    }
    void set_on_central_db_list(std::function<void(const uint8_t*, size_t)> cb){
        on_central_db_list_ = std::move(cb);
    }
    void set_on_central_db_dl_data(std::function<void(const uint8_t*, size_t)> cb){
        on_central_db_dl_data_ = std::move(cb);
    }
    void set_on_central_db_dl_info(std::function<void(const uint8_t*, size_t)> cb){
        on_central_db_dl_info_ = std::move(cb);
    }
    void set_on_central_report_list(std::function<void(const uint8_t*, size_t)> cb){
        on_central_report_list_ = std::move(cb);
    }
};
