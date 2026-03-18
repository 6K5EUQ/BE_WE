#pragma once
#include "../relay/relay_proto.hpp"
#include "net_protocol.hpp"
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <memory>
#include <unordered_map>

// ── RelayClient ────────────────────────────────────────────────────────────
// HOST 모드:
//   open_room() → relay에 연결, HOST_OPEN 전송 → relay_fd 반환
//   이후 relay_fd는 MUX 스트림: JOIN패킷이 [MuxHdr+BEWE]로 들어오고
//   HOST→JOIN 전송도 [MuxHdr+BEWE]로 내보냄
//   NetServer 대신 RelayNetAdapter가 이 fd를 읽어 각 JOIN 가상소켓에 라우팅
//
// JOIN 모드:
//   join_room() → relay에 연결, JOIN_ROOM 전송 → relay_fd 반환
//   이후 relay_fd는 투명 BEWE 스트림 → NetClient::connect_fd()에 전달
//
// 목록 조회:
//   fetch_stations() / start_polling()

class RelayClient {
public:
    struct Station {
        std::string station_id;
        std::string name;
        float       lat = 0, lon = 0;
        uint8_t     host_tier  = 1;
        uint8_t     user_count = 0;
    };

    // relay 서버의 LAN IP 캐시 (LIST_RESP에서 수신)
    // 같은 망에 있으면 LAN IP로 직접 접속 가능
    std::vector<std::string> relay_lan_ips;
    std::mutex               relay_lan_ips_mtx;

    // ── 목록 조회 ─────────────────────────────────────────────────────────
    std::vector<Station> fetch_stations(const std::string& host, int port);
    void start_polling(const std::string& host, int port,
                       std::function<void(const std::vector<Station>&)> cb);
    void stop_polling();

    // ── HOST 모드 ─────────────────────────────────────────────────────────
    // 성공 시 relay_fd(MUX 스트림) 반환, 실패 시 -1
    int open_room(const std::string& relay_host, int relay_port,
                  const std::string& station_id, const std::string& station_name,
                  float lat, float lon, uint8_t host_tier);

    // MUX 스트림을 읽어 각 JOIN의 가상 socketpair에 라우팅하는 어댑터 시작
    // NetServer는 accept_fd(je->local_fd)로 JOIN 연결을 수락함
    // on_new_join(local_fd): 새 JOIN 연결 시 호출 → NetServer에 inject
    void start_mux_adapter(int relay_fd,
                           std::function<void(int local_fd)> on_new_join,
                           std::function<uint8_t()> user_count_fn,
                           std::function<void()> on_disconnect = nullptr);
    void stop_mux_adapter();
    bool is_relay_connected() const { return mux_running_.load(); }

    // relay에 NET_RESET 신호 전송 (0=reset start, 1=open)
    void send_net_reset(uint8_t flag){
        if(mux_relay_fd_ < 0) return;
        std::lock_guard<std::mutex> lk(mux_relay_write_mtx_);
        relay_send_mux(mux_relay_fd_, 0xFFFF, RelayMuxType::NET_RESET, &flag, 1);
    }

    // ── JOIN 모드 ─────────────────────────────────────────────────────────
    // 성공 시 relay_fd 반환 (투명 BEWE 스트림), 실패 시 -1
    int join_room(const std::string& relay_host, int relay_port,
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
    int               mux_relay_fd_ = -1;
    std::mutex        mux_relay_write_mtx_; // relay_fd write 직렬화 (mux_loop HB + pump thr)

    struct JoinPair {
        int local_fd  = -1;  // NetServer 쪽 (accept로 받음)
        int remote_fd = -1;  // relay로 보내는 쪽 (socketpair)
        std::thread thr;     // local→relay pump
    };
    std::mutex mux_joins_mtx_;
    std::unordered_map<uint16_t, std::shared_ptr<JoinPair>> mux_joins_;

    std::function<void()> on_mux_disconnect_;  // mux_loop 종료 시 호출

    void mux_loop(int relay_fd,
                  std::function<void(int)> on_new_join,
                  std::function<uint8_t()> count_fn);
};
