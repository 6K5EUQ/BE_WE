#pragma once
#include <cstdint>
#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <chrono>

// ── IQ 파일 전송 전용 독립 TCP 서버 (포트 7702) ──────────────────────────
//
// 동작 방식:
//   HOST: register_send(req_id, file_path, progress_cb)
//         → 서버가 JOIN 연결 대기 후 파일을 직접 스트리밍
//   JOIN: 7702 포트에 TCP 연결 → req_id(4바이트 LE) 전송
//         → 서버가 파일 데이터 스트리밍
//
// BEWE 메인 스트림과 완전 분리. 별도 포트, 별도 스레드.
//

static constexpr int IQ_PIPE_SERVER_PORT = 7702;

class IqPipeServer {
public:
    // phase: 0=대기중, 1=전송중, 2=완료, -1=에러
    using ProgressCb = std::function<void(uint64_t done, uint64_t total, int phase)>;

    bool start(int port = IQ_PIPE_SERVER_PORT);
    void stop();

    // HOST 측: req_id와 파일 경로 등록 (JOIN 연결 대기 후 자동 전송)
    void register_send(uint32_t req_id, const std::string& file_path, ProgressCb cb);

private:
    int               listen_fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread       accept_thr_;

    struct SendEntry {
        uint32_t    req_id;
        std::string file_path;
        ProgressCb  cb;
        std::chrono::steady_clock::time_point created;
    };
    std::mutex              entries_mtx_;
    std::vector<SendEntry>  entries_;

    void accept_loop();
    void handle_join(int fd);
    int  make_listen_sock(int port);
};
