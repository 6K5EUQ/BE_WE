#include "central_server.hpp"
#include <cstdio>
#include <cstring>
#include <csignal>
#include <atomic>
#include <cstdlib>
#include <chrono>
#include <thread>

// async-signal-safe 정책: sig handler는 atomic flag만 set. mutex/IO/join 금지.
// 두번째 Ctrl+C는 강제 종료 (정상 stop()이 어딘가 stuck인 경우의 비상 탈출).
static std::atomic<int>  g_sigint_count{0};
static std::atomic<bool> g_should_stop{false};
static void sig_handler(int){
    int n = g_sigint_count.fetch_add(1) + 1;
    if(n >= 2){
        // 두번째 신호: 즉시 _exit (stdio flush 안 함 — async-signal-safe)
        _exit(130);
    }
    g_should_stop.store(true);
}

int main(int argc, char** argv){
    setbuf(stdout, nullptr);  // stdout 라인 버퍼링 해제 → 즉시 출력
    setbuf(stderr, nullptr);
    int port = CENTRAL_PORT; // 기본 7700 (단일 포트) 나중에 보안검토 다시 할때 수정하기 ...
    for(int i=1; i<argc; i++){
        if(!strcmp(argv[i],"--port") && i+1<argc) port = atoi(argv[++i]);
    }

    printf("=== BEWE Central Server ===\n");
    printf("  Port: %d (HOST/JOIN/LIST 통합)\n", port);
    printf("Press Ctrl+C to stop. (Twice for force quit.)\n\n");

    CentralServer central_srv;
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    if(!central_srv.start(port)){
        fprintf(stderr,"[Central] start failed\n");
        return 1;
    }
    // sig handler는 flag만 set. 메인 루프가 polling으로 stop() 호출 — 재진입/락 데드락 방지.
    while(!g_should_stop.load()){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    printf("[Central] received shutdown signal, stopping...\n");
    central_srv.stop();
    printf("[Central] shutdown complete\n");
    return 0;
}
