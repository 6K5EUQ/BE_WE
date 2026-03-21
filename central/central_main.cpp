#include "central_server.hpp"
#include <cstdio>
#include <cstring>
#include <csignal>

static CentralServer* g_central = nullptr;
static void sig_handler(int){ if(g_central) g_central->stop(); }

int main(int argc, char** argv){
    setbuf(stdout, nullptr);  // stdout 라인 버퍼링 해제 → 즉시 출력
    setbuf(stderr, nullptr);
    int port = CENTRAL_PORT; // 기본 7700 (단일 포트) 나중에 보안검토 다시 할때 수정하기 ...
    for(int i=1; i<argc; i++){
        if(!strcmp(argv[i],"--port") && i+1<argc) port = atoi(argv[++i]);
    }

    printf("=== BEWE Central Server ===\n");
    printf("  Port: %d (HOST/JOIN/LIST 통합)\n", port);
    printf("Press Ctrl+C to stop.\n\n");

    CentralServer central_srv;
    g_central = &central_srv;
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    if(!central_srv.start(port)){
        fprintf(stderr,"[Central] start failed\n");
        return 1;
    }
    central_srv.run();
    printf("[Central] shutdown complete\n");
    return 0;
}
