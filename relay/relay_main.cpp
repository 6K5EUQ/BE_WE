#include "relay_server.hpp"
#include <cstdio>
#include <cstring>
#include <csignal>

static RelayServer* g_relay = nullptr;
static void sig_handler(int){ if(g_relay) g_relay->stop(); }

int main(int argc, char** argv){
    int port = RELAY_PORT; // 기본 7700 (단일 포트) 나중에 보안검토 다시 할때 수정하기 ...
    for(int i=1; i<argc; i++){
        if(!strcmp(argv[i],"--port") && i+1<argc) port = atoi(argv[++i]);
    }

    printf("=== BEWE Relay Server ===\n");
    printf("  Port: %d (HOST/JOIN/LIST 통합)\n", port);
    printf("Press Ctrl+C to stop.\n\n");

    RelayServer relay;
    g_relay = &relay;
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    if(!relay.start(port)){
        fprintf(stderr,"[Relay] start failed\n");
        return 1;
    }
    relay.run();
    printf("[Relay] shutdown complete\n");
    return 0;
}
