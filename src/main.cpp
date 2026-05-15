#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "session_args.hpp"
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>
#include <atomic>
#include <signal.h>

// SIGINT/SIGTERM 수신 시 main loop가 정상 종료 경로(미션 자동 end + cleanup) 거치도록
// 플래그만 set. ui.cpp / cli_host.cpp 가 매 iteration에서 폴링.
std::atomic<bool> g_signal_shutdown{false};
static void bewe_sig_handler(int){ g_signal_shutdown.store(true); }
static void install_signal_handlers(){
    struct sigaction sa{};
    sa.sa_handler = bewe_sig_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;  // SA_RESTART 없음 — sleep/read를 깨워야 main loop가 깨어남
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

#ifndef BEWE_HEADLESS
#include "sat_view.hpp"
#endif

SessionArgs g_session_args;

static bool starts_with(const char* s, const char* prefix){
    size_t n = std::strlen(prefix);
    return std::strncmp(s, prefix, n) == 0;
}

static void parse_args(int argc, char** argv){
    for(int i = 1; i < argc; i++){
        const char* a = argv[i];
        if(std::strcmp(a, "--sdr") == 0 && i+1 < argc){
            std::string v = argv[i+1];
            if(v == "bladerf" || v == "rtlsdr" || v == "pluto"){
                g_sdr_force = v;
                fprintf(stderr, "[BEWE] forcing SDR = %s\n", v.c_str());
            } else {
                fprintf(stderr, "[BEWE] unknown --sdr '%s' (use bladerf|rtlsdr|pluto)\n", v.c_str());
            }
            i++;
        } else if(starts_with(a, "--session-mode=")){
            std::string v = a + std::strlen("--session-mode=");
            if(v == "host" || v == "join"){
                g_session_args.mode_set = true;
                g_session_args.mode     = v;
            } else {
                fprintf(stderr, "[BEWE] unknown --session-mode '%s' (use host|join)\n", v.c_str());
            }
        } else if(starts_with(a, "--station-id=")){
            g_session_args.station_id = a + std::strlen("--station-id=");
        } else if(starts_with(a, "--station-name=")){
            g_session_args.station_name = a + std::strlen("--station-name=");
        } else if(starts_with(a, "--station-lat=")){
            g_session_args.station_lat = (float)std::atof(a + std::strlen("--station-lat="));
        } else if(starts_with(a, "--station-lon=")){
            g_session_args.station_lon = (float)std::atof(a + std::strlen("--station-lon="));
        } else if(std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0){
            fprintf(stderr,
                "BE_WE options:\n"
                "  --sdr bladerf|rtlsdr|pluto   force a specific SDR backend\n"
                "  --session-mode=host|join     internal: child-process boot mode\n"
                "  --station-id=<id>            internal: Central room id (JOIN)\n"
                "  --station-name=<utf8>        internal: station display name\n"
                "  --station-lat=<deg>          internal: station latitude (HOST)\n"
                "  --station-lon=<deg>          internal: station longitude (HOST)\n"
                "  -h, --help                   this message\n");
        }
    }
}

#ifdef BEWE_HEADLESS
int main(int argc, char** argv){
    parse_args(argc, argv);
    BEWEPaths::ensure_dirs();
    install_signal_handlers();
    run_cli_host();
    return 0;
}
#else
int main(int argc, char** argv){
    parse_args(argc, argv);
    BEWEPaths::ensure_dirs();
    setenv("GTK_IM_MODULE","none",1);
    setenv("QT_IM_MODULE","none",1);
    setenv("XMODIFIERS","@im=none",1);
    setenv("GLFW_IM_MODULE","none",1);
    install_signal_handlers();
    // TLE fetch is now lazy: only when the user picks ALL in the sat tracker.
    run_streaming_viewer();
    return 0;
}
#endif
