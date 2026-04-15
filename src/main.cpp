#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>

static void parse_sdr_flag(int argc, char** argv){
    for(int i = 1; i < argc; i++){
        if(std::strcmp(argv[i], "--sdr") == 0 && i+1 < argc){
            std::string v = argv[i+1];
            if(v == "bladerf" || v == "rtlsdr" || v == "pluto"){
                g_sdr_force = v;
                fprintf(stderr, "[BEWE] forcing SDR = %s\n", v.c_str());
            } else {
                fprintf(stderr, "[BEWE] unknown --sdr '%s' (use bladerf|rtlsdr|pluto)\n", v.c_str());
            }
            i++;
        } else if(std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0){
            fprintf(stderr,
                "BE_WE options:\n"
                "  --sdr bladerf|rtlsdr|pluto   force a specific SDR backend\n"
                "  -h, --help                   this message\n");
        }
    }
}

#ifdef BEWE_HEADLESS
int main(int argc, char** argv){
    parse_sdr_flag(argc, argv);
    BEWEPaths::ensure_dirs();
    run_cli_host();
    return 0;
}
#else
int main(int argc, char** argv){
    parse_sdr_flag(argc, argv);
    BEWEPaths::ensure_dirs();
    setenv("GTK_IM_MODULE","none",1);
    setenv("QT_IM_MODULE","none",1);
    setenv("XMODIFIERS","@im=none",1);
    setenv("GLFW_IM_MODULE","none",1);
    run_streaming_viewer();
    return 0;
}
#endif
