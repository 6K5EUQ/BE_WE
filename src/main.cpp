#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include <cstdlib>

#ifdef BEWE_HEADLESS
int main(){
    BEWEPaths::ensure_dirs();
    run_cli_host();
    return 0;
}
#else
int main(){
    BEWEPaths::ensure_dirs();
    setenv("GTK_IM_MODULE","none",1);
    setenv("QT_IM_MODULE","none",1);
    setenv("XMODIFIERS","@im=none",1);
    setenv("GLFW_IM_MODULE","none",1);
    run_streaming_viewer();
    return 0;
}
#endif