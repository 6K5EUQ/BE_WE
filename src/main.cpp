#include "fft_viewer.hpp"
#include <cstdlib>

int main(){
    setenv("GTK_IM_MODULE","none",1);
    setenv("QT_IM_MODULE","none",1);
    setenv("XMODIFIERS","@im=none",1);
    setenv("GLFW_IM_MODULE","none",1);
    run_streaming_viewer();
    return 0;
}
