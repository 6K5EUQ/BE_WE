cmake_minimum_required(VERSION 3.16)
project(BE_WE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BLADERF REQUIRED libbladeRF)
pkg_check_modules(FFTW    REQUIRED fftw3f)
pkg_check_modules(ALSA    REQUIRED alsa)
pkg_check_modules(MPG123  REQUIRED libmpg123)

find_package(OpenGL REQUIRED)
find_package(GLEW   REQUIRED)
find_package(glfw3  REQUIRED)

set(IMGUI_PATH          /usr/include/imgui)
set(IMGUI_BACKENDS_PATH ${IMGUI_PATH}/backends)

set(IMGUI_SOURCES
    ${IMGUI_PATH}/imgui.cpp
    ${IMGUI_PATH}/imgui_demo.cpp
    ${IMGUI_PATH}/imgui_draw.cpp
    ${IMGUI_PATH}/imgui_tables.cpp
    ${IMGUI_PATH}/imgui_widgets.cpp
    ${IMGUI_BACKENDS_PATH}/imgui_impl_glfw.cpp
    ${IMGUI_BACKENDS_PATH}/imgui_impl_opengl3.cpp
)

add_executable(BE_WE
    src/main.cpp
    src/fft_viewer.cpp
    src/bladerf_io.cpp
    src/demod.cpp
    src/iq_record.cpp
    src/audio.cpp
    src/alert.cpp
    src/ui.cpp
    src/timemachine.cpp
    src/region_save.cpp
    ${IMGUI_SOURCES}
)

target_include_directories(BE_WE PRIVATE
    src
    ${BLADERF_INCLUDE_DIRS}
    ${FFTW_INCLUDE_DIRS}
    ${ALSA_INCLUDE_DIRS}
    ${MPG123_INCLUDE_DIRS}
    ${IMGUI_PATH}
    ${IMGUI_BACKENDS_PATH}
)

target_link_libraries(BE_WE PRIVATE
    ${BLADERF_LIBRARIES}
    ${FFTW_LIBRARIES}
    ${ALSA_LIBRARIES}
    ${MPG123_LIBRARIES}
    OpenGL::GL
    GLEW::GLEW
    glfw
    pthread
)

target_compile_options(BE_WE PRIVATE -O3 -march=native)