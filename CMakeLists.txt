cmake_minimum_required(VERSION 3.16)
project(BE_WE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CLI "Build CLI-only binary (no OpenGL/GLFW/ImGui)" OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BLADERF REQUIRED libbladeRF)
pkg_check_modules(FFTW    REQUIRED fftw3f)
pkg_check_modules(ALSA    REQUIRED alsa)
pkg_check_modules(MPG123  REQUIRED libmpg123)
pkg_check_modules(RTLSDR  REQUIRED librtlsdr)
pkg_check_modules(LIBPNG  REQUIRED libpng)

pkg_check_modules(VOLK REQUIRED volk)
pkg_check_modules(IIO    REQUIRED libiio)
pkg_check_modules(AD9361 REQUIRED libad9361)

if(CLI)
    # ── CLI build (no OpenGL/GLFW/ImGui) ──────────────────────────────────
    add_executable(BE_WE
        src/main.cpp
        src/cli_host.cpp
        src/fft_viewer.cpp
        src/bladerf_io.cpp
        src/demod.cpp
        src/iq_record.cpp
        src/sched_record.cpp
        src/audio.cpp
        src/audio_playback.cpp
        src/timemachine.cpp
        src/region_save.cpp
        src/sa_compute.cpp
        src/eid_compute.cpp
        src/rtlsdr_io.cpp
        src/hw_detect.cpp
        src/ais.cpp
        src/digi_demod.cpp
        src/auto_id.cpp
        src/proto_decode.cpp
        src/login.cpp
        src/net_server.cpp
        src/net_client.cpp
        src/net_stream.cpp
        src/central_client.cpp
        src/host_band_plan.cpp
        src/host_band_categories.cpp
        src/long_waterfall.cpp
        src/lora_demod.cpp
        src/pluto_io.cpp
    )

    target_compile_definitions(BE_WE PRIVATE BEWE_HEADLESS=1)

    target_include_directories(BE_WE PRIVATE
        src
        ${BLADERF_INCLUDE_DIRS}
        ${FFTW_INCLUDE_DIRS}
        ${ALSA_INCLUDE_DIRS}
        ${MPG123_INCLUDE_DIRS}
        ${RTLSDR_INCLUDE_DIRS}
        ${LIBPNG_INCLUDE_DIRS}
        ${VOLK_INCLUDE_DIRS}
        ${IIO_INCLUDE_DIRS}
        ${AD9361_INCLUDE_DIRS}
    )

    target_link_libraries(BE_WE PRIVATE
        ${BLADERF_LIBRARIES}
        ${FFTW_LIBRARIES}
        ${ALSA_LIBRARIES}
        ${MPG123_LIBRARIES}
        ${RTLSDR_LIBRARIES}
        ${LIBPNG_LIBRARIES}
        ${VOLK_LIBRARIES}
        ${IIO_LIBRARIES}
        ${AD9361_LIBRARIES}
        pthread
        m
    )

    target_compile_options(BE_WE PRIVATE -O3 -march=native)

else()
    # ── GUI build (original, unchanged) ───────────────────────────────────
    find_package(OpenGL REQUIRED)
    find_package(GLEW   REQUIRED)
    find_package(glfw3  REQUIRED)

    set(IMGUI_PATH          ${CMAKE_SOURCE_DIR}/libs/imgui)
    set(IMGUI_BACKENDS_PATH ${IMGUI_PATH}/backends)
    set(IMGUI_SOURCES
        ${IMGUI_PATH}/imgui.cpp
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
        src/sched_record.cpp
        src/audio.cpp
        src/audio_playback.cpp
        src/ui.cpp
        src/timemachine.cpp
        src/region_save.cpp
        src/sa_compute.cpp
        src/eid_compute.cpp
        src/rtlsdr_io.cpp
        src/hw_detect.cpp
        src/ais.cpp
        src/digi_demod.cpp
        src/auto_id.cpp
        src/proto_decode.cpp
        src/login.cpp
        src/net_server.cpp
        src/net_client.cpp
        src/net_stream.cpp
        src/globe.cpp
        src/central_client.cpp
        src/host_band_plan.cpp
        src/host_band_categories.cpp
        src/long_waterfall.cpp
        src/long_waterfall_view.cpp
        src/lora_demod.cpp
        src/pluto_io.cpp
        ${IMGUI_SOURCES}
    )

    target_include_directories(BE_WE PRIVATE
        src
        ${BLADERF_INCLUDE_DIRS}
        ${FFTW_INCLUDE_DIRS}
        ${ALSA_INCLUDE_DIRS}
        ${MPG123_INCLUDE_DIRS}
        ${RTLSDR_INCLUDE_DIRS}
        ${LIBPNG_INCLUDE_DIRS}
        ${VOLK_INCLUDE_DIRS}
        ${IIO_INCLUDE_DIRS}
        ${AD9361_INCLUDE_DIRS}
        ${IMGUI_PATH}
        ${IMGUI_BACKENDS_PATH}
    )

    target_link_libraries(BE_WE PRIVATE
        ${BLADERF_LIBRARIES}
        ${FFTW_LIBRARIES}
        ${ALSA_LIBRARIES}
        ${MPG123_LIBRARIES}
        ${RTLSDR_LIBRARIES}
        ${LIBPNG_LIBRARIES}
        OpenGL::GL
        GLEW::GLEW
        glfw
        ${VOLK_LIBRARIES}
        ${IIO_LIBRARIES}
        ${AD9361_LIBRARIES}
        pthread
        m
    )

    target_compile_options(BE_WE PRIVATE -O3 -march=native)

endif()
