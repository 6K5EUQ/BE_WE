cmake_minimum_required(VERSION 3.16)
project(FFT_Analyzer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BLADERF REQUIRED libbladeRF)
pkg_check_modules(FFTW REQUIRED fftw3f)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)

# Paths
set(IMGUI_PATH /usr/include/imgui)
set(IMGUI_BACKENDS_PATH ${IMGUI_PATH}/backends)

# record_with_fft
add_executable(record_with_fft src/record_with_fft.cpp)
target_include_directories(record_with_fft PRIVATE ${BLADERF_INCLUDE_DIRS} ${FFTW_INCLUDE_DIRS})
target_link_libraries(record_with_fft PRIVATE ${BLADERF_LIBRARIES} ${FFTW_LIBRARIES})

# view_fft with ImGui
set(IMGUI_SOURCES
    ${IMGUI_PATH}/imgui.cpp
    ${IMGUI_PATH}/imgui_demo.cpp
    ${IMGUI_PATH}/imgui_draw.cpp
    ${IMGUI_PATH}/imgui_tables.cpp
    ${IMGUI_PATH}/imgui_widgets.cpp
    ${IMGUI_BACKENDS_PATH}/imgui_impl_glfw.cpp
    ${IMGUI_BACKENDS_PATH}/imgui_impl_opengl3.cpp
)

add_executable(view_fft src/view_fft.cpp ${IMGUI_SOURCES})
target_include_directories(view_fft PRIVATE 
    ${IMGUI_PATH}
    ${IMGUI_BACKENDS_PATH}
    /usr/include/GL
    /usr/include/GLFW
    ${FFTW_INCLUDE_DIRS}
)
target_link_libraries(view_fft PRIVATE 
    OpenGL::GL 
    GLEW::GLEW 
    glfw 
    glm::glm 
    ${FFTW_LIBRARIES}
)