cmake_minimum_required(VERSION 3.16)
project(BE_WE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BLADERF REQUIRED libbladeRF)

add_executable(be_we
    src/main.cpp
)

target_include_directories(be_we PRIVATE ${BLADERF_INCLUDE_DIRS})
target_link_libraries(be_we PRIVATE ${BLADERF_LIBRARIES})
