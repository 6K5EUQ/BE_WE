cmake_minimum_required(VERSION 3.16)
project(bewe_relay)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(bewe_relay
    relay_main.cpp
    relay_server.cpp
)
target_include_directories(bewe_relay PRIVATE .)
target_link_libraries(bewe_relay PRIVATE pthread)
target_compile_options(bewe_relay PRIVATE -O2)
