cmake_minimum_required(VERSION 3.16)
project(bewe_central)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(bewe_central
    central_main.cpp
    central_server.cpp
    central_mission_archive.cpp
    emitter_db.cpp
    info_parse.cpp
)
target_include_directories(bewe_central PRIVATE . ${CMAKE_SOURCE_DIR}/../src)
find_package(ZLIB REQUIRED)
target_link_libraries(bewe_central PRIVATE pthread ZLIB::ZLIB)
target_compile_options(bewe_central PRIVATE -O2)
