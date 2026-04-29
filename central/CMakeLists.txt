cmake_minimum_required(VERSION 3.16)
project(bewe_central)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(bewe_central
    central_main.cpp
    central_server.cpp
    emitter_db.cpp
    info_parse.cpp
)
target_include_directories(bewe_central PRIVATE . ${CMAKE_SOURCE_DIR}/../src)
target_link_libraries(bewe_central PRIVATE pthread)
target_compile_options(bewe_central PRIVATE -O2)

# Standalone unit test for InfoParse + EmitterDb (no server deps).
add_executable(bewe_emitter_db_test
    test_emitter_db.cpp
    emitter_db.cpp
    info_parse.cpp
)
target_include_directories(bewe_emitter_db_test PRIVATE .)
target_link_libraries(bewe_emitter_db_test PRIVATE pthread)
target_compile_options(bewe_emitter_db_test PRIVATE -O0 -g)
