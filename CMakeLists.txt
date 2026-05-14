cmake_minimum_required(VERSION 3.16)

project(udl_sink LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_library(udl_sink STATIC
    src/udl_sink.c
)

target_include_directories(udl_sink
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(udl_sink PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()

add_executable(udl_sink_test
    test/test_udl_sink.c
)

add_executable(udl_sink_bench
    test/bench_udl_sink.c
)

target_link_libraries(udl_sink_test PRIVATE udl_sink)
target_link_libraries(udl_sink_bench PRIVATE udl_sink)

if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(udl_sink_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(udl_sink_bench PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME udl_sink_test COMMAND udl_sink_test)