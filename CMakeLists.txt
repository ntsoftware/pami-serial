cmake_minimum_required(VERSION 3.20)
project(pami-serial)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(test_bw16
    test_bw16.cpp
    data_types.h
    bw16/capture.cpp
    bw16/capture.h
    bw16/data.cpp
    bw16/data.h
    bw16/hal.cpp
    bw16/hal.h
)
add_executable(test_teensy
    test_teensy.cpp
    data_types.h
    teensy/data.cpp
    teensy/data.h
    teensy/hal.cpp
    teensy/hal.h
)
target_compile_definitions(test_bw16 PRIVATE
    HAL_RX_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/bw16_rx"
    HAL_TX_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/bw16_tx"
    CAPTURE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/capture"
)
target_compile_definitions(test_teensy PRIVATE
    HAL_RX_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/teensy_rx"
    HAL_TX_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/teensy_tx"
)
