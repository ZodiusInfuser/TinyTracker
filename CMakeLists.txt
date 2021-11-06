cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
project(test_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pimoroni_tiny2040)

pico_sdk_init()

add_executable(tiny_tracker
  tiny_tracker.cpp
  Sensor.cpp
)

pico_enable_stdio_usb(tiny_tracker 1)
pico_enable_stdio_uart(tiny_tracker 1)

pico_add_extra_outputs(tiny_tracker)

pico_generate_pio_header(tiny_tracker ${CMAKE_CURRENT_LIST_DIR}/lighthouse.pio)
pico_generate_pio_header(tiny_tracker ${CMAKE_CURRENT_LIST_DIR}/simulated_lh.pio)

target_link_libraries(tiny_tracker pico_stdlib hardware_pio hardware_pwm)