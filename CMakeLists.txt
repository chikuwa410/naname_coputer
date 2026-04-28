cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_copter C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()
#include_directories(/home/student01/pico/pico-sdk/eigen)
include_directories(../../eigen)

add_executable(pico_copter
  pico_copter.cpp
  sensor.cpp
  ekf.cpp
  lsm9ds1_reg.c
  pwm.cpp
  radio.cpp
  control.cpp
  WS2812.cpp
  rgbled.cpp
  i2c.cpp
  pid.cpp
  pmw3901.cpp
  #vl53l1_platform.c
  #VL53L1X_api.c
  #VL53L1X_calibration.c
)

pico_generate_pio_header(pico_copter ${CMAKE_CURRENT_LIST_DIR}/WS2812.pio)


target_link_libraries(pico_copter 
  hardware_spi 
  hardware_pwm 
  hardware_uart 
  hardware_pio
  hardware_i2c
  hardware_adc
  pico_stdlib
  pico_multicore
)

pico_enable_stdio_usb(pico_copter 1)
pico_enable_stdio_uart(pico_copter 0)

pico_add_extra_outputs(pico_copter)

# Include ToF modules and ST driver
#include_directories(${CMAKE_CURRENT_LIST_DIR}/modules/tof)
include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/ST_VL53L1X)

# Add ToF and ST sources
target_sources(pico_copter PRIVATE
  #modules/tof/tof_bridge.cpp
  #modules/tof/tof_vl53l1x.cpp
  third_party/ST_VL53L1X/VL53L1X_api.c
  third_party/ST_VL53L1X/vl53l1_platform.c
)