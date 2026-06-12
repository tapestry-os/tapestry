cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21br).  Must be before find_package.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)

# tapestry-os/dts/bindings/ holds custom DTS bindings (e.g. bitcraze,cf21-motors).
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

# Board overlay from tapestry-os pairs the BMI088 IMU (and motor PWM) with
# the crazyflie21br board DTS — shared with tapestry-scr-hw and motor-test.
set(DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/crazyflie21br/crazyflie21br.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_imu_test)

set(TAPESTRY_OS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_BOARDS ${TAPESTRY_OS_ROOT}/boards)

target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21br/cf21_imu.c
)

# CRTP radio log backend — routes Zephyr LOG_* to nRF51 → crazyradio2 → read_console.py
target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/crazyflie21br/cf21_crtp_log.c)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_BOARDS}/crazyflie21br     # cf21_imu.h
)
