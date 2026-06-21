cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_lh2_test)

set(TAPESTRY_OS_BOARDS ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards)

target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_iwdg.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_crtp_log.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_lighthouse.c
)
target_include_directories(app PRIVATE ${TAPESTRY_OS_BOARDS}/crazyflie21bl)
