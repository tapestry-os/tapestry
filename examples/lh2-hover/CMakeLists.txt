cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

set(DTC_OVERLAY_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/crazyflie21bl/crazyflie21bl.overlay"
    "${CMAKE_CURRENT_SOURCE_DIR}/boards/crazyflie21bl_lh.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_lh2_hover)

set(TAPESTRY_OS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_BOARDS ${TAPESTRY_OS_ROOT}/boards)

target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/substrate_crazyflie21bl.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/crazyflie21bl.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_stabilizer.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_imu.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_lighthouse.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_iwdg.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_crtp_log.c
)

if(CONFIG_CF21BL_PM)
  target_sources(app PRIVATE
      ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_pm.c)
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl
)
