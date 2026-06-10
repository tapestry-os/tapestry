cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21br).  Must be before find_package.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)

# tapestry-os/dts/bindings/ holds custom DTS bindings (e.g. bitcraze,cf21-motors).
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

# Board overlays from tapestry-os pair hardware drivers with their devicetree.
if("${BOARD}" STREQUAL "bbc_microbit_v2")
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/bbc_microbit_v2/bbc_microbit_v2.overlay)
elseif("${BOARD}" STREQUAL "crazyflie21br")
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/crazyflie21br/crazyflie21br.overlay)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_motor_test)

set(TAPESTRY_OS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_BOARDS ${TAPESTRY_OS_ROOT}/boards)

target_sources(app PRIVATE src/main.c)

# CRTP radio log backend — routes Zephyr LOG_* to nRF51 → crazyradio2 → read_console.py
if("${BOARD}" STREQUAL "crazyflie21br")
    target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/crazyflie21br/cf21_crtp_log.c)
endif()

# Substrate selection: Cutebot (I2C), CF21br (PWM), null stub everywhere else.
if(CONFIG_I2C)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2/substrate_cutebot.c
        ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2/cutebot.c
    )
elseif(CONFIG_PWM)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/crazyflie21br/substrate_crazyflie21br.c
        ${TAPESTRY_OS_BOARDS}/crazyflie21br/crazyflie21br.c
    )
else()
    target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/substrate_null.c)
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include             # <tapestry/substrate.h>
    ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2   # cutebot.h
    ${TAPESTRY_OS_BOARDS}/crazyflie21br     # crazyflie21br.h, crazyflie21br_mix.h
)
