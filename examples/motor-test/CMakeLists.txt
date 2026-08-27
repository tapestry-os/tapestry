cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21bl).  Must be before find_package.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)

# tapestry-os/dts/bindings/ holds custom DTS bindings (e.g. bitcraze,cf21-motors).
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

# Board overlays from tapestry-os pair hardware drivers with their devicetree.
if("${BOARD}" STREQUAL "bbc_microbit_v2")
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/bbc_microbit_v2/bbc_microbit_v2.overlay)
elseif("${BOARD}" STREQUAL "crazyflie21bl")
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/crazyflie21bl/crazyflie21bl.overlay)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_motor_test)

set(TAPESTRY_OS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_BOARDS ${TAPESTRY_OS_ROOT}/boards)

target_sources(app PRIVATE src/main.c)

# CRTP radio log backend — routes Zephyr LOG_* to nRF51 → crazyradio2 → crazyflie_console.py
if("${BOARD}" STREQUAL "crazyflie21bl")
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_crtp_log.c
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_syslink_tx.c)
endif()

# Substrate selection: CF21bl (PWM), Cutebot (I2C), null stub everywhere else.
# PWM must be checked before I2C: the CF21 stabilizer uses both (PWM for motors,
# I2C for the BMI088 IMU), so checking I2C first would wrongly select the Cutebot path.
if(CONFIG_PWM)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/substrate_crazyflie21bl.c
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/crazyflie21bl.c
    )
    if(CONFIG_CF21BL_STABILIZER)
        target_sources(app PRIVATE
            ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_stabilizer.c
            ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_imu.c
        )
        target_link_libraries(app PRIVATE m)
    endif()
elseif(CONFIG_I2C)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2/substrate_cutebot.c
        ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2/cutebot.c
    )
else()
    target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/substrate_null.c)
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include             # <tapestry/substrate.h>
    ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2   # cutebot.h
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl     # crazyflie21bl.h, crazyflie21bl_mix.h
)
