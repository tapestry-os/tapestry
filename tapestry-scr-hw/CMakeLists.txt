cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21bl).  Must be set before find_package.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# tapestry-os/dts/bindings/ holds custom DTS bindings (e.g. bitcraze,cf21-motors).
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os)

if("${BOARD}" STREQUAL "bbc_microbit_v2")
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os/boards/bbc_microbit_v2/bbc_microbit_v2.overlay)
elseif("${BOARD}" STREQUAL "crazyflie21bl")
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os/boards/crazyflie21bl/crazyflie21bl.overlay)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_hw_element)

set(TAPESTRY_OS_ROOT      ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os)
set(TAPESTRY_OS_CSM       ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_SCR       ${TAPESTRY_OS_ROOT}/subsys/scr)
set(TAPESTRY_OS_TRANSPORT ${TAPESTRY_OS_ROOT}/subsys/transport)
set(TAPESTRY_OS_BSE       ${TAPESTRY_OS_ROOT}/subsys/bse)
set(TAPESTRY_OS_CHOREO    ${TAPESTRY_OS_ROOT}/subsys/choreo)
set(TAPESTRY_OS_RUNTIME   ${TAPESTRY_OS_ROOT}/subsys/runtime)
set(TAPESTRY_OS_BOARDS    ${TAPESTRY_OS_ROOT}/boards)
set(TAPESTRY_SDK          ${CMAKE_CURRENT_SOURCE_DIR}/../sdk)

# Core L2 runtime + L4/L5 stack + L6 BSE stub + L7 SDK stub + transport.
target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_RUNTIME}/runtime.c
    ${TAPESTRY_OS_RUNTIME}/power.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_SCR}/scr.c
    ${TAPESTRY_OS_BSE}/bse.c
    ${TAPESTRY_OS_CHOREO}/choreo.c
    ${TAPESTRY_OS_TRANSPORT}/transport.c
    ${TAPESTRY_OS_TRANSPORT}/gossip.c
)

# UDP transceiver + network bring-up — ESP32 and RA8D1 only.
if(CONFIG_NETWORKING)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_TRANSPORT}/transceiver_udp.c
        ${TAPESTRY_OS_TRANSPORT}/net_init.c
    )
endif()

# BLE transceiver — ESP32 (bridge) and micro:bit.
if(CONFIG_BT)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_ble.c)
endif()

# Stub transceiver backends (compiled when the matching Kconfig symbol is set
# by a board overlay; not active in any current build).
if(CONFIG_TAPESTRY_TRANSCEIVER_ACOUSTIC)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_acoustic.c)
endif()
if(CONFIG_TAPESTRY_TRANSCEIVER_OPTICAL)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_optical.c)
endif()
if(CONFIG_TAPESTRY_TRANSCEIVER_CHEMICAL)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_chemical.c)
endif()
if(CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_syslink_p2p.c)
endif()

# CF2.1 early IWDG kick — must be compiled unconditionally so it runs even in
# minimal blink-test builds (CONFIG_PWM=n).  The Bitcraze bootloader arms the
# IWDG before jumping to us; without this, the watchdog fires within ~100-353ms.
#
# CF2.1 CRTP log backend — sends Zephyr LOG_* output as syslink console packets
# over USART6 to the nRF51 co-processor; read_console.py receives them via radio.
# Compiled only when CONFIG_LOG=y (i.e. the log subsystem is present).
if("${BOARD}" STREQUAL "crazyflie21bl")
    target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_iwdg.c)
    if(CONFIG_LOG)
        target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_crtp_log.c)
    endif()
endif()

# Substrate: CF2.1 brushless on crazyflie21bl, Cutebot on micro:bit, null elsewhere.
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
    ${TAPESTRY_OS_ROOT}/include                    # <tapestry/csm.h>, <tapestry/scr.h>, <tapestry/transport.h>, <tapestry/substrate.h>, <tapestry/bse.h>, <tapestry/runtime.h>
    ${TAPESTRY_SDK}/include                        # <tapestry/choreo.h>
    ${TAPESTRY_OS_CSM}                             # world_model.c; types via <tapestry/csm.h>
    ${TAPESTRY_OS_SCR}                             # scr.h internals
    ${TAPESTRY_OS_RUNTIME}                         # power.h (internal)
    ${TAPESTRY_OS_TRANSPORT}                       # gossip.h, transceiver_ble.h, transceiver_udp.h, net_init.h
    ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2          # cutebot.h
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl             # crazyflie21bl_mix.h, crazyflie21bl.h
    src
)

target_link_libraries(app PRIVATE m)
