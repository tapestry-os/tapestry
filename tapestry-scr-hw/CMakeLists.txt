cmake_minimum_required(VERSION 3.20.0)

if("${BOARD}" STREQUAL "bbc_microbit_v2")
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os/boards/bbc_microbit_v2/bbc_microbit_v2.overlay)
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

# Substrate: Cutebot on micro:bit, null stub everywhere else.
if(CONFIG_I2C)
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
    ${TAPESTRY_OS_CSM}                             # world_model.h, state.h
    ${TAPESTRY_OS_SCR}                             # scr.h internals
    ${TAPESTRY_OS_RUNTIME}                         # power.h (internal)
    ${TAPESTRY_OS_TRANSPORT}                       # gossip.h, transceiver_ble.h, transceiver_udp.h, net_init.h
    ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2          # cutebot.h
    src
)

target_link_libraries(app PRIVATE m)
