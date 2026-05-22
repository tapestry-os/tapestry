cmake_minimum_required(VERSION 3.20.0)

if("${BOARD}" STREQUAL "bbc_microbit_v2")
    list(APPEND EXTRA_DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/bbc_microbit_v2/bbc_microbit_v2.overlay)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_collective_formation)

set(TAPESTRY_OS_ROOT      ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_CSM       ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_TRANSPORT ${TAPESTRY_OS_ROOT}/subsys/transport)
set(TAPESTRY_OS_BOARDS    ${TAPESTRY_OS_ROOT}/boards)

# Core L4 stack + transport.
target_sources(app PRIVATE
    src/main.c
    src/formation.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_TRANSPORT}/transport.c
    ${TAPESTRY_OS_TRANSPORT}/gossip.c
)

# BLE transceiver — micro:bit only (CONFIG_BT=y in board overlay).
if(CONFIG_BT)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_ble.c)
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
    ${TAPESTRY_OS_ROOT}/include                    # <tapestry/csm.h>, <tapestry/transport.h>, <tapestry/substrate.h>
    ${TAPESTRY_OS_CSM}                             # world_model.c; types via <tapestry/csm.h>
    ${TAPESTRY_OS_TRANSPORT}                       # gossip.h, transceiver_ble.h, net_init.h
    ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2          # cutebot.h
    src
)

target_link_libraries(app PRIVATE m)
