cmake_minimum_required(VERSION 3.20.0)

# Board overlays from tapestry-os pair hardware drivers with their devicetree
# requirements.  Must be set before find_package(Zephyr).
if("${BOARD}" STREQUAL "bbc_microbit_v2")
    list(APPEND EXTRA_DT_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os/boards/bbc_microbit_v2/bbc_microbit_v2.overlay)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_hw_element)

set(TAPESTRY_OS_ROOT      ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os)
set(TAPESTRY_OS_CSM       ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_SCR       ${TAPESTRY_OS_ROOT}/subsys/scr)
set(TAPESTRY_OS_TRANSPORT ${TAPESTRY_OS_ROOT}/subsys/transport)
set(TAPESTRY_OS_BSE       ${TAPESTRY_OS_ROOT}/subsys/bse)
set(TAPESTRY_OS_BOARDS    ${TAPESTRY_OS_ROOT}/boards)
set(TAPESTRY_SDK          ${CMAKE_CURRENT_SOURCE_DIR}/../sdk)

# Core L4/L5 stack + L6 BSE stub + L7 SDK stub + transport multiplexer.
target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_SCR}/scr.c
    ${TAPESTRY_OS_BSE}/bse_stub.c
    ${TAPESTRY_SDK}/src/app_stub.c
    ${TAPESTRY_OS_TRANSPORT}/transport.c
)

# UDP gossip backend + network bring-up — ESP32 and RA8D1 only.
if(CONFIG_NETWORKING)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_TRANSPORT}/udp/udp_gossip.c
        ${TAPESTRY_OS_TRANSPORT}/udp/net_init.c
    )
endif()

# BLE gossip backend — ESP32 (bridge) and micro:bit.
if(CONFIG_BT)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/ble/ble_gossip.c)
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
    ${TAPESTRY_OS_ROOT}/include                    # <tapestry/csm.h>, <tapestry/scr.h>, <tapestry/transport.h>, <tapestry/substrate.h>, <tapestry/bse.h>
    ${TAPESTRY_SDK}/include                        # <tapestry/app.h>
    ${TAPESTRY_OS_CSM}                             # world_model.h, state.h (used by transport.c)
    ${TAPESTRY_OS_SCR}                             # scr.h internals
    ${TAPESTRY_OS_TRANSPORT}/ble                   # ble_gossip.h (used by transport.c)
    ${TAPESTRY_OS_TRANSPORT}/udp                   # udp_gossip.h, net_init.h (used by transport.c)
    ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2          # cutebot.h (used by substrate_cutebot.c)
    src
)

target_link_libraries(app PRIVATE m)
