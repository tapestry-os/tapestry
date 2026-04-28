cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_hw_element)

set(TAPESTRY_OS_ROOT      ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os)
set(TAPESTRY_OS_CSM       ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_SCR       ${TAPESTRY_OS_ROOT}/subsys/scr)
set(TAPESTRY_OS_TRANSPORT ${TAPESTRY_OS_ROOT}/subsys/transport)

# Core L4/L5 stack — always compiled.
target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_SCR}/scr.c
)

# UDP gossip transport + network bring-up — ESP32 and RA8D1 only.
if(CONFIG_NETWORKING)
    target_sources(app PRIVATE
        src/net_init.c
        ${TAPESTRY_OS_TRANSPORT}/udp/udp_gossip.c
    )
endif()

# BLE gossip transport — ESP32 (bridge) and micro:bit.
if(CONFIG_BT)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/ble/ble_gossip.c)
endif()

# Cutebot motor/LED driver — micro:bit only.
if(CONFIG_I2C)
    target_sources(app PRIVATE src/cutebot.c)
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include          # <tapestry/csm.h>, <tapestry/scr.h>, <tapestry/wire.h>
    ${TAPESTRY_OS_CSM}                   # world_model.h, state.h
    ${TAPESTRY_OS_SCR}                   # scr.h internals
    ${TAPESTRY_OS_TRANSPORT}/udp         # udp_gossip.h
    ${TAPESTRY_OS_TRANSPORT}/ble         # ble_gossip.h
    src                                  # net_init.h, cutebot.h
)

target_link_libraries(app PRIVATE m)
