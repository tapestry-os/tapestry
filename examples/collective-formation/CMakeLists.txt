cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_collective_formation)

set(TAPESTRY_OS_ROOT  ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_CSM   ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_SCR   ${TAPESTRY_OS_ROOT}/subsys/scr)
set(TAPESTRY_HW       ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw/src)
set(TAPESTRY_CSM_SIM  ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-csm-sim)

# Core L4 stack — no L5 SCR compiled or linked.
target_sources(app PRIVATE
    src/main.c
    src/formation.c
    ${TAPESTRY_OS_CSM}/world_model.c
)

# BLE gossip layer — micro:bit only (CONFIG_BT=y in board overlay).
if(CONFIG_BT)
    target_sources(app PRIVATE ${TAPESTRY_HW}/ble_gossip.c)
endif()

# Cutebot motor/LED driver — micro:bit only (CONFIG_I2C=y in board overlay).
if(CONFIG_I2C)
    target_sources(app PRIVATE ${TAPESTRY_HW}/cutebot.c)
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include   # <tapestry/csm.h>, <tapestry/scr.h> (types only)
    ${TAPESTRY_OS_CSM}            # state.h, world_model.h
    ${TAPESTRY_OS_SCR}            # scr.h type defs (needed by cutebot.h)
    ${TAPESTRY_CSM_SIM}           # sim_protocol.h (BLE wire format)
    ${TAPESTRY_HW}                # ble_gossip.h, cutebot.h
    src
)

target_link_libraries(app PRIVATE m)
