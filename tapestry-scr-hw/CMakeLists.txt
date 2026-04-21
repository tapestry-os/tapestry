cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_hw_element)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os)
set(TAPESTRY_OS_CSM  ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_SCR  ${TAPESTRY_OS_ROOT}/subsys/scr)

set(TAPESTRY_CSM_SIM ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-csm-sim)
set(TAPESTRY_SCR_SIM ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-scr-sim)

# Core L4/L5 stack — always compiled.
target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_SCR}/scr.c
)

# UDP comms layer — ESP32 and RA8D1 only (CONFIG_NETWORKING=y in board overlay).
if(CONFIG_NETWORKING)
    target_sources(app PRIVATE
        src/net_init.c
        src/comms.c
    )
endif()

# BLE gossip layer — ESP32 (bridge) and micro:bit (CONFIG_BT=y in board overlay).
if(CONFIG_BT)
    target_sources(app PRIVATE src/ble_gossip.c)
endif()

# Cutebot motor/LED driver — micro:bit only (CONFIG_I2C=y in board overlay).
if(CONFIG_I2C)
    target_sources(app PRIVATE src/cutebot.c)
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include
    ${TAPESTRY_OS_CSM}
    ${TAPESTRY_OS_SCR}
    ${TAPESTRY_CSM_SIM}
    ${TAPESTRY_SCR_SIM}
    src
)

target_link_libraries(app PRIVATE m)
