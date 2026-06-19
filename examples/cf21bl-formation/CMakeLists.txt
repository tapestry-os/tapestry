cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

set(DTC_OVERLAY_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/crazyflie21bl/crazyflie21bl.overlay"
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_cf21bl_formation)

set(TAPESTRY_OS_ROOT      ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_CSM       ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_TRANSPORT ${TAPESTRY_OS_ROOT}/subsys/transport)
set(TAPESTRY_OS_BOARDS    ${TAPESTRY_OS_ROOT}/boards)

# Core gossip stack (L4 CSM, no SCR/L5)
target_sources(app PRIVATE
    src/main.c
    src/formation.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_TRANSPORT}/transport.c
    ${TAPESTRY_OS_TRANSPORT}/gossip.c
    ${TAPESTRY_OS_TRANSPORT}/transceiver_syslink_p2p.c
    # IWDG: keep watchdog alive (bootloader arms it before jumping to us)
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_iwdg.c
    # NOTE: cf21bl_crtp_log.c is intentionally EXCLUDED — it also TX on
    # USART6 and would corrupt syslink P2P frames.
)

# Substrate: motor + stabilizer sources only when PWM is enabled.
# Build with -DCONFIG_PWM=n to run gossip-only with no ESC init and no beeping.
if(CONFIG_PWM)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/substrate_crazyflie21bl.c
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/crazyflie21bl.c
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_stabilizer.c
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_imu.c
    )
else()
    target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/substrate_null.c)
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include           # <tapestry/...>
    ${TAPESTRY_OS_CSM}                    # world_model internals
    ${TAPESTRY_OS_TRANSPORT}              # gossip.h
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl   # cf21bl_stabilizer.h etc.
    src
)

target_link_libraries(app PRIVATE m)
