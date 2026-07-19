cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)
list(APPEND DTS_ROOT   ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

# Second overlay reconfigures USART3 to 230400 baud for the lighthouse deck
# FPGA data stream — this also means the wired USART3 console is
# unavailable; use CRTP radio.
set(DTC_OVERLAY_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/crazyflie21bl/crazyflie21bl.overlay"
    "${CMAKE_CURRENT_SOURCE_DIR}/boards/crazyflie21bl_lh.overlay"
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
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_lighthouse.c
    # IWDG: keep watchdog alive (bootloader arms it before jumping to us)
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_iwdg.c
    # Shared USART6 TX mutex — required whenever the syslink P2P transport
    # is linked; see cf21bl_syslink_tx.h.
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_syslink_tx.c
)

# L6 BSE + L7 Choreographer — the choreo demo mode's target source.
# (No L5 SCR: main.c synthesizes the quorum signal from L4 freshness.)
if(CONFIG_DEMO_MODE_CHOREO)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_ROOT}/subsys/bse/bse.c
        ${TAPESTRY_OS_ROOT}/subsys/choreo/choreo.c
    )
    target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/include   # <tapestry/choreo.h>
    )
endif()

# CRTP radio console — safe to combine with the syslink P2P transport since
# both now serialize USART6 TX through cf21bl_syslink_tx_mutex (previously
# excluded here; see git history).  USART3 is not available as a wired
# console alternative once the lighthouse deck overlay is applied above.
# Build with -DCONFIG_LOG=n for a quieter radio (formation flights with 3
# drones share one nRF51 radio config — no per-drone channel plan exists
# yet, see Phase D notes, so treat concurrent consoles as best-effort).
if(CONFIG_LOG)
    target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_crtp_log.c)
endif()

# Battery telemetry + forced landing on critical voltage (independent per
# drone — see cf21bl_stabilizer.c's PM-critical force-land path).
if(CONFIG_CF21BL_PM)
    target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_pm.c)
endif()

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
