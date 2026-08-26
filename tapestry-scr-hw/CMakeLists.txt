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
elseif("${BOARD}" STREQUAL "apollo510_evb")
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os/boards/apollo510_evb/apollo510_evb.overlay)
elseif("${BOARD}" STREQUAL "ek_ra8d1")
    # Adds only a tapestry,gossip-uart chosen node pointing at UART3, so the
    # RA8D1 can join a wired gossip bus.  Inert in the default Ethernet/UDP
    # build — nothing reads the node unless transceiver_uart.c is compiled in.
    set(DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os/boards/ek_ra8d1/ek_ra8d1.overlay)
endif()

# Apollo510 EVB: make `west flash` reliable.
#
# The Apollo510 is a secure part whose bootloader must run to completion after
# reset.  J-Link's default reset races it: the runner's generated script goes
# `r` straight into `loadfile` with no `halt` between, so the flash loader can
# start while the bootloader is still running.  Measured on hardware
# 2026-08-25: 2 failures in 4 runs, each aborting after "Programming flash
# [100%] Done" and leaving a HALF-WRITTEN image that boots and then hard-faults
# — a failure that reads convincingly as a firmware bug.
#
# Reset type 2 fixes it: 5 clean runs versus 2 failures in 4 with the default
# reset.  Under the old rate a clean run of 5 had roughly a 3% chance, so the
# improvement is real — though 5 trials cannot prove a race never fires.  west
# exits non-zero on failure, so a bad flash is always visible at the shell.
# `--speed 500` was tried and did not help.
#
# Injected via the BOARD_RUNNER_ARGS_<runner> cache variable, which
# board_finalize_runner_args() appends last of all (so it wins over the board's
# own args).  The documented app_set_runner_args() macro hook was tried first
# and did not take effect on Zephyr 4.4.0-rc1 — board_finalize_runner_args()
# demonstrably ran (its --dt-flash=y reached runners.yaml) but did not see the
# macro.  Unexplained; the cache variable is read by the same function two
# lines later and does work.
if("${BOARD}" STREQUAL "apollo510_evb")
    set(BOARD_RUNNER_ARGS_jlink "--reset-type=2" CACHE INTERNAL
        "Apollo510 secure-bootloader-safe JLink reset")
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
    ${TAPESTRY_OS_CSM}/facts.c
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

# Wired serial gossip bus — the only medium available to radio-less boards
# (Apollo510 EVB).  Board-agnostic: needs a spare interrupt-driven UART and a
# `tapestry,gossip-uart` chosen node in the board overlay.
if(CONFIG_TAPESTRY_TRANSCEIVER_UART)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_uart.c)
endif()

# CF2.1 early IWDG kick — must be compiled unconditionally so it runs even in
# minimal blink-test builds (CONFIG_PWM=n).  The Bitcraze bootloader arms the
# IWDG before jumping to us; without this, the watchdog fires within ~100-353ms.
#
# CF2.1 CRTP log backend — sends Zephyr LOG_* output as syslink console packets
# over USART6 to the nRF51 co-processor; crazyflie_console.py receives them via radio.
# Compiled only when CONFIG_LOG=y (i.e. the log subsystem is present).
# cf21bl_syslink_tx.c (the shared USART6 TX mutex) is compiled unconditionally
# alongside it so any combination of USART6 writers (console, PM, syslink P2P
# transport) can be linked together safely.
if("${BOARD}" STREQUAL "crazyflie21bl")
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_iwdg.c
        ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_syslink_tx.c)
    if(CONFIG_LOG)
        target_sources(app PRIVATE ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_crtp_log.c)
    endif()
endif()

# Substrate: CF2.1 brushless on crazyflie21bl, Cutebot on micro:bit,
# LED/button status output on apollo510_evb, null elsewhere.
#
# Boards with a substrate of their own are matched by NAME before the
# CONFIG_* heuristics below.  Those heuristics infer a substrate from which
# buses a board happens to enable, which is only sound while every such board
# has exactly one plausible substrate — apollo510_evb breaks that (it has no
# actuators at all, and enabling I2C or PWM on it for an unrelated reason
# would silently select the Cutebot or Crazyflie driver).
#
# Within the heuristics, PWM must be checked before I2C: the CF21 stabilizer
# uses both (PWM for motors, I2C for the BMI088 IMU), so checking I2C first
# would wrongly select the Cutebot path.
if("${BOARD}" STREQUAL "apollo510_evb")
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/apollo510_evb/substrate_apollo510_evb.c)
elseif(CONFIG_PWM)
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
