cmake_minimum_required(VERSION 3.20.0)

if("${BOARD}" STREQUAL "bbc_microbit_v2")
    list(APPEND EXTRA_DTC_OVERLAY_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards/bbc_microbit_v2/bbc_microbit_v2.overlay)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_cutebot_formation)

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

# #ifndef-guarded in core csm.h — overridden here (app-scoped, every other
# consumer keeps the 1500ms default) because a real 4-robot BLE mesh on
# this hardware showed continuous fresh/stale flicker (active peer count
# bouncing 1-2 for 60+s straight, never reaching 3) at the default's only
# 3 gossip-cycle grace window: passive BLE advertising has no acks/
# retries, and collision/loss with 4 simultaneous broadcasters is real.
# 3000ms gives 6 cycles of tolerance instead of 3 — a lab/demo tradeoff
# (slower to detect a GENUINE peer loss) not appropriate to apply to
# cf21bl-formation's flight-tested default without its own validation.
target_compile_definitions(app PRIVATE WM_STALE_THRESHOLD_MS=3000)

# L5 SCR + L6 BSE + L7 Choreographer — the choreo demo mode's target source.
if(CONFIG_DEMO_MODE_CHOREO)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_ROOT}/subsys/scr/scr.c
        ${TAPESTRY_OS_ROOT}/subsys/bse/bse.c
        ${TAPESTRY_OS_ROOT}/subsys/choreo/choreo.c
    )
    target_include_directories(app PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/include   # <tapestry/choreo.h>
    )
    # Both #ifndef-guarded in core choreo.c/choreo.h with a documented
    # "must stay equal" invariant between them — overridden here (app-
    # scoped, cf21bl-formation/webots-formation keep the flight-tested
    # 2000ms default) because real BLE gossip jitter on this hardware
    # needs a bit more margin before element_lost/element_joined fires:
    # 2026-09-11 hardware run showed the ring settle, pause ~3s, then
    # keep re-adjusting — traced to bse.c recomputing each element's FORM
    # vertex from live rank/count every tick, with a single dropped/
    # delayed gossip frame enough to shift it. 2000ms wasn't quite long
    # enough margin above that jitter; 3000ms is the first bump to try.
    target_compile_definitions(app PRIVATE
        CHOREO_MEMBERSHIP_HOLD_MS=3000u
        CHOREO_REMOTE_ADOPT_HOLD_MS=3000u
    )
endif()

# BLE transceiver — micro:bit only (CONFIG_BT=y in board overlay).
if(CONFIG_BT)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_ble.c)
endif()

# Substrate: Cutebot on micro:bit, null stub everywhere else.
if(CONFIG_I2C)
    target_sources(app PRIVATE
        ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2/substrate_cutebot.c
        ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2/cutebot.c
        ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2/cutebot_line.c
        ${TAPESTRY_OS_BOARDS}/bbc_microbit_v2/cutebot_line_bench.c
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
