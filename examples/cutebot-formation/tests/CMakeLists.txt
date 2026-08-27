cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_cutebot_formation)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../tapestry-os)

target_sources(app PRIVATE
    src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/formation.c
    # L6/L7 under test: the choreo demo mode's script engine + BSE
    ${TAPESTRY_OS_ROOT}/subsys/scr/scr.c
    ${TAPESTRY_OS_ROOT}/subsys/bse/bse.c
    ${TAPESTRY_OS_ROOT}/subsys/choreo/choreo.c
    # scr_tick()'s trusted-peer scan reads the world model through
    # wm_get_entry(); the suite builds its own wm.entries[] by hand, but
    # the accessor itself lives here.
    ${TAPESTRY_OS_ROOT}/subsys/csm/world_model.c
    # demo_set_leds() calls substrate_set_signal(); the null substrate
    # provides it (same trick as cf21bl-formation's own tests).
    ${TAPESTRY_OS_ROOT}/boards/substrate_null.c)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include                # <tapestry/csm.h>, <tapestry/substrate.h>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../sdk/include   # <tapestry/choreo.h>
    ${CMAKE_CURRENT_SOURCE_DIR}/../src)        # "formation.h"

# No `target_link_libraries(app PRIVATE m)` here, however natural that
# looks for bse.c's sqrtf/sinf/cosf/ceilf: Zephyr links the application
# image with -nostdlib, so -lm resolves to the STATIC libm.a and drags in
# ifunc resolvers (__ceilf_ifunc, __sincosf_ifunc) that reference
# _dl_x86_cpu_features, a glibc-internal symbol the native_simulator's
# final link cannot resolve.  zephyr/scripts/native_simulator/Makefile
# already links -lm dynamically into the runner executable, which is what
# actually resolves these — cf21bl-formation's suite builds the same math
# and names no libraries at all.
