cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_cf21bl_formation)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../tapestry-os)

target_sources(app PRIVATE
    src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/formation.c
    # L6/L7 under test: the choreo demo mode's script engine + BSE
    ${TAPESTRY_OS_ROOT}/subsys/bse/bse.c
    ${TAPESTRY_OS_ROOT}/subsys/choreo/choreo.c
    # L5 rank/role state: bse.c and choreo.c read the swarm view through
    # scr_get_*()/scr_peer_is_trusted(), and the frame/track tests drive
    # scr_init()/scr_tick() directly to set up the anchor selectors.
    ${TAPESTRY_OS_ROOT}/subsys/scr/scr.c
    # scr_tick()'s trusted-peer scan reads the world model through
    # wm_get_entry(); the suite builds its own wm.entries[] by hand, but
    # the accessor itself lives here.
    ${TAPESTRY_OS_ROOT}/subsys/csm/world_model.c
    # demo_set_leds() calls substrate_set_signal(); the null substrate
    # provides it (same trick as the -DCONFIG_PWM=n gossip-only build).
    ${TAPESTRY_OS_ROOT}/boards/substrate_null.c)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include                # <tapestry/csm.h>, <tapestry/substrate.h>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../sdk/include   # <tapestry/choreo.h>
    ${CMAKE_CURRENT_SOURCE_DIR}/../src)        # "formation.h"

# formation.h derives DEMO_ARENA_LIMIT_M from this board Kconfig symbol,
# which doesn't exist on native_sim — provide the flight value.
target_compile_definitions(app PRIVATE CONFIG_CF21BL_POS_MAX_M=3)
