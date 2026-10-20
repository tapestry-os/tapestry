cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_leak_sensor)

set(TAPESTRY_OS_ROOT      ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_CSM       ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_TRANSPORT ${TAPESTRY_OS_ROOT}/subsys/transport)

# L4 world model + shared facts, transport, and L5/L6/L7: the sensor runs
# the same Choreo as the robots (it lands on the script's catch-all track).
target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_CSM}/facts.c
    ${TAPESTRY_OS_TRANSPORT}/transport.c
    ${TAPESTRY_OS_TRANSPORT}/gossip.c
    ${TAPESTRY_OS_ROOT}/subsys/scr/scr.c
    ${TAPESTRY_OS_ROOT}/subsys/bse/bse.c
    ${TAPESTRY_OS_ROOT}/subsys/choreo/choreo.c
)

if(CONFIG_BT)
    target_sources(app PRIVATE ${TAPESTRY_OS_TRANSPORT}/transceiver_ble.c)
endif()

# Must match cutebot-formation's override (its CMakeLists.txt explains the
# value): the membership debounce is part of how the script behaves.
target_compile_definitions(app PRIVATE
    CHOREO_MEMBERSHIP_HOLD_MS=3000u
    CHOREO_REMOTE_ADOPT_HOLD_MS=3000u
)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include                    # <tapestry/csm.h>, <tapestry/transport.h>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/include  # <tapestry/choreo.h>
    ${TAPESTRY_OS_CSM}
    ${TAPESTRY_OS_TRANSPORT}                       # gossip.h, transceiver_ble.h
    src
)

target_link_libraries(app PRIVATE m)
