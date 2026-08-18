cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21bl lives in tapestry-scr-hw/boards/).
# Must be set before find_package so Zephyr finds the board DTS.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../tapestry-scr-hw)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_transport)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# gossip.c is the unit under test; world_model.c is the far end of the round
# trip (a drained frame has to land somewhere observable).  No transceiver
# backend is linked — the test supplies its own loopback vtable, which is
# what lets this run on native_sim with no radio, no network and no board.
target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_ROOT}/subsys/csm/world_model.c
    ${TAPESTRY_OS_ROOT}/subsys/transport/gossip.c
)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include              # <tapestry/csm.h>, wire.h, transceiver.h
    ${TAPESTRY_OS_ROOT}/subsys/transport     # gossip.h
)

target_link_libraries(app PRIVATE m)
