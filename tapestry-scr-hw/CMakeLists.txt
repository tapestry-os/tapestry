cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_hw_element)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-os)
set(TAPESTRY_OS_CSM  ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_SCR  ${TAPESTRY_OS_ROOT}/subsys/scr)

# Protocol headers live in the sim directories (shared wire format).
# scr_protocol.h includes ../tapestry-csm-sim/sim_protocol.h via a relative
# path that resolves correctly when tapestry-scr-sim/ is on the include path.
set(TAPESTRY_CSM_SIM ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-csm-sim)
set(TAPESTRY_SCR_SIM ${CMAKE_CURRENT_SOURCE_DIR}/../tapestry-scr-sim)

target_sources(app PRIVATE
    src/main.c
    src/net_init.c
    src/comms.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_SCR}/scr.c
)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include   # <tapestry/csm.h>, <tapestry/scr.h>
    ${TAPESTRY_OS_CSM}            # world_model.h
    ${TAPESTRY_OS_SCR}            # scr.h (internal)
    ${TAPESTRY_CSM_SIM}           # sim_protocol.h
    ${TAPESTRY_SCR_SIM}           # scr_protocol.h
    src
)

target_link_libraries(app PRIVATE m)
