cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scr_element)

set(TAPESTRY_OS_ROOT  ${CMAKE_CURRENT_SOURCE_DIR}/../../../tapestry-os)
set(TAPESTRY_OS_CSM   ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_SCR   ${TAPESTRY_OS_ROOT}/subsys/scr)
set(TAPESTRY_OS_BSE   ${TAPESTRY_OS_ROOT}/subsys/bse)
set(TAPESTRY_SDK      ${CMAKE_CURRENT_SOURCE_DIR}/../../../sdk)
set(TAPESTRY_CSM_SIM  ${CMAKE_CURRENT_SOURCE_DIR}/../../../tapestry-csm-sim)
set(CSM_ELEM_SRC      ${TAPESTRY_CSM_SIM}/zephyr/element/src)

target_sources(app PRIVATE
    src/main.c
    src/comms_scr.c
    ${CSM_ELEM_SRC}/comms.c        # L4 UDP transport: reused verbatim
    ${CSM_ELEM_SRC}/movement.c     # random walk + repulsion: reused verbatim
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_SCR}/scr.c
    ${TAPESTRY_OS_BSE}/bse_stub.c
    ${TAPESTRY_SDK}/src/choreo_stub.c
)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include    # <tapestry/scr.h>, <tapestry/bse.h>
    ${TAPESTRY_SDK}/include        # <tapestry/choreo.h>
    ${TAPESTRY_OS_CSM}             # internal: world_model.h, state.h
    ${CSM_ELEM_SRC}                # comms.h, movement.h (from csm element)
    ${TAPESTRY_CSM_SIM}            # sim_protocol.h
    src/                           # comms_scr.h
)

# Do not link -lm here: zephyr.elf is built as a relocatable (-r), which
# forces static libm and pulls in glibc ifunc resolvers that reference
# _dl_x86_cpu_features from ld.so — unavailable in native_sim's final link.
# The native_sim Makefile's FINALLINK_FLAGS already supplies -lm as shared
# libm, where sinf/cosf resolve cleanly at runtime.
