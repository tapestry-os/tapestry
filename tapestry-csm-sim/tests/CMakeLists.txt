cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_csm)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_CSM  ${TAPESTRY_OS_ROOT}/subsys/csm)

target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_CSM}/world_model.c
)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include   # <tapestry/csm.h>
    ${TAPESTRY_OS_CSM}            # internal: world_model.c includes world_model.h
)

target_link_libraries(app PRIVATE m)
