cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_ble_observer)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

target_sources(app PRIVATE src/main.c)

# Headers only: <tapestry/wire.h> for frame offsets, <tapestry/csm.h> for
# ELEMENT_ID_INVALID.  None of the Tapestry stack is linked.
target_include_directories(app PRIVATE ${TAPESTRY_OS_ROOT}/include)
