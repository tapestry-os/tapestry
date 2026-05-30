cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21).  Must be set before find_package.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_cf21_mix)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include              # <tapestry/substrate.h>
    ${TAPESTRY_OS_ROOT}/boards/crazyflie21  # "crazyflie21_mix.h"
)

# crazyflie21_mix.h is header-only — no additional C sources required.
# Link -lm for fabsf() used in test epsilon comparisons.
target_link_libraries(app PRIVATE m)
