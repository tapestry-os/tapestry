cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21bl).  Must be set before find_package.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_cf21bl_mix)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

target_sources(app PRIVATE src/main.c)

# crazyflie21bl hardware additions:
#   cf21bl_iwdg.c — extends the bootloader IWDG to ~32 s so tests can run without
#                 the watchdog resetting the board mid-suite.
#   cf21bl_crtp_log.c — routes ztest output (via CONFIG_LOG_PRINTK) over USART6
#                     to the nRF51 radio co-processor; crazyflie_console.py shows
#                     PASS/FAIL results on the host without a serial cable.
if("${BOARD}" STREQUAL "crazyflie21bl")
    target_sources(app PRIVATE ${TAPESTRY_OS_ROOT}/boards/crazyflie21bl/cf21bl_iwdg.c)
    if(CONFIG_LOG)
        target_sources(app PRIVATE
            ${TAPESTRY_OS_ROOT}/boards/crazyflie21bl/cf21bl_crtp_log.c
            ${TAPESTRY_OS_ROOT}/boards/crazyflie21bl/cf21bl_syslink_tx.c)
    endif()
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include              # <tapestry/substrate.h>
    ${TAPESTRY_OS_ROOT}/boards/crazyflie21bl  # "crazyflie21bl_mix.h"
)

# crazyflie21bl_mix.h is header-only — no additional C sources required.
# Link -lm for fabsf() used in test epsilon comparisons.
target_link_libraries(app PRIVATE m)
