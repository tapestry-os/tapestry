cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21br).  Must be set before find_package.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_cf21_mix)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

target_sources(app PRIVATE src/main.c)

# crazyflie21br hardware additions:
#   cf21_iwdg.c — extends the bootloader IWDG to ~32 s so tests can run without
#                 the watchdog resetting the board mid-suite.
#   cf21_crtp_log.c — routes ztest output (via CONFIG_LOG_PRINTK) over USART6
#                     to the nRF51 radio co-processor; read_console.py shows
#                     PASS/FAIL results on the host without a serial cable.
if("${BOARD}" STREQUAL "crazyflie21br")
    target_sources(app PRIVATE ${TAPESTRY_OS_ROOT}/boards/crazyflie21br/cf21_iwdg.c)
    if(CONFIG_LOG)
        target_sources(app PRIVATE ${TAPESTRY_OS_ROOT}/boards/crazyflie21br/cf21_crtp_log.c)
    endif()
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include              # <tapestry/substrate.h>
    ${TAPESTRY_OS_ROOT}/boards/crazyflie21br  # "crazyflie21br_mix.h"
)

# crazyflie21br_mix.h is header-only — no additional C sources required.
# Link -lm for fabsf() used in test epsilon comparisons.
target_link_libraries(app PRIVATE m)
