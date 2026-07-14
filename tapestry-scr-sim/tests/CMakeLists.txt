cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21bl lives in tapestry-scr-hw/boards/).
# Must be set before find_package so Zephyr finds the board DTS.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_scr)

set(TAPESTRY_OS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)
set(TAPESTRY_OS_CSM  ${TAPESTRY_OS_ROOT}/subsys/csm)
set(TAPESTRY_OS_SCR  ${TAPESTRY_OS_ROOT}/subsys/scr)

target_sources(app PRIVATE
    src/main.c
    ${TAPESTRY_OS_CSM}/world_model.c
    ${TAPESTRY_OS_SCR}/scr.c
)

# crazyflie21bl hardware additions:
#   cf21bl_iwdg.c     — extends bootloader IWDG to ~32 s so the full test suite
#                     completes before the watchdog resets the board.
#   cf21bl_crtp_log.c — routes ztest output over USART6 to the nRF51 radio;
#                     read_console.py shows PASS/FAIL on the host.
if("${BOARD}" STREQUAL "crazyflie21bl")
    target_sources(app PRIVATE ${TAPESTRY_OS_ROOT}/boards/crazyflie21bl/cf21bl_iwdg.c)
    if(CONFIG_LOG)
        target_sources(app PRIVATE
            ${TAPESTRY_OS_ROOT}/boards/crazyflie21bl/cf21bl_crtp_log.c
            ${TAPESTRY_OS_ROOT}/boards/crazyflie21bl/cf21bl_syslink_tx.c)
    endif()
endif()

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_ROOT}/include   # <tapestry/csm.h>, <tapestry/scr.h>
    ${TAPESTRY_OS_CSM}            # world_model.c; types via <tapestry/csm.h>
)

target_link_libraries(app PRIVATE m)
