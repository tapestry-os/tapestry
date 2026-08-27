cmake_minimum_required(VERSION 3.20.0)

# Out-of-tree board definitions (crazyflie21bl).  Must be before find_package.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-scr-hw)

# tapestry-os/dts/bindings/ holds custom DTS bindings.
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os)

# boards/crazyflie21bl.overlay in this directory is auto-applied by Zephyr.
# It contains only I2C3 + BMP388 (no BMI088 nodes, so the bmi08x driver is
# not pulled in and the Zephyr 4.4.0-rc1 rtio_bus compile bug is avoided).

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tapestry_baro_test)

set(TAPESTRY_OS_BOARDS ${CMAKE_CURRENT_SOURCE_DIR}/../../tapestry-os/boards)

target_sources(app PRIVATE
    src/main.c
    # IWDG: Bitcraze bootloader arms the watchdog; without this the board
    # resets after ~32 s.  The timer in cf21bl_iwdg.c kicks it every 10 s.
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_iwdg.c
    # CRTP log backend: routes Zephyr LOG_* to nRF51 → crazyradio2 → crazyflie_console.py
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_crtp_log.c
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_syslink_tx.c
)

target_include_directories(app PRIVATE
    ${TAPESTRY_OS_BOARDS}/crazyflie21bl
)
