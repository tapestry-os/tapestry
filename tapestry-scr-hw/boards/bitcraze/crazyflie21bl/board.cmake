# Programming via ST-Link + OpenOCD (SWD).
# The CF2.1 also supports DFU via USB; use "west flash --runner dfu-util"
# once the Crazyflie bootloader is preserved in the first 64 kB of flash.
board_runner_args(openocd
  "--config=interface/stlink.cfg"
  "--config=target/stm32f4x.cfg"
)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
