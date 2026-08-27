# Demo — Motor Test

A motor characterisation sweep used to find the usable throttle/speed range
of a robot's drive system before running a higher-level demo. Two boards are
supported, each with its own sweep:

- **crazyflie21bl** — collective thrust sweep to find the minimum ESC PWM
  width at which all four motors spin (BLHeli_S RC PWM mode).
- **bbc_microbit_v2** (Cutebot) — forward-speed sweep to find the stiction
  threshold and measure speed vs. throttle.

Results feed into `examples/cutebot-formation` (`DEMO_MAX_SPEED`,
`DEMO_WHEEL_TRACK`) and into `tapestry-os/boards/crazyflie21bl/crazyflie21bl.c`
(`CF21BL_PWM_MIN_NS`).

## crazyflie21bl — ESC characterisation

**Procedure**

1. **Remove all propellers.** Secure the frame to the bench (tape or clamps).
2. Build and flash:
   ```sh
   west build -p always -b crazyflie21bl tapestry/examples/motor-test
   cfloader flash build/zephyr/zephyr.bin stm32-dfu   # activate ~/code/tapestry/.venv first
   ```
3. Connect a console (see below).
4. Power on. ESC arming runs automatically (idle PWM + PC15 reset pulse,
   ~3 s hold).
5. The sweep runs steps `100%, 5%, 8%, 10%, 12%, 15%, 18%, 20%, 25%, 30%,
   40%, 50%`, each driving for 3 s then pausing 2 s. The 100% step is first,
   as a wiring sanity check — if it doesn't spin, the ESCs likely still need
   reflashing to RC PWM mode.
6. Note the lowest step where **all four** motors audibly/tactilely spin.
7. Compute `CF21BL_PWM_MIN_NS = 1000000 + threshold_pct * 10000` and set it in
   `tapestry-os/boards/crazyflie21bl/crazyflie21bl.c`.

**Measured result (2026-06-09):** all four motors spin from 18% (1180 µs),
clean at 20% (1200 µs). `CF21BL_PWM_MIN_NS = 1180000` is already set.

### Console

USART3 (PC10/PC11) and the CRTP radio backend are both enabled simultaneously:

```sh
minicom -D /dev/ttyUSB0 -b 115200          # wired USART3
python3 tapestry/tapestry-os/tools/crazyflie_console.py    # CRTP radio (crazyradio2)
```

## bbc_microbit_v2 (Cutebot) — speed/stiction sweep

**Procedure**

1. Place the robot on a flat surface with at least 600 mm clear ahead.
2. Mark the start position.
3. Build and flash (appears as a USB mass storage device):
   ```sh
   west build -p always -b bbc_microbit_v2 tapestry/examples/motor-test
   cp build/zephyr/zephyr.hex /media/$USER/MICROBIT/
   ```
4. Watch serial output — each step is announced before the motors run.
5. The sweep runs steps `20%, 25%, 30%, 35%, 40%, 50%, 75%, 100%`, each
   driving for 3 s then pausing 2 s.
6. Note the first step that causes movement (stiction threshold).
7. For each moving step, measure the distance from the mark to where the
   robot stopped:
   ```
   speed_mm_per_s = distance_mm / 3000
   DEMO_MAX_SPEED = speed_mm_per_s / 0.22 * 100 / arena_width_mm   (calibrate at 22%)
   ```
