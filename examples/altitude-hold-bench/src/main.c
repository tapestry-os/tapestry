/*
 * altitude-hold-bench — WS3 estimator/cascade validation, ESCs never armed
 *
 * Boots the stabilizer with CONFIG_CF21BL_ALTITUDE_HOLD active and sends one
 * non-idle setpoint so the full complementary-filter + cascaded position/
 * velocity/thrust path in cf21bl_stabilizer.c actually runs (it resets to
 * zero and skips computation entirely at the idle sentinel). Its
 * "alt=... vz=... target=... vz_sp=... T=..." log line then fires at ~2 Hz
 * for as long as this app runs.
 *
 * Motors cannot spin regardless of the thrust the stabilizer computes:
 * substrate_set_power() is never called here, so crazyflie21bl.c's g_armed
 * stays false and every PWM write is forced to the 1 ms idle pulse
 * (see cf21bl_set_motors()/motor_to_ns() — the armed check gates output
 * independently of whatever value the stabilizer thread produces).
 *
 * Use this to sanity-check the estimator before ever risking hardware in
 * flight: hand-lift/tilt/set-down the drone and confirm alt/vz track the
 * motion in the right direction and settle back near zero at rest.
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/tapestry/examples/altitude-hold-bench
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu
 * Read:   minicom -D /dev/ttyUSB0 -b 115200
 *   or:   picocom /dev/ttyACM0
 *   or:   python3 tapestry/tapestry-os/tools/crazyflie_console.py
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/substrate.h>

LOG_MODULE_REGISTER(alt_hold_bench, LOG_LEVEL_INF);

/* target_alt = linear.z + CF21BL_ALT_SP_OFFSET (1.0 m, cf21bl_stabilizer.c),
 * so -0.7 → 0.3 m above home. Comfortably clear of the -0.9 idle threshold
 * so the estimator/cascade actually runs instead of resetting to zero. */
#define BENCH_SETPOINT_LZ   -0.7f

int main(void)
{
    LOG_INF("=== altitude-hold-bench: WS3 estimator check — ESCs NEVER armed ===");

    if (substrate_init() != 0) {
        LOG_ERR("substrate_init failed");
        return -1;
    }

    /* cf21bl_stabilizer_start() (called from substrate_init()) runs gyro
     * calibration and averages ~1 s of baro readings for the home altitude
     * before its main loop starts — give it a moment. */
    k_msleep(2000);

    LOG_INF("Sending setpoint (target=0.3 m above home). Motors are NOT armed — "
            "hand-lift/tilt the drone and watch alt/vz below track it.");

    substrate_twist_t sp = { .linear = { .z = BENCH_SETPOINT_LZ } };

    /* Resend well inside CF21BL_SP_STALE_MS (default 500 ms, WS2 watchdog)
     * or the stabilizer forces idle after the timeout and the estimator
     * never runs — a single one-shot substrate_move() looked like a
     * silent no-op for exactly this reason. */
    while (true) {
        substrate_move(&sp);
        k_msleep(200);
    }
}
