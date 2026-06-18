/*
 * altitude-hold-test — Explicit, timed altitude hold test sequence
 *
 * Does NOT use the tapestry runtime (no quorum, no auto-arming).
 * Runs a fixed sequence so you can observe each stage safely:
 *
 *   Stage 0  boot + ESC arming + baro calibration  (≈ 5 s, motors silent)
 *   Stage 1  PLACE ON GROUND AND STAND CLEAR — 5 s countdown
 *   Stage 2  arm ESCs (idle speed), hold 3 s
 *   Stage 3  altitude hold at 0.5 m, hold 15 s
 *   Stage 4  idle (altitude PID off), hold 3 s
 *   Stage 5  disarm — motors off
 *
 * Console output during stage 3: "alt=X.XXX m  target=0.500 m  T=0.XX"
 * (logged every 0.5 s from inside cf21bl_stabilizer.c).
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/tapestry/examples/altitude-hold-test
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu
 * Read:   minicom -D /dev/ttyUSB0 -b 115200
 *   or:   python3 ~/code/tapestry/read_console.py
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <tapestry/substrate.h>

LOG_MODULE_REGISTER(alt_hold_test, LOG_LEVEL_INF);

/* Altitude setpoint: linear.z in [-1, +1], altitude = linear.z + 1.0 m.
 * linear.z < -0.9  → idle (altitude PID disarmed, motors at minimum)
 * linear.z = -0.7  → 0.3 m above boot altitude  (conservative first test)
 * linear.z = -0.5  → 0.5 m above boot altitude
 * linear.z =  0.0  → 1.0 m above boot altitude  */
#define ALT_TARGET_LZ   (-0.7f)   /* → 0.3 m: low, catchable, survivable drop */

static const substrate_twist_t IDLE  = { .linear = { .z = -1.0f } };
static const substrate_twist_t HOVER = { .linear = { .z = ALT_TARGET_LZ } };

int main(void)
{
    LOG_INF("=== Altitude hold test (no tapestry runtime) ===");

    /* substrate_init() → cf21bl_init(): ESC reset + 3 s arming melody,
     * then starts the stabilizer thread (which spends ~1 s averaging
     * the baro home baseline before its main loop begins). */
    LOG_INF("Stage 0: ESC arming + baro calibration (~5 s) ...");
    if (substrate_init() != 0) {
        LOG_ERR("substrate_init failed — aborting");
        return -1;
    }

    /* Wait for the stabilizer thread's baro home averaging to finish.
     * cf21bl_init() returns immediately after launching the thread;
     * give it 2 s on top of the 1 s calibration loop. */
    k_msleep(2000);
    LOG_INF("Stage 0 complete — baro home established");

    /* ── Stage 1: tether window ─────────────────────────────────────── */
    LOG_INF("Stage 1: PLACE ON GROUND AND STAND CLEAR — arming in 5 s ...");
    for (int i = 5; i > 0; i--) {
        LOG_INF("  %d ...", i);
        k_msleep(1000);
    }

    /* ── Stage 2: arm at idle ────────────────────────────────────────── */
    LOG_INF("Stage 2: arming ESCs (idle speed for 3 s)");
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);
    substrate_move(&IDLE);   /* linear.z = -1.0 → altitude PID disarmed */
    k_msleep(3000);

    /* ── Stage 3: altitude hold ──────────────────────────────────────── */
    LOG_INF("Stage 3: altitude hold at %.1f m for 15 s",
            (double)(ALT_TARGET_LZ + 1.0f));
    substrate_move(&HOVER);
    k_msleep(15000);

    /* ── Stage 4: return to idle ─────────────────────────────────────── */
    LOG_INF("Stage 4: idle (altitude PID off) for 3 s — land the drone");
    substrate_move(&IDLE);
    k_msleep(3000);

    /* ── Stage 5: disarm ─────────────────────────────────────────────── */
    substrate_set_power(SUBSTRATE_POWER_SLEEP);
    LOG_INF("Stage 5: disarmed — test complete");
    return 0;
}
