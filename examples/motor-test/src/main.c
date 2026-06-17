/*
 * motor_test — Motor characterisation sweep / yaw-sign verification / hover test
 *
 * crazyflie21br (default, no mode flag set):
 *   Collective thrust sweep to find minimum ESC spin threshold.
 *
 *   Procedure:
 *   1. REMOVE ALL PROPELLERS. Secure frame to bench (tape or clamps).
 *   2. west build -p always -b crazyflie21br tapestry/examples/motor-test
 *      cfloader flash build/zephyr/zephyr.bin stm32-dfu
 *   3. Connect serial: picocom /dev/ttyUSB0 -b 115200 (USART3 PC10/PC11)
 *   4. Power on — ESC arming runs automatically (2 s idle pulse).
 *   5. Watch serial: each step logs its PWM width and runs for 3 s.
 *   6. Note the first step where ALL FOUR motors audibly/tactilely spin.
 *   7. Compute: CF21_PWM_MIN_NS = 1000000 + threshold_pct * 10000
 *      Set CF21_PWM_MIN_NS in tapestry-os/boards/crazyflie21br/crazyflie21br.c.
 *
 * crazyflie21br hover test (CONFIG_CF21_HOVER_TEST=y, CONFIG_CF21_ANGLE_MODE=y):
 *   Arms ESCs and holds a fixed collective indefinitely with angular setpoints = 0.
 *   The outer angle loop drives roll/pitch toward level — tilt the tethered frame
 *   by hand and observe the restoring response.
 *
 *   Procedure:
 *   1. INSTALL PROPELLERS. Tether frame (string + weight through prop arc).
 *   2. Set CONFIG_CF21_HOVER_TEST=y, CONFIG_CF21_ANGLE_MODE=y, and
 *      CONFIG_CF21_HOVER_THROTTLE_PCT to a starting value (default 30).
 *      Rebuild and flash.
 *   3. Power on. After ESC arming (3 s) the frame spins up automatically.
 *   4. Gently tilt in roll and pitch — the angle loop should resist each tilt.
 *   5. Power off (no software disarm — just kill the battery) to stop.
 *   6. Increase CONFIG_CF21_HOVER_THROTTLE_PCT in steps until the tether
 *      goes taut, confirming the frame reaches hover thrust.
 *
 * crazyflie21br yaw-sign verify (CONFIG_MOTOR_TEST_YAW_VERIFY=y):
 *   Spins each diagonal motor pair to confirm the yaw-reaction sign
 *   convention assumed in crazyflie21br_mix.h.
 *
 *   Procedure:
 *   1. INSTALL PROPELLERS (correct orientation per motor spin direction).
 *      Hold frame loosely on bench — do NOT clamp; it needs to rotate freely.
 *   2. Add CONFIG_MOTOR_TEST_YAW_VERIFY=y to boards/crazyflie21br.conf,
 *      rebuild and flash:
 *        west build -p always -b crazyflie21br tapestry/examples/motor-test
 *        cfloader flash build/zephyr/zephyr.bin stm32-dfu
 *   3. Connect serial; watch for phase prompts.
 *   4. M2+M4 phase (CW props, BR+FL): frame should rotate CCW (yaw left, +Z).
 *      If it rotates CW instead: negate all Y terms in cf21_mix() in
 *      tapestry-os/boards/crazyflie21br/crazyflie21br_mix.h.
 *   5. M1+M3 phase (CCW props, FR+BL): frame should rotate CW (yaw right, -Z).
 *
 * bbc_microbit_v2 (Cutebot): forward-speed sweep to characterise stiction.
 *
 *   Procedure:
 *   1. Place robot on flat surface with at least 600 mm clear ahead.
 *   2. Mark the start position.
 *   3. Watch serial output — each step is announced before the motors run.
 *   4. Note the first step that causes movement (stiction threshold %).
 *   5. For each moving step, measure distance from the mark to where it stopped.
 *      Speed (mm/s) = distance_mm / 3000
 *      SPEED_SCALE  = speed_mm_per_s * 100 / arena_mm  (arena_mm = 500 for 0.5 m)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <tapestry/substrate.h>

LOG_MODULE_REGISTER(motor_test, LOG_LEVEL_INF);

#if defined(CONFIG_BOARD_CRAZYFLIE21BR)

/* T=0, 1 ms idle pulse — does NOT spin motors but holds ESC armed */
static const substrate_twist_t k_stop = { .linear = { .z = -1.0f } };

#ifdef CONFIG_CF21_HOVER_TEST

/* ── Tethered hover (props on, angle loop self-leveling) ─────────────────── */

/* Collective at configured %, all angular setpoints zero.
 * linear.z = pct/50 - 1  →  T = pct/100 in cf21_mix. */
static const substrate_twist_t k_hover = {
    .linear = { .z = CONFIG_CF21_HOVER_THROTTLE_PCT / 50.0f - 1.0f },
};

#elif defined(CONFIG_MOTOR_TEST_YAW_VERIFY)

/* ── Yaw-sign verification (props on, frame loose on bench) ──────────────── */

/*
 * T = 0.1, Y = -0.1  →  cf21_mix gives M1=M3=0.20, M2=M4=0.00 (CCW diagonal, 20%).
 * Body reaction: M1(CCW)+M3(CCW) produce CW torque → frame should yaw CW right.
 * If frame rotates CCW instead, negate Y in all four equations in cf21_mix().
 */
static const substrate_twist_t k_m13 = {
    .linear  = { .z = -0.8f },   /* T = 0.1 */
    .angular = { .z = -0.1f },   /* Y = -0.1 → M1+M3 (CCW props) only */
};

/*
 * T = Y = 0.1  →  cf21_mix gives M2=M4=0.20, M1=M3=0.00 (CW diagonal, 20%).
 * Body reaction: M2(CW)+M4(CW) produce CCW torque → frame should yaw CCW left.
 */
static const substrate_twist_t k_m24 = {
    .linear  = { .z = -0.8f },   /* T = 0.1 */
    .angular = { .z =  0.1f },   /* Y = +0.1 → M2+M4 (CW props) only */
};

#else  /* collective sweep (default) */

/* ── Collective sweep (props off, frame secured) ──────────────────────────── */

struct step {
    int pct;
    int drive_ms;
    int pause_ms;
};

/* Fine sweep from 5–50%.
 * PWM mapping: pct% → T = pct/100 → PWM = (1000 + pct*10) µs. */
static const struct step steps[] = {
    {   5, 3000, 2000 },
    {   8, 3000, 2000 },
    {  10, 3000, 2000 },
    {  12, 3000, 2000 },
    {  15, 3000, 2000 },
    {  18, 3000, 2000 },
    {  20, 3000, 2000 },
    {  25, 3000, 2000 },
    {  30, 3000, 2000 },
    {  40, 3000, 2000 },
    {  50, 3000, 2000 },
};

/* linear.z maps to collective thrust: T = (z + 1) / 2, so z = pct/50 - 1 */
static void make_twist(int pct, substrate_twist_t *out)
{
    *out = (substrate_twist_t){ .linear = { .z = pct / 50.0f - 1.0f } };
}

#endif /* CF21_HOVER_TEST / MOTOR_TEST_YAW_VERIFY */

#else  /* bbc_microbit_v2 / Cutebot */

struct step {
    int pct;
    int drive_ms;
    int pause_ms;
};

static const struct step steps[] = {
    {  20, 3000, 2000 },
    {  25, 3000, 2000 },
    {  30, 3000, 2000 },
    {  35, 3000, 2000 },
    {  40, 3000, 2000 },
    {  50, 3000, 2000 },
    {  75, 3000, 2000 },
    { 100, 3000, 2000 },
};

static void make_twist(int pct, substrate_twist_t *out)
{
    *out = (substrate_twist_t){ .linear = { .x = pct / 100.0f } };
}

static const substrate_twist_t k_stop = {0};

#endif /* board selection */

int main(void)
{
#if defined(CONFIG_BOARD_CRAZYFLIE21BR) && defined(CONFIG_CF21_HOVER_TEST)
    LOG_INF("=== CF21br hover test (%d%% collective%s) ===",
            CONFIG_CF21_HOVER_THROTTLE_PCT,
            IS_ENABLED(CONFIG_CF21_ANGLE_MODE) ? ", angle loop" : ", rate loop");
    LOG_INF("PROPS ON, frame tethered — spins up automatically after arming");
    LOG_INF("Starting in 5 s...");
    k_msleep(5000);
#elif defined(CONFIG_BOARD_CRAZYFLIE21BR) && defined(CONFIG_MOTOR_TEST_YAW_VERIFY)
    LOG_INF("=== CF21br yaw-sign verification ===");
    LOG_INF("PROPELLERS ON, frame held loosely on bench — starting in 5 s...");
    k_msleep(5000);
#elif defined(CONFIG_BOARD_CRAZYFLIE21BR)
    LOG_INF("=== CF21br ESC characterisation ===");
    LOG_INF("PROPELLERS OFF, frame secured — starting in 5 s...");
    k_msleep(5000);
#else
    LOG_INF("=== Cutebot motor test ===");
    LOG_INF("Place robot on flat surface with 600 mm clear ahead");
    LOG_INF("Mark start position — starting in 3 s...");
    k_msleep(3000);
#endif

    if (substrate_init() != 0) {
        LOG_ERR("substrate_init failed — aborting");
        return -1;
    }

#if defined(CONFIG_BOARD_CRAZYFLIE21BR)
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);
#if defined(CONFIG_CF21_HOVER_TEST)
    LOG_INF("ESCs armed — holding at %d%% collective", CONFIG_CF21_HOVER_THROTTLE_PCT);
#elif defined(CONFIG_MOTOR_TEST_YAW_VERIFY)
    LOG_INF("ESCs armed — beginning yaw verification");
#else
    LOG_INF("ESCs armed — beginning sweep");
#endif
#endif

#if defined(CONFIG_BOARD_CRAZYFLIE21BR) && defined(CONFIG_CF21_HOVER_TEST)

    substrate_move(&k_hover);
    while (true) {
        k_msleep(5000);
        LOG_INF("Hover running at %d%% — tilt frame to test self-leveling",
                CONFIG_CF21_HOVER_THROTTLE_PCT);
    }

#elif defined(CONFIG_BOARD_CRAZYFLIE21BR) && defined(CONFIG_MOTOR_TEST_YAW_VERIFY)

    LOG_INF("--- M2+M4 (BR+FL, CW props) at 20%% ---");
    LOG_INF("    Expected: frame rotates CCW (yaw left, +Z).");
    LOG_INF("    If CW: negate Y in all 4 equations in cf21_mix() (crazyflie21br_mix.h).");
    substrate_move(&k_m24);
    k_msleep(3000);
    substrate_move(&k_stop);
    k_msleep(2000);

    LOG_INF("--- M1+M3 (FR+BL, CCW props) at 20%% ---");
    LOG_INF("    Expected: frame rotates CW (yaw right, -Z).");
    substrate_move(&k_m13);
    k_msleep(3000);
    substrate_move(&k_stop);
    k_msleep(2000);

    substrate_set_power(SUBSTRATE_POWER_SLEEP);
    LOG_INF("=== Done — ESCs disarmed ===");
    LOG_INF("If M2+M4 spun CW (not CCW): negate Y in cf21_mix().");

#else  /* collective sweep or Cutebot */

    for (int i = 0; i < (int)ARRAY_SIZE(steps); i++) {
        const struct step *s = &steps[i];
        substrate_twist_t twist;

        make_twist(s->pct, &twist);

#if defined(CONFIG_BOARD_CRAZYFLIE21BR)
        LOG_INF("--- Step %2d: %3d%% (PWM %d us) --- MOTORS ON",
                i + 1, s->pct, 1000 + s->pct * 10);
#else
        LOG_INF("--- Step %d: %d%% --- robot drives now", i + 1, s->pct);
#endif

        substrate_move(&twist);
        k_msleep(s->drive_ms);
        substrate_move(&k_stop);

#if defined(CONFIG_BOARD_CRAZYFLIE21BR)
        LOG_INF("    STOP — all 4 motors spin? (2 s pause)");
#else
        LOG_INF("    STOP — measure distance (speed = dist_mm / 3000 mm/s)");
#endif
        k_msleep(s->pause_ms);
    }

#if defined(CONFIG_BOARD_CRAZYFLIE21BR)
    substrate_set_power(SUBSTRATE_POWER_SLEEP);   /* disarm ESCs */
    LOG_INF("=== Done — ESCs disarmed ===");
    LOG_INF("Find lowest step with all 4 motors spinning.");
    LOG_INF("CF21_PWM_MIN_NS = 1000000 + threshold_pct * 10000");
    LOG_INF("Set in tapestry-os/boards/crazyflie21br/crazyflie21br.c");
#else
    LOG_INF("=== Done ===");
    LOG_INF("SPEED_SCALE = speed_mm_per_s * 100 / arena_mm  (500 for 0.5 m)");
#endif

#endif /* hover test / yaw verify / collective sweep */

    return 0;
}
