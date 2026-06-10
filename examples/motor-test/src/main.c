/*
 * motor_test — Motor characterisation sweep
 *
 * crazyflie21br: collective thrust sweep to find minimum ESC spin threshold.
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

struct step {
    int pct;
    int drive_ms;
    int pause_ms;
};

#if defined(CONFIG_BOARD_CRAZYFLIE21BR)

/* Sanity check at 100% first, then fine sweep from 5–50%.
 * PWM mapping: pct% → T = pct/100 → PWM = (1000 + pct*10) µs. */
static const struct step steps[] = {
    { 100, 3000, 2000 },   /* wiring check — if this doesn't spin, ESCs need reflash */
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

/* T=0, 1 ms idle pulse — does NOT spin motors but holds ESC armed */
static const substrate_twist_t k_stop = { .linear = { .z = -1.0f } };

#else  /* bbc_microbit_v2 / Cutebot */

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
#if defined(CONFIG_BOARD_CRAZYFLIE21BR)
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
    LOG_INF("ESCs armed — beginning sweep");
#endif

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

    return 0;
}
