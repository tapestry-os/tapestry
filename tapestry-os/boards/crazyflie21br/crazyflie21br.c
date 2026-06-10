/*
 * crazyflie21br.c — Crazyflie 2.1 brushless motor driver (Zephyr PWM)
 *
 * Motor layout (view from above):
 *   M4(CW)  M1(CCW)   ← front
 *   M3(CW)  M2(CCW)   ← back
 *
 * ESC PWM signal (400 Hz, standard RC):
 *   Period    = CF21_PWM_PERIOD_NS   (2 500 000 ns = 2.5 ms = 400 Hz)
 *   Idle      = CF21_PWM_IDLE_NS     (1 000 000 ns = 1.0 ms)
 *   Full      = CF21_PWM_FULL_NS     (2 000 000 ns = 2.0 ms)
 *
 * DTS alias "cf21-motors" must expose four PWM channels in order M1..M4.
 * Pin assignments live in crazyflie21br.overlay.
 */

#include "crazyflie21br.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(crazyflie21br, LOG_LEVEL_INF);

/* ── Timing constants ────────────────────────────────────────────────────── */

#define CF21_PWM_PERIOD_NS   2500000U   /* 400 Hz period              */
#define CF21_PWM_IDLE_NS     1000000U   /* 1 ms = armed / idle        */
#define CF21_PWM_FULL_NS     2000000U   /* 2 ms = full throttle       */
#define CF21_ARM_MS          3000U      /* ESC arm sequence hold time (≥ startup + arming) */

/* Minimum PWM width at which all four ESCs spin.
 * Measured 2026-06-09 on hardware with BLHeli_S 16.7, RC PWM mode:
 *   18% → 1180 µs — all four motors spinning (first confirmed step).
 *   20% → 1200 µs — confirmed cleanly. */
#define CF21_PWM_MIN_NS      1180000U   /* 1180 µs = 18% of [1000, 2000] range */

/* ── DTS bindings ────────────────────────────────────────────────────────── */

#define CF21_MOTORS_NODE    DT_ALIAS(cf21_motors)

static const struct gpio_dt_spec cf21_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct pwm_dt_spec motors[4] = {
    PWM_DT_SPEC_GET_BY_IDX(CF21_MOTORS_NODE, 0),   /* M1 front-right CCW */
    PWM_DT_SPEC_GET_BY_IDX(CF21_MOTORS_NODE, 1),   /* M2 back-right  CCW */
    PWM_DT_SPEC_GET_BY_IDX(CF21_MOTORS_NODE, 2),   /* M3 back-left   CW  */
    PWM_DT_SPEC_GET_BY_IDX(CF21_MOTORS_NODE, 3),   /* M4 front-left  CW  */
};

/* ── State ───────────────────────────────────────────────────────────────── */

static bool  g_ready  = false;
static bool  g_armed  = false;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static uint32_t motor_to_ns(float v)
{
    /* v in [0.0, 1.0]; map to [CF21_PWM_MIN_NS, CF21_PWM_FULL_NS] */
    uint32_t range = CF21_PWM_FULL_NS - CF21_PWM_MIN_NS;
    return CF21_PWM_MIN_NS + (uint32_t)(v * (float)range);
}

static void write_motor(int idx, float value)
{
    uint32_t pulse = g_armed ? motor_to_ns(value) : CF21_PWM_IDLE_NS;
    int ret = pwm_set_dt(&motors[idx], CF21_PWM_PERIOD_NS, pulse);
    if (ret) {
        LOG_WRN("M%d PWM write failed: %d", idx + 1, ret);
    }
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int cf21_init(void)
{
    /* PC15 = shared ESC reset, open-drain active-low, pull-up to ensure clean
     * rising edge.  Hold HIGH initially so the line is defined while we set up
     * PWM.  The actual reset pulse comes AFTER idle PWM is running — BLHeli_S
     * must see a valid signal the instant reset is released or it enters its
     * bootloader instead of normal mode (per crazyflie-firmware/motors.c). */
    const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    bool have_esc_reset = device_is_ready(gpioc);
    if (have_esc_reset) {
        gpio_pin_configure(gpioc, 15, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN | GPIO_PULL_UP);
    } else {
        LOG_WRN("GPIOC not ready — ESC reset pin not driven");
    }

    if (!gpio_is_ready_dt(&cf21_led)) {
        LOG_ERR("Status LED GPIO not ready");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&cf21_led, GPIO_OUTPUT_ACTIVE);   /* LED on: boot reached */
    k_msleep(200);
    gpio_pin_set_dt(&cf21_led, 0);                          /* LED off */
    k_msleep(200);
    gpio_pin_set_dt(&cf21_led, 1);                          /* LED on: stay on until runtime */

    for (int i = 0; i < 4; i++) {
        if (!pwm_is_ready_dt(&motors[i])) {
            LOG_ERR("PWM device not ready for M%d", i + 1);
            return -ENODEV;
        }
    }

    /* Start idle PWM on all channels — signal must be live before ESC reset. */
    for (int i = 0; i < 4; i++) {
        pwm_set_dt(&motors[i], CF21_PWM_PERIOD_NS, CF21_PWM_IDLE_NS);
    }

    /* Pulse PC15 LOW→HIGH with PWM already running so BLHeli_S sees the RC
     * signal immediately on release and enters normal mode (not bootloader). */
    if (have_esc_reset) {
        k_msleep(50);
        gpio_pin_set(gpioc, 15, 0);   /* assert reset */
        k_msleep(1);
        gpio_pin_set(gpioc, 15, 1);   /* release into live PWM */
        LOG_INF("CF21 ESC reset pulse sent");
    }

    /* Hold idle PWM for ESC startup melody + arming sequence. */
    k_msleep(CF21_ARM_MS);

    g_ready = true;
    g_armed = false;
    LOG_INF("Crazyflie 2.1 ESCs armed (idle)");
    return 0;
}

void cf21_set_motors(const cf21_motors_t *motors_out)
{
    if (!g_ready) {
        return;
    }
    write_motor(0, motors_out->m1);
    write_motor(1, motors_out->m2);
    write_motor(2, motors_out->m3);
    write_motor(3, motors_out->m4);
}

void cf21_set_armed(bool armed)
{
    if (!g_ready) {
        return;
    }
    g_armed = armed;
    if (!armed) {
        /* Return all channels to idle immediately */
        for (int i = 0; i < 4; i++) {
            pwm_set_dt(&motors[i], CF21_PWM_PERIOD_NS, CF21_PWM_IDLE_NS);
        }
        LOG_INF("CF21 disarmed");
    } else {
        LOG_INF("CF21 armed");
    }
}

void cf21_set_led(uint8_t brightness)
{
    gpio_pin_set_dt(&cf21_led, brightness > 0);
}
