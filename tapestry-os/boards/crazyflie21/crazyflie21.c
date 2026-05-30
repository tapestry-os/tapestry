/*
 * crazyflie21.c — Crazyflie 2.1 brushless motor driver (Zephyr PWM)
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
 * Pin assignments live in crazyflie21.overlay.
 */

#include "crazyflie21.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(crazyflie21, LOG_LEVEL_INF);

/* ── Timing constants ────────────────────────────────────────────────────── */

#define CF21_PWM_PERIOD_NS   2500000U   /* 400 Hz period              */
#define CF21_PWM_IDLE_NS     1000000U   /* 1 ms = armed / idle        */
#define CF21_PWM_FULL_NS     2000000U   /* 2 ms = full throttle       */
#define CF21_ARM_MS          2000U      /* ESC arm sequence hold time */

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
    /* v in [0.0, 1.0]; map to [CF21_PWM_IDLE_NS, CF21_PWM_FULL_NS] */
    uint32_t range = CF21_PWM_FULL_NS - CF21_PWM_IDLE_NS;
    return CF21_PWM_IDLE_NS + (uint32_t)(v * (float)range);
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
    if (!gpio_is_ready_dt(&cf21_led)) {
        LOG_ERR("Status LED GPIO not ready");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&cf21_led, GPIO_OUTPUT_INACTIVE);

    for (int i = 0; i < 4; i++) {
        if (!pwm_is_ready_dt(&motors[i])) {
            LOG_ERR("PWM device not ready for M%d", i + 1);
            return -ENODEV;
        }
    }

    /* Drive idle pulse to all ESCs and hold for arming sequence */
    for (int i = 0; i < 4; i++) {
        pwm_set_dt(&motors[i], CF21_PWM_PERIOD_NS, CF21_PWM_IDLE_NS);
    }
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
