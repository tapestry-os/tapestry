/*
 * cutebot_pro.c — ELECFREAKS Cutebot Pro (V2.1.0) driver (I2C)
 *
 * See cutebot_pro.h for the protocol overview and what's verified vs.
 * unverified. Motor, encoder and line-gray paths are hardware-verified
 * (2026-09-22/23); the background line watcher is not yet.
 */

#include "cutebot_pro.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cutebot_pro, LOG_LEVEL_INF);

#define CUTEBOT_PRO_I2C_ADDR   0x10u

#define FRAME_HDR_0            0xFFu
#define FRAME_HDR_1            0xF9u

#define CMD_MOTOR_PID_SPEED    0x80u
#define CMD_LED                0x20u
#define CMD_CLEAR_ENCODER      0x50u
#define CMD_LINE_SENSOR        0x60u
#define CMD_ENCODER            0xA0u

#define LINE_SENSOR_SUB_STATE  0x00u
#define LINE_SENSOR_SUB_GRAY   0x02u

#define ENCODER_SUB_SPEED_L    0x01u
#define ENCODER_SUB_SPEED_R    0x02u
#define ENCODER_SUB_DIST_L     0x03u
#define ENCODER_SUB_DIST_R     0x04u

/* Firmware-enforced legal range for a nonzero pidSpeedControl target
 * (elecfreaks/pxt-Cutebot-Pro's own v2.ts clamps here) — see
 * cutebot_pro_drive_cms()'s doc. */
#define PID_SPEED_MIN_CMS      20.0f
#define PID_SPEED_MAX_CMS      50.0f

#define LIGHT_BOTH             0x02u

static const struct device *i2c_dev;

/* Serializes whole command/reply exchanges. The I2C driver only locks
 * each transfer; a command's reply is a separate transfer, so without
 * this the line-watch thread could slip its own command in between
 * another caller's command and reply and steal (or corrupt) the reply. */
static K_MUTEX_DEFINE(s_bus_lock);

/* ── Helpers ─────────────────────────────────────────────────────────────── */

/* [0xFF][0xF9][CMD][PARAM_LEN][params...] — see cutebot_pro.h's top doc.
 * Caller holds s_bus_lock. */
static int frame_write(uint8_t cmd, const uint8_t *params, uint8_t param_len)
{
    uint8_t buf[4 + 8];   /* every command used here has <= 8 params */

    if (!i2c_dev || param_len > 8) {
        return -EINVAL;
    }

    buf[0] = FRAME_HDR_0;
    buf[1] = FRAME_HDR_1;
    buf[2] = cmd;
    buf[3] = param_len;
    for (uint8_t i = 0; i < param_len; i++) {
        buf[4 + i] = params[i];
    }

    return i2c_write(i2c_dev, buf, (uint32_t)(4 + param_len), CUTEBOT_PRO_I2C_ADDR);
}

static int frame_send(uint8_t cmd, const uint8_t *params, uint8_t param_len)
{
    k_mutex_lock(&s_bus_lock, K_FOREVER);
    int ret = frame_write(cmd, params, param_len);
    k_mutex_unlock(&s_bus_lock);
    return ret;
}

/* A command's reply is a SEPARATE I2C transaction after the command frame
 * (matches elecfreaks/pxt-Cutebot-Pro's own i2cCommandSend() followed by a
 * distinct i2cReadBuffer()/i2cReadNumber() call — not a single repeated-
 * START transaction), so this issues the frame then a plain i2c_read(),
 * both under s_bus_lock. */
static int frame_send_and_read(uint8_t cmd, const uint8_t *params, uint8_t param_len,
                                uint8_t *reply, size_t reply_len)
{
    k_mutex_lock(&s_bus_lock, K_FOREVER);
    int ret = frame_write(cmd, params, param_len);
    if (ret == 0) {
        ret = i2c_read(i2c_dev, reply, (uint32_t)reply_len, CUTEBOT_PRO_I2C_ADDR);
    }
    k_mutex_unlock(&s_bus_lock);
    return ret;
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int cutebot_pro_init(void)
{
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(cutebot_i2c));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready — Cutebot Pro unavailable");
        i2c_dev = NULL;
        return -ENODEV;
    }

    cutebot_pro_drive_cms(0.0f, 0.0f);
    cutebot_pro_set_leds(0, 0, 0);

    LOG_INF("Cutebot Pro ready on I2C addr 0x%02X", CUTEBOT_PRO_I2C_ADDR);
    return 0;
}

void cutebot_pro_drive_cms(float left_cms, float right_cms)
{
    if (!i2c_dev) {
        return;
    }

    uint8_t direction = 0;
    if (left_cms  < 0.0f) { direction |= 0x01u; }
    if (right_cms < 0.0f) { direction |= 0x02u; }

    float l_mag = fabsf(left_cms);
    float r_mag = fabsf(right_cms);

    /* Firmware clamps any nonzero speed into [20, 50] cm/s on its own —
     * but clamping each wheel INDEPENDENTLY here (the original bug, found
     * 2026-09-22 on real 4-robot hardware: a triangle re-form's turn
     * requirement produced pure straight-line "rocking," never turning)
     * silently erases any turn whose natural differential is smaller
     * than the floor: formation.c's angular.z is inherently much smaller
     * in magnitude than linear.x (see substrate_cutebot_pro.c's own
     * doc), so BOTH wheels' natural cm/s values land below 20 for nearly
     * every real turn except an exact 90-degree in-place pivot — flooring
     * them separately then snaps both to the SAME 20 cm/s, leaving no
     * differential at all. Fix: when both wheels want nonzero motion,
     * scale BOTH by whatever factor brings the WEAKER one up to the
     * floor, preserving their ratio (the actual turn radius/direction)
     * instead of destroying it. A single-wheel-only command (pivoting
     * around one stationary wheel) still floors independently — there is
     * no ratio to preserve against a wheel that's deliberately at 0. */
    if (l_mag > 0.0f && r_mag > 0.0f) {
        float smaller = (l_mag < r_mag) ? l_mag : r_mag;
        if (smaller < PID_SPEED_MIN_CMS) {
            float scale = PID_SPEED_MIN_CMS / smaller;
            l_mag *= scale;
            r_mag *= scale;
        }
        /* Re-clamp to the ceiling AFTER scaling. If the original ratio
         * exceeds 50/20 = 2.5:1, this clips the faster wheel and the
         * exact ratio is lost — the firmware's [20,50] range simply
         * cannot represent a sharper turn than that at all — but
         * direction and SOME differential survive, unlike the old bug
         * where it silently collapsed to zero differential. */
        if (l_mag > PID_SPEED_MAX_CMS) { l_mag = PID_SPEED_MAX_CMS; }
        if (r_mag > PID_SPEED_MAX_CMS) { r_mag = PID_SPEED_MAX_CMS; }
    } else if (l_mag > 0.0f) {
        l_mag = (l_mag < PID_SPEED_MIN_CMS) ? PID_SPEED_MIN_CMS
              : (l_mag > PID_SPEED_MAX_CMS) ? PID_SPEED_MAX_CMS
              : l_mag;
    } else if (r_mag > 0.0f) {
        r_mag = (r_mag < PID_SPEED_MIN_CMS) ? PID_SPEED_MIN_CMS
              : (r_mag > PID_SPEED_MAX_CMS) ? PID_SPEED_MAX_CMS
              : r_mag;
    }

    /* pidSpeedControl's wire value is cm/s * 10 (elecfreaks/pxt-Cutebot-Pro's
     * v2.ts: `lspeed *= 10` for the cm/s unit branch), big-endian 16-bit. */
    uint16_t l_wire = (uint16_t)(l_mag * 10.0f);
    uint16_t r_wire = (uint16_t)(r_mag * 10.0f);

    uint8_t params[5] = {
        (uint8_t)(l_wire >> 8), (uint8_t)(l_wire & 0xFFu),
        (uint8_t)(r_wire >> 8), (uint8_t)(r_wire & 0xFFu),
        direction,
    };

    int ret = frame_send(CMD_MOTOR_PID_SPEED, params, sizeof(params));
    if (ret) { LOG_WRN("motor PID speed write failed: %d", ret); }
}

void cutebot_pro_set_leds(uint8_t r, uint8_t g, uint8_t b)
{
    if (!i2c_dev) {
        return;
    }
    uint8_t params[4] = { LIGHT_BOTH, r, g, b };
    int ret = frame_send(CMD_LED, params, sizeof(params));
    if (ret) { LOG_WRN("LED write failed: %d", ret); }
}

void cutebot_pro_read_encoders(cutebot_pro_encoders_t *out)
{
    if (!i2c_dev || !out) {
        return;
    }

    uint8_t speed_l[1], speed_r[1];
    uint8_t dist_l[4], dist_r[4];

    uint8_t sub;
    int ret;

    sub = ENCODER_SUB_SPEED_L;
    ret = frame_send_and_read(CMD_ENCODER, &sub, 1, speed_l, sizeof(speed_l));
    out->left_speed_cms = ret ? 0.0f : (float)speed_l[0];

    sub = ENCODER_SUB_SPEED_R;
    ret = frame_send_and_read(CMD_ENCODER, &sub, 1, speed_r, sizeof(speed_r));
    out->right_speed_cms = ret ? 0.0f : (float)speed_r[0];

    /* Int32LE per elecfreaks/pxt-Cutebot-Pro's v2.ts (NumberFormat.Int32LE) —
     * see cutebot_pro.h's doc for why the physical unit is still unverified. */
    sub = ENCODER_SUB_DIST_L;
    ret = frame_send_and_read(CMD_ENCODER, &sub, 1, dist_l, sizeof(dist_l));
    out->left_distance = ret ? 0 :
        (int32_t)((uint32_t)dist_l[0] | ((uint32_t)dist_l[1] << 8) |
                   ((uint32_t)dist_l[2] << 16) | ((uint32_t)dist_l[3] << 24));

    sub = ENCODER_SUB_DIST_R;
    ret = frame_send_and_read(CMD_ENCODER, &sub, 1, dist_r, sizeof(dist_r));
    out->right_distance = ret ? 0 :
        (int32_t)((uint32_t)dist_r[0] | ((uint32_t)dist_r[1] << 8) |
                   ((uint32_t)dist_r[2] << 16) | ((uint32_t)dist_r[3] << 24));
}

void cutebot_pro_clear_encoders(void)
{
    if (!i2c_dev) {
        return;
    }
    /* elecfreaks/pxt-Cutebot-Pro's clearWheelTurn() sends one motor index
     * per call (0=left, 1=right) — no "both" value documented, so issue
     * both explicitly rather than guess one. */
    uint8_t left_motor = 0, right_motor = 1;
    int ret;
    ret = frame_send(CMD_CLEAR_ENCODER, &left_motor, 1);
    if (ret) { LOG_WRN("clear left encoder failed: %d", ret); }
    ret = frame_send(CMD_CLEAR_ENCODER, &right_motor, 1);
    if (ret) { LOG_WRN("clear right encoder failed: %d", ret); }
}

int cutebot_pro_read_line_state(uint8_t *state)
{
    if (!i2c_dev || !state) {
        return -EINVAL;
    }
    uint8_t sub = LINE_SENSOR_SUB_STATE;
    uint8_t reply[1];
    int ret = frame_send_and_read(CMD_LINE_SENSOR, &sub, 1, reply, sizeof(reply));
    if (ret) {
        return ret;
    }
    *state = reply[0];
    return 0;
}

int cutebot_pro_read_line_gray(uint8_t channel, uint8_t *gray)
{
    if (!i2c_dev || !gray || channel < 1 || channel > 4) {
        return -EINVAL;
    }
    /* Wire is 0-indexed — see elecfreaks/pxt-Cutebot-Pro's own
     * trackbitgetGray(channel - 1) call. */
    uint8_t params[2] = { LINE_SENSOR_SUB_GRAY, (uint8_t)(channel - 1) };
    uint8_t reply[1];
    int ret = frame_send_and_read(CMD_LINE_SENSOR, params, sizeof(params), reply, sizeof(reply));
    if (ret) {
        return ret;
    }
    *gray = reply[0];
    return 0;
}

/* ── Background line watcher (see cutebot_pro.h) ────────────────────────── */

#define LINE_WATCH_STACK_SIZE  1024
/* Same priority as main (CONFIG_MAIN_THREAD_PRIORITY=0): time slicing
 * (CONFIG_TIMESLICE_SIZE) then bounds how long either can hold the CPU, so
 * a busy main tick delays a sample by at most one slice. Each sample is
 * four bit-banged I2C exchanges (a few ms of CPU), so the period is also
 * the knob for how much CPU this costs. */
#define LINE_WATCH_PRIORITY    0

K_THREAD_STACK_DEFINE(s_lw_stack, LINE_WATCH_STACK_SIZE);
static struct k_thread s_lw_thread;
static struct k_spinlock s_lw_lock;
static cutebot_pro_line_sample_t s_lw;
static uint8_t  s_lw_threshold;
static uint32_t s_lw_period_ms;
static bool     s_lw_started;

static void line_watch_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        uint8_t gray[4];
        bool ok = true;
        for (uint8_t ch = 1; ch <= 4 && ok; ch++) {
            ok = (cutebot_pro_read_line_gray(ch, &gray[ch - 1]) == 0);
        }

        uint8_t mask = 0;
        if (ok) {
            for (int i = 0; i < 4; i++) {
                if (gray[i] >= s_lw_threshold) {
                    mask |= (uint8_t)(1u << i);
                }
            }
        }

        k_spinlock_key_t key = k_spin_lock(&s_lw_lock);
        if (ok) {
            for (int i = 0; i < 4; i++) {
                s_lw.gray[i] = gray[i];
            }
            s_lw.dark_mask  = mask;
            s_lw.seen_mask |= mask;
            s_lw.samples++;
            s_lw.valid = true;
        } else {
            s_lw.errors++;
        }
        k_spin_unlock(&s_lw_lock, key);

        k_msleep(s_lw_period_ms);
    }
}

int cutebot_pro_line_watch_start(uint8_t dark_threshold, uint32_t period_ms)
{
    if (!i2c_dev) {
        return -ENODEV;
    }
    if (s_lw_started) {
        return -EALREADY;
    }
    s_lw_threshold = dark_threshold;
    s_lw_period_ms = period_ms;
    s_lw_started   = true;

    k_thread_create(&s_lw_thread, s_lw_stack, K_THREAD_STACK_SIZEOF(s_lw_stack),
                    line_watch_fn, NULL, NULL, NULL,
                    LINE_WATCH_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&s_lw_thread, "cutebot_pro_lw");
    LOG_INF("line watch: threshold %u, period %u ms",
            (unsigned)dark_threshold, (unsigned)period_ms);
    return 0;
}

void cutebot_pro_line_watch_poll(cutebot_pro_line_sample_t *out)
{
    k_spinlock_key_t key = k_spin_lock(&s_lw_lock);
    *out = s_lw;
    s_lw.seen_mask = 0;
    s_lw.samples   = 0;
    s_lw.errors    = 0;
    k_spin_unlock(&s_lw_lock, key);
}
