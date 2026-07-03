/*
 * cf21bl_lighthouse.c — Lighthouse v2 positioning for the Crazyflie 2.1 Brushless
 *
 * See cf21bl_lighthouse.h for API, calibration instructions, and geometry notes.
 *
 * Hardware interface (verified against Bitcraze crazyflie-firmware):
 *   src/drivers/interface/uart1.h     → USART3, PC10=TX, PC11=RX
 *   src/modules/src/lighthouse/lighthouse_core.c → uart1Init(230400), 12-byte frames
 *
 * CONSOLE CONFLICT: USART3 PC10/PC11 is also the Zephyr wired debug console.
 * When the lighthouse deck is attached, use the CRTP radio console instead.
 * The per-example overlay (boards/crazyflie21bl.overlay) reconfigures USART3
 * to 230400 baud and disables it as the Zephyr console.
 *
 * Frame format (from lighthouse_core.c getUartFrameRaw, UART_FRAME_LENGTH = 12)
 * -------------------------------------------------------------------------------
 * Sync frame:  all 12 bytes = 0xFF.  Marks liveness; no measurement data.
 * Data frame:
 *   byte 0       bit[7]    = !channelFound (0 = channel valid)
 *                bits[6:3] = channel (0-indexed BS channel, 0-15)
 *                bit[2]    = slowBit (distinguishes the two sweep planes: 0 or 1)
 *                bits[1:0] = sensor (photodiode 0-3 on the deck)
 *   bytes 1-2   width     (uint16 LE, pulse width — not used for positioning)
 *   bytes 3-5   offset    (24-bit LE, in 6 MHz ticks → ×4 → 24 MHz ticks)
 *   bytes 6-8   beamData  (24-bit LE, LFSR beam data — not used in our driver)
 *   bytes 9-11  timestamp (24-bit LE, 24 MHz ticks, wraps at 2^24)
 *
 * LH2 angle extraction (from pulse_processor_v2.c calculateAngles)
 * -----------------------------------------------------------------
 * The deck sees two consecutive sweep blocks from the same BS channel.
 * Each block's `offset` (in 24 MHz ticks) gives one raw beam angle:
 *
 *   firstBeam  = (offset0 × 2π / CYCLE_PERIODS[ch]) − π + π/3
 *   secondBeam = (offset1 × 2π / CYCLE_PERIODS[ch]) − π − π/3
 *
 * CYCLE_PERIODS[ch] (24 MHz ticks, from pulse_processor_v2.c):
 *   ch 0: 479500,  ch 1: 478500,  ch 2: 476500,  ch 3: 474500,
 *   ch 4: 473500,  ch 5: 471500,  ch 6: 470500,  ch 7: 469500,
 *   ch 8: 468500,  ch 9: 464500, ch10: 459500, ch11: 455500,
 *  ch12: 453500, ch13: 450500, ch14: 446500, ch15: 443500
 *
 * Azimuth and elevation (from pulseProcessorV2ConvertToV1Angles):
 *   azimuth   = (firstBeam + secondBeam) / 2
 *   elevation = atan2f(sinf(secondBeam − firstBeam),
 *                      tan(π/6) × (cosf(firstBeam) + cosf(secondBeam)))
 *
 * Direction in BS-local frame (X=right, Y=up, Z=forward out of face):
 *   d_local = (sin(az)·cos(el),  sin(el),  cos(az)·cos(el))
 *
 * 3D position is the midpoint of closest approach between two rays
 * (one per base station) — see lh2_triangulate().
 *
 * Channel → base station mapping
 * --------------------------------
 * The channel number in the frame (0-indexed) is the SteamVR channel minus 1.
 * Two BSes are assigned two different channels in the SteamVR room setup.
 * cf21bl_lighthouse_set_bs_channel() maps each channel to a pose (BS0 / BS1).
 * Default: channel 0 → BS pose 0,  channel 1 → BS pose 1.
 */

#include "cf21bl_lighthouse.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cf21bl_lh2, LOG_LEVEL_INF);

/* ── Constants ─────────────────────────────────────────────────────────────── */

#define LH2_BS_COUNT      2
#define LH2_FRAME_LEN     12   /* bytes per UART frame, confirmed from source */

/*
 * Cycle periods in 24 MHz ticks for each LH2 channel (0-indexed).
 * Source: pulse_processor_v2.c CYCLE_PERIODS[] = {959000,957000,...} / 2.
 */
static const uint32_t LH2_CYCLE_PERIODS[16] = {
    479500, 478500, 476500, 474500,
    473500, 471500, 470500, 469500,
    468500, 464500, 459500, 455500,
    453500, 450500, 446500, 443500,
};

/* tan(π/6) = 1/√3, used in the elevation formula */
#define LH2_TANT   0.5773502691896258f

/* A sweep block is fresh if seen within this many ms */
#define LH2_BLOCK_FRESH_MS  200u

/*
 * 5-sample sliding median filter on the triangulated position.
 * Rejects single-sample outliers (bad rotor pairing, brief frame errors)
 * before the position reaches any control loop.  At ~30 Hz fix rate the
 * window covers ~170 ms — enough to catch spikes without smearing steps.
 */
#define LH2_MEDIAN_N  5

/* Stack size and priority of the UART reader thread */
#define LH2_STACK_SIZE     1536
#define LH2_THREAD_PRIO    K_PRIO_PREEMPT(1)

/* ── Device reference ───────────────────────────────────────────────────────── */

/* USART3: PC10=TX (STM32→deck for LED commands), PC11=RX (deck→STM32 for data) */
static const struct device *const g_uart = DEVICE_DT_GET(DT_NODELABEL(usart3));

/*
 * Bit-bang I2C on PB6 (SCL) / PB7 (SDA) — deck expansion connector pins.
 *
 * The lighthouse deck bootloader MCU sits at I2C address 0x2F and holds the
 * FPGA in reset until it receives a BOOT_TO_FW byte (0x00) over I2C.
 * Source: crazyflie-firmware lh_bootloader.c, LIGHTHOUSE_DECK_I2C_ADDRESS=0x2F.
 *
 * We bit-bang rather than use CONFIG_I2C=y because the Zephyr STM32 I2C1
 * hardware driver runs bus-recovery at SYS_INIT (before main()), toggling
 * PB6 with bus-wide side-effects that corrupt USART6 and kill the CRTP log
 * backend.  Pure GPIO bit-banging avoids the hardware I2C peripheral entirely.
 *
 * GPIOB is already enabled in crazyflie21bl.dts.  No extra Kconfig needed.
 */
static const struct device *const g_gpiob = DEVICE_DT_GET(DT_NODELABEL(gpiob));

#define LH2_SCL_PIN          6    /* PB6 = DECK_SCL */
#define LH2_SDA_PIN          7    /* PB7 = DECK_SDA */
#define LH2_BOOTLOADER_ADDR  0x2F /* 7-bit I2C address of deck bootloader MCU */
#define LHBL_BOOT_TO_FW      0x00 /* command: load FPGA bitstream and start */

/* Half-period delay for ~100 kHz I2C (SCL high/low ≥ 5 µs each) */
#define I2C_HALF_PERIOD_US   5

static inline void scl_high(void) { gpio_pin_set(g_gpiob, LH2_SCL_PIN, 1); k_busy_wait(I2C_HALF_PERIOD_US); }
static inline void scl_low(void)  { gpio_pin_set(g_gpiob, LH2_SCL_PIN, 0); k_busy_wait(I2C_HALF_PERIOD_US); }
static inline void sda_high(void) { gpio_pin_set(g_gpiob, LH2_SDA_PIN, 1); }
static inline void sda_low(void)  { gpio_pin_set(g_gpiob, LH2_SDA_PIN, 0); }

static void bb_start(void)
{
    sda_high(); scl_high();
    sda_low();  /* SDA falls while SCL high = START */
    scl_low();
}

static void bb_stop(void)
{
    sda_low(); scl_high();
    sda_high(); /* SDA rises while SCL high = STOP */
    k_busy_wait(I2C_HALF_PERIOD_US);
}

/* Writes one byte, returns true if ACKed by device */
static bool bb_write_byte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if ((byte >> i) & 1) { sda_high(); } else { sda_low(); }
        scl_high(); scl_low();
    }
    /* Release SDA and sample ACK */
    gpio_pin_configure(g_gpiob, LH2_SDA_PIN,
                       GPIO_INPUT | GPIO_PULL_UP);
    scl_high();
    bool ack = (gpio_pin_get(g_gpiob, LH2_SDA_PIN) == 0);
    scl_low();
    gpio_pin_configure(g_gpiob, LH2_SDA_PIN,
                       GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
    return ack;
}

/* ── Sweep block state ──────────────────────────────────────────────────────── */

/*
 * Per-channel, per-rotor offset storage.
 * The UART frame's slow_bit field identifies which of the two rotors generated
 * each measurement (0 = rotor 0 / firstBeam, 1 = rotor 1 / secondBeam).
 * We store the latest offset for each (channel, rotor) pair and compute angles
 * only when we have a fresh measurement from both rotors of the same channel.
 *
 * (Declared as static inside process_frame() so the arrays live adjacent to
 * their use and the compiler can see they don't alias anything else.)
 */

/* Calibrated BS poses and the channel→BS index mapping */
static lh2_bs_pose_t g_bs_pose[LH2_BS_COUNT];
static bool          g_bs_pose_set[LH2_BS_COUNT];
static uint8_t       g_bs_channel[LH2_BS_COUNT] = { 0, 1 }; /* default mapping */

/* Output position + velocity under spinlock */
static struct k_spinlock g_pos_lock;
static lh2_position_t g_pos;
static lh2_position_t g_vel;    /* m/s, world frame, from position derivative */
static bool           g_pos_valid;

/* LPF coefficient for the position-derivative velocity estimate (applied per
 * accepted fix, ~50 Hz → time constant ≈ 60 ms). */
#define LH2_VEL_LPF_ALPHA  0.3f

/* Median filter state — circular buffer of raw triangulation results */
static float g_med_buf[3][LH2_MEDIAN_N];
static int   g_med_head;
static int   g_med_count;

/*
 * RX byte queue: the UART ISR drains the hardware FIFO into this queue so that
 * RXNE is cleared before the ISR returns.  Without this, RXNE stays asserted
 * and the ISR re-fires immediately in an infinite loop, starving every thread.
 *
 * 256 bytes ≈ 11 ms of FPGA data at 230400 baud (23040 bytes/sec).  The reader
 * thread must drain this within 11 ms to avoid drops; at our 200 ms main-thread
 * sleep interval bytes will be lost, but the LH2 protocol tolerates missing
 * frames — synchronize() retries until 12× 0xFF appear.
 */
K_MSGQ_DEFINE(g_rx_msgq, 1, 256, 1);

/* ── UART callback ──────────────────────────────────────────────────────────── */

static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    uart_irq_update(dev);
    /*
     * Drain ALL available bytes from the hardware FIFO before returning.
     * Calling uart_fifo_read() reads the USART DR register, which clears
     * the RXNE flag.  Without this drain, RXNE stays set after the ISR
     * returns and the interrupt immediately re-fires — an infinite loop
     * that starves every Zephyr thread.
     */
    while (uart_irq_rx_ready(dev)) {
        uint8_t c;
        uart_fifo_read(dev, &c, 1);
        k_msgq_put(&g_rx_msgq, &c, K_NO_WAIT);   /* ISR-safe, non-blocking */
    }
}

/* ── Frame reading ──────────────────────────────────────────────────────────── */

/*
 * read_byte — blocking read of exactly one byte from USART3.
 * The reader thread calls this; it blocks until a byte arrives.
 */
static uint8_t read_byte(void)
{
    uint8_t c;
    k_msgq_get(&g_rx_msgq, &c, K_FOREVER);
    return c;
}

/*
 * synchronize — drain bytes until we see LH2_FRAME_LEN consecutive 0xFF bytes.
 * Called once at startup before normal frame processing begins.
 */
static void synchronize(void)
{
    int count = 0;
    while (count < LH2_FRAME_LEN) {
        uint8_t c = read_byte();
        count = (c == 0xFF) ? count + 1 : 0;
    }
    LOG_INF("LH2 UART synchronized");
}

/* ── Math helpers ───────────────────────────────────────────────────────────── */

static void mat3_mul_vec(const float m[9], const float v[3], float out[3])
{
    out[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
    out[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
    out[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

/*
 * lh2_offsets_to_direction — convert two consecutive sweep offsets from one BS
 * into a unit direction vector in world frame.
 *
 * Source for angle formulas: pulse_processor_v2.c calculateAngles() and
 * pulseProcessorV2ConvertToV1Angles().
 *
 *   firstBeam  = (offset0 × 2π / period) − π + π/3
 *   secondBeam = (offset1 × 2π / period) − π − π/3
 *   azimuth    = (firstBeam + secondBeam) / 2
 *   elevation  = atan2(sin(secondBeam − firstBeam),
 *                      tan(π/6) × (cos(firstBeam) + cos(secondBeam)))
 *   d_local    = (sin(az)·cos(el), sin(el), cos(az)·cos(el))
 *   d_world    = R_bs × d_local
 */
static void lh2_offsets_to_direction(uint32_t offset0, uint32_t offset1,
                                     uint8_t channel,
                                     const lh2_bs_pose_t *pose, float out[3])
{
    uint32_t period = LH2_CYCLE_PERIODS[channel & 0x0F];
    float    twopi_over_period = 2.0f * (float)M_PI / (float)period;

    float first_beam  = (float)offset0 * twopi_over_period - (float)M_PI + (float)M_PI / 3.0f;
    float second_beam = (float)offset1 * twopi_over_period - (float)M_PI - (float)M_PI / 3.0f;

    float az = (first_beam + second_beam) * 0.5f;
    float el = atan2f(sinf(second_beam - first_beam),
                      LH2_TANT * (cosf(first_beam) + cosf(second_beam)));

    float cos_el   = cosf(el);
    float d_local[3] = { sinf(az) * cos_el, sinf(el), cosf(az) * cos_el };

    mat3_mul_vec(pose->rot, d_local, out);
}

/*
 * lh2_triangulate — midpoint of closest approach between two rays.
 * Returns false when rays are parallel (degenerate geometry).
 */
static bool lh2_triangulate(const float pa[3], const float da[3],
                             const float pb[3], const float db[3],
                             float out[3])
{
    float b = da[0]*db[0] + da[1]*db[1] + da[2]*db[2];
    float denom = 1.0f - b * b;
    if (fabsf(denom) < 1e-6f) {
        return false;
    }
    float w[3] = { pa[0]-pb[0], pa[1]-pb[1], pa[2]-pb[2] };
    float d = da[0]*w[0] + da[1]*w[1] + da[2]*w[2];
    float e = db[0]*w[0] + db[1]*w[1] + db[2]*w[2];
    float t = (b*e - d) / denom;
    float s = (e - b*d) / denom;
    out[0] = 0.5f * ((pa[0] + t*da[0]) + (pb[0] + s*db[0]));
    out[1] = 0.5f * ((pa[1] + t*da[1]) + (pb[1] + s*db[1]));
    out[2] = 0.5f * ((pa[2] + t*da[2]) + (pb[2] + s*db[2]));
    return true;
}

/* ── Median filter ──────────────────────────────────────────────────────────── */

/* Return the median of LH2_MEDIAN_N values (insertion-sorts a local copy). */
static float lh2_median(const float src[LH2_MEDIAN_N])
{
    float s[LH2_MEDIAN_N];
    for (int i = 0; i < LH2_MEDIAN_N; i++) { s[i] = src[i]; }
    for (int i = 1; i < LH2_MEDIAN_N; i++) {
        float key = s[i];
        int j = i - 1;
        while (j >= 0 && s[j] > key) { s[j + 1] = s[j]; j--; }
        s[j + 1] = key;
    }
    return s[LH2_MEDIAN_N / 2];
}

/* ── Frame processing ───────────────────────────────────────────────────────── */

/*
 * process_frame — parse one 12-byte data frame and update sweep block state.
 *
 * We use sensor 0 only (the first photodiode) for simplicity.  The Bitcraze
 * firmware aggregates all 4 sensors; a future enhancement could do the same.
 *
 * When a channel has two consecutive blocks, attempt position estimation.
 */
static void process_frame(const uint8_t data[LH2_FRAME_LEN])
{
    /* Sync frame: all 0xFF */
    bool is_sync = true;
    for (int i = 0; i < LH2_FRAME_LEN; i++) {
        if (data[i] != 0xFF) { is_sync = false; break; }
    }
    if (is_sync) {
        return;
    }

    bool    channel_found = (data[0] & 0x80) == 0;
    uint8_t channel       = (data[0] >> 3) & 0x0F;
    uint8_t slow_bit      = (data[0] >> 2) & 0x01;   /* 0 = rotor 0, 1 = rotor 1 */
    uint8_t sensor        = data[0] & 0x03;

    /* Validity checks */
    bool is_valid = channel_found && (sensor == 0);
    bool padding_ok = ((data[5] | data[8]) & 0xFE) == 0;
    if (!is_valid || !padding_ok) {
        return;
    }

    /* Extract offset (24-bit LE, 6 MHz ticks) → multiply by 4 → 24 MHz ticks */
    uint32_t offset_raw = (uint32_t)data[3]
                        | ((uint32_t)data[4] << 8)
                        | ((uint32_t)data[5] << 16);
    uint32_t offset = offset_raw * 4;

    if (offset == 0) {
        return;   /* no offset on this frame */
    }

    /*
     * Store this sweep in the slot for (channel, slow_bit).
     * slow_bit=0 → rotor 0 (firstBeam), slow_bit=1 → rotor 1 (secondBeam).
     * We need one measurement from each rotor to compute angles; previously
     * we used consecutive blocks regardless of rotor, which paired two frames
     * from the same rotor and produced wildly wrong directions.
     */
    static uint32_t g_rotor_offset[16][2];   /* [channel][slow_bit] */
    static uint32_t g_rotor_age_ms[16][2];
    static bool     g_rotor_valid[16][2];

    g_rotor_offset[channel][slow_bit] = offset;
    g_rotor_age_ms[channel][slow_bit] = k_uptime_get_32();
    g_rotor_valid[channel][slow_bit]  = true;

    /* Need both rotors to compute angles */
    int other_rotor = 1 - slow_bit;
    if (!g_rotor_valid[channel][other_rotor]) {
        return;
    }

    /* Check both measurements are fresh */
    uint32_t now      = k_uptime_get_32();
    uint32_t age_this = now - g_rotor_age_ms[channel][slow_bit];
    uint32_t age_other = now - g_rotor_age_ms[channel][other_rotor];
    if (age_this > LH2_BLOCK_FRESH_MS || age_other > LH2_BLOCK_FRESH_MS) {
        return;
    }

    /* offset0 = rotor-0 (slow_bit=0), offset1 = rotor-1 (slow_bit=1) */
    uint32_t offset0 = g_rotor_offset[channel][0];
    uint32_t offset1 = g_rotor_offset[channel][1];

    /* Find which pose this channel maps to */
    int bs_idx = -1;
    for (int i = 0; i < LH2_BS_COUNT; i++) {
        if (g_bs_channel[i] == channel) { bs_idx = i; break; }
    }
    if (bs_idx < 0 || !g_bs_pose_set[bs_idx]) {
        return;
    }

    /* Compute direction vector for this BS */
    float dir[3];
    lh2_offsets_to_direction(offset0, offset1,
                             channel, &g_bs_pose[bs_idx], dir);

    /*
     * Store direction.  We keep one direction per BS index.
     * If the other BS also has a fresh direction, triangulate.
     */
    static float    g_dir[LH2_BS_COUNT][3];
    static uint32_t g_dir_age[LH2_BS_COUNT];
    static bool     g_dir_valid[LH2_BS_COUNT];

    g_dir[bs_idx][0] = dir[0];
    g_dir[bs_idx][1] = dir[1];
    g_dir[bs_idx][2] = dir[2];
    g_dir_age[bs_idx]   = now;
    g_dir_valid[bs_idx] = true;

    /* Check other BS */
    int other = 1 - bs_idx;
    if (!g_dir_valid[other]) {
        return;
    }
    if ((now - g_dir_age[other]) > LH2_BLOCK_FRESH_MS) {
        return;
    }

    float result[3];
    bool ok = lh2_triangulate(g_bs_pose[0].origin, g_dir[0],
                              g_bs_pose[1].origin, g_dir[1],
                              result);
    if (!ok) {
        return;
    }

    /* Push raw result into the median buffer */
    g_med_buf[0][g_med_head] = result[0];
    g_med_buf[1][g_med_head] = result[1];
    g_med_buf[2][g_med_head] = result[2];
    g_med_head = (g_med_head + 1) % LH2_MEDIAN_N;
    if (g_med_count < LH2_MEDIAN_N) { g_med_count++; }

    /* Only publish once the window is full (avoids cold-start outliers) */
    if (g_med_count < LH2_MEDIAN_N) {
        return;
    }

    float mx = lh2_median(g_med_buf[0]);
    float my = lh2_median(g_med_buf[1]);
    float mz = lh2_median(g_med_buf[2]);

    /* Velocity from the filtered-position derivative, low-passed.  Feeds the
     * stabilizer's position-hold damping term — a P-only position loop has no
     * damping and limit-cycles (stock runs a full velocity PID between its
     * position and attitude loops for the same reason). */
    static int64_t g_vel_t_ms;
    static float   g_prev[3];
    int64_t t_ms  = k_uptime_get();
    int64_t dt_ms = t_ms - g_vel_t_ms;
    float vx = g_vel.x, vy = g_vel.y, vz = g_vel.z;

    if (g_vel_t_ms != 0 && dt_ms >= 5 && dt_ms <= 500) {
        float inv_dt = 1000.0f / (float)dt_ms;
        float a = LH2_VEL_LPF_ALPHA;
        vx += a * ((mx - g_prev[0]) * inv_dt - vx);
        vy += a * ((my - g_prev[1]) * inv_dt - vy);
        vz += a * ((mz - g_prev[2]) * inv_dt - vz);
    } else {
        /* First fix, or a gap long enough that the derivative is meaningless */
        vx = 0.0f; vy = 0.0f; vz = 0.0f;
    }
    g_prev[0] = mx; g_prev[1] = my; g_prev[2] = mz;
    g_vel_t_ms = t_ms;

    k_spinlock_key_t key = k_spin_lock(&g_pos_lock);
    g_pos.x     = mx;
    g_pos.y     = my;
    g_pos.z     = mz;
    g_vel.x     = vx;
    g_vel.y     = vy;
    g_vel.z     = vz;
    g_pos_valid = true;
    k_spin_unlock(&g_pos_lock, key);
}

/* ── Deck boot (bit-bang I2C) ────────────────────────────────────────────────── */

/*
 * bb_write_seq — one I2C write transaction to LH2_BOOTLOADER_ADDR.
 * Returns true if address and every data byte are ACK'd.
 */
static bool bb_write_seq(const uint8_t *data, size_t len)
{
    bb_start();
    bool ok = bb_write_byte((LH2_BOOTLOADER_ADDR << 1) | 0);
    for (size_t i = 0; ok && i < len; i++) {
        ok = bb_write_byte(data[i]);
    }
    bb_stop();
    return ok;
}

/*
 * bb_read_byte_nack — read one byte from the slave, then send NAK + STOP.
 * Call only after a successful write of the read-address byte (R bit = 1).
 * SDA is left as OUTPUT_HIGH_OPEN_DRAIN on return; caller sends bb_stop().
 */
static uint8_t bb_read_byte_nack(void)
{
    uint8_t val = 0;
    gpio_pin_configure(g_gpiob, LH2_SDA_PIN, GPIO_INPUT | GPIO_PULL_UP);
    for (int i = 7; i >= 0; i--) {
        scl_high();
        val |= (uint8_t)(gpio_pin_get(g_gpiob, LH2_SDA_PIN) << i);
        scl_low();
    }
    /* NAK: master drives SDA HIGH (open-drain release = pulled up = NAK) */
    gpio_pin_configure(g_gpiob, LH2_SDA_PIN, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
    scl_high();   /* slave sees the NAK */
    scl_low();
    return val;
}

/*
 * lh2_boot_deck — boot the lighthouse deck FPGA via the deck bootloader (I2C 0x2F).
 *
 * Implements the full Bitcraze pre-boot sequence from lighthouse_deck_flasher.c:
 *
 *   1. GET_VERSION  write [0x02, 0x42] → read 1 byte (advances the bootloader
 *                  state machine; required before FLASH_WAKEUP and BOOT_TO_FW
 *                  will be accepted).
 *                  Source: lhblGetVersion() — dummy byte 0x42 works around an
 *                  I2C library bug in the CF firmware that sends 2 bytes when
 *                  writeLen=1.
 *
 *   2. FLASH_WAKEUP write [0x01, 0x01, 0x00, 0x00, 0x00, 0xAB]
 *                  Wakes the deck's SPI flash (lhblFlashWakeup via lhblHeaderPrepare).
 *
 *   3. BOOT_TO_FW  write [0x00] — deck bootloader loads FPGA bitstream from flash.
 *                  Source: lhblBootToFW().
 *
 * Skipped vs. Bitcraze: CRC verification of the bitstream (steps 3-5 in the
 * original).  Without the CF firmware's embedded bitstream we cannot verify CRC;
 * we rely on the deck's own internal CRC check (if any) or assume the bitstream
 * stored on the deck is valid.
 */
static int lh2_boot_deck(void)
{
    if (!device_is_ready(g_gpiob)) {
        LOG_ERR("GPIOB not ready");
        return -ENODEV;
    }

    gpio_pin_configure(g_gpiob, LH2_SCL_PIN, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
    gpio_pin_configure(g_gpiob, LH2_SDA_PIN, GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN);
    k_busy_wait(I2C_HALF_PERIOD_US);

    int ret = 0;

    /* ── Step 1: GET_VERSION ─────────────────────────────────────────────────
     * Write [0x02, 0x42] then read 1 byte (bootloader version).
     * The read transaction is required to advance the bootloader state machine.
     */
    static const uint8_t ver_w[] = { 0x02, 0x42 };
    if (!bb_write_seq(ver_w, sizeof(ver_w))) {
        LOG_ERR("GET_VERSION write NAK — bootloader at 0x%02X not responding; "
                "deck seated? power-cycled? PB6=SCL PB7=SDA?",
                LH2_BOOTLOADER_ADDR);
        ret = -EIO;
        goto release;
    }

    /* Read transaction: START + read-address + 1 byte + NAK + STOP */
    bb_start();
    if (!bb_write_byte((LH2_BOOTLOADER_ADDR << 1) | 1)) {
        bb_stop();
        LOG_ERR("GET_VERSION read-address NAK");
        ret = -EIO;
        goto release;
    }
    uint8_t bl_version = bb_read_byte_nack();
    bb_stop();
    LOG_INF("LH2 bootloader version %u", bl_version);

    /* ── Step 2: FLASH_WAKEUP ────────────────────────────────────────────────
     * LHBL_BL_CMD header + FLASH_CMD_WAKEUP (0xAB).
     */
    static const uint8_t wakeup[] = { 0x01, 0x01, 0x00, 0x00, 0x00, 0xAB };
    if (!bb_write_seq(wakeup, sizeof(wakeup))) {
        LOG_ERR("FLASH_WAKEUP NAK");
        ret = -EIO;
        goto release;
    }
    /*
     * After FLASH_WAKEUP the deck's bootloader MCU reads the full FPGA
     * bitstream (104 KB) from SPI flash and runs its own internal CRC check
     * before it will accept BOOT_TO_FW.  The stock CF firmware occupies this
     * window reading the bitstream itself over I2C (which takes several seconds
     * at 100 kHz).  We skip that read, so we must wait explicitly.
     * At SPI 4 MHz: 104 KB / 500 KB/s ≈ 0.2 s of flash reading + CRC time.
     * Use 3 s to be conservative.
     */
    LOG_INF("LH2 waiting for deck CRC check (~3 s)...");
    k_msleep(3000);

    /* ── Step 3: BOOT_TO_FW ─────────────────────────────────────────────────
     * Send BOOT_TO_FW and log the result, but do NOT fail on NAK.
     *
     * The deck's bootloader performs its own internal CRC check after
     * FLASH_WAKEUP and auto-boots the FPGA when the check passes — which
     * it has already done during our 3-second wait.  By the time we send
     * BOOT_TO_FW, the bootloader MCU has jumped to the FPGA application
     * and is no longer serving I2C at 0x2F, so BOOT_TO_FW NAKs.
     *
     * Since GET_VERSION and FLASH_WAKEUP both succeeded, the boot was
     * initiated.  We proceed to synchronize() regardless of the BOOT_TO_FW
     * result; if the FPGA is streaming, synchronize() will detect it.
     */
    bb_start();
    bool addr_ok = bb_write_byte((LH2_BOOTLOADER_ADDR << 1) | 0);
    bool data_ok = addr_ok && bb_write_byte(LHBL_BOOT_TO_FW);
    bb_stop();

    if (!addr_ok) {
        LOG_INF("BOOT_TO_FW: address NAK — deck already auto-booted (expected)");
    } else if (!data_ok) {
        LOG_INF("BOOT_TO_FW: data NAK — deck booting from internal CRC pass");
    } else {
        LOG_INF("BOOT_TO_FW: ACK — explicit boot command accepted");
        k_msleep(10);   /* FPGA configuration time */
    }
    LOG_INF("LH2 deck boot sequence complete — proceeding to UART sync");

release:
    gpio_pin_configure(g_gpiob, LH2_SCL_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(g_gpiob, LH2_SDA_PIN, GPIO_INPUT | GPIO_PULL_UP);
    return ret;
}

/* ── Reader thread ──────────────────────────────────────────────────────────── */

K_THREAD_STACK_DEFINE(g_lh2_stack, LH2_STACK_SIZE);
static struct k_thread g_lh2_thread;

static void lh2_reader_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    /*
     * Wait for the deck bootloader MCU to finish initializing.
     * The deck powers on with the CF but its MCU takes ~1-2 s before it
     * responds to I2C.  Without this delay, the first BOOT_TO_FW attempt
     * arrives while the MCU is still starting up, causing it to NAK and
     * then enter a non-responsive state for subsequent attempts.
     */
    k_msleep(2000);
    LOG_INF("LH2 attempting deck boot...");

    /* Boot the deck FPGA via bit-bang I2C.  Retry every 10 s on failure
     * — long enough for the bootloader to finish any internal processing
     * and become responsive again after a failed attempt. */
    while (lh2_boot_deck() != 0) {
        LOG_WRN("Deck boot failed — retrying in 10 s");
        k_msleep(10000);
    }

    synchronize();

    uint8_t frame[LH2_FRAME_LEN];
    while (true) {
        for (int i = 0; i < LH2_FRAME_LEN; i++) {
            frame[i] = read_byte();
        }
        process_frame(frame);
    }
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

int cf21bl_lighthouse_init(void)
{
    if (!device_is_ready(g_uart)) {
        LOG_ERR("USART3 not ready — check CONFIG_UART_INTERRUPT_DRIVEN=y");
        return -ENODEV;
    }

    uart_irq_callback_set(g_uart, uart_cb);
    uart_irq_rx_enable(g_uart);

    k_thread_create(&g_lh2_thread, g_lh2_stack,
                    K_THREAD_STACK_SIZEOF(g_lh2_stack),
                    lh2_reader_fn, NULL, NULL, NULL,
                    LH2_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&g_lh2_thread, "cf21bl_lh2");

    LOG_INF("LH2 reader started (USART3 230400 baud, PC10/PC11)");
    LOG_INF("  BS0 channel=%u  BS1 channel=%u", g_bs_channel[0], g_bs_channel[1]);
    return 0;
}

void cf21bl_lighthouse_set_bs_pose(int id, const lh2_bs_pose_t *pose)
{
    if (id < 0 || id >= LH2_BS_COUNT) {
        return;
    }
    g_bs_pose[id]     = *pose;
    g_bs_pose_set[id] = true;
    LOG_INF("LH2 BS%d pose set: origin=(%.3f, %.3f, %.3f), channel=%u",
            id,
            (double)pose->origin[0],
            (double)pose->origin[1],
            (double)pose->origin[2],
            g_bs_channel[id]);
}

void cf21bl_lighthouse_set_bs_channel(int bs_id, uint8_t channel)
{
    if (bs_id < 0 || bs_id >= LH2_BS_COUNT || channel > 15) {
        return;
    }
    g_bs_channel[bs_id] = channel;
}

int cf21bl_lighthouse_get_position(lh2_position_t *pos)
{
    k_spinlock_key_t key = k_spin_lock(&g_pos_lock);
    bool valid = g_pos_valid;
    if (valid) {
        *pos = g_pos;
    }
    k_spin_unlock(&g_pos_lock, key);
    return valid ? 0 : -EAGAIN;
}

int cf21bl_lighthouse_get_velocity(lh2_position_t *vel)
{
    k_spinlock_key_t key = k_spin_lock(&g_pos_lock);
    bool valid = g_pos_valid;
    if (valid) {
        *vel = g_vel;
    }
    k_spin_unlock(&g_pos_lock, key);
    return valid ? 0 : -EAGAIN;
}

bool cf21bl_lighthouse_is_valid(void)
{
    return g_pos_valid;
}
