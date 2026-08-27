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
 *                bit[2]    = slowBit — one bit of the OOTX calibration data
 *                            stream (NOT a sweep-plane id; see process_frame)
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
 * Direction in BS-local frame (X=forward out of face, Y=+azimuth/left, Z=up
 * — the cfclient/stock lighthouse_geometry.c convention the calibration
 * rotation matrices are estimated for):
 *   d_local = normalize(cos(el)·cos(az), cos(el)·sin(az), sin(el)·cos(az))
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

/* CONFIG_CF21BL_LH2_LOG_LEVEL lets an application quiet the per-fix info
 * stream on a shared multi-drone console (see cf21bl-formation's
 * DEMO_CONSOLE_VERBOSE).  Apps that do not define it keep LOG_LEVEL_INF. */
#ifndef CONFIG_CF21BL_LH2_LOG_LEVEL
#define CONFIG_CF21BL_LH2_LOG_LEVEL LOG_LEVEL_INF
#endif
LOG_MODULE_REGISTER(cf21bl_lh2, CONFIG_CF21BL_LH2_LOG_LEVEL);

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
 * Position staleness timeout.
 *
 * g_pos_valid used to be a LATCH: set true on the first fix and never
 * cleared, so cf21bl_lighthouse_is_valid() reported "valid" forever no
 * matter how old g_pos was.  Occlude the base stations mid-flight (a body
 * in the way, a drone turning through its own shadow) and the position
 * simply FREEZES at its last value while still reporting valid.
 *
 * That defeated both safety nets at once, because both read the same
 * estimate: the consumer's fix-loss path never armed (it gates on
 * is_valid()), and its geofence kept measuring the frozen — legal —
 * position.  Meanwhile the stabilizer saw a position error that could
 * never shrink, wound up, and flew the airframe away.  2026-08-26
 * flight 24: pos frozen at (-0.37, 0.62) for 21 s, no fix-loss warning, no
 * geofence breach, no landing trigger of any kind — the drone flew until
 * it hit something.
 *
 * 400 ms = 2x LH2_BLOCK_FRESH_MS: long enough that a normal ~30 Hz fix
 * stream never trips it and a brief single-station dropout rides through
 * on the other station, short enough that the consumer's own
 * FIX_LOSS_GRACE_MS (2 s of SUSTAINED loss before landing) still dominates
 * the actual land/no-land decision.  This timeout decides "is this
 * estimate still real", not "should we land".
 */
#define LH2_POS_STALE_MS    400u

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
 * Per-channel sweep-pair storage: one pending sweep per channel, matched to
 * its revolution partner by rotor-zero time (timestamp − offset) — see the
 * pairing comment in process_frame().  Beam roles come from arrival order
 * within the matched pair, exactly like stock calculateAngles.
 *
 * (Declared as static inside process_frame() so the arrays live adjacent to
 * their use and the compiler can see they don't alias anything else.)
 */

/* Calibrated BS poses and the channel→BS index mapping */
static lh2_bs_pose_t g_bs_pose[LH2_BS_COUNT];
static bool          g_bs_pose_set[LH2_BS_COUNT];
static uint8_t       g_bs_channel[LH2_BS_COUNT] = { 0, 1 }; /* default mapping */

/* OOTX sweep calibration (optional — angles used raw when not set) */
static lh2_bs_calib_t g_bs_calib[LH2_BS_COUNT];
static bool           g_bs_calib_set[LH2_BS_COUNT];

/* Per-channel ray health counters — quantify asymmetric BS quality (e.g.
 * flights showing one BS's rays rejected/mispaired far more than the
 * other's) instead of eyeballing sparsely-throttled debug lines. Indexed
 * by raw channel (0-15), incremented in lh2_offsets_to_direction() (ok/
 * rej_angle) and process_frame()'s pairing block (pair_miss); dumped and
 * reset every LH2_STAT_PERIOD_MS by lh2_log_stats_maybe(). */
#define LH2_STAT_PERIOD_MS 2000u
static uint32_t g_ch_ok[16];
static uint32_t g_ch_rej_angle[16];
static uint32_t g_ch_pair_miss[16];
static uint32_t g_stat_last_ms;

/* Output position + velocity under spinlock */
static struct k_spinlock g_pos_lock;
static lh2_position_t g_pos;
static lh2_position_t g_vel;    /* m/s, world frame, from position derivative */
static bool           g_pos_valid;
static uint32_t       g_pos_ms;      /* uptime of the last accepted fix */

/* LPF coefficient for the position-derivative velocity estimate (applied per
 * accepted fix, ~50 Hz → time constant ≈ 60 ms). */
/* Velocity estimate: finite difference over a ~150 ms baseline (ring
 * buffer of accepted fixes) instead of per-publish (~10–20 ms) deltas.
 * Position noise (median quantization, ~cm) is amplified by 1/dt when
 * differentiating: the short-baseline version needed a heavy LPF
 * (α=0.3) whose lag+attenuation cut the delivered damping ~4× — the
 * 2026-07-05 30 s-hold flight measured real |v|≈0.12 m/s while the
 * estimate logged 0.03, so the Kd term flew at a quarter strength.  A
 * 150 ms baseline divides the same noise by ~10× more time; only light
 * smoothing is needed and the estimate tracks real motion. */
#define LH2_VEL_BASELINE_MS      150   /* target finite-difference baseline  */
#define LH2_VEL_MIN_BASELINE_MS  60    /* shortest usable baseline (startup) */
#define LH2_VEL_MAX_BASELINE_MS  400   /* older ⇒ a fix outage — restart     */
#define LH2_VEL_BUF_N            32    /* ≥320 ms of history at ~100 fix/s   */
#define LH2_VEL_LPF_ALPHA        0.5f
/* When the derivative can't be formed (jump-gate re-seed flushes the
 * history, fix outage, startup), the velocity estimate DECAYS with this
 * half-life instead of snapping to zero.  Flight logs 2026-07-05 showed
 * vx/vy=0.000 exactly at peak position errors — every hard-zero turned
 * the position loop P-only right when damping mattered most. */
#define LH2_VEL_DECAY_HALF_MS  100.0f

/* Sweep pairing: two frames belong to the same revolution when their
 * rotor-zero times (timestamp − offset) agree within this many 24 MHz
 * ticks.  Stock pulse_processor_v2.c uses 10 for block-to-block matching,
 * but that proved too tight on our single-sensor frame path (no pairs
 * formed at all on the bench, 2026-07-05).  Physics allows a much looser
 * gate: the only events that can agree in rotor-zero time are the two
 * sweeps of one revolution — the same plane next revolution is a full
 * cycle (~480 000 ticks) away, so anything ≪ half a period discriminates
 * perfectly.  2 000 ticks = 83 µs. */
#define LH2_MAX_T0_DIFF    2000u

/* Within one revolution, offsets closer than this are the SAME sweep seen
 * by different photodiodes (sensor spread ≤ ~1k ticks), not the second
 * plane (~period/3 ≈ 160k ticks away). */
#define LH2_SAME_SWEEP_MAX 20000u

/* Max believable ray closest-approach distance for a published fix.
 * Nominal is < 0.1 m without OOTX sweep calibration applied. */
#define LH2_MAX_MISS_M     0.3f

/* Angle-plausibility gate: reject sweep pairs whose az/el cannot be a real
 * drone position.  TWO repeatable phantom pairs observed (2026-07-05, OLD
 * pre-recalibration base-station placement):
 *   ch1 az≈+71° el≈−75° — three flights, incl. once while stationary on
 *     the ground; won the jump gate's consecutive-N re-seed and ended one
 *     flight on a phantom "Touchdown at z=-19.8 m".
 *   ch0 az≈−36° el≈−44° — two flights; triangulates with a CLEAN miss
 *     (0.004 m — a phantom ray still crosses the other BS's real ray, at
 *     the wrong depth) producing fixes 0.3–0.4 m off that beat the miss
 *     gate AND the old 0.5 m jump gate.
 * Mechanism (specular reflection vs. driver pairing artifact) undetermined
 * — the reject log below includes raw offsets to discriminate: a genuine
 * plane pair has Δoff ≈ 160k ticks; a real+reflected optical pair shows a
 * different but consistent same-revolution Δoff; a mis-pair that slipped
 * the t0 guard shows anomalous offsets.  The gate works either way.
 *
 * Bounds are the LH2 hardware's optical FOV (~150°×110° → az ±75°,
 * el ±55°) — a fixed physical property of the base station, independent of
 * room geometry or where the stations are mounted, so it can never go
 * stale on a recalibration.  Anything a station can physically illuminate
 * passes; only decode/pairing artifacts (like the el≈−75° family above,
 * impossible optics) are rejected.
 *
 * HISTORY (2026-07-1x): the previous floor, el ≥ −35°, was an empirical
 * fit to the OLD placement ("legit extreme −26.8°, phantom −43.7°, split
 * the gap").  The 2026-07-06 recalibrations moved and retilted both
 * stations; nobody re-derived the bound.  Replaying this driver's own
 * az/el math over the current calibration (lighthouse_cal.yaml)
 * shows legitimate flight-volume elevations reach −44.6° on BS0
 * (the calibration origin itself sits at −35.3°, already outside the old
 * floor), and the flight-4 "reject implausible angles ch=0 el=−38..−42,
 * az=−11..+7" burst maps exactly onto real room positions ~0.3–0.5 m from
 * directly under BS0 — genuine rays, wrongly rejected, starving the BS0
 * direction (stale-ray fixes for LH2_BLOCK_FRESH_MS, then no fixes) right
 * as the drone flew that zone.  That gate-induced dropout↔XY-excursion
 * coupling recurred across 4 flights/3 airframes before being traced.
 *
 * CAUTION: the old ch0 phantom family (el −43..−50°) is INSIDE the FOV
 * bound, so this gate alone no longer rejects it.  That family was only
 * ever observed under the old placement/aim, and the defenses added since
 * it last did damage (miss gate 0.3 m, median-of-5, 0.2 m-floor speed
 * jump gate, velocity decay, ±0.75 m stabilizer error saturation, and the
 * corrected position-hold sign) all stand between a re-admitted phantom
 * and the controller.  The watch log below keeps the formerly-rejected
 * band visible in flight logs (with the Δoff discriminator) so a
 * re-emergence is caught from telemetry, not from another excursion hunt.
 * Do NOT re-tighten these to a room-derived envelope: three prior
 * attempts (2026-07-06) showed any bound tuned to a "legitimate small
 * envelope" clips exactly the large excursions it is meant to diagnose. */
#define LH2_AZ_LIMIT_RAD   (75.0f * (float)M_PI / 180.0f)
#define LH2_EL_MIN_RAD     (-55.0f * (float)M_PI / 180.0f)
#define LH2_EL_MAX_RAD     (55.0f * (float)M_PI / 180.0f)

/* Watch band: rays that the pre-2026-07-1x bounds (az ±60°, el −35..+30°)
 * would have rejected but the FOV bounds accept.  Logged (throttled) so the
 * old phantom signatures remain visible if they resurface under the new
 * base-station aim.  Diagnostic only — no gating. */
#define LH2_AZ_WATCH_RAD   (60.0f * (float)M_PI / 180.0f)
#define LH2_EL_WATCH_MIN_RAD (-35.0f * (float)M_PI / 180.0f)
#define LH2_EL_WATCH_MAX_RAD (30.0f * (float)M_PI / 180.0f)

/* Jump gate (see comment at the publish site): max believable displacement
 * between consecutive accepted fixes, and how many consecutive far fixes
 * constitute a genuine re-acquisition rather than a wrong-geometry branch.
 * At ~50 Hz fix rate, 0.2 m/fix would still be 10 m/s — far beyond real
 * motion.  Tightened 0.5→0.2 (2026-07-05): phantom #2's clean-miss fixes
 * landed only 0.3–0.4 m off and sailed under the 0.5 m threshold; the
 * flip-flop counter already protects against interleaved phantoms
 * stealing a re-seed.
 *
 * REVISED 2026-07-06: LH2_JUMP_M alone assumes the ~50 Hz fix rate holds
 * continuously.  This room's 2026-07-06 base-station placement produces
 * noticeably more pair-miss/angle-reject traffic than the old placement
 * (visible throughout every flight log) — when the accepted-fix rate dips,
 * two consecutive accepted fixes are further apart in TIME, so genuine,
 * physically real motion can legitimately cover more than a fixed 0.2 m
 * between them.  Confirmed on a real flight log (2026-07-06): the velocity
 * estimate disagreed in SIGN with the velocity implied by real consecutive
 * position readings, at exactly the points the drone was moving fastest —
 * consistent with the jump gate misreading a real fast movement (across a
 * temporarily slower fix cadence) as a teleport, discarding it or wiping
 * the velocity history via re-seed, and leaving a stale/decayed velocity
 * feeding CF21BL_POS_KD right when accurate damping mattered most (a
 * wrong-signed "damping" term adds energy instead of removing it).  The
 * jump limit is now a SPEED limit (LH2_MAX_SPEED_MPS), scaled by the
 * actual elapsed time since the last accepted fix, so a real gap in fix
 * rate no longer misclassifies real motion as a teleport — only motion
 * that would require exceeding this drone's plausible top speed still
 * gets rejected. */
#define LH2_JUMP_M         0.2f
#define LH2_JUMP_REJECT_N  10
#define LH2_MAX_SPEED_MPS  3.0f   /* generous ceiling for this airframe */

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

    /* Read-and-clear UART error flags — an uncleared error condition
     * re-asserts the interrupt forever (see pm_uart_cb in cf21bl_pm.c for
     * the full failure mode).  The 230400-baud FPGA stream tolerates the
     * occasional dropped byte; a latched error flag is fatal. */
    (void)uart_err_check(dev);

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

/* ── OOTX sweep calibration ─────────────────────────────────────────────────
 * Port of stock lighthouse_calibration.c (lighthouseCalibrationApplyV2 and
 * lighthouseCalibrationMeasurementModelLh2).  The base station broadcasts a
 * distortion model of its own sweep planes; the measured beam angles are the
 * DISTORTED ones, so we invert the ideal→distorted model by fixed-point
 * iteration to recover the ideal angles the triangulation assumes.
 * Stock uses only tilt/phase/gibphase/gibmag for LH2 (curve and ogee are an
 * acknowledged TODO there), so this port has the same limitation.
 */

static inline float lh2_clip1(float v)
{
    if (v >  1.0f) { return  1.0f; }
    if (v < -1.0f) { return -1.0f; }
    return v;
}

/* Distorted (measured) beam angle a rotor with distortion `sweep` reports
 * when its ideally-tilted plane (tilt t = ∓π/6) crosses direction (x,y,z).
 * Stock: lighthouseCalibrationMeasurementModelLh2(). */
static float lh2_calib_model(float x, float y, float z, float t,
                             const lh2_bs_calib_sweep_t *sweep)
{
    float ax = atan2f(y, x);
    float r  = sqrtf(x * x + y * y);

    float base     = ax + asinf(lh2_clip1(z * tanf(t - sweep->tilt) / r));
    float comp_gib = -sweep->gibmag * cosf(ax + sweep->gibphase);

    return base - (sweep->phase + comp_gib);
}

/* Ideal beam-angle pair → the distorted pair the BS would measure.
 * Stock: idealToDistortedV2(). */
static void lh2_ideal_to_distorted(const lh2_bs_calib_t *calib,
                                   const float ideal[2], float distorted[2])
{
    const float t30   = (float)M_PI / 6.0f;
    const float a1    = ideal[0];
    const float a2    = ideal[1];

    /* Direction implied by the ideal pair (unnormalized; the model only
     * uses ratios).  z denominator = tan30·(cos a1 + cos a2) folded into
     * the same form stock uses. */
    float x = 1.0f;
    float y = tanf((a2 + a1) / 2.0f);
    float z = sinf(a2 - a1) / (LH2_TANT * (cosf(a2) + cosf(a1)));

    distorted[0] = lh2_calib_model(x, y, z, -t30, &calib->sweep[0]);
    distorted[1] = lh2_calib_model(x, y, z,  t30, &calib->sweep[1]);
}

/* Invert the distortion: measured (raw) pair → ideal pair, by fixed-point
 * iteration (the distortion is small, so convergence is fast — stock caps
 * at 5 iterations / 0.0005 rad).  Stock: lighthouseCalibrationApply(). */
static void lh2_calib_apply(const lh2_bs_calib_t *calib,
                            const float raw[2], float corrected[2])
{
    const float max_delta = 0.0005f;

    corrected[0] = raw[0];
    corrected[1] = raw[1];

    for (int i = 0; i < 5; i++) {
        float distorted[2];
        lh2_ideal_to_distorted(calib, corrected, distorted);

        float delta0 = raw[0] - distorted[0];
        float delta1 = raw[1] - distorted[1];

        corrected[0] += delta0;
        corrected[1] += delta1;

        if (fabsf(delta0) < max_delta && fabsf(delta1) < max_delta) {
            break;
        }
    }
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
static bool lh2_offsets_to_direction(uint32_t offset0, uint32_t offset1,
                                     uint8_t channel,
                                     const lh2_bs_pose_t *pose,
                                     const lh2_bs_calib_t *calib, float out[3])
{
    uint32_t period = LH2_CYCLE_PERIODS[channel & 0x0F];
    float    twopi_over_period = 2.0f * (float)M_PI / (float)period;

    float first_beam  = (float)offset0 * twopi_over_period - (float)M_PI + (float)M_PI / 3.0f;
    float second_beam = (float)offset1 * twopi_over_period - (float)M_PI - (float)M_PI / 3.0f;

    /* Undo the base station's broadcast sweep-plane distortion BEFORE the
     * az/el conversion, in raw beam-angle space — same pipeline position as
     * stock (pulseProcessorApplyCalibration runs on measurement->angles,
     * then ConvertToV1Angles consumes correctedAngles). */
    if (calib != NULL) {
        float raw[2] = { first_beam, second_beam };
        float cor[2];
        lh2_calib_apply(calib, raw, cor);
        first_beam  = cor[0];
        second_beam = cor[1];
    }

    float az = (first_beam + second_beam) * 0.5f;
    float el = atan2f(sinf(second_beam - first_beam),
                      LH2_TANT * (cosf(first_beam) + cosf(second_beam)));

    /* Angle-plausibility gate (see LH2_AZ_LIMIT_RAD block comment): kill
     * phantom pairs here, before they can reach the triangulator and win
     * the jump gate's re-seed logic.  Raw offsets included so the phantom
     * mechanism can be identified from the log (Δoff signature — see the
     * block comment). */
    if (az > LH2_AZ_LIMIT_RAD || az < -LH2_AZ_LIMIT_RAD ||
        el > LH2_EL_MAX_RAD   || el < LH2_EL_MIN_RAD) {
        g_ch_rej_angle[channel]++;
        static int rej_div;
        if (++rej_div >= 8) {
            rej_div = 0;
            LOG_INF("reject implausible angles ch=%u az=%.1f el=%.1f "
                    "off0=%u off1=%u",
                    channel, (double)(az * 180.0f / (float)M_PI),
                    (double)(el * 180.0f / (float)M_PI),
                    offset0, offset1);
        }
        return false;
    }

    /* Accepted, but inside the watch band the old empirical bounds would
     * have rejected (see LH2_AZ_WATCH_RAD block comment).  Offsets included
     * so a resurfaced reflection shows its Δoff signature in flight logs. */
    if (az > LH2_AZ_WATCH_RAD || az < -LH2_AZ_WATCH_RAD ||
        el > LH2_EL_WATCH_MAX_RAD || el < LH2_EL_WATCH_MIN_RAD) {
        static int watch_div;
        if (++watch_div >= 8) {
            watch_div = 0;
            LOG_INF("accept beyond old bounds ch=%u az=%.1f el=%.1f "
                    "off0=%u off1=%u",
                    channel, (double)(az * 180.0f / (float)M_PI),
                    (double)(el * 180.0f / (float)M_PI),
                    offset0, offset1);
        }
    }

    /*
     * Base-station local frame MUST match the one the calibration rotation
     * matrix was estimated for.  cfclient geometry (and stock
     * lighthouse_geometry.c lighthouseGeometryGetRay()) uses:
     *   X = forward (out of the face), Y = +azimuth (left), Z = up
     * with the ray built as the intersection of the two sweep planes:
     *   a = ( sin H, -cos H, 0 )         b = ( -sin V, 0, cos V )
     *   ray = normalize(b × a) = normalize( cosV·cosH, cosV·sinH, sinV·cosH )
     * The original code here used a Z-forward/Y-up spherical convention —
     * applying the cfclient matrix to that permuted vector produced a
     * consistently rotated world (drone on the floor read z ≈ +1.9 m and
     * XY position hold pushed in wrong directions → wandering).
     */
    /* Convention-debug log (~1 Hz per BS at typical fix rates): compare
     * against the expected angles computed from the calibrated geometry for
     * a drone at a known position.  Sign errors in az/el (sweep pairing,
     * rotation direction) show up here directly, decoupled from the
     * triangulation. */
    static int dbg_div;
    if (++dbg_div >= 128) {
        dbg_div = 0;
        LOG_INF("angles ch=%u az=%.1f el=%.1f deg",
                channel, (double)(az * 180.0f / (float)M_PI),
                (double)(el * 180.0f / (float)M_PI));
    }

    float cH = cosf(az), sH = sinf(az);
    float cV = cosf(el), sV = sinf(el);
    float d_local[3] = { cV * cH, cV * sH, sV * cH };
    float len = sqrtf(d_local[0] * d_local[0] +
                      d_local[1] * d_local[1] +
                      d_local[2] * d_local[2]);
    if (len > 0.0f) {
        d_local[0] /= len; d_local[1] /= len; d_local[2] /= len;
    }

    mat3_mul_vec(pose->rot, d_local, out);
    g_ch_ok[channel]++;
    return true;
}

/*
 * lh2_triangulate — midpoint of closest approach between two rays.
 * Returns false when rays are parallel (degenerate geometry).
 */
static bool lh2_triangulate(const float pa[3], const float da[3],
                             const float pb[3], const float db[3],
                             float out[3], float *miss_m)
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
    float qa[3] = { pa[0] + t*da[0], pa[1] + t*da[1], pa[2] + t*da[2] };
    float qb[3] = { pb[0] + s*db[0], pb[1] + s*db[1], pb[2] + s*db[2] };
    out[0] = 0.5f * (qa[0] + qb[0]);
    out[1] = 0.5f * (qa[1] + qb[1]);
    out[2] = 0.5f * (qa[2] + qb[2]);

    /* Closest-approach miss distance: the health metric for the whole
     * geometry chain.  With correct angles + frames the two rays nearly
     * intersect (centimetres); a frame/convention error leaves them
     * wildly skew while still producing a stable-looking midpoint. */
    if (miss_m) {
        float mx = qa[0]-qb[0], my = qa[1]-qb[1], mz = qa[2]-qb[2];
        *miss_m = sqrtf(mx*mx + my*my + mz*mz);
    }
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
static void lh2_log_stats_maybe(void)
{
    uint32_t now = k_uptime_get_32();
    if (now - g_stat_last_ms < LH2_STAT_PERIOD_MS) {
        return;
    }
    g_stat_last_ms = now;

    for (int i = 0; i < LH2_BS_COUNT; i++) {
        uint8_t ch = g_bs_channel[i];
        LOG_INF("lh2 stats bs%d(ch%u): ok=%u rej_angle=%u pair_miss=%u",
                i, ch, g_ch_ok[ch], g_ch_rej_angle[ch], g_ch_pair_miss[ch]);
        g_ch_ok[ch]        = 0;
        g_ch_rej_angle[ch] = 0;
        g_ch_pair_miss[ch] = 0;
    }
}

static void process_frame(const uint8_t data[LH2_FRAME_LEN])
{
    /* Per-BS ray health, dumped every LH2_STAT_PERIOD_MS regardless of this
     * frame's content — see g_ch_ok/g_ch_rej_angle/g_ch_pair_miss above. */
    lh2_log_stats_maybe();

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
    /* data[0] bit 2 is the slowBit: one bit of the base station's OOTX
     * calibration data stream (stock feeds it to ootxDecoderProcessBit and
     * nothing else).  It is NOT a sweep-plane identifier — a previous
     * revision of this driver keyed sweep pairing on it, which paired
     * same-plane offsets from different revolutions and produced stable
     * but wildly wrong angles (bench 2026-07-05: az off by ~60°, el by
     * 50-80°, ray miss distance 1.7 m). */
    uint8_t sensor        = data[0] & 0x03;

    /* Validity checks.  Accept frames from ANY photodiode: the FPGA cannot
     * fill in the channel on the first sensor a sweep crosses (it needs a
     * second sensor to decode — see stock augmentFramesInWorkspace), so
     * which sensors carry channelFound depends on sweep geometry.  A
     * previous sensor==0-only filter starved one plane of BS1 entirely
     * (bench 2026-07-05: ch1 never paired, t0_diff always one full
     * revolution).  Sensor spacing on the deck (~1–2 cm) is far below our
     * accuracy needs, so mixing sensors is fine. */
    bool padding_ok = ((data[5] | data[8]) & 0xFE) == 0;
    if (!channel_found || !padding_ok) {
        return;
    }
    (void)sensor;

    /* Extract offset (24-bit LE, 6 MHz ticks) → multiply by 4 → 24 MHz ticks */
    uint32_t offset_raw = (uint32_t)data[3]
                        | ((uint32_t)data[4] << 8)
                        | ((uint32_t)data[5] << 16);
    uint32_t offset = offset_raw * 4;

    if (offset == 0) {
        return;   /* no offset on this frame */
    }

    /* Frame timestamp: 24-bit LE, 24 MHz ticks (bytes 9-11) */
    uint32_t timestamp = (uint32_t)data[9]
                       | ((uint32_t)data[10] << 8)
                       | ((uint32_t)data[11] << 16);

    /*
     * Sweep pairing — stock pulse_processor_v2.c principle:
     * timestamp0 = timestamp − offset is the rotor-zero time of the
     * revolution this sweep belongs to (24-bit wraparound arithmetic).
     * The two sweep planes of ONE revolution share timestamp0 to within
     * LH2_MAX_T0_DIFF ticks (stock: 10), while the same plane one
     * revolution later is a full cycle period (~480k ticks) away.
     * Within a matched pair, arrival order gives the beam roles exactly
     * as stock calculateAngles does: earlier frame → firstBeam (+π/3),
     * later frame → secondBeam (−π/3).
     */
    uint32_t t0 = (timestamp - offset) & 0xFFFFFFu;

    static uint32_t g_pair_offset[16];
    static uint32_t g_pair_t0[16];
    static bool     g_pair_valid[16];

    uint32_t fwd     = (t0 - g_pair_t0[channel]) & 0xFFFFFFu;
    uint32_t t0_diff = (fwd <= 0x800000u) ? fwd : (0x1000000u - fwd);

    if (!g_pair_valid[channel] || t0_diff > LH2_MAX_T0_DIFF) {
        /* Only count as a miss if we actually had a pending partner to miss
         * against — the very first sweep ever seen on a channel isn't a
         * failure, just cold start. */
        if (g_pair_valid[channel]) {
            g_ch_pair_miss[channel]++;
        }

        /* Pairing diagnostic (~1 Hz at typical frame rates): the observed
         * t0_diff distribution tells us where the same-revolution cluster
         * actually sits.  Expect a bimodal split: small values (the pair
         * partner) vs ~a full cycle period (~480k, next revolution). */
        static int pair_dbg_div;
        if (g_pair_valid[channel] && ++pair_dbg_div >= 100) {
            pair_dbg_div = 0;
            LOG_INF("pair miss ch=%u t0_diff=%u off0=%u off1=%u",
                    channel, t0_diff, g_pair_offset[channel], offset);
        }

        /* First sweep of a new revolution: stash it and wait for its pair */
        g_pair_offset[channel] = offset;
        g_pair_t0[channel]     = t0;
        g_pair_valid[channel]  = true;
        return;
    }

    /* Same revolution — but is it the OTHER plane, or the same sweep seen
     * by another photodiode?  All four sensors share the rotor-zero time;
     * their offsets differ only by the beam's transit across the deck
     * (≤ ~1 000 ticks), while the two planes sit ~period/3 ≈ 160 000 ticks
     * apart.  Ignore same-sweep duplicates and keep waiting for the
     * genuine partner. */
    uint32_t off_delta = (offset > g_pair_offset[channel])
                         ? offset - g_pair_offset[channel]
                         : g_pair_offset[channel] - offset;
    if (off_delta < LH2_SAME_SWEEP_MAX) {
        return;
    }

    /* Genuine plane pair: complete it (and consume the stored sweep) */
    uint32_t offset0 = g_pair_offset[channel];   /* earlier → firstBeam  */
    uint32_t offset1 = offset;                   /* later   → secondBeam */
    g_pair_valid[channel] = false;

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
    if (!lh2_offsets_to_direction(offset0, offset1,
                                  channel, &g_bs_pose[bs_idx],
                                  g_bs_calib_set[bs_idx] ? &g_bs_calib[bs_idx]
                                                         : NULL,
                                  dir)) {
        return;    /* implausible angles — phantom reflection */
    }

    /*
     * Store direction.  We keep one direction per BS index.
     * If the other BS also has a fresh direction, triangulate.
     */
    static float    g_dir[LH2_BS_COUNT][3];
    static uint32_t g_dir_age[LH2_BS_COUNT];
    static bool     g_dir_valid[LH2_BS_COUNT];

    uint32_t now = k_uptime_get_32();

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
    float miss_m = 0.0f;
    bool ok = lh2_triangulate(g_bs_pose[0].origin, g_dir[0],
                              g_bs_pose[1].origin, g_dir[1],
                              result, &miss_m);
    if (!ok) {
        return;
    }

    /* Reject bad sweep pairs outright: with correct geometry the rays
     * intersect within ~0.1 m (bench 2026-07-05: 0.045–0.09 m); a large
     * miss means at least one beam angle is garbage (flight log showed a
     * miss=0.97 m outlier pair mid-flight that dragged the position).
     * Cheaper and earlier than letting the median/jump-gate fight it. */
    if (miss_m > LH2_MAX_MISS_M) {
        return;
    }

    static int miss_div;
    if (++miss_div >= 64) {
        miss_div = 0;
        /* age0/age1: how old each BS's contributing ray was at this fix, ms.
         * The BS that triggered this triangulation reads ~0; the other can
         * be up to LH2_BLOCK_FRESH_MS old — a fix built from a stale ray
         * paired with a fresh one is a real geometry error (the stale ray
         * assumes the drone hasn't moved since it was measured), not just
         * noise, and this makes that directly visible per-fix instead of
         * inferred from scattered accept/reject counts. */
        LOG_INF("fix (%.2f, %.2f, %.2f) miss=%.3f m age0=%u age1=%u",
                (double)result[0], (double)result[1], (double)result[2],
                (double)miss_m,
                now - g_dir_age[0], now - g_dir_age[1]);
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

    static int64_t g_vel_t_ms;
    static float   g_prev[3];

    /* ── Jump gate ─────────────────────────────────────────────────────────
     * LH2 wrong-geometry solutions (wrong rotor pair / sweep pairing) can
     * survive the median filter for over a second and land meters away from
     * the true position — flight test 2026-07-03 saw a 1.76 m X teleport
     * sustained ~1.7 s that drove position hold into a phantom correction on
     * the ground, and a z=2.8 m burst that tripped the app's climb ceiling.
     * A real drone cannot teleport: reject any fix further than LH2_JUMP_M
     * from the last accepted one.  If LH2_JUMP_REJECT_N *consecutive* fixes
     * agree on the far location, accept it as a genuine re-acquisition and
     * re-seed (velocity restarts from zero).  Alternating good/bad solutions
     * keep resetting the counter, so sustained flip-flopping never re-seeds
     * onto the wrong branch. */
    /* t_ms/dt_ms computed here (moved up from the velocity section below)
     * so the jump gate can scale its distance threshold by actual elapsed
     * time since the last accepted fix, not assume a fixed fix rate. */
    int64_t t_ms  = k_uptime_get();
    int64_t dt_ms = t_ms - g_vel_t_ms;

    static bool  g_jump_seeded;
    static int   g_jump_far_n;
    bool derivative_valid = true;
    if (g_jump_seeded) {
        float jx = mx - g_prev[0];
        float jy = my - g_prev[1];
        float jz = mz - g_prev[2];
        float jump_limit_m = LH2_MAX_SPEED_MPS * (float)dt_ms / 1000.0f;
        if (jump_limit_m < LH2_JUMP_M) { jump_limit_m = LH2_JUMP_M; }
        if (jx * jx + jy * jy + jz * jz > jump_limit_m * jump_limit_m) {
            if (++g_jump_far_n < LH2_JUMP_REJECT_N) {
                return;                      /* discard outlier fix */
            }
            /* Re-seed: the position step is a teleport, not motion — the
             * derivative would read tens of m/s.  Skip it, but keep the
             * (decaying) previous velocity rather than zeroing: if this was
             * a genuine re-acquisition after a gap, the drone's real
             * velocity didn't reset just because tracking did. */
            derivative_valid = false;
        }
        g_jump_far_n = 0;
    } else {
        g_jump_seeded    = true;
        derivative_valid = false;
    }

    /* Velocity from a long-baseline finite difference over the accepted-fix
     * history (see LH2_VEL_BASELINE_MS block comment).  Feeds the
     * stabilizer's position-hold damping term — a P-only position loop has no
     * damping and limit-cycles (stock runs a full velocity PID between its
     * position and attitude loops for the same reason). */
    float vx = g_vel.x, vy = g_vel.y, vz = g_vel.z;

    static float   g_vhist[LH2_VEL_BUF_N][3];
    static int64_t g_vhist_t[LH2_VEL_BUF_N];
    static int     g_vhist_head;    /* next write slot */
    static int     g_vhist_n;

    if (!derivative_valid) {
        /* Re-seed/first fix: history belongs to the old solution branch —
         * a difference across the teleport would read tens of m/s. */
        g_vhist_n = 0;
    }

    /* Most recent history sample at least BASELINE_MS old; fall back to
     * the oldest entry during startup if it spans MIN_BASELINE_MS. */
    const float *old_m = NULL;
    int64_t      old_t = 0;
    for (int i = 1; i <= g_vhist_n; i++) {
        int idx = (g_vhist_head - i + LH2_VEL_BUF_N) % LH2_VEL_BUF_N;
        if (t_ms - g_vhist_t[idx] >= LH2_VEL_BASELINE_MS) {
            old_m = g_vhist[idx];
            old_t = g_vhist_t[idx];
            break;
        }
    }
    if (old_m == NULL && g_vhist_n > 0) {
        int idx = (g_vhist_head - g_vhist_n + LH2_VEL_BUF_N) % LH2_VEL_BUF_N;
        if (t_ms - g_vhist_t[idx] >= LH2_VEL_MIN_BASELINE_MS) {
            old_m = g_vhist[idx];
            old_t = g_vhist_t[idx];
        }
    }

    if (old_m != NULL && (t_ms - old_t) <= LH2_VEL_MAX_BASELINE_MS) {
        float inv_dt = 1000.0f / (float)(t_ms - old_t);
        float a = LH2_VEL_LPF_ALPHA;
        vx += a * ((mx - old_m[0]) * inv_dt - vx);
        vy += a * ((my - old_m[1]) * inv_dt - vy);
        vz += a * ((mz - old_m[2]) * inv_dt - vz);
    } else {
        if (old_m != NULL) {
            /* Nearest usable sample predates a fix outage — restart the
             * history rather than difference across the gap. */
            g_vhist_n = 0;
        }
        /* No usable baseline: decay the estimate by elapsed time instead
         * of zeroing it — a hard zero here is what kept stripping the
         * position loop's damping mid-flight. */
        float k = exp2f(-(float)dt_ms / LH2_VEL_DECAY_HALF_MS);
        vx *= k; vy *= k; vz *= k;
    }

    /* Push this fix into the history ring */
    g_vhist[g_vhist_head][0] = mx;
    g_vhist[g_vhist_head][1] = my;
    g_vhist[g_vhist_head][2] = mz;
    g_vhist_t[g_vhist_head]  = t_ms;
    g_vhist_head = (g_vhist_head + 1) % LH2_VEL_BUF_N;
    if (g_vhist_n < LH2_VEL_BUF_N) { g_vhist_n++; }

    g_prev[0] = mx; g_prev[1] = my; g_prev[2] = mz;   /* jump-gate reference */
    g_vel_t_ms = t_ms;

    k_spinlock_key_t key = k_spin_lock(&g_pos_lock);
    g_pos.x     = mx;
    g_pos.y     = my;
    g_pos.z     = mz;
    g_vel.x     = vx;
    g_vel.y     = vy;
    g_vel.z     = vz;
    g_pos_valid = true;
    g_pos_ms    = k_uptime_get_32();
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

void cf21bl_lighthouse_set_bs_calib(int id, const lh2_bs_calib_t *calib)
{
    if (id < 0 || id >= LH2_BS_COUNT) {
        return;
    }
    g_bs_calib[id]     = *calib;
    g_bs_calib_set[id] = true;
    LOG_INF("LH2 BS%d OOTX calib set (uid=%u, tilt %+0.4f/%+0.4f rad)",
            id, calib->uid,
            (double)calib->sweep[0].tilt, (double)calib->sweep[1].tilt);
}

void cf21bl_lighthouse_set_bs_channel(int bs_id, uint8_t channel)
{
    if (bs_id < 0 || bs_id >= LH2_BS_COUNT || channel > 15) {
        return;
    }
    g_bs_channel[bs_id] = channel;
}

/* Fresh == we have ever had a fix AND the last one is younger than
 * LH2_POS_STALE_MS (see that constant: a frozen-but-"valid" position is
 * what crashed flight 24).  Caller must hold g_pos_lock. */
static bool pos_fresh_locked(void)
{
    if (!g_pos_valid) {
        return false;
    }
    return (k_uptime_get_32() - g_pos_ms) < LH2_POS_STALE_MS;
}

int cf21bl_lighthouse_get_position(lh2_position_t *pos)
{
    k_spinlock_key_t key = k_spin_lock(&g_pos_lock);
    bool valid = pos_fresh_locked();
    if (valid) {
        *pos = g_pos;
    }
    k_spin_unlock(&g_pos_lock, key);
    return valid ? 0 : -EAGAIN;
}

int cf21bl_lighthouse_get_velocity(lh2_position_t *vel)
{
    k_spinlock_key_t key = k_spin_lock(&g_pos_lock);
    bool valid = pos_fresh_locked();
    if (valid) {
        *vel = g_vel;
    }
    k_spin_unlock(&g_pos_lock, key);
    return valid ? 0 : -EAGAIN;
}

bool cf21bl_lighthouse_is_valid(void)
{
    k_spinlock_key_t key = k_spin_lock(&g_pos_lock);
    bool valid = pos_fresh_locked();
    k_spin_unlock(&g_pos_lock, key);
    return valid;
}

uint32_t cf21bl_lighthouse_fix_age_ms(void)
{
    k_spinlock_key_t key = k_spin_lock(&g_pos_lock);
    uint32_t age = g_pos_valid ? (k_uptime_get_32() - g_pos_ms) : UINT32_MAX;
    k_spin_unlock(&g_pos_lock, key);
    return age;
}
