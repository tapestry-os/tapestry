/*
 * cutebot_pro.h — ELECFREAKS Cutebot Pro (V2.1.0) driver
 *
 * NOT the same protocol as cutebot.h's Cutebot Mini, despite sharing the
 * same I2C address (0x10) and bus (&cutebot_i2c, bbc_microbit_v2.overlay —
 * no devicetree changes needed for Pro). Verified against ELECFREAKS' own
 * elecfreaks/pxt-Cutebot-Pro firmware source (v2.ts/main.ts), V2-generation
 * protocol only (V1 hardware uses a different, non-framed protocol — out of
 * scope; see Kconfig's CUTEBOT_VARIANT_PRO help for the version note).
 *
 * All commands share one frame: [0xFF][0xF9][CMD][PARAM_LEN][params...].
 *
 * Motor: cmd 0x80 (pidSpeedControl) is a CONTINUOUS, non-blocking
 * closed-loop wheel-speed target in cm/s, driven by the controller's own
 * firmware PID — safe to reissue every control-loop tick, unlike cmd
 * 0x81/0x83 (pidRunDistance/pidRunAngle), which block for seconds via an
 * internal firmware-side pause and are NOT implemented here (wrong shape
 * for this codebase's 100 ms substrate_move() tick). Nonzero speed is
 * firmware-clamped to [20, 50] cm/s per wheel; 0 is a real stop.
 *
 * Encoders: cmd 0xA0 sub 1/2 read measured per-wheel speed (cm/s);
 * sub 3/4 read cumulative per-wheel distance (Int32LE). The physical unit
 * of that distance value is UNVERIFIED — ELECFREAKS' own source comments
 * it "rotation degrees" but returns a raw 4-byte value with no documented
 * scale factor anywhere (official firmware source, an independent
 * V2.0.2/V2.1.0-specific community driver, and the vendor wiki were all
 * checked). Treat cutebot_pro_encoders_t.left_distance/right_distance as
 * an opaque, monotonic-within-a-run raw count until DEMO_MODE_ENCODER_BENCH
 * (Kconfig) pins down the real scale against a physically measured
 * distance.
 *
 * 4-way line sensor: cmd 0x60 sub 0x00 reads one I2C-polled byte, bit0..
 * bit3 = channel 1..4 (1 = line/dark). Unlike Mini's cutebot_line.c, this
 * is genuinely I2C-only — there is no GPIO/interrupt path for this sensor,
 * so cutebot_pro_read_line_state() must be polled (no crossing-timing
 * signal is available the way cutebot_line.c's entry-edge timestamps are).
 */

#ifndef TAPESTRY_CUTEBOT_PRO_H
#define TAPESTRY_CUTEBOT_PRO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One encoder read. speed is signed cm/s (matches the commanded direction);
 * distance is the raw, unit-unverified cumulative count (see header doc). */
typedef struct {
    float   left_speed_cms;
    float   right_speed_cms;
    int32_t left_distance;
    int32_t right_distance;
} cutebot_pro_encoders_t;

/* Obtain the I2C device handle and verify the Cutebot Pro is accessible.
 * Returns 0 on success, negative errno if the I2C bus or device is missing. */
int cutebot_pro_init(void);

/*
 * Command a continuous closed-loop wheel speed via the firmware's own PID
 * (cmd 0x80). Each of left_cms/right_cms is independently: 0 = stop, or
 * clamped in magnitude to the firmware's legal [20, 50] cm/s nonzero range
 * (a caller wanting less than 20 cm/s gets 20 cm/s, not a proportionally
 * weaker command — there is no way to ask this firmware for less; see
 * substrate_cutebot_pro.c for how the shared force/twist control law's
 * near-zero residual is handled at this boundary).
 */
void cutebot_pro_drive_cms(float left_cms, float right_cms);

/* Set both RGB headlights to the given color (cmd 0x20, LIGHT=2/both). */
void cutebot_pro_set_leds(uint8_t r, uint8_t g, uint8_t b);

/* Read both wheels' measured speed and cumulative distance (cmd 0xA0). */
void cutebot_pro_read_encoders(cutebot_pro_encoders_t *out);

/* Clear both wheels' cumulative distance counters (cmd 0x50). */
void cutebot_pro_clear_encoders(void);

/*
 * Read the 4-way line sensor's raw state byte (cmd 0x60/0x00): bit0..bit3
 * = channel 1..4, 1 = line/dark. Value 15 = crossroads (all 4), any 3-bit
 * value (7/11/13/14) = a T-junction with the zero bit marking the open
 * side. Polled, not interrupt-driven — see this header's top comment.
 * Returns 0 on success, negative errno if the I2C transaction failed (out
 * left unmodified).
 */
int cutebot_pro_read_line_state(uint8_t *state);

/*
 * Read one channel's raw reflectance (cmd 0x60/0x02, 0-255, no inversion —
 * NOT the pre-thresholded bit cutebot_pro_read_line_state() returns).
 * channel is 1-4, matching that function's ch1..ch4 numbering (converted
 * to the wire's 0-indexed convention internally — see
 * elecfreaks/pxt-Cutebot-Pro's own trackbitgetGray(channel - 1) call).
 *
 * Added 2026-09-22 to diagnose a real hardware finding: ch1/ch4 read
 * "dark" (bit=1) via cutebot_pro_read_line_state() on a genuinely bare
 * table, every poll, regardless of surface — this reads the underlying
 * analog value so a near-0 reading (genuinely dark to the sensor) can be
 * told apart from a borderline value sitting right at whatever fixed
 * threshold the board's own firmware applies internally (no calibration
 * command exists in the protocol — checked; the digital state is however
 * the firmware itself thresholds this raw value, not something tunable
 * from here).
 *
 * Returns 0 on success, negative errno if the I2C transaction failed (out
 * left unmodified).
 */
int cutebot_pro_read_line_gray(uint8_t channel, uint8_t *gray);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CUTEBOT_PRO_H */
