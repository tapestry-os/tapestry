/*
 * cf21bl_lighthouse.h — Lighthouse v2 positioning for the Crazyflie 2.1 Brushless
 *
 * Reads LH2 sweep timing frames from the deck FPGA via SPI2, converts timing
 * offsets to sweep angles using exact LH2 geometry, and triangulates a 3D
 * position from two SteamVR 2.0 base stations.
 *
 * Enabling
 * --------
 * Add to your application's boards/crazyflie21bl.conf:
 *   CONFIG_CF21BL_LIGHTHOUSE=y
 *   CONFIG_SPI=y
 *
 * Add to CMakeLists.txt inside the if(CONFIG_PWM) block:
 *   if(CONFIG_CF21BL_LIGHTHOUSE)
 *     target_sources(app PRIVATE
 *       ${TAPESTRY_OS_BOARDS}/crazyflie21bl/cf21bl_lighthouse.c)
 *   endif()
 *
 * Calibration
 * -----------
 * Call cf21bl_lighthouse_set_bs_pose() for both base stations (ID 0 and 1)
 * before expecting valid positions.  Poses come from the Bitcraze geometry
 * estimation tool (cfclient → Lighthouse → Manage geometry → Estimate).
 *
 * Base station pose convention (right-handed, Z-up world frame):
 *   origin[3]: position of the base station optical centre in metres
 *   rot[9]:    row-major 3×3 rotation matrix mapping BS-local to world frame
 *
 *   BS-local frame axes (looking straight out of the BS face):
 *     X = right,   Y = up,   Z = forward (toward the flying space)
 *   At azimuth θ=0, elevation β=0 the direction vector is (0,0,1) in BS-local.
 *
 * Position frame
 * --------------
 * Reported positions are in the world frame defined by the base station
 * calibration — typically the SteamVR chaperone origin (floor level, roughly
 * centred in the tracking volume).
 *
 * SPI frame format (verify against Bitcraze lhDriver.c / lighthouse2-fpga repo)
 * -------------------------------------------------------------------------------
 * The FPGA asserts irq-gpios when a frame is available.  The STM32 reads one
 * SPI frame per IRQ assertion.  Frame layout (all little-endian):
 *
 *   byte 0        count of events in this frame (0–LH2_MAX_EVENTS)
 *   per event (LH2_BYTES_PER_EVENT = 7 bytes each):
 *     byte 0      bits[7:6]=bs_id (0 or 1), bits[5:4]=sensor (0–3),
 *                 bit[3]=rotor (0 or 1), bit[2]=is_sync, bits[1:0]=reserved
 *     bytes 1–4   timestamp (uint32 LE, 24 MHz ticks, 41.67 ns/tick)
 *     bytes 5–6   pulse width (uint16 LE, 24 MHz ticks)
 *
 * Sync events (is_sync=1) mark the start of a rotor revolution.
 * Sweep events (is_sync=0) carry the measurement timing used for positioning.
 *
 * LH2 geometry (exact, no small-angle approximation)
 * ---------------------------------------------------
 * Each base station emits two counter-tilted (±LH2_TILT_DEG = ±15°) sweep
 * planes per revolution.  For rotor angles θ₀, θ₁ (rad, sync-relative):
 *
 *   azimuth   α = (θ₀ + θ₁) / 2
 *   elevation β = atan( sin((θ₀ − θ₁)/2) / tan(LH2_TILT_DEG × π/180) )
 *
 * Direction in BS-local frame:  d = (sin(α)·cos(β), sin(β), cos(α)·cos(β))
 * Direction in world frame:     d_w = R_bs × d
 * Ray from BS i:                P_bs_i + t × d_w_i
 *
 * 3D position is the midpoint of the closest approach between the two rays
 * (least-squares, since real measurements add noise and the rays are skew).
 */

#ifndef TAPESTRY_CF21BL_LIGHTHOUSE_H
#define TAPESTRY_CF21BL_LIGHTHOUSE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Types ────────────────────────────────────────────────────────────────── */

/*
 * lh2_bs_pose_t — calibrated pose of one base station in the world frame.
 *
 * Obtain from the Bitcraze cfclient geometry estimator or the Python SDK:
 *   python3 -m cflib.crazyflie.mem.lighthouseConfigWriter --estimate-geometry
 *
 * The JSON output "rotation_matrix" is rot[0..8] row-major, and "origin" is
 * origin[0..2] in metres.
 */
typedef struct {
    float origin[3];   /* [x, y, z] BS optical centre, metres, world frame   */
    float rot[9];      /* row-major R: rot[r*3+c] = R_world←BS[r][c]         */
} lh2_bs_pose_t;

/* 3D position estimate in the world frame, metres */
typedef struct {
    float x, y, z;
} lh2_position_t;

/* ── API ──────────────────────────────────────────────────────────────────── */

/*
 * cf21bl_lighthouse_init — Open the SPI device, register the FPGA IRQ GPIO,
 * and launch the SPI reader thread.  Returns 0 on success, negative errno if
 * the SPI device or IRQ GPIO is not ready.
 *
 * Call once at startup before cf21bl_init() arms the ESCs, so the first
 * pose packets arrive before the drone lifts off.
 */
int cf21bl_lighthouse_init(void);

/*
 * cf21bl_lighthouse_set_bs_pose — Load calibrated pose for base station id
 * (0 or 1).  Thread-safe; may be called at any time after init.  Must be
 * called for both stations before cf21bl_lighthouse_is_valid() returns true.
 */
void cf21bl_lighthouse_set_bs_pose(int id, const lh2_bs_pose_t *pose);

/*
 * cf21bl_lighthouse_set_bs_channel — Map a SteamVR channel number to a BS pose
 * index.  The channel in UART frames is 0-indexed (SteamVR channel − 1).
 * Default mapping: BS0 → channel 0, BS1 → channel 1.
 * Call this if your SteamVR room uses different channel assignments.
 */
void cf21bl_lighthouse_set_bs_channel(int bs_id, uint8_t channel);

/*
 * cf21bl_lighthouse_get_position — Copy the latest position estimate into *pos.
 * Returns 0 when valid (both base stations seen within LH2_VALID_MS ms).
 * Returns -EAGAIN when no fix.
 */
int cf21bl_lighthouse_get_position(lh2_position_t *pos);

/*
 * cf21bl_lighthouse_is_valid — True when get_position() would succeed.
 * Lock-free; safe to poll from the stabilizer thread.
 */
bool cf21bl_lighthouse_is_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CF21BL_LIGHTHOUSE_H */
