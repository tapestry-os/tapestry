/*
 * cf21bl_lighthouse.h — Lighthouse v2 positioning for the Crazyflie 2.1 Brushless
 *
 * Reads 12-byte LH2 sweep frames from the deck FPGA over USART3 (PC10/PC11,
 * 230400 baud — see cf21bl_lighthouse.c for the frame format), converts
 * timing offsets to sweep angles using exact LH2 geometry, and triangulates
 * a 3D position from two SteamVR 2.0 base stations.
 *
 * Enabling
 * --------
 * Add to your application's prj.conf:
 *   CONFIG_CF21BL_LIGHTHOUSE=y
 *   CONFIG_UART_INTERRUPT_DRIVEN=y
 * plus the per-example overlay that sets USART3 to 230400 baud and moves the
 * Zephyr console (see the CF21BL_LIGHTHOUSE Kconfig help).
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
 * Optionally call cf21bl_lighthouse_set_bs_calib() with the per-sweep OOTX
 * calibration each base station broadcasts (cfclient saves it in the same
 * YAML as the geometry, under "calibs:").  Without it the sweep planes are
 * treated as geometrically ideal, which leaves a systematic ray error
 * (observed as a 5–9 cm triangulation miss at the calibration origin,
 * growing with distance from it).  With it, measured sweep angles are
 * corrected through the stock measurement model before triangulation.
 *
 * Base station pose convention (right-handed, Z-up world frame):
 *   origin[3]: position of the base station optical center in meters
 *   rot[9]:    row-major 3×3 rotation matrix mapping BS-local to world frame
 *
 *   BS-local frame axes (stock lighthouse_geometry.c convention — the frame
 *   cfclient's rotation matrices are estimated for):
 *     X = forward (out of the face, toward the flying space),
 *     Y = +azimuth (left),   Z = up
 *   At azimuth θ=0, elevation β=0 the direction vector is (1,0,0) in BS-local.
 *   The cfclient YAML "rotation" rows map directly onto rot[0..8] row-major;
 *   no transpose.
 *
 * Position frame
 * --------------
 * Reported positions are in the world frame defined by the base station
 * calibration — typically the SteamVR chaperone origin (floor level, roughly
 * centered in the tracking volume).
 *
 * Frame transport
 * ---------------
 * The deck FPGA streams 12-byte frames continuously over the deck UART; see
 * the frame-format table at the top of cf21bl_lighthouse.c (byte layout,
 * sync frames, channel/slow-bit/sensor packing, offset and timestamp fields).
 *
 * LH2 geometry (exact, no small-angle approximation)
 * ---------------------------------------------------
 * Each base station emits two counter-tilted (±30°) sweep planes per
 * revolution.  For beam angles θ₀, θ₁ (rad, sync-relative, ±π/3 offsets
 * already applied — stock pulseProcessorV2ConvertToV1Angles):
 *
 *   azimuth   α = (θ₀ + θ₁) / 2
 *   elevation β = atan2( sin(θ₁ − θ₀), tan(π/6) × (cos θ₀ + cos θ₁) )
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
 * origin[0..2] in meters.
 */
typedef struct {
    float origin[3];   /* [x, y, z] BS optical center, meters, world frame   */
    float rot[9];      /* row-major R: rot[r*3+c] = R_world←BS[r][c]         */
} lh2_bs_pose_t;

/*
 * lh2_bs_calib_sweep_t — OOTX factory calibration for ONE sweep plane
 * (each base station broadcasts one set per plane).  Values come from the
 * cfclient YAML "calibs:" section; units are radians (angles) and unitless
 * distortion coefficients, exactly as broadcast.
 *
 * Only tilt, phase, gibphase and gibmag are used by the LH2 correction
 * model (same as stock lighthouse_calibration.c, where curve/ogee handling
 * is an open TODO).  curve/ogee* are stored so the data survives a future
 * model upgrade without re-entering it.
 */
typedef struct {
    float phase;
    float tilt;
    float curve;
    float gibphase;
    float gibmag;
    float ogeephase;
    float ogeemag;
} lh2_bs_calib_sweep_t;

/* Full per-base-station calibration: sweep[0] = first beam (t = −30°),
 * sweep[1] = second beam (t = +30°) — same order as the YAML lists them. */
typedef struct {
    lh2_bs_calib_sweep_t sweep[2];
    uint32_t             uid;      /* OOTX id from the YAML; informational */
} lh2_bs_calib_t;

/* 3D position estimate in the world frame, meters */
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
 * cf21bl_lighthouse_set_bs_calib — Load OOTX sweep calibration for base
 * station id (0 or 1, same index as set_bs_pose).  Optional: without it,
 * raw sweep angles are used uncorrected.  Call any time; takes effect on
 * the next processed sweep pair.
 */
void cf21bl_lighthouse_set_bs_calib(int id, const lh2_bs_calib_t *calib);

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
 * cf21bl_lighthouse_get_velocity — Copy the latest velocity estimate (m/s,
 * world frame; low-passed derivative of the median-filtered position) into
 * *vel.  Same validity condition as get_position; returns -EAGAIN when no
 * fix.  Used by the stabilizer's position-hold damping term.
 */
int cf21bl_lighthouse_get_velocity(lh2_position_t *vel);

/*
 * cf21bl_lighthouse_is_valid — True when get_position() would succeed.
 * Lock-free; safe to poll from the stabilizer thread.
 */
bool cf21bl_lighthouse_is_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CF21BL_LIGHTHOUSE_H */
