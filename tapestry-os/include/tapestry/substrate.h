/*
 * tapestry/substrate.h — Tapestry L1 Physical Substrate Interface (PSI)
 *
 * Hardware Abstraction Layer over the physical substrate an agent inhabits.
 * Application code and upper layers include only this header; the concrete
 * implementation is selected by the build system.
 *
 * Add support for a new substrate:
 *   1. Create tapestry-os/boards/<your_board>/substrate_<name>.c
 *   2. Implement every function declared below.
 *   3. In your app's CMakeLists.txt, compile that file for your board config.
 *      Use boards/substrate_null.c as the fallback / simulation target.
 *
 * Design invariants:
 *   - No OS-specific or Zephyr types cross this boundary. Pure C99 + stdint.h.
 *   - Motion is expressed in body frame, normalized [-1.0, 1.0].
 *   - Unimplemented primitives (bond, release, emit, sense, set_power) are
 *     no-ops or return a negative value; callers must tolerate this gracefully.
 *   - No dependency on any other Tapestry layer header.
 */

#ifndef TAPESTRY_SUBSTRATE_H
#define TAPESTRY_SUBSTRATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Geometric primitives ────────────────────────────────────────────────── */

typedef struct { float x, y, z;       } substrate_vec3_t;
typedef struct { float w, x, y, z;    } substrate_quat_t;  /* unit quaternion */

/*
 * substrate_twist_t — 6-DOF body-frame motion command.
 *
 * All components are normalized to [-1.0, 1.0]:
 *   linear.x   forward (+) / backward (-)
 *   linear.y   left    (+) / right    (-)   (holonomic platforms only)
 *   linear.z   up      (+) / down     (-)   (flying / swimming agents)
 *   angular.x  roll  rate (positive = right-side-down by right-hand rule)
 *   angular.y  pitch rate (positive = nose-up)
 *   angular.z  yaw   rate (positive = counterclockwise / turn left)
 *
 * Implementations read only the axes they can act on; unused axes are ignored.
 * Differential drive platforms use linear.x and angular.z exclusively.
 *
 * substrate_quat_t is available for sensor output (orientation reporting) and
 * future pose-command extensions; it is not used in substrate_move today.
 */
typedef struct {
    substrate_vec3_t linear;
    substrate_vec3_t angular;
} substrate_twist_t;

/* ── Power domain ────────────────────────────────────────────────────────── */

typedef enum {
    SUBSTRATE_POWER_ACTIVE  = 0,   /* full sensing, actuation, and communication */
    SUBSTRATE_POWER_IDLE    = 1,   /* communication only; actuation paused       */
    SUBSTRATE_POWER_SLEEP   = 2,   /* deep sleep; wakes on timer or interrupt    */
    SUBSTRATE_POWER_HARVEST = 3,   /* energy harvesting; minimal activity        */
} substrate_power_state_t;

/* ── Signal output ───────────────────────────────────────────────────────── */

/*
 * Semantic signal states for agent status output.
 * The physical form (LED color, acoustic tone, chemical marker) is
 * implementation-defined; the meaning is substrate-neutral.
 */
typedef enum {
    SUBSTRATE_SIGNAL_NONE     = 0,   /* no output (off / silent)          */
    SUBSTRATE_SIGNAL_IDLE     = 1,   /* agent present, no active goal      */
    SUBSTRATE_SIGNAL_ACTIVE   = 2,   /* goal in progress, quorum healthy   */
    SUBSTRATE_SIGNAL_DEGRADED = 3,   /* goal in progress, quorum reduced   */
    SUBSTRATE_SIGNAL_FAILED   = 4,   /* quorum lost or goal unachievable   */
} substrate_signal_t;

/* ── Sensor type ─────────────────────────────────────────────────────────── */

typedef enum {
    SUBSTRATE_SENSOR_PROXIMITY = 0,  /* normalized [0=touching, 1=max range clear] */
    SUBSTRATE_SENSOR_BATTERY   = 1,  /* normalized [0=empty, 1=full]               */
} substrate_sensor_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/*
 * substrate_init — Initialise the substrate hardware.
 * Must be called once before any other substrate_* function.
 * Returns 0 on success, negative errno if hardware is unreachable.
 * All other substrate calls are no-ops when init returns non-zero.
 */
int substrate_init(void);

/*
 * substrate_move — Command a 6-DOF body-frame motion.
 * Components are normalized [-1.0, 1.0]; see substrate_twist_t for axis
 * conventions. Implementations clamp to their physical limits.
 * Passing a zero-initialised twist stops all motion.
 */
void substrate_move(const substrate_twist_t *twist);

/*
 * substrate_set_signal — Set the agent's status output.
 * The physical representation (LED, tone, chemical) is implementation-defined.
 */
void substrate_set_signal(substrate_signal_t signal);

/*
 * substrate_set_power — Transition the substrate to a power state.
 * No-op on substrates without power management support.
 */
void substrate_set_power(substrate_power_state_t state);

/*
 * substrate_sense — Read a sensor channel into *out (normalized [0.0, 1.0]).
 * Returns 0 on success, negative value if the sensor is unsupported or
 * unavailable on this substrate.
 */
int substrate_sense(substrate_sensor_t type, float *out);

/*
 * substrate_bond    — Actuate a bonding mechanism (e.g. electrostatic, magnetic).
 * substrate_release — Release a previously formed bond.
 * substrate_emit    — Emit energy or matter (photons, molecules, acoustic pulse).
 *
 * No-op stubs on all current substrates; defined for future nanoscale agents.
 */
void substrate_bond(void);
void substrate_release(void);
void substrate_emit(void);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_SUBSTRATE_H */
