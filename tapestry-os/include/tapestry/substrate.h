/*
 * tapestry/substrate.h — Tapestry L1 Physical Substrate Interface (PSI)
 *
 * Hardware Abstraction Layer over the physical substrate an element inhabits.
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

/* ── Geometric primitives ────────────────────────────────────────────────── */

typedef struct { float x, y, z;       } substrate_vec3_t;
typedef struct { float w, x, y, z;    } substrate_quat_t;  /* unit quaternion */

/*
 * substrate_twist_t — 6-DOF body-frame motion command.
 *
 * All components are normalized to [-1.0, 1.0]:
 *   linear.x   forward (+) / backward (-)
 *   linear.y   left    (+) / right    (-)   (holonomic platforms only)
 *   linear.z   up      (+) / down     (-)   (flying / swimming elements)
 *   angular.x  roll  rate (positive = right-side-down by right-hand rule)
 *   angular.y  pitch rate (positive = nose-up)
 *   angular.z  yaw   rate (positive = counterclockwise / turn left)
 *
 * Implementations read only the axes they can act on; unused axes are ignored.
 * Differential drive platforms use linear.x and angular.z exclusively.
 *
 * substrate_quat_t is available for sensor output (orientation reporting) and
 * future pose-command extensions; it is not used in substrate_move today.
 * When sensor output does get reported as one (e.g. into csm.h's
 * element_state_t.orientation for gossip), it must be expressed in the
 * platform's world frame, not this device's local/body frame or wherever
 * it happened to be pointed at boot — see orientation_t's comment in
 * csm.h for the full reference-frame convention this feeds into.
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
 * Semantic signal states for element status output.
 * The physical form (LED color, acoustic tone, chemical marker) is
 * implementation-defined; the meaning is substrate-neutral.
 */
typedef enum {
    SUBSTRATE_SIGNAL_NONE     = 0,   /* no output (off / silent)           */
    SUBSTRATE_SIGNAL_IDLE     = 1,   /* element present, no active goal    */
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
 * substrate_init — Initialize the substrate hardware.
 * Must be called once before any other substrate_* function.
 * Returns 0 on success, negative errno if hardware is unreachable.
 * All other substrate calls are no-ops when init returns non-zero.
 */
int substrate_init(void);

/*
 * substrate_move — Command a 6-DOF body-frame motion.
 * Components are normalized [-1.0, 1.0]; see substrate_twist_t for axis
 * conventions. Implementations clamp to their physical limits.
 * Passing a zero-initialized twist stops all motion.
 */
void substrate_move(const substrate_twist_t *twist);

/*
 * substrate_set_signal — Set the element's status output.
 * The physical representation (LED, tone, chemical) is implementation-defined.
 */
void substrate_set_signal(substrate_signal_t signal);

/*
 * substrate_set_power — Transition the substrate to a power state.
 * No-op on substrates without power management support.
 */
void substrate_set_power(substrate_power_state_t state);

/*
 * substrate_identify — Physically announce this element's ordinal so a
 * human can tell WHICH element they are looking at, before any auto-
 * negotiated identity (element_id from transport_negotiate_id(), a slot
 * index, etc.) is otherwise visible to the naked eye.  Intended to be
 * called once, right after that identity is known and before motion
 * starts — e.g. so a person can then physically place/orient the element
 * correctly for a formation that assumes a specific id-to-position
 * mapping (see cutebot-formation's compute_start_pos()), instead of
 * guessing or pre-labelling hardware via a one-time serial connection.
 *
 * `ordinal` is caller-defined (typically element_id) — substrate has no
 * opinion on what it means, only that it should be rendered as something
 * a person can count or read off without instrumentation.
 *
 * No-op on substrates with no addressable indicator (see substrate_bond()
 * etc. for the same "no-op stub, defined for substrates that can't do
 * this" pattern) or where a physical announcement makes no sense (e.g. a
 * simulated element with no viewport to look at).
 */
void substrate_identify(uint8_t ordinal);

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
 * No-op stubs on all current substrates; defined for future nanoscale elements.
 */
void substrate_bond(void);
void substrate_release(void);
void substrate_emit(void);

#endif /* TAPESTRY_SUBSTRATE_H */
