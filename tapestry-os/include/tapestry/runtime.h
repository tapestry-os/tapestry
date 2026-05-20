/*
 * tapestry/runtime.h — Tapestry L2 Element Runtime
 *
 * Owns the per-element main-loop cadence and power state machine for the
 * full L4+L5+L6 stack.  Consuming applications:
 *
 *   1. tapestry_runtime_init(&cfg)   — init transport, substrate, WM, SCR, choreo
 *   2. choreo_submit_goal(&goal)     — optional: set L7 goal (from <tapestry/choreo.h>)
 *   3. while (true) {
 *          tapestry_runtime_update_pos(x, y);  — optional: feed odometry back in
 *          tapestry_runtime_tick();             — drain, age, elect, BSE, gossip, telemetry
 *          // read state, drive substrate, sleep
 *      }
 *
 * Not for use by demos that bypass L5/L6 (e.g. collective-formation, which
 * implements its own control law directly on L4).
 *
 * Design invariants:
 *   - No OS-specific types in this header.  Pure C99 + stdint.
 *   - Power transitions route through substrate_set_power() (L1 HAL); the
 *     L2 layer owns the state machine but never calls Zephyr PM directly.
 *   - tapestry_runtime_tick() does not call substrate_move(); that remains
 *     the application's responsibility after reading tapestry_runtime_scr().
 */

#ifndef TAPESTRY_RUNTIME_H
#define TAPESTRY_RUNTIME_H

#include <stdint.h>
#include <tapestry/substrate.h>   /* substrate_power_state_t                  */
#include <tapestry/scr.h>         /* scr_state_t, world_model_t, element_id_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ───────────────────────────────────────────────────────── */

typedef struct {
    element_id_t     self_id;           /* Unique element ID for this node          */
    float            pos_x;             /* Initial logical X position [0, 100]      */
    float            pos_y;             /* Initial logical Y position [0, 100]      */
    float            consistency_bias;  /* L4 AP/CP dial: 0.0 = pure AP, 1.0 = CP  */
    uint8_t          quorum_min;        /* SCR: minimum fresh peers for DEGRADED    */
    uint8_t          quorum_target;     /* SCR: minimum fresh peers for HEALTHY     */
    scr_capability_t capabilities;      /* SCR: SCR_CAP_* flags for this element    */
} tapestry_runtime_config_t;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

/*
 * tapestry_runtime_init — Initialise all runtime subsystems.
 *
 * Calls substrate_init(), transport_init(), wm_init(), scr_init(), and
 * choreo_init() (L7) in order.  substrate_init() failure is non-fatal
 * (logged as warning).  transport_init() failure returns -1 immediately.
 *
 * Returns 0 on success, -1 if a fatal subsystem fails to start.
 */
int tapestry_runtime_init(const tapestry_runtime_config_t *cfg);

/*
 * tapestry_runtime_tick — Drive one full runtime cycle.
 *
 * Executes in order:
 *   1. transport_drain()       — receive gossip from all transports
 *   2. wm_tick()               — age L4 entries, recompute consistency
 *   3. wm_update_self()        — refresh own entry so L6 sees current position
 *   4. scr_tick()              — recompute role and quorum; on completion,
 *                                invokes choreo_tick() via scr_state_t::on_tick
 *                                (L5 post-tick hook → L6 directive synthesis)
 *   5. transport_send()        — broadcast gossip (throttled to GOSSIP_INTERVAL_MS)
 *   6. transport_send_telemetry() — emit metric frames
 *   7. tapestry_power_tick()   — run auto power-stepping policy (if enabled)
 *
 * Does not call substrate_move() or substrate_set_signal(); those are the
 * application's responsibility based on tapestry_runtime_scr().
 *
 * Must be called from a single thread.  Not reentrant.
 */
void tapestry_runtime_tick(void);

/* ── State accessors ─────────────────────────────────────────────────────── */

/*
 * tapestry_runtime_wm  — Return the live L4 world model (read-only).
 * tapestry_runtime_scr — Return the live L5 SCR state (read-only).
 *
 * Valid after tapestry_runtime_init().  Pointers are stable for the lifetime
 * of the process; contents change each tapestry_runtime_tick().  Do not hold
 * references across ticks.
 */
const world_model_t *tapestry_runtime_wm(void);
const scr_state_t   *tapestry_runtime_scr(void);

/*
 * tapestry_runtime_update_pos — Feed the element's current position back in.
 *
 * Call before tapestry_runtime_tick() each cycle with dead-reckoning or
 * sensor-fusion output.  The value is written into s_own and picked up by
 * wm_update_self() inside the tick, so choreo_tick() sees a fresh position.
 *
 * x and y must be in the logical world coordinate space [0, 100].
 * No-op until tapestry_runtime_init() has been called.
 */
void tapestry_runtime_update_pos(float x, float y);

/* ── Power management ────────────────────────────────────────────────────── */

/*
 * tapestry_power_request — Request a power state transition.
 *
 * If the requested state differs from the current state, calls
 * substrate_set_power() to hand the transition to the L1 HAL.
 * No-op when state is already current.
 */
void                    tapestry_power_request(substrate_power_state_t state);

/*
 * tapestry_power_get — Return the current power state.
 */
substrate_power_state_t tapestry_power_get(void);

/* ── Future subsystems (interface reserved, not implemented) ─────────────── */
/*
 * tapestry_enclave_*   — secure computation partition
 * tapestry_ota_*       — over-the-air firmware update
 * tapestry_percept_*   — local sensor fusion pipeline
 */

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_RUNTIME_H */
