/*
 * main.c — Tapestry element entry point
 *
 * Reads configuration from environment variables (native_sim runs as a
 * Linux host process, so host libc getenv is available):
 *
 *   ELEMENT_ID    integer element ID [0..MAX_ELEMENTS)  default 0
 *   ORCH_PORT     orchestrator UDP port                  default SIM_ORCH_PORT
 *
 * The element port is always SIM_ELEMENT_BASE_PORT + ELEMENT_ID.
 *
 * Main loop (WM_CYCLE_MS per iteration):
 *   1. Drain inbox   — gossip + control messages from orchestrator
 *   2. wm_tick       — age entries, recompute consistency metric
 *   3. Movement      — random walk + repulsion (skipped if degraded
 *                      or power state is not POWER_ACTIVE)
 *   4. wm_update_self
 *   5. Send gossip   — every GOSSIP_INTERVAL_MS
 *   6. Send metric   — every cycle
 *   7. k_msleep      — rest of the cycle period
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "state.h"
#include "world_model.h"
#include "movement.h"
#include "comms.h"
#include "sim_protocol.h"

LOG_MODULE_REGISTER(element, LOG_LEVEL_INF);

/* native_sim is a Linux process — host libc getenv is accessible */
extern char   *getenv(const char *name);
extern int     atoi(const char *s);
extern double  atof(const char *s);

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static int env_int(const char *name, int default_val)
{
    const char *v = getenv(name);
    return (v != NULL) ? atoi(v) : default_val;
}

static float env_float(const char *name, float default_val)
{
    const char *v = getenv(name);
    return (v != NULL) ? (float)atof(v) : default_val;
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    const int   element_id        = env_int("ELEMENT_ID",        0);
    const int   orch_port         = env_int("ORCH_PORT",         SIM_ORCH_PORT);
    const float consistency_bias  = env_float("CONSISTENCY_BIAS", 0.0f);

    /* ---- Initialise own state ------------------------------------------ */
    element_state_t own_state = {0};
    own_state.id               = (element_id_t)element_id;
    own_state.power_state      = POWER_ACTIVE;
    own_state.partition_island = 0;
    own_state.logical_clock    = 0;
    own_state.update_seq       = 0;

    /* Spread initial positions deterministically so elements don't start
     * on top of each other.  The multipliers are chosen to distribute
     * 20 elements reasonably across the 100×100 world. */
    own_state.position.x = 10.0f + (float)((element_id * 17) % 80);
    own_state.position.y = 10.0f + (float)((element_id * 23) % 80);

    LOG_INF("element %d starting at (%.1f, %.1f)",
            element_id,
            (double)own_state.position.x,
            (double)own_state.position.y);

    /* ---- Initialise world model --------------------------------------- */
    world_model_t wm;
    wm_init(&wm, own_state.id, &own_state, consistency_bias);

    /* ---- Initialise comms --------------------------------------------- */
    comms_t comms;
    if (comms_init(&comms, own_state.id, (uint16_t)orch_port) != 0) {
        LOG_ERR("comms_init failed — aborting");
        return -1;
    }

    /* ---- Main loop ---------------------------------------------------- */
    uint32_t gossip_accum_ms = 0;

    while (own_state.power_state != POWER_SLEEP) {

        /* 1. Process any messages that arrived since the last cycle */
        comms_drain_inbox(&comms, &wm, &own_state);

        /* 2. Age world model entries and recompute consistency metric */
        wm_tick(&wm, WM_CYCLE_MS);

        /* 3. Update position (only when active and not CP-frozen) */
        const wm_consistency_metric_t *metric = wm_get_metric(&wm);

        if (own_state.power_state == POWER_ACTIVE && !metric->degraded) {
            movement_tick(&own_state, &wm);
            wm_update_self(&wm, &own_state);
        }

        /* 4. Send gossip on interval */
        gossip_accum_ms += WM_CYCLE_MS;
        if (gossip_accum_ms >= GOSSIP_INTERVAL_MS) {
            comms_send_gossip(&comms, &own_state);
            gossip_accum_ms = 0;
        }

        /* 5. Send consistency metric every cycle */
        comms_send_metric(&comms, &wm, own_state.id);

        /* 6. Sleep for the rest of the cycle */
        k_msleep(WM_CYCLE_MS);
    }

    LOG_INF("element %d shutting down", element_id);
    return 0;
}
