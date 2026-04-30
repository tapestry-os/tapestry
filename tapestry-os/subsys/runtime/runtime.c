/*
 * runtime.c — Tapestry L2 Element Runtime
 *
 * Owns the per-element cycle for the full L4+L5+L6 stack:
 *   transport_drain → wm_tick → scr_tick → wm_update_self →
 *   tapestry_tick → gossip send → telemetry → power policy
 *
 * Applications call tapestry_runtime_tick() once per WM_CYCLE_MS and then
 * read tapestry_runtime_scr() to drive substrate_move() and
 * substrate_set_signal() themselves.
 */

#include "power.h"
#include <tapestry/runtime.h>
#include <tapestry/transport.h>
#include <tapestry/app.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tapestry_runtime, LOG_LEVEL_INF);

/* ── Static storage ──────────────────────────────────────────────────────── */

static element_state_t s_own;
static world_model_t   s_wm;
static scr_state_t     s_scr;

static uint32_t     s_gossip_accum_ms;
static uint32_t     s_election_count;
static element_id_t s_last_leader;
static bool         s_ready;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

int tapestry_runtime_init(const tapestry_runtime_config_t *cfg)
{
    if (substrate_init() != 0) {
        LOG_WRN("substrate init failed — movement and signal disabled");
    }

    if (transport_init() != 0) {
        LOG_ERR("transport init failed — halting");
        return -1;
    }

    s_own = (element_state_t){
        .id            = cfg->self_id,
        .power_state   = POWER_ACTIVE,
        .position.x    = cfg->pos_x,
        .position.y    = cfg->pos_y,
        .logical_clock = 0,
        .update_seq    = 0,
    };

    wm_init(&s_wm, cfg->self_id, &s_own, cfg->consistency_bias);
    scr_init(&s_scr, cfg->self_id, cfg->quorum_min, cfg->quorum_target);
    tapestry_init(cfg->self_id);
    tapestry_power_init();

    s_gossip_accum_ms = 0;
    s_election_count  = 0;
    s_last_leader     = ELEMENT_ID_INVALID;
    s_ready           = true;

    LOG_INF("runtime: element %u at (%.1f, %.1f) quorum=%u/%u",
            (unsigned)cfg->self_id,
            (double)cfg->pos_x, (double)cfg->pos_y,
            (unsigned)cfg->quorum_min, (unsigned)cfg->quorum_target);
    return 0;
}

/* ── Tick ────────────────────────────────────────────────────────────────── */

void tapestry_runtime_tick(void)
{
    if (!s_ready) {
        return;
    }

    /* 1. Receive gossip */
    transport_drain(&s_wm, s_own.id);

    /* 2. Age L4 entries */
    wm_tick(&s_wm, WM_CYCLE_MS);

    /* 3. Recompute L5 role and quorum */
    scr_tick(&s_scr, &s_wm);

    /* 4. Track leader changes for telemetry election counter */
    if (s_scr.leader_id != s_last_leader) {
        if (s_last_leader   != ELEMENT_ID_INVALID ||
            s_scr.leader_id != ELEMENT_ID_INVALID) {
            s_election_count++;
        }
        s_last_leader = s_scr.leader_id;
        LOG_INF("election #%u: leader=%u role=%u quorum=%u",
                s_election_count,
                (unsigned)s_scr.leader_id,
                (unsigned)s_scr.role,
                (unsigned)s_scr.quorum_state);
    }

    /* 5. Refresh own entry */
    wm_update_self(&s_wm, &s_own);

    /* 6. L6 BSE: synthesise per-element directive */
    tapestry_tick(&s_wm, &s_scr);

    /* 7. Gossip send on interval */
    s_gossip_accum_ms += WM_CYCLE_MS;
    if (s_gossip_accum_ms >= GOSSIP_INTERVAL_MS) {
        s_own.update_seq++;
        transport_send(&s_own);
        s_gossip_accum_ms = 0;
    }

    /* 8. Telemetry */
    transport_send_telemetry(&s_wm, s_own.id, &s_scr, s_election_count);

    /* 9. Power auto-stepping policy */
    tapestry_power_tick(s_scr.quorum_state);
}

/* ── State accessors ─────────────────────────────────────────────────────── */

const world_model_t *tapestry_runtime_wm(void)  { return &s_wm; }
const scr_state_t   *tapestry_runtime_scr(void) { return &s_scr; }
