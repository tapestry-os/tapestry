/*
 * power.c — Tapestry L2 power state machine
 *
 * Manages the element's power state (ACTIVE / IDLE / SLEEP / HARVEST).
 *
 * Transition sequence on every state change:
 *   1. substrate_set_power(state) — L1 cleans up actuators (motors off, etc.)
 *   2. L2 issues the OS-level Zephyr PM transition.
 *
 * SLEEP   → PM_STATE_SUSPEND_TO_IDLE; wake via BLE event or k_timer.
 * HARVEST → PM_STATE_SOFT_OFF; wake via threshold-gpios from a
 *            "tapestry,harvester" DT node (tapestry-os/dts/bindings/).
 *            On nRF52833 SOFT_OFF wakes with a full reset.
 *            Falls back to SUSPEND_TO_IDLE if no threshold-gpios are present.
 * ACTIVE / IDLE do not require explicit PM calls; Zephyr's idle thread
 * enters PM_STATE_RUNTIME_IDLE automatically when the CPU has no work.
 */

#include "power.h"
#include <tapestry/runtime.h>
#include <tapestry/substrate.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(tapestry_runtime, LOG_LEVEL_INF);

#ifdef CONFIG_PM
#include <zephyr/pm/pm.h>
#endif

/* ── Harvester DT detection ──────────────────────────────────────────────── */

#if DT_HAS_COMPAT_STATUS_OKAY(tapestry_harvester)
#include <zephyr/drivers/gpio.h>
#define HARVESTER_NODE DT_INST(0, tapestry_harvester)
#if DT_NODE_HAS_PROP(HARVESTER_NODE, threshold_gpios)
static const struct gpio_dt_spec s_harvest_thr =
    GPIO_DT_SPEC_GET(HARVESTER_NODE, threshold_gpios);
#define HAS_HARVEST_THR 1
#endif
#endif

/* ── Internal state ──────────────────────────────────────────────────────── */

static substrate_power_state_t s_state           = SUBSTRATE_POWER_ACTIVE;
static uint32_t                s_quorum_lost_cyc  = 0;

#ifndef CONFIG_TAPESTRY_POWER_SLEEP_GRACE_CYCLES
#define CONFIG_TAPESTRY_POWER_SLEEP_GRACE_CYCLES 50
#endif

/* ── PM helpers ──────────────────────────────────────────────────────────── */

static void enter_sleep(void)
{
#ifdef CONFIG_PM
    pm_state_force(0, &(struct pm_state_info){ .state = PM_STATE_SUSPEND_TO_IDLE });
#endif
}

static void enter_harvest(void)
{
#ifdef CONFIG_PM
#ifdef HAS_HARVEST_THR
    if (gpio_is_ready_dt(&s_harvest_thr)) {
        gpio_pin_configure_dt(&s_harvest_thr, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&s_harvest_thr, GPIO_INT_EDGE_RISING);
        LOG_INF("harvest: wake source armed on %s pin %d",
                s_harvest_thr.port->name, s_harvest_thr.pin);
    }
    pm_state_force(0, &(struct pm_state_info){ .state = PM_STATE_SOFT_OFF });
#else
    /* No tapestry,harvester threshold-gpios in DT — fall back to deep sleep. */
    LOG_WRN("harvest: no threshold-gpios defined; falling back to SUSPEND_TO_IDLE");
    pm_state_force(0, &(struct pm_state_info){ .state = PM_STATE_SUSPEND_TO_IDLE });
#endif
#endif /* CONFIG_PM */
}

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void tapestry_power_init(void)
{
    s_state           = SUBSTRATE_POWER_ACTIVE;
    s_quorum_lost_cyc = 0;
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void tapestry_power_request(substrate_power_state_t state)
{
    if (state == s_state) {
        return;
    }
    LOG_INF("power: %d -> %d", (int)s_state, (int)state);
    s_state = state;
    substrate_set_power(state);   /* L1: actuator cleanup before sleep */

    switch (state) {
    case SUBSTRATE_POWER_SLEEP:
        enter_sleep();
        break;
    case SUBSTRATE_POWER_HARVEST:
        enter_harvest();
        break;
    default:
        break;
    }
}

substrate_power_state_t tapestry_power_get(void)
{
    return s_state;
}

/* ── Internal tick ───────────────────────────────────────────────────────── */

void tapestry_power_tick(scr_quorum_state_t quorum_state)
{
#ifdef CONFIG_TAPESTRY_POWER_AUTO_POLICY
    if (quorum_state == SCR_QUORUM_LOST) {
        s_quorum_lost_cyc++;
        if (s_quorum_lost_cyc >= CONFIG_TAPESTRY_POWER_SLEEP_GRACE_CYCLES &&
            s_state == SUBSTRATE_POWER_ACTIVE) {
            tapestry_power_request(SUBSTRATE_POWER_IDLE);
        }
    } else {
        s_quorum_lost_cyc = 0;
        if (s_state == SUBSTRATE_POWER_IDLE) {
            tapestry_power_request(SUBSTRATE_POWER_ACTIVE);
        }
    }
#else
    (void)quorum_state;
#endif
}
