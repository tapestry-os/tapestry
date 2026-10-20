/*
 * main.c — stationary leak sensor (examples/leak-sensor)
 *
 * A Tapestry element that never moves: a dev board placed on the robots'
 * board at a configured position (CONFIG_LEAK_SENSOR_X/Y), standing in for
 * a fixed leak detector.  Pressing sw0 reports a leak; the robots running
 * ../cutebot-formation/leak-sensor.choreo.toml then ring it.
 *
 * What makes it an element the robots understand:
 *
 *   - It gossips its state like any element, with ELEMENT_HEALTH_STATIONARY
 *     set (csm.h): the robots keep a smaller keep-out around it than around
 *     each other, and their auto-ID leaves it out of n_total.  It takes a
 *     fixed ID (CONFIG_TAPESTRY_ELEMENT_ID, default 31) and never runs
 *     auto-ID itself.
 *
 *   - It runs the robots' own Choreo.  Lacking locomotion, it lands on the
 *     script's catch-all track and gossips that, so FORM and membership
 *     events on the robots' track never count it.
 *
 *   - The leak is a TARGET shared fact at its own position.  The state
 *     lives in the fact table every element holds, not in this program: a
 *     sensor that reboots relearns the fact from the robots, and LED2
 *     follows the table, not the button.
 *
 * Buttons and LEDs (devicetree aliases, so any board with them works):
 *
 *   sw0   report a leak — publishes the TARGET fact (retried each cycle
 *         until the table has synced with a peer, ~1-2 s after boot)
 *   sw1   clear — retracts it (a tombstone).  The robots' script answers
 *         "landmark_retracted" by forming a ring about the board centre and
 *         waiting for the next leak: the reset between runs.
 *   led0  on: running
 *   led1  on: at least one robot heard (fresh); off: alone
 *   led2  on: a leak is held in the fact table; blinking: publish pending
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <tapestry/csm.h>
#include <tapestry/scr.h>
#include <tapestry/transport.h>
#include <tapestry/choreo.h>

#include "choreo_script.h"

LOG_MODULE_REGISTER(leak_sensor, LOG_LEVEL_INF);

#ifndef CHOREO_N_TRACKS
#error "leak-sensor needs a [[tracks]] Choreo (see choreo_script.h)"
#endif

static const struct gpio_dt_spec btn_leak  = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static const struct gpio_dt_spec btn_clear = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw1), gpios, {0});
static const struct gpio_dt_spec led_up    = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec led_peers = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});
static const struct gpio_dt_spec led_leak  = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0});

/* A missing button or LED is not fatal — the element still gossips, and a
 * leak can still be held (and shown) from a peer. */
static bool pin_setup(const struct gpio_dt_spec *spec, gpio_flags_t flags,
                      const char *what)
{
    if (spec->port == NULL || !gpio_is_ready_dt(spec) ||
        gpio_pin_configure_dt(spec, flags) != 0) {
        LOG_WRN("%s not available", what);
        return false;
    }
    return true;
}

static void led_set(const struct gpio_dt_spec *spec, bool ok, bool on)
{
    if (ok) {
        gpio_pin_set_dt(spec, on ? 1 : 0);
    }
}

/* Rising edge of a button sampled once per cycle.  WM_CYCLE_MS is longer
 * than contact bounce, so one sample per cycle is its own debounce. */
static bool pressed_edge(const struct gpio_dt_spec *spec, bool ok, bool *was)
{
    bool now = ok && gpio_pin_get_dt(spec) > 0;
    bool edge = now && !*was;
    *was = now;
    return edge;
}

static int fresh_robots(const world_model_t *wm)
{
    int n = 0;
    for (int i = 0; i < MAX_ELEMENTS; i++) {
        const wm_entry_t *e = &wm->entries[i];
        if (e->is_active && !e->is_self && !e->is_stale &&
            !element_is_stationary(e)) {
            n++;
        }
    }
    return n;
}

int main(void)
{
    const element_id_t id = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;
    const position_t   at = { (float)CONFIG_LEAK_SENSOR_X,
                              (float)CONFIG_LEAK_SENSOR_Y, 0.0f };

    bool leak_ok  = pin_setup(&btn_leak,  GPIO_INPUT, "sw0 (leak)");
    bool clear_ok = pin_setup(&btn_clear, GPIO_INPUT, "sw1 (clear)");
    bool up_ok    = pin_setup(&led_up,    GPIO_OUTPUT_INACTIVE, "led0");
    bool peers_ok = pin_setup(&led_peers, GPIO_OUTPUT_INACTIVE, "led1");
    bool lk_ok    = pin_setup(&led_leak,  GPIO_OUTPUT_INACTIVE, "led2");

    LOG_INF("leak sensor id=%u at (%.1f, %.1f) — stationary, fixed id",
            (unsigned)id, (double)at.x, (double)at.y);
    /* Board silkscreens number buttons and LEDs their own way; the pin
     * finds the part on the schematic. */
    const struct { const struct gpio_dt_spec *s; bool ok; const char *what; } pins[] = {
        { &btn_leak, leak_ok, "sw0  LEAK " }, { &btn_clear, clear_ok, "sw1  CLEAR" },
        { &led_up, up_ok, "led0 up   " }, { &led_peers, peers_ok, "led1 peers" },
        { &led_leak, lk_ok, "led2 leak " },
    };
    for (size_t i = 0; i < ARRAY_SIZE(pins); i++) {
        if (pins[i].ok) {
            LOG_INF("  %s = %s pin %u", pins[i].what, pins[i].s->port->name,
                    (unsigned)pins[i].s->pin);
        }
    }

    if (transport_init() != 0) {
        LOG_ERR("no transport — this sensor cannot be heard");
        return -1;
    }
    led_set(&led_up, up_ok, true);

    element_state_t own_state = {0};
    own_state.id           = id;
    own_state.position     = at;
    own_state.orientation  = orientation_identity();
    own_state.energy_level = 100u;
    own_state.health_flags = ELEMENT_HEALTH_STATIONARY;

    world_model_t wm;
    wm_init(&wm, id, &own_state, 0.0f);   /* pure AP, like the robots */

    /* Sensing, and an absolute position by construction (configured).  No
     * actuator: the script's "responders" track requires locomotion, so
     * this element takes the catch-all. */
    scr_state_t scr;
    scr_init(&scr, id, 1u, 1u, SCR_CAP_SENSOR | SCR_CAP_ABS_POSITION);

    choreo_init(id);
    choreo_register_scr(&scr);
    if (choreo_submit_tracks(&wm, k_choreo_tracks, CHOREO_N_TRACKS) != 0) {
        LOG_ERR("choreo \"%s\" rejected", CHOREO_NAME);
        return -1;
    }
    /* Gossip the track from the first frame on, not track 0 by default. */
    choreo_publish_state(&own_state);
    LOG_INF("choreo \"%s\" loaded — track %u", CHOREO_NAME,
            (unsigned)choreo_current_track());

    bool       leak_was = false, clear_was = false;
    bool       pending  = false;
    uint32_t   gossip_accum = GOSSIP_INTERVAL_MS;   /* send on the first tick */
    uint32_t   blink        = 0;
    int        last_robots  = -1;
    fact_key_t logged_key   = {0};
    bool       logged_held  = false;

    while (true) {
        transport_drain(&wm, id);   /* also runs the shared-fact duties */
        wm_tick(&wm, WM_CYCLE_MS);
        wm_update_self(&wm, &own_state);
        scr_tick(&scr, &wm);
        choreo_tick(&wm, &scr);
        choreo_publish_state(&own_state);

        if (pressed_edge(&btn_leak, leak_ok, &leak_was)) {
            LOG_INF("LEAK reported (sw0)");
            pending = true;
        }
        if (pending) {
            fact_key_t key;
            int rc = wm_fact_publish(&wm, FACT_TYPE_TARGET, &at, 0u,
                                     FACT_IMMUTABLE | FACT_PRIO(3), &key);
            if (rc == WM_FACT_CREATED || rc == WM_FACT_REUSED) {
                pending = false;
                LOG_INF("%s target %u/%04x/%u at (%.1f, %.1f)",
                        rc == WM_FACT_CREATED ? "PUBLISHED" : "already held:",
                        (unsigned)key.origin, (unsigned)key.epoch,
                        (unsigned)key.seq, (double)at.x, (double)at.y);
            } else if (rc != WM_FACT_ERR_UNSYNCED && rc != WM_FACT_ERR_NOEPOCH) {
                pending = false;
                LOG_ERR("target fact NOT published (%d)", rc);
            }
        }

        fact_t target;
        bool held = wm_fact_find(&wm, FACT_TYPE_TARGET, 0, &target);
        if (pressed_edge(&btn_clear, clear_ok, &clear_was)) {
            pending = false;
            if (held && wm_fact_retract(&wm, &target.key) == 0) {
                LOG_INF("leak CLEARED (sw1): target %u/%04x/%u retracted",
                        (unsigned)target.key.origin,
                        (unsigned)target.key.epoch,
                        (unsigned)target.key.seq);
                held = wm_fact_find(&wm, FACT_TYPE_TARGET, 0, &target);
            } else {
                LOG_INF("clear (sw1): no leak held");
            }
        }
        if (held && (!logged_held || !fact_key_equal(&logged_key, &target.key))) {
            LOG_INF("leak held: target %u/%04x/%u at (%.1f, %.1f)",
                    (unsigned)target.key.origin, (unsigned)target.key.epoch,
                    (unsigned)target.key.seq,
                    (double)target.pos.x, (double)target.pos.y);
            logged_key = target.key;
        } else if (!held && logged_held) {
            LOG_INF("no leak held");
        }
        logged_held = held;

        int robots = fresh_robots(&wm);
        if (robots != last_robots) {
            LOG_INF("robots heard: %d -> %d", last_robots, robots);
            last_robots = robots;
        }

        blink++;
        led_set(&led_peers, peers_ok, robots > 0);
        led_set(&led_leak, lk_ok, held || (pending && (blink & 2u)));

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
            own_state.update_seq++;
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }

        k_msleep(WM_CYCLE_MS);
    }

    return 0;
}
