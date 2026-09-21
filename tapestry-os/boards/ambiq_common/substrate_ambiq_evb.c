/*
 * tapestry-os/boards/ambiq_common/substrate_ambiq_evb.c
 * Tapestry L1 substrate for Ambiq evaluation boards (Apollo510 EVB,
 * Apollo330mP EVB).
 *
 * Implements <tapestry/substrate.h> for an element that has compute, memory,
 * and a gossip bus but no actuators — the EVB is a bare SoC evaluation board.
 * That is a legitimate element class, not a degenerate one: it participates
 * fully in L3 gossip, L4 world state, and L5 quorum, and simply has nothing
 * to move.  Every motion primitive is therefore a documented no-op.
 *
 * What the board DOES give the substrate layer:
 *
 *   3 × LED   (aliases led0/led1/led2, all active-low) → status signal
 *   2 × button (aliases sw0/sw1, active-low with pull-up)
 *
 * Signal encoding — the three LEDs read left to right as one status word:
 *
 *   signal      led0  led1  led2   meaning
 *   NONE        off   off   off    output suppressed
 *   IDLE        on    off   off    element present, no active goal
 *   ACTIVE      on    on    off    goal in progress, quorum healthy
 *   DEGRADED    on    off   on     goal in progress, quorum reduced
 *   FAILED      off   off   on     quorum lost or goal unachievable
 *
 *   led0 = "this element is up"      led1 = "quorum healthy"
 *   led2 = "something is wrong"
 *
 * Proximity sensing is wired to BTN0 as a human-in-the-loop stand-in: the
 * EVB has no range sensor, so pressing BTN0 reports an obstacle at contact
 * range and releasing it reports clear.  This is deliberately a manual
 * injection point for testing obstacle-reactive behaviour on a board that
 * cannot sense anything — it is NOT a sensor reading, and the comment on
 * apollo_sense() below says so at the call site too.
 *
 * Entirely devicetree-driven — led0/led1/led2 and sw0 aliases — so it serves
 * any Ambiq EVB with the standard three LEDs and two buttons.  Selected by
 * board name in the app's CMakeLists.txt.
 */

#include <tapestry/substrate.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(substrate_ambiq, LOG_LEVEL_INF);

/* ── Devicetree handles ──────────────────────────────────────────────────── */

#define LED_SPEC(alias) GPIO_DT_SPEC_GET_OR(DT_ALIAS(alias), gpios, {0})

static const struct gpio_dt_spec leds[3] = {
    LED_SPEC(led0),
    LED_SPEC(led1),
    LED_SPEC(led2),
};

/*
 * BTN0 is read directly rather than through the gpio-keys input driver:
 * this build does not enable CONFIG_INPUT, so nothing else claims the pin,
 * and substrate_sense() is a polled call with no need for an event stream.
 */
static const struct gpio_dt_spec btn0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});

/* ── Private state ───────────────────────────────────────────────────────── */

static bool leds_ready;
static bool btn0_ready;

/* Suppressed while the substrate is in a reduced power state, so a
 * set_signal() arriving from L2 after the element has been put to sleep
 * cannot silently light the board back up. */
static bool output_enabled = true;

/* Retained so the LEDs can be restored on the way back to ACTIVE. */
static substrate_signal_t last_signal = SUBSTRATE_SIGNAL_NONE;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static void leds_write(bool l0, bool l1, bool l2)
{
    if (!leds_ready) {
        return;
    }

    const bool want[3] = { l0, l1, l2 };

    for (int i = 0; i < 3; i++) {
        /* GPIO_ACTIVE_LOW is declared in the board DTS, so the logical
         * value here is "lit", not "pin high". */
        (void)gpio_pin_set_dt(&leds[i], want[i] ? 1 : 0);
    }
}

static void apply_signal(substrate_signal_t signal)
{
    switch (signal) {
    case SUBSTRATE_SIGNAL_IDLE:     leds_write(true,  false, false); break;
    case SUBSTRATE_SIGNAL_ACTIVE:   leds_write(true,  true,  false); break;
    case SUBSTRATE_SIGNAL_DEGRADED: leds_write(true,  false, true);  break;
    case SUBSTRATE_SIGNAL_FAILED:   leds_write(false, false, true);  break;
    case SUBSTRATE_SIGNAL_NONE:
    default:                        leds_write(false, false, false); break;
    }
}

/* ── API ─────────────────────────────────────────────────────────────────── */

int substrate_init(void)
{
#ifdef CONFIG_TAPESTRY_AP510_PIN_FINDER
    extern void ap510_pin_finder(void);
    ap510_pin_finder();   /* never returns */
#endif

    leds_ready = true;

    for (int i = 0; i < 3; i++) {
        if (!gpio_is_ready_dt(&leds[i])) {
            LOG_WRN("led%d not ready — status output disabled", i);
            leds_ready = false;
            break;
        }
        if (gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE) != 0) {
            LOG_WRN("led%d configure failed — status output disabled", i);
            leds_ready = false;
            break;
        }
    }

    btn0_ready = gpio_is_ready_dt(&btn0) &&
                 gpio_pin_configure_dt(&btn0, GPIO_INPUT) == 0;
    if (!btn0_ready) {
        LOG_WRN("sw0 not ready — proximity injection unavailable");
    }

#ifdef CONFIG_TAPESTRY_AP510_LED_WALK
    /*
     * Bring-up aid: light each LED alone so the physical LED behind each
     * devicetree alias can be identified by eye.  The EVB silkscreen numbers
     * its LEDs independently of Zephyr's aliases and the board also carries
     * indicator LEDs that software cannot drive, so this mapping cannot be
     * read off the board files.
     */
    if (leds_ready) {
        for (int i = 0; i < 3; i++) {
            /* Port name + pin, read from devicetree — the SoC pin number is
             * that bank's base plus this offset. */
            LOG_INF("LED WALK: led%d (%s pin %u) alone for 1.5 s",
                    i, leds[i].port->name, (unsigned)leds[i].pin);
            leds_write(i == 0, i == 1, i == 2);
            k_msleep(1500);
        }
        leds_write(false, false, false);
        LOG_INF("LED WALK: done — all off");
    }
#endif

    /*
     * A missing LED or button is not fatal.  The element's job is to gossip
     * and hold up its share of the quorum; losing the status display costs
     * observability, not participation, and returning an error here would
     * stop tapestry_runtime_init() from bringing the element up at all.
     */
    LOG_INF("ambiq evb substrate ready  leds=%s  btn0=%s  (no actuators)",
            leds_ready ? "yes" : "no", btn0_ready ? "yes" : "no");
    return 0;
}

void substrate_move(const substrate_twist_t *twist)
{
    /*
     * No actuators on an evaluation board.  Motion commands are accepted and
     * discarded — the substrate contract requires callers to tolerate exactly
     * this, and upper layers keep planning as normal.  Watch the element's
     * commanded twist in the telemetry CSV on UART0, not on the hardware.
     */
    ARG_UNUSED(twist);
}

void substrate_set_signal(substrate_signal_t signal)
{
    last_signal = signal;

    if (output_enabled) {
        apply_signal(signal);
    }
}

void substrate_set_power(substrate_power_state_t state)
{
    switch (state) {
    case SUBSTRATE_POWER_ACTIVE:
        output_enabled = true;
        apply_signal(last_signal);
        break;

    case SUBSTRATE_POWER_IDLE:
        /* Communication only.  The LEDs are the only load this substrate
         * controls and they are worth keeping: an element that has gone
         * quiet but is still gossiping should still be visible.  Repaint —
         * arriving here from SLEEP leaves them blanked otherwise. */
        output_enabled = true;
        apply_signal(last_signal);
        break;

    case SUBSTRATE_POWER_SLEEP:
    case SUBSTRATE_POWER_HARVEST:
        /* Shed the only load there is.  L2 (power.c) handles the SoC-side
         * transition; note that transceiver_uart.c holds a policy lock on
         * PM_STATE_SUSPEND_TO_RAM so the gossip bus stays clocked. */
        output_enabled = false;
        leds_write(false, false, false);
        break;
    }
}

int substrate_sense(substrate_sensor_t type, float *out)
{
    if (out == NULL) {
        return -1;
    }

    switch (type) {
    case SUBSTRATE_SENSOR_PROXIMITY:
        /*
         * NOT a sensor.  BTN0 stands in for a range sensor the EVB does not
         * have, so obstacle-reactive behaviour can be exercised by hand:
         * held down = obstacle at contact range, released = clear.
         */
        if (!btn0_ready) {
            return -1;
        }
        {
            const int pressed = gpio_pin_get_dt(&btn0);
            if (pressed < 0) {
                return -1;
            }
            *out = pressed ? 0.0f : 1.0f;
        }
        return 0;

    case SUBSTRATE_SENSOR_BATTERY:
        /* Bench-powered evaluation board — no battery to report on.  A
         * hardcoded 1.0 would look like a healthy cell to L4's energy
         * aggregation, which would be a lie about a board that cannot run
         * off a battery at all. */
        return -1;

    default:
        return -1;
    }
}

void substrate_bond(void)    {}
void substrate_release(void) {}
void substrate_emit(void)    {}
