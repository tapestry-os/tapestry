/*
 * tapestry-os/boards/apollo510_evb/substrate_apollo510_evb.c
 * Tapestry L1 substrate implementation for the Ambiq Apollo510 EVB.
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
 * Selected at build time when BOARD is apollo510_evb (see the app's
 * CMakeLists.txt).
 */

#include <tapestry/substrate.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(substrate_ap510, LOG_LEVEL_INF);

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

#ifdef CONFIG_TAPESTRY_AP510_PIN_FINDER
/* ── Header pin finder (bring-up aid) ────────────────────────────────────── */
/*
 * The EVB silkscreens its headers with circled numbers whose meaning is
 * ambiguous without the schematic — they may be connector pin POSITIONS
 * rather than SoC GPIO numbers, and position 1 on a header is very often a
 * power rail.  Jumpering two of them on the assumption that they are GPIOs
 * risks shorting a rail.
 *
 * This resolves the mapping empirically instead.  Every UART1/2/3 TX or RX
 * pin the SoC offers is armed as a pull-up input; grounding one through a
 * series resistor names it on the console:
 *
 *     PIN FINDER: P23 GROUNDED  <<<<
 *
 * So: touch the probe to a header pin, read which SoC pin it is.  Use a
 * ~1 kOhm resistor in the probe lead — then touching a power rail by mistake
 * costs a few milliamps instead of a short.
 *
 * Pins claimed by an enabled peripheral are excluded: P53 (mspi1 chip enable)
 * and P55 (UART0 console RX).  Everything on the list is otherwise idle on
 * this board, and the SDIO/MSPI/USB/LED/button pins are all above this range.
 *
 * Never returns — the element does not come up in this mode.  Bring-up only.
 */

static const struct device *const pf_banks[] = {
    DEVICE_DT_GET(DT_NODELABEL(gpio0_31)),     /* P0  – P31 */
    DEVICE_DT_GET(DT_NODELABEL(gpio32_63)),    /* P32 – P63 */
    DEVICE_DT_GET(DT_NODELABEL(gpio64_95)),    /* P64 – P95 */
};

/* Every UART1/2/3 TX or RX pin, minus the two in use.  Derived from
 * modules/hal/ambiq/dts/apollo510/ambiq-apollo510-pinmux.h. */
static const uint8_t pf_pins[] = {
    0, 1, 2, 3, 11, 12, 13, 14, 15, 20, 21, 22, 23, 25,
    26, 31, 33, 34, 35, 36, 37, 41, 42, 43, 44, 45, 46, 47,
    48, 54, 56, 61, 62, 63, 66, 67, 68, 69, 72, 73, 75, 78,
};

static void pin_finder(void)
{
    /* Edge-detect state, seeded to "not grounded" so the first poll reports a
     * pin that is ALREADY grounded at start-up and stays silent about the ~40
     * that are not.  Seeding the other way round inverts both behaviours. */
    static bool was_low[ARRAY_SIZE(pf_pins)];
    static bool armed_ok[ARRAY_SIZE(pf_pins)];
    int armed = 0;

    for (size_t b = 0; b < ARRAY_SIZE(pf_banks); b++) {
        if (!device_is_ready(pf_banks[b])) {
            LOG_ERR("PIN FINDER: GPIO bank %u not ready — aborting", (unsigned)b);
            return;
        }
    }

    for (size_t i = 0; i < ARRAY_SIZE(pf_pins); i++) {
        const uint8_t pin = pf_pins[i];

        if (gpio_pin_configure(pf_banks[pin / 32u], pin % 32u,
                               GPIO_INPUT | GPIO_PULL_UP) != 0) {
            LOG_WRN("PIN FINDER: P%u could not be armed", (unsigned)pin);
            continue;
        }
        armed_ok[i] = true;
        was_low[i]  = false;
        armed++;
    }

    LOG_INF("PIN FINDER: %d pins armed as pull-up inputs", armed);
    LOG_INF("PIN FINDER: ground a header pin through ~1k to identify it");

    while (true) {
        for (size_t i = 0; i < ARRAY_SIZE(pf_pins); i++) {
            if (!armed_ok[i]) {
                continue;   /* never configured — its reading is meaningless */
            }

            const uint8_t pin = pf_pins[i];
            const int     val = gpio_pin_get_raw(pf_banks[pin / 32u], pin % 32u);

            if (val < 0) {
                continue;
            }

            const bool low = (val == 0);

            if (low != was_low[i]) {
                was_low[i] = low;
                LOG_INF("PIN FINDER: P%u %s", (unsigned)pin,
                        low ? "GROUNDED  <<<<" : "released");
            }
        }
        k_msleep(20);
    }
}
#endif /* CONFIG_TAPESTRY_AP510_PIN_FINDER */

/* ── API ─────────────────────────────────────────────────────────────────── */

int substrate_init(void)
{
#ifdef CONFIG_TAPESTRY_AP510_PIN_FINDER
    pin_finder();   /* never returns */
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
        static const char *const walk_pin[3] = { "P165", "P89", "P92" };

        for (int i = 0; i < 3; i++) {
            LOG_INF("LED WALK: led%d (%s) alone for 1.5 s", i, walk_pin[i]);
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
    LOG_INF("apollo510_evb substrate ready  leds=%s  btn0=%s  (no actuators)",
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
