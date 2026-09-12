/*
 * cutebot_line.c — see cutebot_line.h for the interface contract and the
 * pin/signal-convention doc in bbc_microbit_v2.overlay.
 */

#include "cutebot_line.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cutebot_line, LOG_LEVEL_INF);

/*
 * On `zephyr,user`, not a custom node — a devicetree node with no
 * `compatible` gets zero property-type inference (confirmed against a
 * real build attempt: a bare `cutebot-line-sensors { left-gpios = ...; }`
 * node generated no macros for its "-gpios" properties at all).
 * `zephyr,user` is the one path Zephyr's tooling special-cases for
 * exactly this ad-hoc-GPIO case — see the overlay's comment, which also
 * cites the in-tree sample (line_follower_robot) using this identical
 * pattern for the same P13/P14 pins.
 */
#define LINE_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(LINE_NODE, cutebot_line_left_gpios) && \
    DT_NODE_HAS_PROP(LINE_NODE, cutebot_line_right_gpios)

static const struct gpio_dt_spec s_left  =
    GPIO_DT_SPEC_GET(LINE_NODE, cutebot_line_left_gpios);
static const struct gpio_dt_spec s_right =
    GPIO_DT_SPEC_GET(LINE_NODE, cutebot_line_right_gpios);

static struct gpio_callback s_left_cb;
static struct gpio_callback s_right_cb;

/* Written only from GPIO ISR context, read-and-cleared only from
 * cutebot_line_poll() under irq_lock() — see that function. Plain
 * volatile counters, not atomic_t: the critical section is a handful of
 * instructions on a single core, and this driver has no other
 * concurrent writer to race against. */
static volatile uint16_t s_left_edges;
static volatile uint16_t s_right_edges;

/* Line-ENTRY (LOW-going) edge timestamps, same synchronization as above
 * — see cutebot_line_sample_t's doc for why these exist (heading-error
 * sensing from the left/right crossing-time delta). */
static volatile bool     s_left_entered;
static volatile bool     s_right_entered;
static volatile uint32_t s_left_entry_ms;
static volatile uint32_t s_right_entry_ms;

static void line_isr(const struct device *port, struct gpio_callback *cb,
                      gpio_port_pins_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    bool is_left = (cb == &s_left_cb);
    const struct gpio_dt_spec *spec = is_left ? &s_left : &s_right;

    if (is_left) {
        s_left_edges++;
    } else {
        s_right_edges++;
    }

    /* gpio_pin_get_dt() is a plain register read on this GPIO backend —
     * safe from ISR context, no blocking call. LOW (0) == line, per the
     * overlay's documented sensor convention — this is the ENTRY edge,
     * not the later rising/exit edge. */
    if (gpio_pin_get_dt(spec) == 0) {
        uint32_t now = k_uptime_get_32();
        if (is_left) {
            s_left_entry_ms = now;
            s_left_entered  = true;
        } else {
            s_right_entry_ms = now;
            s_right_entered  = true;
        }
    }
}

int cutebot_line_init(void)
{
    if (!gpio_is_ready_dt(&s_left) || !gpio_is_ready_dt(&s_right)) {
        LOG_ERR("line sensor GPIO device(s) not ready");
        return -ENODEV;
    }

    int rc;

    /* No pull configured — matches pxt-cutebot's PullNone: the sensor
     * module drives its own output, an internal pull would just fight
     * it. */
    rc = gpio_pin_configure_dt(&s_left, GPIO_INPUT);
    if (rc != 0) {
        return rc;
    }
    rc = gpio_pin_configure_dt(&s_right, GPIO_INPUT);
    if (rc != 0) {
        return rc;
    }

    gpio_init_callback(&s_left_cb, line_isr, BIT(s_left.pin));
    gpio_init_callback(&s_right_cb, line_isr, BIT(s_right.pin));

    rc = gpio_add_callback(s_left.port, &s_left_cb);
    if (rc != 0) {
        return rc;
    }
    rc = gpio_add_callback(s_right.port, &s_right_cb);
    if (rc != 0) {
        return rc;
    }

    rc = gpio_pin_interrupt_configure_dt(&s_left, GPIO_INT_EDGE_BOTH);
    if (rc != 0) {
        return rc;
    }
    rc = gpio_pin_interrupt_configure_dt(&s_right, GPIO_INT_EDGE_BOTH);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

void cutebot_line_poll(cutebot_line_sample_t *out)
{
    /* Raw GPIO level is LOW-means-line (see overlay comment) — invert
     * here so every caller of this driver sees a plain "true == line". */
    out->left  = (gpio_pin_get_dt(&s_left)  == 0);
    out->right = (gpio_pin_get_dt(&s_right) == 0);

    unsigned int key = irq_lock();
    out->left_edges  = s_left_edges;
    out->right_edges = s_right_edges;
    s_left_edges  = 0;
    s_right_edges = 0;

    out->left_entered  = s_left_entered;
    out->right_entered = s_right_entered;
    out->left_entry_ms  = s_left_entry_ms;
    out->right_entry_ms = s_right_entry_ms;
    s_left_entered  = false;
    s_right_entered = false;
    irq_unlock(key);
}

#else /* line-sensor properties absent from zephyr,user on this build */

int cutebot_line_init(void)
{
    return -ENODEV;
}

void cutebot_line_poll(cutebot_line_sample_t *out)
{
    out->left = false;
    out->right = false;
    out->left_edges = 0;
    out->right_edges = 0;
    out->left_entered = false;
    out->right_entered = false;
    out->left_entry_ms = 0;
    out->right_entry_ms = 0;
}

#endif
