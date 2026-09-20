/*
 * cutebot_line_bench.c — see cutebot_line_bench.h.
 */

#include "cutebot_line_bench.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define LINE_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(LINE_NODE, cutebot_line_left_gpios) && \
    DT_NODE_HAS_PROP(LINE_NODE, cutebot_line_right_gpios)

static const struct gpio_dt_spec s_p13 =
    GPIO_DT_SPEC_GET(LINE_NODE, cutebot_line_left_gpios);
static const struct gpio_dt_spec s_p14 =
    GPIO_DT_SPEC_GET(LINE_NODE, cutebot_line_right_gpios);

int cutebot_line_bench_init(void)
{
    const struct gpio_dt_spec *specs[] = { &s_p13, &s_p14 };

    for (size_t i = 0; i < ARRAY_SIZE(specs); i++) {
        if (!gpio_is_ready_dt(specs[i])) {
            return -ENODEV;
        }
        int rc = gpio_pin_configure_dt(specs[i], GPIO_INPUT);
        if (rc != 0) {
            return rc;
        }
    }

    return 0;
}

void cutebot_line_bench_read(cutebot_line_bench_sample_t *out)
{
    out->p13 = gpio_pin_get_dt(&s_p13);
    out->p14 = gpio_pin_get_dt(&s_p14);
}

#else /* one or both line-sensor pin properties absent from zephyr,user */

int cutebot_line_bench_init(void)
{
    return -ENODEV;
}

void cutebot_line_bench_read(cutebot_line_bench_sample_t *out)
{
    out->p13 = -1;
    out->p14 = -1;
}

#endif
