/*
 * tapestry-os/boards/apollo510_evb/ap510_pin_finder.c
 * Header pin finder for the Ambiq Apollo510 EVB — bring-up aid.
 *
 * Split out of the substrate because its candidate pin list is specific to the
 * Apollo510's UART pinmux; the substrate itself is board-neutral and shared
 * with the other Ambiq EVBs (see boards/ambiq_common/substrate_ambiq_evb.c).
 *
 * Runs from substrate_init() via ap510_pin_finder(), which never returns.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ap510_pins, LOG_LEVEL_INF);

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

void ap510_pin_finder(void)
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

