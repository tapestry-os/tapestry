/*
 * tapestry-os/subsys/transport/transceiver_optical.c
 * Tapestry L1 optical transceiver — stub
 *
 * Template for an optical (IR / visible-light) communication medium.
 * Gate on CONFIG_TAPESTRY_TRANSCEIVER_OPTICAL and wire into the board's
 * Kconfig overlay once the hardware driver exists.
 */

#include "transceiver_optical.h"

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_OPTICAL

#include <errno.h>
#include <tapestry/transceiver.h>

static int optical_init(void)      { return -ENOTSUP; }
static int optical_tx(const uint8_t *d, uint16_t l) { ARG_UNUSED(d); ARG_UNUSED(l); return -ENOTSUP; }
static int optical_rx(uint8_t *b,  uint16_t l)      { ARG_UNUSED(b); ARG_UNUSED(l); return -ENOTSUP; }
static void optical_set_power(float v)               { ARG_UNUSED(v); }

const tapestry_transceiver_t transceiver_optical = {
    .type      = TRANSCEIVER_TYPE_OPTICAL,
    .init      = optical_init,
    .tx        = optical_tx,
    .rx        = optical_rx,
    .set_power = optical_set_power,
};

#endif /* CONFIG_TAPESTRY_TRANSCEIVER_OPTICAL */
