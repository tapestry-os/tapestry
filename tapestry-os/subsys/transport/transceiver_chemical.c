/*
 * tapestry-os/subsys/transport/transceiver_chemical.c
 * Tapestry L1 chemical transceiver — stub
 *
 * Template for a chemical (pheromone / diffusion-based) communication medium.
 * Gate on CONFIG_TAPESTRY_TRANSCEIVER_CHEMICAL and wire into the board's
 * Kconfig overlay once the hardware driver exists.
 */

#include "transceiver_chemical.h"

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_CHEMICAL

#include <errno.h>
#include <tapestry/transceiver.h>

static int chemical_init(void)      { return -ENOTSUP; }
static int chemical_tx(const uint8_t *d, uint16_t l) { ARG_UNUSED(d); ARG_UNUSED(l); return -ENOTSUP; }
static int chemical_rx(uint8_t *b,  uint16_t l)      { ARG_UNUSED(b); ARG_UNUSED(l); return -ENOTSUP; }
static void chemical_set_power(float v)               { ARG_UNUSED(v); }

const tapestry_transceiver_t transceiver_chemical = {
    .type      = TRANSCEIVER_TYPE_CHEMICAL,
    .init      = chemical_init,
    .tx        = chemical_tx,
    .rx        = chemical_rx,
    .set_power = chemical_set_power,
};

#endif /* CONFIG_TAPESTRY_TRANSCEIVER_CHEMICAL */
