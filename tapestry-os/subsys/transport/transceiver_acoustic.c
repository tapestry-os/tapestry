/*
 * tapestry-os/subsys/transport/transceiver_acoustic.c
 * Tapestry L1 acoustic transceiver — stub
 *
 * Template for an acoustic (ultrasonic / audible) communication medium.
 * Gate on CONFIG_TAPESTRY_TRANSCEIVER_ACOUSTIC and wire into the board's
 * Kconfig overlay once the hardware driver exists.
 *
 * The init/tx/rx ops return -ENOTSUP until a real driver is provided.
 * This file compiles cleanly so the vtable shape is always verified.
 */

#include "transceiver_acoustic.h"

#ifdef CONFIG_TAPESTRY_TRANSCEIVER_ACOUSTIC

#include <errno.h>
#include <tapestry/transceiver.h>

static int acoustic_init(void)      { return -ENOTSUP; }
static int acoustic_tx(const uint8_t *d, uint16_t l) { ARG_UNUSED(d); ARG_UNUSED(l); return -ENOTSUP; }
static int acoustic_rx(uint8_t *b,  uint16_t l)      { ARG_UNUSED(b); ARG_UNUSED(l); return -ENOTSUP; }
static void acoustic_set_power(float v)               { ARG_UNUSED(v); }

const tapestry_transceiver_t transceiver_acoustic = {
    .type      = TRANSCEIVER_TYPE_ACOUSTIC,
    .init      = acoustic_init,
    .tx        = acoustic_tx,
    .rx        = acoustic_rx,
    .set_power = acoustic_set_power,
};

#endif /* CONFIG_TAPESTRY_TRANSCEIVER_ACOUSTIC */
