/*
 * tapestry-os/boards/bbc_microbit_v2/actuation_cutebot.c
 * Tapestry actuation implementation for the ELECFREAKS Cutebot Mini
 *
 * Implements <tapestry/actuation.h> by delegating to the Cutebot I2C driver.
 * Selected at build time when CONFIG_I2C=y (see app CMakeLists.txt).
 */

#include <tapestry/actuation.h>
#include "cutebot.h"

int actuation_init(void)
{
    return cutebot_init();
}

void actuation_drive(int left_pct, int right_pct)
{
    cutebot_drive(left_pct, right_pct);
}

void actuation_set_leds(uint8_t r, uint8_t g, uint8_t b)
{
    cutebot_set_leds(r, g, b);
}

void actuation_update(scr_role_t role, scr_quorum_state_t quorum)
{
    cutebot_update(role, quorum);
}
