/*
 * cloud_occlusion.c — see cloud_occlusion.h
 */

#include "cloud_occlusion.h"

typedef enum { CSIDE_SOUTH, CSIDE_UNDER, CSIDE_NORTH } cloud_side_t;

static cloud_side_t cloud_side_of(float y)
{
    if (y < CLOUD_RF_DECK_Y_MIN) { return CSIDE_SOUTH; }
    if (y > CLOUD_RF_DECK_Y_MAX) { return CSIDE_NORTH; }
    return CSIDE_UNDER;
}

bool cloud_link_obstructed(float ax, float ay, float bx, float by)
{
    (void)ax;
    (void)bx;
    if (cloud_side_of(ay) == cloud_side_of(by)) {
        return false;
    }
    float lo = ax < bx ? ax : bx;
    float hi = ax < bx ? bx : ax;
    return (hi >= CLOUD_RF_DECK_X_MIN && lo <= CLOUD_RF_DECK_X_MAX);
}
