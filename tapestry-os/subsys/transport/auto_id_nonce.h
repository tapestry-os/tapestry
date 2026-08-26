/*
 * auto_id_nonce.h — the auto-ID boot nonce from a hardware device id.
 *
 * Internal to the transport subsystem (transport.c's hw_nonce()); a header
 * of its own so the native transport test can check it without linking a
 * radio.
 */

#ifndef TAPESTRY_AUTO_ID_NONCE_H
#define TAPESTRY_AUTO_ID_NONCE_H

#include <stddef.h>
#include <stdint.h>

/* FNV-1a over EVERY byte of the device id.  The first 4 bytes alone are
 * not unique: on STM32F4 they are the die's X/Y wafer coordinates, and
 * both cf21bl drones reported 0x36333233 — each dropped the other's
 * beacons as its own, and two drones powered on together took the same
 * id (flight 57).  On Ambiq Apollo5x only CHIPID0 and CHIPID1 together
 * identify a die.  0 sorts first in the ranking, so it is shifted to 1. */
static inline uint32_t auto_id_nonce_from_id(const uint8_t *id, size_t len)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= id[i];
        h *= 16777619u;
    }
    return h ? h : 1u;
}

#endif /* TAPESTRY_AUTO_ID_NONCE_H */
