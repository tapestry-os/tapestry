/*
 * zephyr/random/random.h — host stand-in for Zephyr's sys_rand32_get().
 *
 * gossip.c seeds each element's shared-fact epoch and Trickle jitter from
 * it.  Webots controllers are separate host processes, so mixing the pid
 * and the clock into a PRNG gives every controller (and every restart of
 * one) a different stream — all the facts layer needs, since the epoch only
 * has to differ between boots of the same element id.
 *
 * rand_r() because the controllers build with -D_POSIX_C_SOURCE=200809L,
 * which declares it but hides random()/srandom().
 */
#ifndef ZEPHYR_SHIM_RANDOM_H
#define ZEPHYR_SHIM_RANDOM_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

static inline uint32_t sys_rand32_get(void)
{
    static unsigned state;
    static int      seeded;

    if (!seeded) {
        struct timespec ts;

        clock_gettime(CLOCK_REALTIME, &ts);
        state = (unsigned)getpid() * 2654435761u ^ (unsigned)ts.tv_nsec ^
                (unsigned)ts.tv_sec;
        seeded = 1;
    }
    /* rand_r() yields at least 15 bits; take 11 from each of three calls. */
    uint32_t r = 0;

    for (int i = 0; i < 3; i++) {
        r = (r << 11) ^ ((uint32_t)rand_r(&state) & 0x7FFu);
    }
    return r ^ ((uint32_t)rand_r(&state) << 22);
}

#endif /* ZEPHYR_SHIM_RANDOM_H */
