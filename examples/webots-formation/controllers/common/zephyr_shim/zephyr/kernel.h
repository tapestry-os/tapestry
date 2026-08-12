/*
 * zephyr_shim/zephyr/kernel.h — fake header, NOT part of Zephyr.
 *
 * tapestry-os/subsys/transport/gossip.c is written against Zephyr and
 * includes <zephyr/kernel.h> unconditionally. This shim satisfies that
 * include with a plain POSIX clock so gossip.c compiles completely
 * unmodified into this Webots controller (a plain host C build, no
 * Zephyr/west involved). Only the symbols gossip.c actually references
 * are provided — see examples/webots-formation/README.md. This shim is
 * shared by every substrate in this example, not cf21bl-specific.
 */

#ifndef TAPESTRY_WEBOTS_ZEPHYR_SHIM_KERNEL_H
#define TAPESTRY_WEBOTS_ZEPHYR_SHIM_KERNEL_H

#include <stdint.h>
#include <time.h>

/* IS_ENABLED(CONFIG_TAPESTRY_MESH_RELAY) in gossip.c: this build never
 * defines CONFIG_TAPESTRY_MESH_RELAY, so it is always false. */
#define IS_ENABLED(macro) 0

static inline uint32_t k_uptime_get_32(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
}

static inline int64_t k_uptime_get(void)
{
    return (int64_t)k_uptime_get_32();
}

#endif /* TAPESTRY_WEBOTS_ZEPHYR_SHIM_KERNEL_H */
