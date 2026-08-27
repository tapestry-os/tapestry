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

/* clock_gettime()/CLOCK_MONOTONIC below are POSIX, not ISO C, and glibc
 * hides them under __STRICT_ANSI__ (any -std=c99 build without a POSIX
 * feature-test macro).  The failure is otherwise an opaque "storage size
 * of 'ts' isn't known" pointing at a line that looks perfectly ordinary —
 * name the actual cause instead.  Tested for after <time.h>, so a build
 * that reaches POSIX by any route (-D_POSIX_C_SOURCE, -std=gnu*, Webots'
 * own default flags, Apple libc) passes without having to enumerate them. */
#ifndef CLOCK_MONOTONIC
#error "zephyr_shim/kernel.h needs POSIX clocks: build with -D_POSIX_C_SOURCE=200809L (see examples/webots-formation/ci-check/Makefile) or -std=gnu99"
#endif

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
