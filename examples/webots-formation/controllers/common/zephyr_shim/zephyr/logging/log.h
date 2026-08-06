/*
 * zephyr_shim/zephyr/logging/log.h — fake header, NOT part of Zephyr.
 *
 * See ../kernel.h for why this shim exists. Only the logging macros
 * gossip.c actually uses are provided; all levels route to stderr so they
 * interleave correctly with the printf() lines main.c writes to stdout.
 */

#ifndef TAPESTRY_WEBOTS_ZEPHYR_SHIM_LOG_H
#define TAPESTRY_WEBOTS_ZEPHYR_SHIM_LOG_H

#include <stdio.h>

#define LOG_LEVEL_ERR 1
#define LOG_LEVEL_WRN 2
#define LOG_LEVEL_INF 3
#define LOG_LEVEL_DBG 4

#define LOG_MODULE_REGISTER(name, level) \
    static const char *_log_module_##name##_unused __attribute__((unused)) = #name

#define LOG_ERR(fmt, ...) fprintf(stderr, "<err> " fmt "\n", ##__VA_ARGS__)
#define LOG_WRN(fmt, ...) fprintf(stderr, "<wrn> " fmt "\n", ##__VA_ARGS__)
#define LOG_INF(fmt, ...) fprintf(stderr, "<inf> " fmt "\n", ##__VA_ARGS__)
#define LOG_DBG(fmt, ...) fprintf(stderr, "<dbg> " fmt "\n", ##__VA_ARGS__)

#endif /* TAPESTRY_WEBOTS_ZEPHYR_SHIM_LOG_H */
