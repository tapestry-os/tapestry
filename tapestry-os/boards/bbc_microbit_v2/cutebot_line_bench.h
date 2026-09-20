/*
 * cutebot_line_bench.h — raw line-sensor IR bring-up diagnostic.
 *
 * NOT the real line-sensor driver (see cutebot_line.h for that) — this
 * exists purely to check whether the sensors toggle in response to a
 * real dark/light change at all, isolating a possible sensor/wiring/
 * power problem from anything about driving dynamics or crossing
 * timing.
 *
 * Originally logged four pins (P11-P14) to also settle which pins are
 * real, after a report that a physical unit might have four IR modules
 * wired instead of the two (P13/P14) ELECFREAKS' own pxt-cutebot source
 * documents. Confirmed on the bench: P11/P12 never toggle — only P13/P14
 * are real, matching pxt-cutebot and cutebot_line.c's own driver. Trimmed
 * back to those two.
 *
 * Deliberately different from cutebot_line.c in every way that matters
 * for a bring-up tool: no interrupts (plain polled reads — nothing here
 * needs to catch a brief transition, a human is watching it live),
 * RAW levels, not inverted to "true == line" (see cutebot_line.c's
 * comment on that inversion) — this reports exactly what the pin
 * electrically reads, on purpose, so a polarity assumption bug can't
 * hide itself the way it could behind the real driver's interpretation.
 */

#ifndef TAPESTRY_CUTEBOT_LINE_BENCH_H
#define TAPESTRY_CUTEBOT_LINE_BENCH_H

/*
 * Raw GPIO level per pin: 1 or 0 exactly as gpio_pin_get_dt() returns
 * it, no inversion, no "line means X" interpretation at all.
 */
typedef struct {
    int p13;
    int p14;
} cutebot_line_bench_sample_t;

/* Configure both pins as plain inputs, no pull (matches pxt-cutebot's
 * PullNone — see bbc_microbit_v2.overlay), no interrupt. Returns 0 on
 * success, negative errno if either GPIO device isn't ready. */
int cutebot_line_bench_init(void);

/* Read both pins' current raw levels. */
void cutebot_line_bench_read(cutebot_line_bench_sample_t *out);

#endif /* TAPESTRY_CUTEBOT_LINE_BENCH_H */
