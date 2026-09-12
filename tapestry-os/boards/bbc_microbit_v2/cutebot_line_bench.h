/*
 * cutebot_line_bench.h — raw 4-pin IR sensor bring-up diagnostic.
 *
 * NOT the real line-sensor driver (see cutebot_line.h for that) — this
 * exists purely to answer two open hardware questions from the bench:
 * (1) does ANY of these pins actually toggle in response to a real
 * dark/light change at all (isolating a possible sensor/wiring/power
 * problem from anything about driving dynamics or crossing timing), and
 * (2) a report that FOUR IR modules might be wired (P11-P14), not just
 * the two (P13/P14) ELECFREAKS' own pxt-cutebot source documents.
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
    int p11;
    int p12;
    int p13;
    int p14;
} cutebot_line_bench_sample_t;

/* Configure all four pins as plain inputs, no pull (matches pxt-cutebot's
 * PullNone — see bbc_microbit_v2.overlay), no interrupt. Returns 0 on
 * success, negative errno if any GPIO device isn't ready. */
int cutebot_line_bench_init(void);

/* Read all four pins' current raw levels. */
void cutebot_line_bench_read(cutebot_line_bench_sample_t *out);

#endif /* TAPESTRY_CUTEBOT_LINE_BENCH_H */
