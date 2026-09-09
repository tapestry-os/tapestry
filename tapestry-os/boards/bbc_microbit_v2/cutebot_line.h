/*
 * cutebot_line.h — ELECFREAKS Cutebot Mini ground-facing line-tracking
 * sensor driver (edge-connector GPIO, not the I2C motor/LED bus).
 *
 * Pins, signal convention and the two independent sources that confirm
 * them (ELECFREAKS' own pxt-cutebot MakeCode source, cross-checked
 * against this Zephyr tree's bbc_microbit_v2.dts edge-connector map) are
 * documented in bbc_microbit_v2.overlay, next to the devicetree nodes
 * this driver consumes.
 *
 * Interrupt-driven, not polled: WM_CYCLE_MS (the 100 ms main-loop period
 * every caller of this driver runs at) is far coarser than a line
 * crossing — at the fleet's measured ~238 mm/s and a 5 mm line, a
 * crossing lasts roughly 21 ms, so a 10 Hz poll would alias it into
 * noise or miss it outright (roughly 4 in 5 crossings would fall
 * entirely between two 100 ms samples). Both pins are therefore
 * interrupt-driven on both edges, counted in cutebot_line_isr_edges_t
 * independent of when the main loop happens to ask, and
 * cutebot_line_poll() is a read-and-clear over whatever accumulated
 * since the previous call — exact regardless of poll cadence.
 */

#ifndef TAPESTRY_CUTEBOT_LINE_H
#define TAPESTRY_CUTEBOT_LINE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * One poll's worth of line-sensor state.
 *
 *   left / right       — CURRENT level at the moment of the poll call:
 *                         true = line (dark) under that sensor right now,
 *                         false = no line (bright). Already inverted
 *                         from the raw LOW-means-line GPIO convention
 *                         (see the overlay comment) so callers never
 *                         have to think about sensor polarity.
 *   left_edges/right_edges — number of level CHANGES on that sensor
 *                         since the previous cutebot_line_poll() call,
 *                         cleared on read. A clean single perpendicular
 *                         line crossing produces exactly 2 (one falling,
 *                         one rising) on whichever sensor(s) passed over
 *                         the line; 0 means "no transition this cycle,"
 *                         not "no line under the sensor" (see `left`/
 *                         `right` for the latter).
 */
typedef struct {
    bool     left;
    bool     right;
    uint16_t left_edges;
    uint16_t right_edges;
} cutebot_line_sample_t;

/*
 * cutebot_line_init — configure both sensor pins as interrupt-driven
 * inputs. Returns 0 on success, negative errno if either GPIO device is
 * not ready or configuration fails.
 *
 * Must be called once before cutebot_line_poll(). Safe to call even if
 * bbc_microbit_v2.overlay's zephyr,user cutebot-line-{left,right}-gpios
 * properties are absent from a given build (returns -ENODEV) — callers
 * that don't need line sensing simply never call this.
 */
int cutebot_line_init(void);

/*
 * cutebot_line_poll — read current levels and the edge counts
 * accumulated since the last call, then clear those counts. Call once
 * per main-loop cycle, at any cadence — see this header's top comment
 * for why the cadence does not affect correctness.
 */
void cutebot_line_poll(cutebot_line_sample_t *out);

#endif /* TAPESTRY_CUTEBOT_LINE_H */
