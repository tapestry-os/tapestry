/*
 * rf_geom_check.c — assertions on the scene 2 RF partition geometry.
 *
 * Unlike the link check next to it (which is compiled and thrown away),
 * this one RUNS. It exists because scene 2's whole claim — "the fleet
 * splits 3/5 because of where the script sends it" — is a geometric
 * coincidence between three files that have no compile-time relationship:
 * the deck footprint in rf_occlusion.h, the FORM/MOVE targets in
 * scene2-partition.choreo.toml, and the deck Solid in
 * worlds/scene2_partition_vs_cloud.wbt (scene 2 runs exclusively as its
 * cloud-comparison variant — there is no standalone scene2_partition.wbt
 * any more). Nudge any one of them and the scene quietly
 * stops demonstrating anything — the fleet stays connected, or splits in a
 * way nobody intended, and the only symptom is a less interesting movie.
 *
 * So the positions the script actually produces are recomputed here from
 * bse.c's own layout math and checked against the occlusion model.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "rf_occlusion.h"

/* rf_occlusion.c pulls in the transceiver for udp_posix_set_rx_filter();
 * this check only exercises the pure geometry predicate. */

#define N_ELEMENTS 8

/* Mirrors bse.c's TAPESTRY_BSE_SHAPE_GRID case. */
static void form_grid(int count, float radius, float cx, float cy,
                      position_t out[])
{
    int cols = (int)ceilf(sqrtf((float)count));
    int rows = (int)ceilf((float)count / (float)cols);
    for (int r = 0; r < count; r++) {
        out[r].x = cx + ((float)(r % cols) - 0.5f * (float)(cols - 1)) * radius;
        out[r].y = cy + ((float)(r / cols) - 0.5f * (float)(rows - 1)) * radius;
        out[r].z = 0.0f;
    }
}

/* Mirrors bse.c's TAPESTRY_BSE_INTENT_MOVE case: each element's offset from
 * the participant centroid is snapshot at activation, then re-applied about
 * the new target. */
static void move_formation(int count, const position_t in[],
                           float tx, float ty, position_t out[])
{
    float cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < count; i++) { cx += in[i].x; cy += in[i].y; }
    cx /= (float)count;
    cy /= (float)count;
    for (int i = 0; i < count; i++) {
        out[i].x = tx + (in[i].x - cx);
        out[i].y = ty + (in[i].y - cy);
        out[i].z = 0.0f;
    }
}

static int failures;

static void check(bool cond, const char *what)
{
    printf("  %-58s %s\n", what, cond ? "ok" : "FAIL");
    if (!cond) { failures++; }
}

int main(void)
{
    position_t formed[N_ELEMENTS], moved[N_ELEMENTS];

    /* scene2-partition.choreo.toml step 2, then step 3. */
    form_grid(N_ELEMENTS, 3.0f, -6.0f, -4.0f, formed);
    move_formation(N_ELEMENTS, formed, 0.0f, 3.2f, moved);

    puts("scene 2 — after the FORM step, the fleet must be fully connected:");
    int obstructed_at_dock = 0;
    for (int a = 0; a < N_ELEMENTS; a++) {
        for (int b = a + 1; b < N_ELEMENTS; b++) {
            if (rf_link_obstructed(&formed[a], &formed[b])) {
                obstructed_at_dock++;
            }
        }
    }
    check(obstructed_at_dock == 0, "no obstructed links while formed at the dock");

    puts("\nscene 2 — after the MOVE step, the fleet must split 3 south / 5 north:");
    int south = 0, north = 0;
    for (int i = 0; i < N_ELEMENTS; i++) {
        if (moved[i].y < RF_DECK_Y_MIN)      { south++; }
        else if (moved[i].y > RF_DECK_Y_MAX) { north++; }
    }
    check(south == 3, "exactly 3 elements end south of the deck");
    check(north == 5, "exactly 5 elements end north of the deck");
    check(south + north == N_ELEMENTS, "no element parks inside the deck band");

    /* Every cross-island pair blocked, every same-island pair clear — this
     * is what makes it a clean partition rather than a lossy mess. */
    bool all_cross_blocked = true, all_same_clear = true;
    for (int a = 0; a < N_ELEMENTS; a++) {
        for (int b = 0; b < N_ELEMENTS; b++) {
            if (a == b) { continue; }
            bool same_side = (moved[a].y < RF_DECK_Y_MIN) ==
                             (moved[b].y < RF_DECK_Y_MIN);
            bool blocked = rf_link_obstructed(&moved[a], &moved[b]);
            if (same_side  &&  blocked) { all_same_clear   = false; }
            if (!same_side && !blocked) { all_cross_blocked = false; }
        }
    }
    check(all_cross_blocked, "every cross-island link is obstructed");
    check(all_same_clear,    "every same-island link is clear");

    /* The margin is what makes the split survive tracking error. */
    float worst = 1e9f;
    for (int i = 0; i < N_ELEMENTS; i++) {
        float m = (moved[i].y < RF_DECK_Y_MIN)
                  ? RF_DECK_Y_MIN - moved[i].y
                  : moved[i].y - RF_DECK_Y_MAX;
        if (m < worst) { worst = m; }
    }
    printf("  %-58s %.3f m\n", "tightest clearance to the deck band", (double)worst);
    check(worst > 0.5f, "clearance exceeds 0.5 m of tracking slack");

    /* The frozen-sample case that produced a one-way partition: an element
     * on an open side, and a peer whose last known position is UNDER the
     * deck because that is where it stopped being audible. Both ends must
     * agree the link is down, or the partition is asymmetric. */
    puts("\nRF model — a peer last seen UNDER the deck is unreachable:");
    position_t open_s = { 0.0f, -1.0f, 0.0f };
    position_t under  = { 0.0f,  2.0f, 0.0f };   /* inside [1.4, 2.6] */
    position_t open_n = { 0.0f,  5.0f, 0.0f };
    check(rf_link_obstructed(&open_s, &under), "south element cannot reach a peer under the deck");
    check(rf_link_obstructed(&under, &open_s), "and that is symmetric");
    check(rf_link_obstructed(&open_n, &under), "north element cannot reach it either");
    position_t under2 = { 3.0f, 2.2f, 0.0f };
    check(!rf_link_obstructed(&under, &under2), "two elements both under the deck can hear each other");

    /* The link must reach AROUND the ends of the deck, or the model is just
     * "different y = blocked" wearing a costume. */
    puts("\nRF model sanity — the deck has ends:");
    position_t w_s = { -15.0f, -3.0f, 0.0f }, w_n = { -15.0f, 6.0f, 0.0f };
    check(!rf_link_obstructed(&w_s, &w_n), "a link clear of the deck's X extent is not blocked");
    position_t c_s = { 0.0f, -3.0f, 0.0f }, c_n = { 0.0f, 6.0f, 0.0f };
    check(rf_link_obstructed(&c_s, &c_n), "a link straight through the deck is blocked");
    check(rf_link_obstructed(&c_n, &c_s), "obstruction is symmetric");

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
