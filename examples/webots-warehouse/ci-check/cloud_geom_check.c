/*
 * cloud_geom_check.c — assertions on the cloud-fleet comparison scenes'
 * geometry (scenes 2/3).
 *
 * THE BUG THIS EXISTS TO CATCH. The first build of scene 2 placed the
 * cloud fleet's northbound crossing path at x = 6/7/8/9 m. The deck's east
 * support leg sits at x = 9.5 m (half-width 0.15 m); the AMR body is 0.5 m
 * long (half-length 0.25 m). x=9 left only ~0.25 m of clearance to the
 * leg's near face — not enough — and the robot at x=9 physically wedged
 * against the leg mid-crossing, stalled at (9.3, 1.15) for the rest of the
 * run. It never showed up as an RF-occlusion problem (it never even
 * reached the deck's shadow); it looked, from the coordinator's messages
 * alone, exactly like a working link to a robot that had simply stopped
 * moving. That combination — silent, physically caused, and
 * indistinguishable from the intended demonstration by log inspection
 * alone — is exactly the kind of thing worth an automated geometry check
 * rather than re-discovering by watching a robot stall in a live run.
 *
 * This also checks cloud_occlusion.c's geometry against rf_occlusion.c's
 * (the Tapestry fleet's own RF model) for actual functional agreement, not
 * just eyeballed matching constants — the two are deliberately independent
 * source files (see cloud_occlusion.h's header comment for why), so
 * nothing at compile time stops them from silently drifting apart.
 */

#include <stdio.h>

#include "cloud_occlusion.h"
#include "rf_occlusion.h"

static int failures;

static void check(bool cond, const char *what)
{
    printf("  %-58s %s\n", what, cond ? "ok" : "FAIL");
    if (!cond) { failures++; }
}

int main(void)
{
    /* Cloud fleet geometry, matching worlds/scene2_partition_vs_cloud.wbt:
     * x in {1,2,3,4,5,6,7,8}, start y=-6, coordinator at (6.5, -7.5) -- 8
     * robots, apples-to-apples with the 8-element Tapestry fleet. Deck
     * support legs at x=+-9.5 (half-width 0.15), y in [1.4, 2.6] (matching
     * rf_occlusion.h's Y band). scene3_failover_vs_cloud.wbt has its own,
     * unrelated orbit-based layout -- it has no deck to clear, so this
     * check's leg-clearance/deck-crossing assertions don't apply there;
     * see the ring/orbit check further down for that scene's own geometry
     * invariant. */
    const float cloud_x[]  = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f};
    const float leg_x[]    = {-9.5f, 9.5f};
    const float leg_half_w = 0.15f;
    const float body_half  = 0.25f;   /* AMR is 0.5 m long */
    const float margin     = 0.3f;

    puts("scene 2 — cloud fleet crossing path clears the deck's support legs:");
    float worst = 1e9f;
    for (unsigned i = 0; i < sizeof(cloud_x) / sizeof(cloud_x[0]); i++) {
        for (unsigned j = 0; j < 2; j++) {
            float clearance = (cloud_x[i] > leg_x[j] ? cloud_x[i] - leg_x[j]
                                                      : leg_x[j] - cloud_x[i])
                              - leg_half_w - body_half;
            if (clearance < worst) { worst = clearance; }
        }
    }
    printf("  %-58s %.2f m\n", "tightest clearance from any cloud bot to either leg",
           (double)worst);
    check(worst >= margin, "clearance meets the 0.3 m margin");

    puts("\nscene 2 — the cloud fleet's crossing must still actually cross the deck's shadow:");
    /* If this ever fails, the comparison has gone silent for the opposite
     * reason — the fleet would sail straight through with no RF story to
     * tell at all. */
    bool clear_start = !cloud_link_obstructed(cloud_x[0], -6.0f, 6.5f, -7.5f);
    bool blocked_mid = cloud_link_obstructed(cloud_x[0], 2.0f, 6.5f, -7.5f);
    check(clear_start, "clear before the deck (both south of it)");
    check(blocked_mid, "obstructed while under the deck");

    puts("\ncloud_occlusion.c vs rf_occlusion.c — same predicate, independent files:");
    /* A grid of representative points around the deck band, checked against
     * BOTH implementations — they must agree everywhere, not just at the
     * points either file's own test happens to probe. */
    bool all_agree = true;
    for (float ay = -3.0f; ay <= 5.0f; ay += 0.5f) {
        for (float by = -3.0f; by <= 5.0f; by += 0.5f) {
            position_t a = {0.0f, ay, 0.0f};
            position_t b = {0.0f, by, 0.0f};
            bool tapestry_says = rf_link_obstructed(&a, &b);
            bool cloud_says    = cloud_link_obstructed(a.x, a.y, b.x, b.y);
            if (tapestry_says != cloud_says) {
                printf("  disagreement at ay=%.1f by=%.1f: tapestry=%d cloud=%d\n",
                       (double)ay, (double)by, tapestry_says, cloud_says);
                all_agree = false;
            }
        }
    }
    check(all_agree, "agree at every sampled point along the Y axis");

    /* Scene 3: Tapestry ring at (-5,0) r=4 and the cloud fleet's orbit at
     * (5,0) r=4, matching worlds/scene3_failover_vs_cloud.wbt and
     * scene3-failover.choreo.toml. Neither scene has a physical deck, so
     * this isn't an occlusion check — it's the same class of bug as the
     * leg-clearance one above: two independently-authored geometries (a
     * Choreo script's FORM/CIRCLE target and a coordinator's orbit center)
     * that must not overlap, with nothing at compile time linking them. */
    puts("\nscene 3 — the Tapestry ring and the cloud orbit must not overlap:");
    float tap_cx = -5.0f, tap_cy = 0.0f, tap_r = 4.0f;
    float cloud_cx = 5.0f, cloud_cy = 0.0f, cloud_r = 4.0f;
    float center_dist = sqrtf((cloud_cx - tap_cx) * (cloud_cx - tap_cx) +
                              (cloud_cy - tap_cy) * (cloud_cy - tap_cy));
    float gap = center_dist - tap_r - cloud_r;
    printf("  %-58s %.2f m\n", "gap between the two rings", (double)gap);
    check(gap >= 0.5f, "gap is at least 0.5 m (comfortable AMR clearance)");

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
