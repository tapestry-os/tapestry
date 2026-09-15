/*
 * rf_occlusion.c — see rf_occlusion.h
 */

#include "rf_occlusion.h"
#include "transceiver_udp_posix.h"

#include <string.h>
#include <tapestry/wire.h>

/*
 * This element's own live position, refreshed every coordination tick by
 * rf_occlusion_set_own_position(). The receive filter runs inside
 * gossip_drain() and cannot be passed context, so it reads this.
 *
 * Note what is NOT here any more: a table of remembered peer positions. An
 * earlier version masked TRANSMISSIONS using the last position it had heard
 * from each peer, and that could not work — see the long note on
 * udp_posix_set_rx_filter() in transceiver_udp_posix.h. Both endpoints of
 * this predicate are now true, live positions: ours from our own GPS, the
 * sender's from the frame in hand.
 */
static position_t s_own_pos;
static bool       s_enabled;

/* Peers whose frames were dropped since the last rf_occlusion_blocked_count()
 * call. Display only — nothing in the model reads it. */
static uint32_t s_dropped;

/*
 * Which RF cell a point is in. The deck's own footprint is a cell of its
 * own (UNDER) rather than a boundary, and that is load-bearing — see
 * rf_link_obstructed() below.
 */
typedef enum {
    RF_SIDE_SOUTH = 0,
    RF_SIDE_UNDER = 1,
    RF_SIDE_NORTH = 2,
} rf_side_t;

static rf_side_t rf_side(const position_t *p)
{
    if (p->y < RF_DECK_Y_MIN) { return RF_SIDE_SOUTH; }
    if (p->y > RF_DECK_Y_MAX) { return RF_SIDE_NORTH; }
    return RF_SIDE_UNDER;
}

bool rf_link_obstructed(const position_t *a, const position_t *b)
{
    /*
     * Different cells means the steel is in the way. Treating UNDER as its
     * own opaque cell — rather than as transparent, which an earlier version
     * did — is what makes the partition SYMMETRIC, and the reason is worth
     * recording because the failure it fixes is not obvious.
     *
     * This predicate is evaluated by each element against its OWN live
     * position and its LAST KNOWN position for the peer. A peer's last known
     * position is, by construction, the last one it broadcast before it
     * became unreachable — which is the moment it entered the deck's
     * footprint. If the footprint counts as transparent, that frozen sample
     * sits exactly where the predicate reports "clear", so the element on the
     * near side never starts blocking, keeps transmitting, and keeps being
     * heard. The far side blocks, the near side does not, and scene 2 ran
     * with a stable ONE-WAY link: the north island reported swarm_size 8
     * while the south island reported 3.
     *
     * Making the footprint opaque puts that frozen sample in a cell that
     * differs from either open side, so both ends block and the partition is
     * clean. Healing is unaffected and still works for the right reason:
     * whichever element physically crosses back evaluates its OWN live
     * position into the far cell, unblocks, transmits, and is heard — at
     * which point the other side refreshes its stale sample and unblocks too.
     *
     * It is also the better physical model. A robot parked under a steel
     * mezzanine deck is shadowed from everything outside it, and two robots
     * both under it can see each other; that is exactly what this says.
     */
    if (rf_side(a) == rf_side(b)) {
        return false;
    }

    /*
     * Different cells, so the link is obstructed only if it passes through
     * the deck rather than around one of its ends. Tested as an overlap
     * between the link's X span and the deck's, which is deliberately
     * CONSERVATIVE: a link that clips past a corner is called blocked. The
     * alternative — solving the segment for X at the deck's midline — is
     * exact for two endpoints on opposite open sides but degenerate when one
     * of them is UNDER the deck (the midline may not lie between them at
     * all, and the solution extrapolates off the segment). Since the deck
     * spans 20 m of a 24 m floor, the two agree everywhere any scene
     * actually operates, and the robust one is the better trade.
     */
    float lo = a->x < b->x ? a->x : b->x;
    float hi = a->x < b->x ? b->x : a->x;

    return (hi >= RF_DECK_X_MIN && lo <= RF_DECK_X_MAX);
}

/* The receive predicate handed to the transceiver. Runs per datagram. */
static bool rf_rx_filter(const uint8_t *data, uint16_t len)
{
    if (!s_enabled) {
        return true;
    }
    /* This transport puts one gossip frame's bytes directly in the datagram
     * with no tapestry_msg_header_t (see transceiver_udp_posix.h), so a
     * full-size datagram is exactly a tapestry_gossip_frame_t. Anything else
     * is passed through untouched rather than guessed at — the RF model has
     * no business dropping a frame it cannot parse. */
    if (len < (uint16_t)sizeof(tapestry_gossip_frame_t)) {
        return true;
    }

    tapestry_gossip_frame_t f;
    memcpy(&f, data, sizeof(f));

    position_t sender = { f.x, f.y, f.z };
    if (!rf_link_obstructed(&s_own_pos, &sender)) {
        return true;
    }

    if (f.id < 32u) {
        s_dropped |= (1u << f.id);
    }
    return false;   /* the steel is in the way */
}

void rf_occlusion_configure(bool enabled)
{
    s_enabled = enabled;
    /* Installed unconditionally so that enabling is a pure data change; the
     * filter itself short-circuits when disabled. */
    udp_posix_set_rx_filter(rf_rx_filter);
}

void rf_occlusion_set_own_position(const position_t *p)
{
    if (p != NULL) {
        s_own_pos = *p;
    }
}

uint8_t rf_occlusion_blocked_count(void)
{
    uint8_t n = 0;
    for (int i = 0; i < 32; i++) {
        if (s_dropped & (1u << i)) { n++; }
    }
    s_dropped = 0u;
    return n;
}
