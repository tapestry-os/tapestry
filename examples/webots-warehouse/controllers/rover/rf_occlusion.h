/*
 * rf_occlusion.h — physical RF obstruction model (scene 2)
 *
 * WHAT THIS IS. worlds/scene2_partition_vs_cloud.wbt (scene 2 runs
 * exclusively as its cloud-comparison variant) contains a steel mezzanine
 * deck spanning the middle of the warehouse, raised on legs so AMRs drive
 * UNDERNEATH it freely. It is opaque to 2.4 GHz. Two elements on opposite
 * sides of it cannot hear each other; two on the same side can.
 *
 * WHY IT IS BUILT THIS WAY. The obvious way to demonstrate partition
 * tolerance is to inject a fault on a timer. This does not do that, and the
 * difference is the point of the scene: here the partition is a CONSEQUENCE
 * OF THE FLEET DOING ITS JOB. The script's `move` step sends part of the
 * formation to pick faces on the far side of the deck; the fleet splits
 * because it went where it was told to go. Nothing in the script, the
 * world, or the supervisor knows a partition is about to happen.
 *
 * WHERE THE DECISION IS MADE. Entirely locally, in each element, with no
 * coordinator and no privileged view:
 *
 *   - The deck's geometry is a compile-time constant here, the same way a
 *     real robot would carry a site map.
 *   - Its OWN position comes from its own GPS.
 *   - The SENDER's position comes out of the gossip frame being delivered,
 *     which already carries it (tapestry_gossip_frame_t, wire.h).
 *
 * so an element decides "did that transmission reach me" from information it
 * genuinely has, at the moment it would have arrived.
 *
 * IT IS A RECEIVE FILTER, AND THAT MATTERS. The intuitive place to model an
 * obstruction is at the transmitter — the frame never leaves. That version
 * was written first and does not work, for a reason worth recording: a
 * transmitter can only consult what it last HEARD about a peer, and that
 * belief freezes at the instant the peer became unreachable, which is while
 * the peer was still on the near side. The element that stayed put therefore
 * never starts blocking, keeps transmitting, and keeps being heard. Scene 2
 * ran with a stable ONE-WAY link — the north island reporting swarm_size 8
 * while the south island reported 3 — through two separate attempts to fix
 * it from the transmit side.
 *
 * Obstruction is a function of where BOTH endpoints actually are, and the
 * receiver is the only party that holds both of them as live truth. Hence
 * udp_posix_set_rx_filter() rather than a transmit mask.
 *
 * SCOPE. This is a hard binary geometric model — no path loss, no fading,
 * no partial delivery, no antenna pattern. It exists to produce a clean,
 * reproducible, physically-motivated partition, not to be an RF simulator.
 */

#ifndef TAPESTRY_WAREHOUSE_RF_OCCLUSION_H
#define TAPESTRY_WAREHOUSE_RF_OCCLUSION_H

#include <stdbool.h>
#include <stdint.h>
#include <tapestry/csm.h>

/*
 * Deck footprint in world-frame meters. The band runs along X; a link is
 * obstructed when it crosses the band in Y within the X extent.
 * Must match the Solid in worlds/scene2_partition_vs_cloud.wbt.
 */
#define RF_DECK_X_MIN  (-10.0f)
#define RF_DECK_X_MAX  ( 10.0f)
#define RF_DECK_Y_MIN  (  1.4f)
#define RF_DECK_Y_MAX  (  2.6f)

/*
 * Install the receive filter. Call once at startup. enabled == false leaves
 * the model inert (scenes 1 and 3 have no deck), so one binary runs every
 * scene and the RF model only bites where the world actually contains the
 * structure.
 */
void rf_occlusion_configure(bool enabled);

/* Publish this element's own live position for the filter to use. Call once
 * per coordination tick, BEFORE gossip_drain(). */
void rf_occlusion_set_own_position(const position_t *p);

/* How many distinct peers have had frames dropped since the last call.
 * Display and logging only — nothing in the model reads it. Clears on read. */
uint8_t rf_occlusion_blocked_count(void);

/* True if a link between the two points is obstructed by the deck.
 *
 * The deck's own footprint counts as a THIRD cell that is opaque to both
 * open sides — a peer under the deck is unreachable from outside it, and two
 * elements both under it can hear each other. That is not a detail: it is
 * what keeps the partition symmetric when one endpoint is a frozen
 * last-known sample taken at the moment the peer went quiet. See the long
 * comment on the implementation.
 *
 * Exposed for the unit check in ../../ci-check/. */
bool rf_link_obstructed(const position_t *a, const position_t *b);

#endif /* TAPESTRY_WAREHOUSE_RF_OCCLUSION_H */
