/*
 * main.c — a robot in the "cloud"-coordinated comparison fleet (scenes
 * 2/3). See ../cloud_common/cloud_protocol.h for what this stand-in is
 * and is not, and why the honesty framing matters.
 *
 * DELIBERATELY THIN, AND THAT IS THE POINT. This file has no gossip, no
 * world model, no quorum, no peer awareness of any other robot — cloud or
 * Tapestry — whatsoever. It knows exactly one thing: the last target its
 * coordinator told it to drive to, and how long ago that was. That is an
 * honest reflection of what a robot in a centrally-coordinated architecture
 * actually is: an actuator at the edge of someone else's intelligence. Every
 * line of sophistication in ../rover/main.c (the Tapestry equivalent) is
 * sophistication this file deliberately does not have, because a Tapestry
 * element carries its own copy of the coordination stack and this one does
 * not.
 *
 * Reuses ../rover/substrate_webots.{c,h} (the differential-drive L1 HAL)
 * and ../rover/status_tx.{c,h} (the presentation-only telemetry feed) by
 * relative path, unmodified. Both are pure, Tapestry-stack-free utility
 * code — the first is a motor/sensor driver, the second a raw UDP struct
 * send with no coordination logic in it — so reusing them here is not
 * borrowing Tapestry's coordination, only its hardware plumbing and its
 * display plumbing, which is not the thing under comparison.
 *
 * Usage (set by controllerArgs, not run manually):
 *   cloud_bot <robot_id> <coord_x> <coord_y> <target_x> <target_y> <apply_deck 0|1>
 *
 *   coord_x/coord_y   the coordinator's fixed physical position (must match
 *                     its Robot node's translation in the .wbt — see
 *                     ../cloud_coordinator/main.c's matching comment).
 *   apply_deck        1 in scene 2 (the steel mezzanine deck can obstruct
 *                     this robot's link to the coordinator); 0 in scene 3
 *                     (no deck — the only way this robot loses its
 *                     coordinator there is the coordinator actually dying).
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <webots/robot.h>

#include "../rover/substrate_webots.h"
#include "../rover/status_tx.h"
#include "../cloud_common/cloud_protocol.h"
#include "cloud_occlusion.h"

/* Same physical constants as the Tapestry rover (0.5 x 0.34 m body) — this
 * is the identical WarehouseAMR PROTO, just a different controller and
 * bodyColor, so the steering tuning below is deliberately copied from
 * rover/main.c rather than re-derived: same platform, same numbers. */
#define ARRIVE_M   0.06f
#define TRACK_KP   1.5f
#define YAW_KP     1.4f
#define MAX_SPEED_FRAC  1.0f

#define COORD_CYCLE_MS  100u   /* control-loop cadence; independent of
                                * CLOUD_BROADCAST_MS (150 ms) — this robot's
                                * own tick rate, not the coordinator's send
                                * rate */

/* The supervisor's status socket is bound at a FIXED port derived from the
 * Tapestry rover's own base port (GOSSIP_BASE_PORT=5800 in rover/main.c,
 * +WH_STATUS_PORT_OFFSET), not from CLOUD_BASE_PORT — the readout is one
 * shared listener for both fleets. This must match rover/main.c's own
 * status_tx_init() argument exactly, or cloud fleet status silently never
 * arrives. */
#define ROVER_GOSSIP_BASE_PORT  5800u

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float wrap_pi(float a)
{
    while (a >  (float)M_PI) { a -= 2.0f * (float)M_PI; }
    while (a < -(float)M_PI) { a += 2.0f * (float)M_PI; }
    return a;
}

int main(int argc, char **argv)
{
    if (argc < 7) {
        fprintf(stderr, "usage: cloud_bot <id> <coord_x> <coord_y> "
                        "<target_x> <target_y> <apply_deck 0|1>\n");
        return -1;
    }
    int   robot_id    = atoi(argv[1]);
    float coord_x     = (float)atof(argv[2]);
    float coord_y     = (float)atof(argv[3]);
    float target_x    = (float)atof(argv[4]);
    float target_y    = (float)atof(argv[5]);
    bool  apply_deck  = atoi(argv[6]) != 0;

    if (substrate_init() != 0) {
        fprintf(stderr, "cloud_bot id=%d substrate_init failed\n", robot_id);
    }

    /* Receive socket for coordinator broadcasts — one fixed port per robot,
     * matching CLOUD_BASE_PORT + robot_id on the coordinator's send side. */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock >= 0) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port        = htons((uint16_t)(CLOUD_BASE_PORT + robot_id));
        int on = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            fprintf(stderr, "cloud_bot id=%d bind failed — no coordinator "
                            "link, will freeze immediately\n", robot_id);
            close(sock);
            sock = -1;
        } else {
            int flags = fcntl(sock, F_GETFL, 0);
            if (flags >= 0) { fcntl(sock, F_SETFL, flags | O_NONBLOCK); }
        }
    }

    status_tx_init(ROVER_GOSSIP_BASE_PORT);

    int timestep = (int)wb_robot_get_basic_time_step();
    double  coord_accum_ms = 0.0;
    uint32_t elapsed_ms    = 0;

    bool     have_directive     = false;
    float    accepted_target_x  = target_x;   /* pre-seeded with the initial
                                               * assignment so a robot that
                                               * never hears the coordinator
                                               * at all still has SOMETHING
                                               * to drive toward, exactly
                                               * like a real fleet robot
                                               * booted with a last-known
                                               * plan */
    float    accepted_target_y  = target_y;
    uint32_t last_accepted_ms   = 0;
    bool     frozen             = false;
    bool     printed_freeze     = false;

    while (wb_robot_step(timestep) != -1) {
        substrate_webots_step((double)timestep / 1000.0);

        coord_accum_ms += timestep;
        if (coord_accum_ms < (double)COORD_CYCLE_MS) {
            continue;
        }
        coord_accum_ms -= (double)COORD_CYCLE_MS;
        elapsed_ms += COORD_CYCLE_MS;

        float px, py, pz;
        substrate_webots_get_position(&px, &py, &pz);

        /* Drain every pending directive datagram, keeping only the latest —
         * same "read until genuinely empty" reasoning as
         * transceiver_udp_posix.c's rx loop, so a burst of buffered packets
         * can't starve delivery of the most recent one. */
        if (sock >= 0) {
            for (;;) {
                cloud_directive_t d;
                ssize_t n = recvfrom(sock, &d, sizeof(d), 0, NULL, NULL);
                if (n < 0) { break; }
                if (n != (ssize_t)sizeof(d) || d.magic != CLOUD_PROTOCOL_MAGIC ||
                    d.robot_id != (uint8_t)robot_id) {
                    continue;
                }
                /* THE occlusion decision: is the straight line from where I
                 * am RIGHT NOW to the coordinator's known fixed position
                 * clear? Evaluated at receive time, using this robot's own
                 * live position — not the coordinator's belief about where
                 * I am, which it doesn't even track (see
                 * cloud_protocol.h's one-way design note). */
                bool blocked = apply_deck &&
                              cloud_link_obstructed(px, py, coord_x, coord_y);
                if (blocked) {
                    continue;   /* the deck ate it — treat as never arrived */
                }
                accepted_target_x = d.target_x;
                accepted_target_y = d.target_y;
                last_accepted_ms  = elapsed_ms;
                have_directive    = true;
            }
        }

        /* Freeze once the grace period since the last ACCEPTED directive
         * exceeds CLOUD_COORD_TIMEOUT_MS. This is the one piece of local
         * intelligence this robot has, and it is a safety stop, not a
         * recovery mechanism — nothing here ever un-freezes itself; only a
         * resumed, unobstructed stream of directives can, and only because
         * that happens to look like "coming back online" from the
         * outside. */
        uint32_t since_accepted = have_directive
            ? (elapsed_ms - last_accepted_ms) : 0xFFFFFFFFu;
        frozen = !have_directive || since_accepted > CLOUD_COORD_TIMEOUT_MS;

        if (frozen && !printed_freeze) {
            printf("cloud_bot id=%d FROZEN at t=%.1fs (pos %.2f,%.2f) — "
                   "no coordinator directive in %ums; nothing here can "
                   "resume without one\n",
                   robot_id, (double)elapsed_ms * 0.001, (double)px,
                   (double)py, (unsigned)CLOUD_COORD_TIMEOUT_MS);
            printed_freeze = true;
        } else if (!frozen && printed_freeze) {
            printf("cloud_bot id=%d coordinator link restored at t=%.1fs\n",
                   robot_id, (double)elapsed_ms * 0.001);
            printed_freeze = false;
        }

        substrate_twist_t sp = {0};
        if (!frozen) {
            float ex   = accepted_target_x - px;
            float ey   = accepted_target_y - py;
            float dist = sqrtf(ex * ex + ey * ey);
            if (dist > ARRIVE_M) {
                float yaw     = substrate_webots_get_yaw();
                float yaw_err = wrap_pi(atan2f(ey, ex) - yaw);
                float align   = cosf(yaw_err);
                sp.angular.z = clampf(YAW_KP * yaw_err, -1.0f, 1.0f);
                sp.linear.x  = (align > 0.0f)
                               ? clampf(dist * TRACK_KP, 0.0f, MAX_SPEED_FRAC) * align
                               : 0.0f;
            }
        }
        substrate_move(&sp);

        wh_status_t st = {0};
        st.magic      = WH_STATUS_MAGIC;
        st.element_id = (uint8_t)robot_id;
        st.fleet      = WH_FLEET_CLOUD;
        /* Tapestry-only fields: explicit "n/a" sentinel rather than a
         * misleading 0 — a hub-and-spoke robot has no quorum, role, swarm
         * size, task slot, or script step to report, and printing zeros for
         * those would look like real (if degenerate) values instead of
         * "this concept doesn't apply here". See status_tx.h. */
        st.quorum     = 0xFFu;
        st.role       = 0xFFu;
        st.swarm_size = 0xFFu;
        st.task_slot  = 0xFFu;
        st.step       = 0xFFu;
        if (frozen) { st.flags |= WH_STATUS_F_NO_COORD; }
        st.x = px;
        st.y = py;
        status_tx_send(&st);
    }

    if (sock >= 0) { close(sock); }
    return 0;
}
