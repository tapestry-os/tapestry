/*
 * main.c — the central "cloud" coordinator (scenes 2/3). See
 * ../cloud_common/cloud_protocol.h for what this stand-in is and is not.
 *
 * THE ENTIRE INTELLIGENCE OF THE CLOUD FLEET LIVES IN THIS ONE PROCESS.
 * Every robot's target is computed HERE, centrally, from a fixed plan
 * this process was launched with — never adapted to anything a robot
 * reports, because robots never report anything back (see
 * cloud_protocol.h's one-way design note). That is what makes this file so
 * short, and it is exactly why, in scene 3, killing this ONE process is
 * enough to stop every robot in the fleet: there is no other place holding
 * a copy of the plan, and no peer mechanism for the robots to improvise
 * one.
 *
 * WHY IT NEEDS NO KNOWLEDGE OF THE DECK. In scene 2 the steel mezzanine
 * deck can obstruct a robot's link to this coordinator — but that decision
 * is made entirely on the ROBOT's receiving side (cloud_bot/main.c),
 * using the robot's own live position. This process just broadcasts,
 * unconditionally, to every robot it knows about, on a fixed schedule,
 * whether or not any given broadcast actually arrives. It does not know,
 * and does not need to know, which robots can currently hear it — a real
 * cloud service usually can't tell either, until a robot's own liveness
 * report goes stale, and this protocol doesn't even have that channel (see
 * cloud_protocol.h).
 *
 * TWO MOTION MODES, BECAUSE THE TWO SCENES NEED GENUINELY DIFFERENT MOTION.
 * Both are pure functions of elapsed time and each robot's OWN starting
 * position — never of anything a robot reports back (there is no such
 * channel), which is what keeps this process this simple regardless of
 * which mode it runs.
 *
 *   MODE_WAYPOINT (0) — target = start + one of two fixed offsets,
 *     toggling every leg_ms. Scene 2 passes the SAME offset twice (north
 *     across the deck) — the toggle never visibly changes anything, which
 *     is exactly "go there once and stay", with no special case needed.
 *
 *   MODE_ORBIT (1) — target orbits a fixed center at a fixed radius and
 *     period; each robot's phase is derived from its OWN start position
 *     (phase = atan2(start_y - cy, start_x - cx)), so a robot staged
 *     exactly on the circle begins orbiting immediately with no initial
 *     "catching up" transient. Scene 3 uses this: a continuously moving
 *     target is what makes the coordinator's death read as an abrupt stop
 *     rather than nothing changing — robots already standing still
 *     staying standing still would prove nothing.
 *
 * Usage (set by controllerArgs, not run manually):
 *   cloud_coordinator <fail_at_s|-1> <mode> <mode-args...> \
 *       <n_robots> <id:x:y> [<id:x:y> ...]
 *
 *   mode 0 args:  <leg_ms> <dx0> <dy0> <dx1> <dy1>
 *                 the two offsets (world-frame meters), toggled every
 *                 leg_ms of simulated time.
 *   mode 1 args:  <cx> <cy> <radius> <period_s>
 *                 orbit center, radius (meters), and seconds per
 *                 revolution.
 *   <id:x:y>      one robot's id and START position. Must match that
 *                 robot's own cloud_bot controllerArgs and its
 *                 WarehouseAMR's initial translation in the .wbt. In
 *                 MODE_ORBIT this position also determines the robot's
 *                 orbital phase — see above.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <webots/robot.h>

#include "../rover/status_tx.h"
#include "../cloud_common/cloud_protocol.h"

#define MAX_ROBOTS  8

#define MODE_WAYPOINT 0
#define MODE_ORBIT    1

typedef struct {
    int   id;
    float start_x, start_y;
    float phase_rad;   /* MODE_ORBIT only; unused in MODE_WAYPOINT */
} robot_cfg_t;

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr, "usage: cloud_coordinator <fail_at_s|-1> <mode:0|1> "
                        "<mode-args...> <n_robots> <id:x:y> [...]\n");
        return -1;
    }
    float fail_at_s = (float)atof(argv[1]);
    int   mode      = atoi(argv[2]);

    /* Mode-specific fixed args, then <n_robots>, then that many "id:x:y"
     * tokens. arg_i tracks the next unconsumed argv index. */
    int   arg_i = 3;
    uint32_t leg_ms = 0;
    float dx0 = 0, dy0 = 0, dx1 = 0, dy1 = 0;
    float orbit_cx = 0, orbit_cy = 0, orbit_r = 0, orbit_period_s = 0;

    if (mode == MODE_WAYPOINT) {
        if (argc < arg_i + 5) {
            fprintf(stderr, "cloud_coordinator: mode 0 needs <leg_ms> <dx0> "
                            "<dy0> <dx1> <dy1>\n");
            return -1;
        }
        leg_ms = (uint32_t)atol(argv[arg_i++]);
        dx0 = (float)atof(argv[arg_i++]); dy0 = (float)atof(argv[arg_i++]);
        dx1 = (float)atof(argv[arg_i++]); dy1 = (float)atof(argv[arg_i++]);
        if (leg_ms == 0u) {
            fprintf(stderr, "cloud_coordinator: leg_ms must be nonzero\n");
            return -1;
        }
    } else if (mode == MODE_ORBIT) {
        if (argc < arg_i + 4) {
            fprintf(stderr, "cloud_coordinator: mode 1 needs <cx> <cy> "
                            "<radius> <period_s>\n");
            return -1;
        }
        orbit_cx = (float)atof(argv[arg_i++]);
        orbit_cy = (float)atof(argv[arg_i++]);
        orbit_r  = (float)atof(argv[arg_i++]);
        orbit_period_s = (float)atof(argv[arg_i++]);
        if (orbit_period_s <= 0.0f) {
            fprintf(stderr, "cloud_coordinator: period_s must be positive\n");
            return -1;
        }
    } else {
        fprintf(stderr, "cloud_coordinator: unknown mode %d (expected 0 or 1)\n",
                mode);
        return -1;
    }

    if (argc < arg_i + 1) {
        fprintf(stderr, "cloud_coordinator: missing <n_robots>\n");
        return -1;
    }
    int n_robots = atoi(argv[arg_i++]);
    if (n_robots < 1 || n_robots > MAX_ROBOTS || argc < arg_i + n_robots) {
        fprintf(stderr, "cloud_coordinator: bad robot count/arg list\n");
        return -1;
    }

    robot_cfg_t robots[MAX_ROBOTS];
    for (int i = 0; i < n_robots; i++) {
        char buf[64];
        strncpy(buf, argv[arg_i + i], sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        char *id_s = strtok(buf, ":");
        char *x_s  = strtok(NULL, ":");
        char *y_s  = strtok(NULL, ":");
        if (!id_s || !x_s || !y_s) {
            fprintf(stderr, "cloud_coordinator: bad id:x:y token %s\n",
                    argv[arg_i + i]);
            return -1;
        }
        robots[i].id      = atoi(id_s);
        robots[i].start_x = (float)atof(x_s);
        robots[i].start_y = (float)atof(y_s);
        robots[i].phase_rad = atan2f(robots[i].start_y - orbit_cy,
                                     robots[i].start_x - orbit_cx);
    }

    wb_robot_init();

    /* One send socket, reused for every robot's port — this is the
     * hub-and-spoke shape made literal: one source, N destinations, no mesh. */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        fprintf(stderr, "cloud_coordinator: socket() failed — every robot "
                        "will freeze immediately, forever\n");
    }

    status_tx_init(5800u);   /* must match the supervisor's fixed listening
                              * port — see cloud_bot/main.c's identical
                              * comment on ROVER_GOSSIP_BASE_PORT */

    int      timestep      = (int)wb_robot_get_basic_time_step();
    double   accum_ms      = 0.0;
    uint32_t elapsed_ms    = 0;
    bool     dead          = false;
    bool     printed_death = false;

    printf("cloud_coordinator: %d robots under central control, mode=%d%s\n",
           n_robots, mode, fail_at_s >= 0.0f ? " (scheduled to fail)" : "");

    while (wb_robot_step(timestep) != -1) {
        accum_ms += timestep;
        if (accum_ms < (double)CLOUD_BROADCAST_MS) {
            continue;
        }
        accum_ms -= (double)CLOUD_BROADCAST_MS;
        elapsed_ms += CLOUD_BROADCAST_MS;

        if (!dead && fail_at_s >= 0.0f &&
            (float)elapsed_ms * 0.001f >= fail_at_s) {
            dead = true;
        }

        if (dead) {
            /* Silence, not a goodbye — same philosophy as the Tapestry
             * rover's own hard-failure path (main.c): a real outage doesn't
             * announce itself. No final "I'm dying" packet, just nothing
             * further, ever, from this process. */
            if (!printed_death) {
                printf("cloud_coordinator: *** DEAD at t=%.1fs — going "
                       "silent. Every robot under this coordinator will "
                       "freeze within %ums with nothing here to bring them "
                       "back. ***\n",
                       (double)elapsed_ms * 0.001,
                       (unsigned)CLOUD_COORD_TIMEOUT_MS);
                printed_death = true;
            }
            continue;
        }

        float ox = 0.0f, oy = 0.0f;   /* MODE_WAYPOINT offset, shared by all robots */
        if (mode == MODE_WAYPOINT) {
            bool leg_b = ((elapsed_ms / leg_ms) % 2u) == 1u;
            ox = leg_b ? dx1 : dx0;
            oy = leg_b ? dy1 : dy0;
        }

        for (int i = 0; i < n_robots; i++) {
            cloud_directive_t d;
            d.magic    = CLOUD_PROTOCOL_MAGIC;
            d.robot_id = (uint8_t)robots[i].id;

            if (mode == MODE_ORBIT) {
                float t     = (float)elapsed_ms * 0.001f;
                float angle = robots[i].phase_rad +
                             2.0f * (float)M_PI * t / orbit_period_s;
                d.target_x = orbit_cx + orbit_r * cosf(angle);
                d.target_y = orbit_cy + orbit_r * sinf(angle);
            } else {
                d.target_x = robots[i].start_x + ox;
                d.target_y = robots[i].start_y + oy;
            }

            if (sock >= 0) {
                struct sockaddr_in dst;
                memset(&dst, 0, sizeof(dst));
                dst.sin_family      = AF_INET;
                dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
                dst.sin_port        = htons((uint16_t)(CLOUD_BASE_PORT +
                                                       robots[i].id));
                sendto(sock, &d, sizeof(d), 0, (struct sockaddr *)&dst,
                      sizeof(dst));
            }
        }

        /* Its own heartbeat to the supervisor, using the reserved
         * coordinator id — see status_tx.h's WH_COORDINATOR_ID. Stops the
         * instant `dead` above goes true, the same way a robot's own status
         * feed would stop if IT died — the supervisor infers coordinator
         * death from silence, exactly as the robots themselves do. */
        wh_status_t st = {0};
        st.magic      = WH_STATUS_MAGIC;
        st.element_id = (uint8_t)WH_COORDINATOR_ID;
        st.fleet      = WH_FLEET_CLOUD;
        st.quorum     = 0xFFu;
        st.role       = 0xFFu;
        st.swarm_size = (uint8_t)n_robots;   /* the one field that IS
                                              * meaningful for a coordinator:
                                              * how many robots it is
                                              * driving */
        st.task_slot  = 0xFFu;
        st.step       = 0xFFu;
        status_tx_send(&st);
    }

    if (sock >= 0) { close(sock); }
    return 0;
}
