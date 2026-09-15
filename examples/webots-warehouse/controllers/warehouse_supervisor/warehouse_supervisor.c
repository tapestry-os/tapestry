/*
 * warehouse_supervisor.c — on-screen readout for the warehouse scenes
 *
 * WHAT IT IS NOT, FIRST. This has no authority over anything. It does not
 * send to any element, does not relay between elements, is not a member of
 * the collective, and nothing in L3-L7 reads it. It binds one UDP port and
 * listens. If it is not running, the scenes behave identically.
 *
 * That distinction is load-bearing rather than pedantic. These scenes claim
 * that eight robots coordinate with no coordinator; a supervisor process
 * that quietly shuttled state between them would make the claim false. So
 * the data flow is strictly one-way and outward: each element reports what
 * IT believes (controllers/rover/status_tx.h), and this renders those
 * beliefs side by side.
 *
 * That side-by-side view is the point. The interesting moments in all three
 * scenes are DISAGREEMENTS between elements' world models — during scene
 * 2's partition the north island and the south island report different
 * swarm sizes at the same instant, and seeing both at once is the whole
 * demonstration. A single global readout would show a consensus that does
 * not exist.
 *
 * Rendering is deliberately per-element and unaggregated for the same
 * reason: this prints what each element said, and never averages, resolves
 * or corrects between them.
 *
 * SCENES 2/3 — TWO FLEETS, ONE READOUT. wh_status_t::fleet distinguishes
 * a Tapestry element from a robot in the centralized "cloud" comparison
 * fleet (see controllers/cloud_bot/, controllers/cloud_coordinator/). This
 * process still does nothing but listen and print — it does not relay
 * between the two fleets any more than it does between Tapestry elements,
 * and the cloud fleet's own coordinator process is exactly as unaware of
 * this readout as every Tapestry element already is.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <webots/robot.h>
#include <webots/supervisor.h>

#include "status_tx.h"

#define MAX_TRACKED 32
#define GOSSIP_BASE_PORT 5800     /* must match controllers/rover/main.c */

/* An element is shown as SILENT if nothing has arrived for this long. Set
 * comfortably above WM_STALE_THRESHOLD_MS (1500 ms) so the readout does not
 * declare an element dead before its own PEERS do — the peers' judgement is
 * the one the scenes are about, and this must not appear to pre-empt it. */
#define SILENT_AFTER_MS 2500

typedef struct {
    wh_status_t st;
    bool        seen;
    double      last_ms;
} tracked_t;

static tracked_t g_elems[MAX_TRACKED];
/* The cloud coordinator's own heartbeat (element_id == WH_COORDINATOR_ID,
 * 255) is tracked HERE, never in g_elems[] — that array is indexed
 * directly by element_id and is only 32 slots long, so a raw 255 would be
 * an out-of-bounds write. See drain() below. */
static tracked_t g_coordinator;
static int       g_sock = -1;

static const char *quorum_name(uint8_t q)
{
    switch (q) {
    case 0:  return "LOST";
    case 1:  return "DEGR";
    case 2:  return "HLTH";
    default: return "????";
    }
}

static const char *role_name(uint8_t r)
{
    switch (r) {
    case 0:  return "-";
    case 1:  return "follow";
    case 2:  return "LEADER";
    case 3:  return "relay";
    case 4:  return "sensor";
    case 5:  return "actuat";
    default: return "?";
    }
}

static void open_socket(void)
{
    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) {
        fprintf(stderr, "supervisor: socket() failed — no status readout\n");
        return;
    }
    int on = 1;
    setsockopt(g_sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port        = htons((uint16_t)(GOSSIP_BASE_PORT +
                                            WH_STATUS_PORT_OFFSET));
    if (bind(g_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "supervisor: bind() failed — no status readout\n");
        close(g_sock);
        g_sock = -1;
        return;
    }
    int flags = fcntl(g_sock, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(g_sock, F_SETFL, flags | O_NONBLOCK);
    }
}

static void drain(double now_ms)
{
    if (g_sock < 0) {
        return;
    }
    for (;;) {
        wh_status_t st;
        ssize_t n = recvfrom(g_sock, &st, sizeof(st), 0, NULL, NULL);
        if (n != (ssize_t)sizeof(st)) {
            break;   /* EAGAIN, or a datagram that is not ours */
        }
        if (st.magic != WH_STATUS_MAGIC) {
            continue;
        }
        if (st.element_id == WH_COORDINATOR_ID) {
            g_coordinator.st      = st;
            g_coordinator.seen    = true;
            g_coordinator.last_ms = now_ms;
            continue;
        }
        if (st.element_id >= MAX_TRACKED) {
            continue;
        }
        g_elems[st.element_id].st      = st;
        g_elems[st.element_id].seen    = true;
        g_elems[st.element_id].last_ms = now_ms;
    }
}

int main(void)
{
    wb_robot_init();
    int timestep = (int)wb_robot_get_basic_time_step();
    open_socket();

    double now_ms   = 0.0;
    double label_ms = 0.0;

    while (wb_robot_step(timestep) != -1) {
        now_ms += timestep;
        drain(now_ms);

        /* Repaint at 5 Hz. Labels are a rendering cost and the underlying
         * state only changes at the 100 ms coordination cadence anyway. */
        label_ms += timestep;
        if (label_ms < 200.0) {
            continue;
        }
        label_ms = 0.0;

        char buf[2048];
        int  off = 0;
        int  scene = 0;

        for (int i = 0; i < MAX_TRACKED; i++) {
            if (g_elems[i].seen) { scene = g_elems[i].st.scene; break; }
        }

        off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                        "TAPESTRY WAREHOUSE  scene %d   t=%5.1fs\n"
                        "id  fleet     pos            zone   peers  quorum  role    step  notes\n",
                        scene, now_ms / 1000.0);

        for (int i = 0; i < MAX_TRACKED && off < (int)sizeof(buf) - 160; i++) {
            if (!g_elems[i].seen) {
                continue;
            }
            const wh_status_t *s = &g_elems[i].st;
            bool cloud  = (s->fleet == WH_FLEET_CLOUD);
            bool silent = (now_ms - g_elems[i].last_ms) > SILENT_AFTER_MS;
            const char *fleet_s = cloud ? "CLOUD" : "TAPESTRY";

            char notes[96];
            notes[0] = '\0';
            /* A failed Tapestry element is reported as SILENT rather than
             * FAILED even though this process happens to have been told:
             * what the fleet can observe is silence, and showing more than
             * that here would misrepresent what the survivors are reacting
             * to. A frozen CLOUD robot is different and is reported as
             * such (WH_STATUS_F_NO_COORD) — it keeps sending its own
             * status the whole time, since going quiet was never its
             * failure mode, only its coordinator's. */
            if (silent) {
                snprintf(notes, sizeof(notes), "SILENT (peers infer loss)");
            } else {
                if (s->flags & WH_STATUS_F_CP_FROZEN) {
                    strncat(notes, "CP-FROZEN ", sizeof(notes) - strlen(notes) - 1);
                }
                if (s->flags & WH_STATUS_F_SUSPENDED) {
                    strncat(notes, "SUSPENDED ", sizeof(notes) - strlen(notes) - 1);
                }
                if (s->flags & WH_STATUS_F_NO_COORD) {
                    strncat(notes, "NO-COORD/FROZEN ", sizeof(notes) - strlen(notes) - 1);
                }
                if (s->flags & WH_STATUS_F_RF_BLOCKED) {
                    char rf[40];
                    snprintf(rf, sizeof(rf), "RF-BLOCKED(%u) ",
                             (unsigned)s->blocked_peers);
                    strncat(notes, rf, sizeof(notes) - strlen(notes) - 1);
                }
            }

            if (silent) {
                off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                                "%-3u %-9s (%6.2f,%6.2f)  --     --     ----    --      --    %s\n",
                                (unsigned)i, fleet_s, (double)s->x, (double)s->y, notes);
            } else if (cloud) {
                /* Tapestry-specific columns are genuinely inapplicable to a
                 * hub-and-spoke robot (0xFF sentinel, status_tx.h) — printed
                 * as "n/a", never as a fabricated 0, so a viewer can't read
                 * a real (if degenerate) value where none exists. */
                off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                                "%-3u %-9s (%6.2f,%6.2f)  n/a    n/a    n/a     n/a     n/a   %s\n",
                                (unsigned)i, fleet_s, (double)s->x, (double)s->y, notes);
            } else {
                off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                                "%-3u %-9s (%6.2f,%6.2f)  %u/%-3u  %-3u    %-4s    %-6s  %-3d   %s\n",
                                (unsigned)i, fleet_s, (double)s->x, (double)s->y,
                                (unsigned)s->task_slot, (unsigned)s->swarm_size,
                                (unsigned)s->fresh_peers,
                                quorum_name(s->quorum), role_name(s->role),
                                (int)s->step, notes);
            }
        }

        /* zone is task_slot/swarm_size and peers is the fresh peer count,
         * both as that element believes them (TAPESTRY rows only). When
         * those columns disagree between rows, the fleet's world models
         * have genuinely diverged — that is the partition, not a
         * rendering artifact. */

        if (g_coordinator.seen) {
            bool coord_silent = (now_ms - g_coordinator.last_ms) > SILENT_AFTER_MS;
            off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                            "\nCLOUD COORDINATOR: %s\n",
                            coord_silent
                                ? "SILENT — every cloud robot above is on "
                                  "its own now, permanently"
                                : "alive, driving robots centrally");
        }
        /* "Lucida Console" and not "Monospace": Webots validates the font
         * name against a fixed list and silently substitutes Arial for
         * anything else, which un-aligns every column in the table above. */
        wb_supervisor_set_label(0, buf, 0.01, 0.01, 0.03, 0x00FF00, 0.15,
                                "Lucida Console");
    }

    wb_robot_cleanup();
    return 0;
}
