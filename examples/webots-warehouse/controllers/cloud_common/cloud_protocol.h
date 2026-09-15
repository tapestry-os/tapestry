/*
 * cloud_protocol.h — wire format for the "cloud coordinator" stand-in
 * (scenes 2/3 — Tapestry vs. centralized coordination, same floor, same
 * fault)
 *
 * WHAT THIS IS, AND WHAT IT ISN'T. This is a minimal, honestly-labeled
 * ANALOG for a centralized/cloud-dependent coordination architecture — a
 * single process assigns tasks and pushes them hub-and-spoke to robots that
 * have no peer-to-peer awareness of each other at all. It is not a port of
 * any specific real framework, and no claim is made about matching any real
 * system's resilience properties in every detail. What IS modeled,
 * faithfully, is the one architectural property every system in this
 * family shares by construction: a robot that depends on a central
 * authority for its task has nothing else to fall back on when that
 * authority is unreachable. See ../cloud_bot/main.c and
 * ../cloud_coordinator/main.c for how that plays out.
 *
 * ONE-WAY BY DESIGN. Unlike Tapestry's gossip (every element publishes its
 * own state, every element consumes everyone else's), this protocol only
 * flows coordinator -> robot. A robot never reports its position back to
 * the coordinator, and the coordinator never adapts its plan to anything a
 * robot does — it has a fixed, centrally pre-computed assignment and simply
 * repeats it. This is deliberate: it makes the coordinator itself trivially
 * simple (see its main.c), which is an honest reflection of a system whose
 * intelligence lives in one place rather than distributed across the
 * fleet — and it is precisely why that one place, if it becomes unreachable
 * or dies, takes every robot's task assignment down with it.
 *
 * WHY THE ROBOT DECIDES REACHABILITY, NOT THE COORDINATOR. Same lesson
 * ../../rover/rf_occlusion.h documents at length: obstruction is a function
 * of where BOTH endpoints are, live, and only the receiver can hold both as
 * true — the coordinator here is fixed and known, but the ROBOT is the one
 * that is moving, so the robot's own receive path is where the occlusion
 * check has to run (see cloud_bot's main.c). A transmit-side model would
 * repeat the exact one-way-link bug that motivated rf_occlusion.h's own
 * receive-filter design.
 */

#ifndef TAPESTRY_WAREHOUSE_CLOUD_PROTOCOL_H
#define TAPESTRY_WAREHOUSE_CLOUD_PROTOCOL_H

#include <stdint.h>

#define CLOUD_PROTOCOL_MAGIC  0xC1

/* UDP base port for coordinator -> robot directives. Each robot binds
 * base_port + robot_id; the coordinator sends to that same address for
 * every robot it knows about. Deliberately far from the Tapestry rover's
 * own port ranges (gossip 5800+id, directive 6800+id, status 7800) so both
 * fleets can run in the same Webots process tree with zero collision. */
#define CLOUD_BASE_PORT  6200u

/*
 * Coordinator -> robot directive. One-way: there is no acknowledgment and
 * no reply. The coordinator repeats the CURRENT assignment for every known
 * robot roughly every CLOUD_BROADCAST_MS — see cloud_coordinator/main.c —
 * so a robot's only signal that its coordinator is still reachable is
 * whether these keep arriving, which is exactly the heartbeat cloud_bot's
 * timeout logic depends on.
 */
typedef struct {
    uint8_t  magic;      /* CLOUD_PROTOCOL_MAGIC — rejects stray datagrams */
    uint8_t  robot_id;
    float    target_x;   /* world-frame meters */
    float    target_y;
} cloud_directive_t;

#define CLOUD_BROADCAST_MS  150u

/*
 * How long a robot will keep driving toward its LAST accepted directive
 * after directives stop arriving, before concluding its coordinator is
 * gone and freezing. Deliberately the SAME value as Tapestry's
 * WM_STALE_THRESHOLD_MS (tapestry-os/include/tapestry/csm.h) — the whole
 * point of this comparison is the architecture, not a rigged timeout, so
 * both fleets get the identical grace period before declaring a peer (or,
 * here, their one authority) unreachable.
 */
#define CLOUD_COORD_TIMEOUT_MS  1500u

#endif /* TAPESTRY_WAREHOUSE_CLOUD_PROTOCOL_H */
