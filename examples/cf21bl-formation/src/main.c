/*
 * main.c — CF21BL collective formation demo (lighthouse + syslink P2P)
 *
 * Two build modes (Kconfig choice DEMO_MODE):
 *
 *   DEMO_MODE_CHOREO (default) — the L5/L6/L7 path.  ONE BINARY for all
 *     drones: element IDs are negotiated at boot (transport_negotiate_id,
 *     same auto-ID protocol as cutebot-formation).  A real L5 SCR (see
 *     scr_init()/scr_tick() below) drives quorum/role/task_slot/abort from
 *     the actual world model — no synthetic quorum.  A declarative L7
 *     Choreo script drives the flight through the L6 BSE:
 *       1. hold  — station-keep at the current position (coordinate-free)
 *       2. exchange — swap places: each drone takes its partner's station
 *          via the BSE's centroid-arc maneuver (snapshot targets, mutual
 *          separation preserved by construction), advancing on the L6
 *          achievement predicate
 *       3. hold (bow) — settle on the new stations so both drones finish
 *          their scripts while the collective is still fresh (achievement
 *          is per-element; without this beat the first finisher would land
 *          and suspend its partner mid-maneuver)
 *     Script completion → directive IDLE → quiescence: this platform maps
 *     it to landing in place and disarming.  The Choreo never says "take
 *     off" or "land" — those words don't exist at L7.
 *
 *   DEMO_MODE_SHOWCASE — the flight-validated 2026-07 spring-field
 *     showcase (L4 only, per-drone builds): line → rotating triangle →
 *     member departs → line re-forms.
 *
 * Both modes use REAL lighthouse position (not dead reckoning) for peer
 * gossip and the stabilizer's own X/Y position hold.  See formation.h for
 * the meters unit convention and the shared-calibration requirement.
 *
 * Architecture:
 *   Attitude:    BMI088 rate + angle loops (cf21bl_stabilizer.c, unchanged)
 *   Yaw heading: CONFIG_CF21BL_YAW_HOLD — locked to boot orientation so
 *                world-frame corrections stay meaningful (Mahony yaw is
 *                boot-relative with no absolute reference, hence the
 *                requirement: place each drone with its nose along lighthouse
 *                world +X at power-on).
 *   Altitude:    CONFIG_CF21BL_ALTITUDE_HOLD (baro, closed-loop) — tracks a
 *                rate-limited setpoint (ALT_BASE_M by default, or
 *                directive.target.z once Choreo is driving).
 *   X/Y:         CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD (stabilizer-internal
 *                P+I+D loop) — this file only ever commands an ABSOLUTE
 *                world-frame target, converted to the stabilizer's
 *                home-relative normalized form via
 *                cf21bl_stabilizer_get_pos_home().
 *   Formation:   formation.c's holonomic spring field, computed from REAL
 *                gossiped peer positions (no odometry drift).
 *
 * Safety layer (see the flight_state_t machine below):
 *   - Own battery LOW (cf21bl_pm_battery_low(), DEMO_MODE_CHOREO only):
 *     this file preempts the running Choreo goal/script with a CONVERGE-
 *     to-home goal (choreo_preempt_goal()) — a graceful, L6/L7-mediated
 *     return-to-base rather than an abrupt stop, and the demo for the
 *     v1.0 goal-queue-with-preemption feature. The original goal/script
 *     resumes automatically if this drone somehow un-preempts (it
 *     doesn't in practice — see the call site) rather than being lost.
 *   - Own battery CRITICAL (cf21bl_pm_battery_critical(), a lower/later
 *     threshold than LOW): still handled independently inside
 *     cf21bl_stabilizer.c (CONFIG_CF21BL_PM forced-landing path) — no code
 *     needed here, and it does not depend on or affect other drones. This
 *     is the hard backstop if RTH above doesn't land in time.
 *   - Own lighthouse fix lost: this file must intervene, because the
 *     stabilizer reinterprets stale linear.x/y as velocity feedforward
 *     once POS_HOLD falls back — holding a position-style value through
 *     that transition would command a sustained runaway tilt.  Zero X/Y
 *     immediately on fix loss; land after FIX_LOSS_GRACE_MS sustained loss.
 *   - Geofence: own real position straying past GEOFENCE_RADIUS_M from the
 *     lighthouse origin triggers an individual landing.
 *   - Minimum separation: formation.c applies extra repulsion below
 *     DEMO_MIN_SEP_M; this file only logs a warning if it's violated
 *     anyway (formation.c reports the closest peer distance each tick).
 *   - Mission duration: every drone independently lands after
 *     MISSION_DURATION_S from its own arm time — the closest thing to a
 *     "coordinated" land command without a wireless uplink, since all
 *     drones arm within the same sync window (see DEMO_SYNC_GRACE_MS).
 *   All landing triggers are per-drone local state — one drone landing
 *   never affects another's flight.
 *
 * Placement requirement: place each drone in its own starting spot, nose
 * along lighthouse world +X, BEFORE arming — the stabilizer captures that
 * drone's OWN lighthouse-frame position as its "home" the instant this
 * file first commands a non-idle altitude (see cf21bl_stabilizer.c's
 * position-hold home capture).  Starting positions do not need to match
 * the final formation shape; the spring field converges regardless.
 *
 * Build (one per drone):
 *   west build -p always -b crazyflie21bl tapestry/examples/cf21bl-formation \
 *     -- -DCONFIG_TAPESTRY_ELEMENT_ID=0   # 1, 2 for other drones
 * Flash:
 *   cfloader flash build/zephyr/zephyr.bin stm32-dfu
 * Console:
 *   python3 tapestry/tapestry-os/tools/crazyflie_console.py   (CRTP radio — USART3 is taken
 *   by the lighthouse deck; see boards/crazyflie21bl.conf.  With 3 drones
 *   transmitting on the same nRF51 radio config, treat concurrent consoles
 *   as best-effort — see Phase D fleet bring-up notes.)
 *
 * Flight checklist:
 *   1. Flash all three drones with IDs 0, 1, 2 — SAME lighthouse BS pose
 *      and OOTX calibration constants below on all three (same YAML).
 *   2. Place each drone in the arena, nose along lighthouse world +X,
 *      spaced roughly DEMO_TARGET_SPACING_M apart — exact placement is not
 *      critical, the spring field will converge.
 *   3. Power on all three within the sync window (a few seconds of slack).
 *   4. Each drone: gyro cal, waits for its own lighthouse fix (abort/idle
 *      if none within 30 s), 5 s countdown, arms, gentle altitude ramp to
 *      the default cruise height, then joins the formation.
 *   5. After MISSION_DURATION_S, every drone lands independently and
 *      disarms.  A drone whose battery goes critical, fix is lost, or
 *      strays past the geofence lands early on its own — this does not
 *      affect the others.
 */

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <tapestry/csm.h>
#include <tapestry/transport.h>
#include <tapestry/substrate.h>
#ifdef CONFIG_DEMO_MODE_CHOREO
#include <tapestry/scr.h>      /* real L5 quorum/role/task_slot/abort */
#include <tapestry/choreo.h>
#include "gossip.h"            /* gossip_own_id_frames — duplicate-ID diag */
/* The show itself.  GENERATED from ../change-partners.choreo.toml (the
 * file to edit) by sdk/tools/choreoc.py — see the regeneration command
 * in its banner. */
#include "choreo_script.h"
#endif

#include "cf21bl_lighthouse.h"
#include "cf21bl_stabilizer.h"
#ifdef CONFIG_CF21BL_PM
#include "cf21bl_pm.h"
#endif

#include "formation.h"

LOG_MODULE_REGISTER(cf21bl_formation, LOG_LEVEL_INF);

/* ── Calibrated BS poses + OOTX calibration ──────────────────────────────── */
/* Single shared copy (see that header's comment for the recalibration
 * procedure).  MUST match the physical base-station placement at flight
 * time and MUST be identical across all drones — gossiped positions are
 * only comparable in a shared frame. */
#include "../../lighthouse_cal.h"

/* ── Mission parameters ───────────────────────────────────────────────────── */

/* Default/idle altitude — the ramp target before Choreo has issued a
 * directive at all (boot). Once FLIGHT_FLYING starts ticking Choreo, real
 * altitude is Choreo-commanded (directive.target.z) instead — the
 * automatic per-element stagger this constant used to carry (via
 * ALT_STEP_PER_ID_M, now removed) is gone; vertical separation, if
 * wanted, is something a script must express explicitly now (e.g.
 * distinct z per track/element).
 *
 * IMPORTANT, THIS ROOM SPECIFICALLY: the stagger wasn't just downwash
 * margin — it was tuned against a real lighthouse arrival-angle problem.
 * At 0.80 m the top drone received BS1's light only ~12.6° above its deck
 * horizon and dropped to ok=0 in bursts (grazing incidence — BS1 is
 * mounted low and far in this room); the validated 0.30–0.70 m band
 * bought back enough arrival angle to fix every recorded dropout. Any
 * script commanding z outside that band on THIS hardware risks
 * reintroducing that dropout — this is now the script author's
 * responsibility to respect, not something the platform enforces for
 * you. */
#define ALT_BASE_M           0.30f

/* Gentle altitude ramp on takeoff (same convention as altitude-hold-tether:
 * ramp the closed-loop PID's TARGET, don't jump straight to cruise). */
#define ALT_RAMP_START_M     0.15f
#define ALT_RAMP_RATE_MPS    0.10f

/* Individual landing.
 *
 * The descent itself belongs to cf21bl_stabilizer_request_land(): it walks
 * the altitude target down from the MEASURED altitude and cuts only after
 * the airframe has settled on the ground.  This file used to walk its own
 * target down at LAND_RATE_MPS instead, which cut thrust at a commanded
 * 0.10 m — the stabilizer's idle sentinel, see CF21BL_IDLE_SP_Z — while
 * the airframe was still above it, and the drone dropped the rest.  The
 * 2026-07-19 flight 10 fix addressed the DISARM half of that (the gate
 * below) but not the thrust half, because by the time this gate runs the
 * motors have already been idle for a quarter second.
 *
 * LAND_HOLD_CMD_M is what we keep commanding on linear.z while the
 * stabilizer owns the profile: any value comfortably clear of the idle
 * sentinel works, since the descent overrides the altitude target
 * internally — it only has to not read as "motors off".
 *
 * The gate below is now a BACKSTOP, not the primary mechanism: normally
 * cf21bl_stabilizer_is_landed() reports the settle first.  It stays
 * because the stabilizer's settle uses its own fused altitude estimate,
 * and an independent measured-lighthouse check plus LAND_FORCE_DISARM_MS
 * is cheap insurance against that estimate being biased high. */
#define LAND_HOLD_CMD_M      0.30f
#define LAND_SETTLE_MS       2000
#define LAND_TOUCHDOWN_Z_M   0.08f
#define LAND_FORCE_DISARM_MS 10000

/* Sustained lighthouse fix loss before an individual landing.  The drone
 * brakes (cf21bl_stabilizer.c — see CF21BL_BRAKE_MS there) and then holds
 * level and in place for up to this long, waiting for the fix to return,
 * rather than landing on the first dropout — see the FLIGHT_FLYING
 * !fix_valid case below.
 *
 * 10 s, up from an original 2 s: the 2 s figure predated two changes that
 * made a longer wait safe rather than merely tolerated.  (1) A dropout no
 * longer re-origins the home reference on re-acquisition (see
 * cf21bl_stabilizer.c's g_pos_home_set handling) — waiting out a longer
 * outage used to mean baking in more drift once it re-latched; now the
 * frame the drone resumes into is the same one it left.  (2) The position
 * gossiped during the outage is now flagged
 * (ELEMENT_HEALTH_POSITION_STALE) rather than presented as current, so
 * peers reasoning about this drone during a long dropout know it is a
 * held last-known point, not a fresh one — separation and repulsion still
 * measure against it (see peer_has_position() in formation.c), same as
 * any other stale-but-active peer.
 *
 * A third change closes the gap those two didn't: cf21bl_stabilizer.c's
 * brake pulse is best-effort and open-loop — nothing confirms it actually
 * arrested the drone's motion, because confirming that needs the very
 * position reading that just stopped arriving.  This constant bounds how
 * long a coast is TOLERATED regardless of whether the brake worked; the
 * coast-budget backstop below bounds how FAR one can go before waiting is
 * no longer safe, independent of this timer. */
#define FIX_LOSS_GRACE_MS    10000

/* Geofence: distance from the lighthouse origin (NOT this drone's home —
 * the origin is the one frame shared by every drone).  TUNE to the room's
 * actual coverage; 2.0 m is conservative relative to CF21BL_POS_MAX_M=3. */
#define GEOFENCE_RADIUS_M    2.0f

/* Quorum-recovery hold, fed to scr_set_quorum_hold_ms() below — see that
 * call site's comment. Flight-tested value (2026-07-19 flight 2). */
#define QUORUM_UP_MS 2000

/* Every drone lands independently after this long from its own arm time —
 * a pure safety backstop in choreo mode (the script normally ends the
 * flight well before it), and the "coordinated" land in showcase mode.
 * Showcase keeps it per-build (Kconfig) so one drone can leave the
 * formation early: it lands, goes gossip-silent (see the FLIGHT_LANDED
 * gate at the send site), peers expire it after WM_EXPIRE_THRESHOLD_MS,
 * and the field re-forms without it. */
#ifdef CONFIG_DEMO_MODE_CHOREO
/* The script's own total time bound (choreoc requires every step to be
 * time-bounded, so this is a hard ceiling) + CONFIG_DEMO_MISSION_MARGIN_S.
 *
 * The margin is NOT just ramp + descent, which is what the original 40 s
 * covered.  Step timers freeze while the Choreo is SUSPENDED (quorum
 * lost), so wall-clock runtime exceeds the script's own bound by however
 * long the link was down — on a marginal P2P link, most of the flight.
 * At 40 s the 48 s change-partners script was racing its own backstop:
 * flight 15 completed with ~2-6 s to spare, and flight 16 (with the
 * stricter scope=all predicate, which advances later by design) appears
 * to have lost the race entirely.  A flight that lands on this backstop
 * instead of on script completion looks identical from the ground and
 * means something completely different, so the margin is now generous by
 * default and tunable per build.  See the Kconfig help. */
#define MISSION_DURATION_S   (CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u \
                              + (uint32_t)CONFIG_DEMO_MISSION_MARGIN_S)
#else
#define MISSION_DURATION_S   CONFIG_DEMO_MISSION_DURATION_S
#endif

#define LOOP_DT_S  ((float)WM_CYCLE_MS * 0.001f)

/* Own-state gossip cadence.  Choreo mode sends at 5 Hz instead of the L4
 * default 2 Hz: measured syslink P2P delivery under load is ~20-35%
 * (2026-07-19 runs), and at 2 Hz the peer entry goes stale
 * (WM_STALE_THRESHOLD_MS) in ~half of all windows — the debounced quorum
 * then almost never comes up and the script stays SUSPENDED.  At 5 Hz the
 * same loss rate keeps the stale threshold fed ~85% of the time.  Airtime
 * cost is trivial (~100 B/s).  Showcase mode keeps the flight-validated
 * default. */
#ifdef CONFIG_DEMO_MODE_CHOREO
#define DEMO_GOSSIP_MS 200u
#else
#define DEMO_GOSSIP_MS GOSSIP_INTERVAL_MS
#endif

/* ── Flight state machine ─────────────────────────────────────────────────── */

typedef enum {
    FLIGHT_RAMPING,   /* gentle climb from ALT_RAMP_START_M to cruise altitude */
    FLIGHT_FLYING,    /* formation control active                              */
    FLIGHT_LANDING,   /* alt target ramps to 0; X/Y held where landing began
                       * (fix-loss landings: X/Y zeroed — no position to hold) */
    FLIGHT_LANDED,    /* idle sentinel forever                                  */
} flight_state_t;

/*
 * Why this element started its descent.  Every trigger below announces
 * itself with a one-shot LOG_INF/LOG_ERR, and on a shared CRTP console
 * one-shots are exactly what line splicing destroys — flights 15-17 lost
 * every "choreo complete" and "mission duration elapsed" line.  That is
 * the single most important fact about how a flight ended: a script that
 * ran to completion and one cut off by the backstop look identical from
 * the ground.  Riding the repeating 1 Hz status line means the reason
 * survives even when the announcement does not.
 */
typedef enum {
    LAND_REASON_NONE = 0,   /* still flying                                */
    LAND_REASON_COMPLETE,   /* script finished — quiescence -> land        */
    LAND_REASON_BACKSTOP,   /* MISSION_DURATION_S elapsed                  */
    LAND_REASON_FIXLOSS,    /* lighthouse fix lost > FIX_LOSS_GRACE_MS     */
    LAND_REASON_GEOFENCE,   /* strayed past GEOFENCE_RADIUS_M              */
    LAND_REASON_DEPARTURE,  /* a peer departed; this element's own
                             * on_departure policy responded (land_in_place,
                             * a completed recall, or a HOLD that timed
                             * out) — see choreo_departure_triggered() */
} land_reason_t;

#ifdef CONFIG_DEMO_MODE_CHOREO
/* Only the choreo status line reports why a flight ended; showcase
 * mode has no such field, so this would be an unused static there. */
static const char *land_reason_name(land_reason_t r)
{
    switch (r) {
    case LAND_REASON_COMPLETE: return " why=complete";
    case LAND_REASON_BACKSTOP: return " why=backstop";
    case LAND_REASON_FIXLOSS:  return " why=fixloss";
    case LAND_REASON_GEOFENCE: return " why=geofence";
    case LAND_REASON_DEPARTURE:
        /* Which policy actually executed (RECALL that fell back to
         * landing is reported as LAND_IN_PLACE by choreo.c itself — see
         * choreo_departure_triggered_policy()'s doc) — queried at print
         * time since land_reason_t has no room to carry it. */
        switch (choreo_departure_triggered_policy()) {
        case CHOREO_DEPARTURE_RECALL:        return " why=departure(recall)";
        case CHOREO_DEPARTURE_LAND_IN_PLACE: return " why=departure(land_in_place)";
        default:                             return " why=departure";
        }
    default:                   return "";
    }
}
#endif

/* Maps this platform's land_reason_t onto the wire's narrower
 * tapestry_departure_reason_t (csm.h) — LOST has no entry because it is
 * never self-declared, only inferred by a receiver from expiry.
 * LAND_REASON_NONE cannot reach here in practice (every FLIGHT_LANDING/
 * FLIGHT_LANDED transition sets land_reason in the same step), but a
 * defensive default is cheaper than an assert on a flight-control path. */
static tapestry_departure_reason_t land_departure_reason(land_reason_t r)
{
    switch (r) {
    case LAND_REASON_BACKSTOP: return ELEMENT_DEPARTED_BACKSTOP;
    case LAND_REASON_FIXLOSS:  return ELEMENT_DEPARTED_FIXLOSS;
    case LAND_REASON_GEOFENCE: return ELEMENT_DEPARTED_GEOFENCE;
    /* LAND_REASON_DEPARTURE has no dedicated wire bit of its own — csm.h's
     * 2-bit reason field is already fully used by the other four, and
     * widening it would cost the wire-additive/no-version-bump property
     * this whole mechanism was built to keep (see csm.h's rationale).
     * COMPLETE is the closest fit: like a normal finish, this is a
     * graceful, policy-decided stop, not an emergency backstop/fixloss/
     * geofence condition. */
    case LAND_REASON_DEPARTURE:
    case LAND_REASON_COMPLETE:
    case LAND_REASON_NONE:
    default:
        return ELEMENT_DEPARTED_COMPLETE;
    }
}

#if defined(CONFIG_DEMO_MODE_CHOREO) && defined(CONFIG_PWM)
/* choreo_departure_recall_point_fn callback (CHOREO_DEPARTURE_RECALL).
 * Showcase mode never registers this (it has no on_departure policy
 * surface — see the CONFIG_DEMO_MODE_CHOREO guard around where it's
 * registered, below), so this stays choreo-mode-only to avoid an unused-
 * function warning there.
 *
 * "home" for this platform is the lighthouse-frame position
 * cf21bl_stabilizer.c latches at arming/first fix (same source RTH's own
 * inline CONVERGE goal already uses). A plain C function pointer has no
 * closure over main()'s locals, unlike RTH's inline construction, so z
 * (own current commanded cruise altitude — 0 would ramp to ground level
 * mid-flight while still translating home, same reasoning as RTH's own
 * comment) is mirrored into g_departure_recall_z_m once per tick, right
 * before choreo_tick() runs, the only point this callback can be invoked
 * from. */
static float g_departure_recall_z_m;

static bool cf21bl_departure_recall_point(position_t *out)
{
    float home_x, home_y;
    if (!cf21bl_stabilizer_get_pos_home(&home_x, &home_y)) {
        return false;   /* no fix yet — RECALL falls back to LAND_IN_PLACE */
    }
    out->x = home_x;
    out->y = home_y;
    out->z = g_departure_recall_z_m;
    return true;
}
#endif

static const char *flight_state_name(flight_state_t s)
{
    switch (s) {
    case FLIGHT_RAMPING: return "RAMPING";
    case FLIGHT_FLYING:  return "FLYING";
    case FLIGHT_LANDING: return "LANDING";
    default:             return "LANDED";
    }
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    cf21bl_lighthouse_set_bs_pose(0, &BS0);
    cf21bl_lighthouse_set_bs_pose(1, &BS1);
    cf21bl_lighthouse_set_bs_calib(0, &BS0_CALIB);
    cf21bl_lighthouse_set_bs_calib(1, &BS1_CALIB);
    cf21bl_lighthouse_set_bs_channel(0, BS0_CHANNEL);
    cf21bl_lighthouse_set_bs_channel(1, BS1_CHANNEL);
    if (cf21bl_lighthouse_init() != 0) {
        LOG_ERR("id=%u lighthouse init failed — cannot fly without position",
                (unsigned)CONFIG_TAPESTRY_ELEMENT_ID);
        return -1;
    }

    /* substrate_init() -> cf21bl_init(): ESC arming + gyro cal + stabilizer
     * start (baro home average).  Motors silent throughout. */
    if (substrate_init() != 0) {
        LOG_WRN("id=%u substrate_init failed — actuation disabled",
                (unsigned)CONFIG_TAPESTRY_ELEMENT_ID);
    }

    if (transport_init() != 0) {
        LOG_WRN("id=%u transport_init failed — no peer awareness",
                (unsigned)CONFIG_TAPESTRY_ELEMENT_ID);
    }

#ifdef CONFIG_DEMO_MODE_CHOREO
    /* One binary for every drone: identity is negotiated over the radio in
     * a boot window (nonce = STM32 unique device ID; see transport.h).
     * Power all drones on within a few seconds of each other so their
     * windows overlap.  VERIFY in the logs that every drone reports the
     * expected n_total and a unique id before flight — a drone that heard
     * nobody claims id=0 and will fly a solo script. */
    /* Let the nRF51 radio finish its cold start before opening the
     * discovery window: every 2026-07-19 run showed near-zero P2P delivery
     * in the first seconds (run 11: 2 of 60 frames during the window, and
     * even the Crazyradio got no ACKs until ~t=5 s) with the link
     * recovering to ~25-35% afterwards.  Beaconing into a deaf radio
     * wastes the window and forces the self-heal path (and its script
     * skew). */
    k_msleep(2500);

    int n_total;
    element_id_t element_id = transport_negotiate_id(&n_total);

#ifndef CONFIG_DEMO_ALLOW_SOLO
    if (n_total < 2) {
        /* Auto-ID heard nobody.  A swap needs two drones, and flying solo
         * after a failed negotiation is how duplicate-ID flights happen
         * (both drones alone-in-their-own-mind at id=0, mutually
         * invisible, hovering until the backstop — 2026-07-19 flight 4).
         * Recover WITHOUT a power-cycle instead — see transport_negotiate_
         * id_retry()'s doc for the two ways this resolves.
         * Deliberate solo flights: -DCONFIG_DEMO_ALLOW_SOLO=y. */
        LOG_ERR("id=%u auto-ID heard NO peers — NOT ARMING; grounded "
                "self-healing diagnostic mode (renegotiates until a peer "
                "is found)", (unsigned)element_id);
        substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);

        element_id = transport_negotiate_id_retry(element_id, &n_total);

        LOG_WRN("id=%u recovered — n_total=%d, proceeding to flight prep",
                (unsigned)element_id, n_total);
        substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
    }
#endif /* !CONFIG_DEMO_ALLOW_SOLO */
#else
    const element_id_t element_id = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;
    const int n_total = CONFIG_TAPESTRY_ELEMENT_COUNT;
#endif
    LOG_INF("CF21BL formation — element %u  n_total=%d  default_alt=%.2fm  "
            "target_spacing=%.2fm",
            (unsigned)element_id, n_total, (double)ALT_BASE_M,
            (double)DEMO_TARGET_SPACING_M);

#ifdef CONFIG_DEMO_MODE_CHOREO
    /* Real L5 SCR.  CONFIG_TAPESTRY_QUORUM_MIN/_TARGET (this Kconfig,
     * default 1/1 — this script only ever needs one fresh partner) were
     * previously dead config, unused anywhere in this example; same
     * options tapestry-scr-hw's reference element feeds to scr_init().
     * SCR_CAP_ACTUATOR satisfies the script's CHOREO_CAP_LOCOMOTION
     * requirement, enforced below by choreo_submit_script() now that a
     * real scr is registered.  SCR_CAP_ABS_POSITION satisfies the
     * battery-low RTH preemption's derived CHOREO_CAP_ABS_POSITION
     * requirement (choreo.c's derived_caps(): CONVERGE + frame ==
     * ABSOLUTE, which the RTH goal is) — this drone genuinely has it, via
     * the lighthouse fix cf21bl_stabilizer_get_pos_home() reads from. */
    scr_state_t scr;
    scr_init(&scr, element_id,
        (uint8_t)CONFIG_TAPESTRY_QUORUM_MIN,
        (uint8_t)CONFIG_TAPESTRY_QUORUM_TARGET,
        SCR_CAP_ACTUATOR | SCR_CAP_ABS_POSITION);
    /* Requiring >= QUORUM_UP_MS of SUSTAINED recovery before scr_tick()
     * reports quorum up — a single lucky gossip frame keeps a peer
     * "fresh" for WM_STALE_THRESHOLD_MS (1500 ms), so gating on
     * instantaneous freshness let lone packets flicker the Choreo awake
     * for a second at a time in flight (2026-07-19 flight 2). This used
     * to be a filter applied to a local COPY of scr here in main.c
     * (never written back), duplicated identically in webots-formation's
     * main.c; scr_set_quorum_hold_ms() moved it into L5 itself, where
     * every consumer of `scr` (not just choreo_tick() below) sees the
     * same held view — see that function's doc for the full semantics,
     * including why SCR_ABORT_CLEARED is now delayed by this same amount. */
    scr_set_quorum_hold_ms(&scr, QUORUM_UP_MS);

    choreo_init(element_id);
    choreo_register_scr(&scr);
    /* CHOREO_DEPARTURE_* come from the generated choreo_script.h —
     * .choreo.toml's `mode`/[on_departure] (default: CONTINUE, every
     * script written before this feature existed is unaffected). The
     * recall-point callback is registered even when the compiled policy
     * doesn't need it today — Phase 3's per-step on_departure override
     * can select RECALL for an individual step regardless of the
     * script-level default. */
    choreo_set_departure_policy(CHOREO_DEPARTURE_POLICY, CHOREO_DEPARTURE_REASONS,
                                CHOREO_DEPARTURE_MIN_PARTICIPANTS);
#ifdef CONFIG_PWM
    choreo_set_departure_recall_point_fn(cf21bl_departure_recall_point);
#endif
    /* k_choreo_script comes from the generated choreo_script.h — the
     * authored script is ../change-partners.choreo.toml (coordinate-free:
     * hold references each drone's own station, exchange references the
     * partner's; no takeoff/landing/altitude anywhere — quiescence at
     * script end maps to land-in-place below, and altitude staggering is
     * a platform deconfliction rule the Choreo never sees). */
    if (choreo_submit_script(k_choreo_script, CHOREO_SCRIPT_LEN) != 0) {
        LOG_ERR("id=%u choreo script rejected — staying grounded",
                (unsigned)element_id);
        substrate_set_power(SUBSTRATE_POWER_SLEEP);
        return -1;
    }
    LOG_INF("id=%u choreo \"%s\" loaded — %u steps, time bound %u s",
            (unsigned)element_id, CHOREO_NAME,
            (unsigned)CHOREO_SCRIPT_LEN,
            (unsigned)(CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u));
#endif

    LOG_INF("id=%u Waiting for lighthouse fix (up to 30 s) ...", (unsigned)element_id);
    {
        uint32_t deadline = k_uptime_get_32() + 30000u;
        while (!cf21bl_lighthouse_is_valid()) {
            if (k_uptime_get_32() > deadline) {
                LOG_ERR("id=%u No lighthouse fix — base stations on? poses correct? "
                        "staying grounded.", (unsigned)element_id);
                substrate_set_power(SUBSTRATE_POWER_SLEEP);
                return -1;
            }
            k_msleep(200);
        }
    }
    LOG_INF("id=%u Fix acquired", (unsigned)element_id);

#if defined(CONFIG_DEMO_START_DELAY_S) && (CONFIG_DEMO_START_DELAY_S > 0)
    /* Staggered join: hold on the ground BEFORE the arming countdown.  Not
     * gossiping yet, so already-flying peers don't count this drone until
     * it enters the flight loop — it then joins the formation visibly.
     * (Once it starts gossiping during its own ramp, peers correctly make
     * XY room for it before it reaches cruise — expected, and wanted for
     * separation.) */
    LOG_INF("id=%u staggered start — joining the formation in %d s",
            (unsigned)element_id, CONFIG_DEMO_START_DELAY_S);
    for (int i = CONFIG_DEMO_START_DELAY_S; i > 0; i--) {
        if (i <= 3 || (i % 5) == 0) {
            LOG_INF("id=%u  join in %d ...", (unsigned)element_id, i);
        }
        k_msleep(1000);
    }
#endif

    LOG_INF("id=%u PLACE ON GROUND (nose along lighthouse world +X) AND STAND CLEAR "
            "— arming in 5 s ...", (unsigned)element_id);
    for (int i = 5; i > 0; i--) {
        LOG_INF("id=%u  %d ...", (unsigned)element_id, i);
        k_msleep(1000);
    }

    element_state_t own_state = { 0 };
    own_state.id          = element_id;
    own_state.orientation = orientation_identity();  /* no attitude-estimate
                                                       * accessor exposed by
                                                       * cf21bl_stabilizer.c
                                                       * today — see this
                                                       * README's "Known
                                                       * limitations". If one
                                                       * is added, its zero
                                                       * must be calibrated to
                                                       * lighthouse world +X
                                                       * (the existing boot
                                                       * placement convention
                                                       * below), not raw
                                                       * gyro-relative-to-boot
                                                       * output — see
                                                       * orientation_t's
                                                       * comment in csm.h */
    /* own_state.position.z is set from own_pos_m.z below (declared in the
     * flight loop) — see that variable's comment for why. */

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);

    /* Target is absolute world-frame, meters. Initialized from this drone's
     * own first fix (below, in the flight loop) rather than world (0,0) —
     * with zero fresh peers demo_compute_drive() never moves the target, so
     * seeding it at the origin would command a translation to wherever
     * lighthouse (0,0) physically is instead of holding at the takeoff spot
     * (unlike lh2-hover, which always commands zero offset from home). */
    demo_setpoint_t target = { 0 };
    bool            target_init = false;
#ifdef CONFIG_DEMO_HOLD_STATION
    /* First-fix seed: the pinned member holds here for the whole mission. */
    float           seed_x = 0.0f;
    float           seed_y = 0.0f;
#endif

    uint32_t gossip_accum = DEMO_GOSSIP_MS;

    /* Convergence hold: wait until all expected peers are fresh (proceeds
     * anyway after the grace period — matches the old demo's behavior). */
#define DEMO_SYNC_GRACE_MS 5000
    for (uint32_t waited = 0; waited < DEMO_SYNC_GRACE_MS; waited += WM_CYCLE_MS) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        int fresh = 0;
        for (int i = 0; i < MAX_ELEMENTS; i++) {
            const wm_entry_t *e = &wm.entries[i];
            if (e->is_active && !e->is_self && !e->is_stale) {
                fresh++;
            }
        }
        if (fresh >= n_total - 1) {
            break;
        }

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= DEMO_GOSSIP_MS) {
            own_state.update_seq++;
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }
        k_msleep(WM_CYCLE_MS);
    }

    LOG_INF("id=%u Arming and entering flight loop", (unsigned)element_id);
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);

    flight_state_t state       = FLIGHT_RAMPING;
    float          alt_cmd_m   = ALT_RAMP_START_M;
    /* Choreo-commanded altitude setpoint — starts at the pre-Choreo
     * default (ALT_BASE_M) and is updated to directive.target.z inside
     * FLIGHT_FLYING below, once choreo_tick() has produced a
     * MOVE_TO_POINT directive. alt_cmd_m (above) is rate-limited toward
     * this every tick (see the continuous ramp block below) rather than
     * jumped to — the same gentle-approach property the takeoff ramp
     * always gave, now applied to any Choreo-commanded altitude change,
     * not just takeoff. */
    float          z_setpoint_m = ALT_BASE_M;
#ifdef CONFIG_PWM
    bool           land_requested = false;   /* stabilizer descent engaged */
#endif
    land_reason_t  land_reason   = LAND_REASON_NONE;
    uint32_t       mission_t0_ms = k_uptime_get_32();
    uint32_t       fix_lost_since_ms = 0;
    /* Origin-relative distance at the START of the current outage — see
     * the coast-budget backstop in the FLIGHT_FLYING !fix_valid case
     * below.  Meaningless while fix_lost_since_ms == 0. */
    float          fix_lost_origin_dist_m = 0.0f;
    uint32_t       land_settle_ms = 0;
    /* z: NOT a live baro reading — cf21bl_stabilizer.c exposes no altitude
     * accessor today. Tracks alt_cmd_m every tick (the commanded/ramping
     * trajectory, see below) instead of a value frozen for the whole
     * flight — real enough to be useful (this is where the drone is
     * actually being commanded to go) without fabricating sensor
     * precision that doesn't exist. A live reading is a follow-up, not
     * done here — see this README's "Known limitations". */
    position_t     own_pos_m   = { 0.0f, 0.0f, ALT_RAMP_START_M };
    bool           have_pos    = false;
    /* Land-in-place: latched at the moment a fix-valid landing (mission
     * end, geofence) begins.  The old behavior — holding X/Y at the boot
     * home — sent drones on long low-altitude cross-room transits at the
     * end of a mission (after the rotation phase nobody is near their own
     * pad; the slot swap means not even the survivors are), and the
     * 0.3 m/s descent lands them at an arbitrary point along the way.
     * Freezing the formation's final geometry is predictable and keeps
     * the end of the show clean.  Fix-LOSS landings deliberately do NOT
     * latch (land_hold_valid stays false → X/Y zeroed, the stabilizer's
     * fix-lost velocity-damp fallback): latching a position we can no
     * longer measure is meaningless. */
    bool           land_hold_valid = false;
    float          land_hold_x = 0.0f;
    float          land_hold_y = 0.0f;

    while (true) {
        transport_drain(&wm, element_id);
        wm_tick(&wm, WM_CYCLE_MS);

        lh2_position_t lhpos;
        bool fix_valid = cf21bl_lighthouse_is_valid()
                          && cf21bl_lighthouse_get_position(&lhpos) == 0;
        if (fix_valid) {
            own_pos_m.x = lhpos.x;
            own_pos_m.y = lhpos.y;
            have_pos    = true;
        }

        if (have_pos && !target_init) {
            /* Seed the target at this drone's own position (same fix used
             * for cf21bl_stabilizer.c's home capture on the same first
             * non-idle tick) so zero-peer flight holds station instead of
             * translating to world (0,0). */
            demo_setpoint_init(&target, own_pos_m.x, own_pos_m.y);
#ifdef CONFIG_DEMO_HOLD_STATION
            seed_x = own_pos_m.x;
            seed_y = own_pos_m.y;
#endif
            target_init = true;
        }

        /* Rate-limit alt_cmd_m toward z_setpoint_m every tick — not just
         * during the initial takeoff ramp, so a Choreo-commanded altitude
         * CHANGE mid-mission is approached gently too, the same safety
         * property FLIGHT_RAMPING always gave takeoff. Excluded once
         * FLIGHT_LANDING starts: that state runs its own separate
         * closed-loop descent (cf21bl_stabilizer_request_land()) and must
         * not fight it. */
        if (state == FLIGHT_RAMPING || state == FLIGHT_FLYING) {
            if (alt_cmd_m < z_setpoint_m) {
                alt_cmd_m += ALT_RAMP_RATE_MPS * LOOP_DT_S;
                if (alt_cmd_m > z_setpoint_m) { alt_cmd_m = z_setpoint_m; }
            } else if (alt_cmd_m > z_setpoint_m) {
                alt_cmd_m -= ALT_RAMP_RATE_MPS * LOOP_DT_S;
                if (alt_cmd_m < z_setpoint_m) { alt_cmd_m = z_setpoint_m; }
            }
            own_pos_m.z = alt_cmd_m;
        }

        if (have_pos) {
            own_state.position = own_pos_m;
        }
        /* Until the first fix, own_state.position is still the zero-init
         * placeholder from element_state_t own_state = { 0 } — and we
         * gossip regardless, because discovery and auto-ID recovery depend
         * on peers hearing us before anyone has a fix.  Say so on the wire
         * instead of going silent, so receivers can exclude the phantom
         * without losing us as a peer (see ELEMENT_HEALTH_NO_POSITION).
         *
         * A fix lost mid-flight is a different claim: own_pos_m holds its
         * last real measurement (not zeroed — see the FLIGHT_FLYING
         * !fix_valid case below) and we keep gossiping it, both so peers
         * keep seeing us as present (the flight-12 deadlock this file's
         * "keep gossiping after landing" comment describes further down)
         * and because a real last-known position is still worth having for
         * separation.  ELEMENT_HEALTH_POSITION_STALE says the position is
         * held, not current, without asking receivers to drop us. */
        uint8_t health = ELEMENT_HEALTH_OK;
        if (!have_pos) {
            health |= ELEMENT_HEALTH_NO_POSITION;
        } else if (!fix_valid) {
            health |= ELEMENT_HEALTH_POSITION_STALE;
        }
        /* Self-declare departure the moment descent starts, not just once
         * landed: a peer must be able to exclude us from collective
         * predicates (scope="all", swap-partner selection) as soon as we
         * are no longer a going participant, not only after touchdown.
         * One WM_CYCLE_MS tick of lag versus `state`/`land_reason`
         * (both set later in this same iteration's switch, read here at
         * the top of the NEXT iteration) is immaterial next to
         * WM_EXPIRE_THRESHOLD_MS. */
        if (state == FLIGHT_LANDING || state == FLIGHT_LANDED) {
            health = element_health_set_departed(health,
                                                  land_departure_reason(land_reason));
        }
#ifdef CONFIG_CF21BL_PM
        float vbat = cf21bl_pm_vbat();
        if (vbat > 0.0f) {
            float pct = (vbat - 3.0f) / (4.2f - 3.0f);
            if (pct < 0.0f) { pct = 0.0f; }
            if (pct > 1.0f) { pct = 1.0f; }
            own_state.energy_level = (uint8_t)(pct * 100.0f);
        }
        if (cf21bl_pm_battery_low()) {
            health |= ELEMENT_HEALTH_LOW_BATTERY;
        }
#endif
        own_state.health_flags = health;
        wm_update_self(&wm, &own_state);

        substrate_twist_t sp = { 0 };
#ifdef CONFIG_DEMO_MODE_CHOREO
        bool log_quorum_up = false;   /* debounced quorum, set in FLYING */
        /* Nearest peer, surfaced in the 1 Hz status line below.
         * It used to live only in formation.c's per-tick DBG trace, so
         * throttling or quieting that module took the separation margin
         * with it — and the margin is exactly what you want when a flight
         * runs near DEMO_MIN_SEP_M (flight 16 spent its swap at 0.53 m
         * against a 0.50 m floor and nothing in the default log said so).
         * -1 now means "no ACTIVE peer at all" — separation genuinely
         * UNKNOWN, not merely unrefreshed; log_sep says whether the
         * reported distance came from a fresh measurement or from a stale
         * entry's last-known position (see demo_sep_t). */
        float      log_min_dist_m   = -1.0f;
        demo_sep_t log_sep          = { .stale = false, .age_ms = 0 };
        /* False when no separation measurement was taken this tick at all —
         * distinct from "measured, found nothing".  Without it the status
         * line printed a bare min_d=-1.00 in states that never ran a scan
         * (fix lost, RAMPING, LANDED), which reads identically to the one
         * case -1.00 is supposed to mean: every peer expired.  See the '?'
         * marker at the status line below. */
        bool       log_sep_measured = false;
#endif

        switch (state) {
        case FLIGHT_RAMPING:
            /* alt_cmd_m is rate-limited toward z_setpoint_m above, every
             * tick — this just waits for it to arrive before formation
             * control engages. */
            if (alt_cmd_m >= z_setpoint_m) {
                state = FLIGHT_FLYING;
                LOG_INF("id=%u Default altitude reached — formation control active",
                        (unsigned)element_id);
            }
            sp.linear.z = alt_cmd_m - 1.0f;
            break;

        case FLIGHT_FLYING: {
            if (!fix_valid) {
                uint32_t blind_ms;
                if (fix_lost_since_ms == 0) {
                    fix_lost_since_ms = k_uptime_get_32();
                    fix_lost_origin_dist_m =
                        sqrtf(own_pos_m.x * own_pos_m.x
                              + own_pos_m.y * own_pos_m.y);
                    LOG_WRN("id=%u lighthouse fix lost (age %u ms) — "
                            "braking, then holding level in place, "
                            "gossiping last-known position (flagged "
                            "stale)", (unsigned)element_id,
                            cf21bl_lighthouse_fix_age_ms());
                }
                blind_ms = k_uptime_get_32() - fix_lost_since_ms;

                /* Coast-budget backstop.  cf21bl_stabilizer.c now brakes
                 * on fix loss (CF21BL_BRAKE_MS there), but it is best-
                 * effort and open-loop — main.c has no way to confirm it
                 * actually arrested the drone's motion, since confirming
                 * that needs the very position reading that just stopped
                 * arriving.  So bound the worst case independently of
                 * whether braking worked: assume the drone COULD be
                 * coasting at the demo's own top commanded speed, aimed
                 * straight away from the origin, for the entire blind
                 * duration so far, and land the instant that projection
                 * would put it past the geofence — well before waiting
                 * out the full FIX_LOSS_GRACE_MS.  Deliberately
                 * pessimistic (most outages end well inside this bound,
                 * and braking should mean the true coast speed is far
                 * below DEMO_MAX_SPEED_MPS after the first
                 * CF21BL_BRAKE_MS or so) rather than tracking real drift,
                 * which needs a position this branch by definition does
                 * not have.  (2026-09-01 flight 49: a real coast averaged
                 * almost exactly DEMO_MAX_SPEED_MPS over ~5.3 s blind,
                 * breaching the 2.0 m geofence — this bound would have
                 * caught it at ~2.6 s from a starting distance of 1.22 m.) */
                float coast_budget_m =
                    GEOFENCE_RADIUS_M - fix_lost_origin_dist_m
                    - DEMO_MAX_SPEED_MPS * ((float)blind_ms * 0.001f);
                if (coast_budget_m <= 0.0f) {
                    LOG_ERR("id=%u fix lost %u ms from %.2f m out — a "
                            "coast at top speed could reach the geofence — "
                            "landing independently", (unsigned)element_id,
                            blind_ms, (double)fix_lost_origin_dist_m);
                    state = FLIGHT_LANDING;
                    land_reason = LAND_REASON_FIXLOSS;
                    land_hold_valid = false;   /* no fix — no position to hold */
                    sp.linear.z = LAND_HOLD_CMD_M - 1.0f;
                    break;
                }

                if (blind_ms > FIX_LOSS_GRACE_MS) {
                    LOG_ERR("id=%u fix lost > %d ms — landing independently",
                            (unsigned)element_id, FIX_LOSS_GRACE_MS);
                    state = FLIGHT_LANDING;
                    land_reason = LAND_REASON_FIXLOSS;
                    land_hold_valid = false;   /* no fix — no position to hold */
                    sp.linear.z = LAND_HOLD_CMD_M - 1.0f;
                    break;
                }
                /* Zero X/Y.  With CONFIG_CF21BL_LIGHTHOUSE_POS_HOLD (this
                 * build) there is no velocity feedforward for a zeroed
                 * linear.x/y to inherit — the stabilizer's own fix-lost
                 * path applies a short braking pulse from the last known
                 * velocity and then drops the position correction to
                 * zero (see CF21BL_BRAKE_MS in cf21bl_stabilizer.c),
                 * i.e. level attitude, no commanded lean: stop
                 * accelerating rather than continuing to steer, with a
                 * best-effort attempt to cancel existing motion first
                 * instead of riding it out on drag alone.  Setting these
                 * explicitly is still correct defense in depth against a
                 * stale caller-side value. */
                sp.linear.x = 0.0f;
                sp.linear.y = 0.0f;
                sp.linear.z = alt_cmd_m - 1.0f;
                break;
            }
            fix_lost_since_ms = 0;

            /* Deliberately horizontal-only, re-examined (not left stale)
             * now that own_pos_m.z is Choreo-commanded and genuinely
             * varies rather than a fixed per-ID constant: kept as a
             * radius, not a sphere, since this room has a flat floor and
             * no modeled ceiling — a horizontal-only boundary is still
             * the more natural safety envelope. (formation.c's peer
             * separation folds in z per a separate, explicit choice —
             * see position_distance()'s comment in csm.h — this geofence
             * is a different check, against the room origin, not a peer.) */
            float origin_dist = sqrtf(own_pos_m.x * own_pos_m.x
                                       + own_pos_m.y * own_pos_m.y);
            if (origin_dist > GEOFENCE_RADIUS_M) {
                LOG_ERR("id=%u geofence breach (%.2f m > %.2f m) — landing independently",
                        (unsigned)element_id,
                        (double)origin_dist, (double)GEOFENCE_RADIUS_M);
                state = FLIGHT_LANDING;
                land_reason = LAND_REASON_GEOFENCE;
                land_hold_x = own_pos_m.x;
                land_hold_y = own_pos_m.y;
                land_hold_valid = true;
                sp.linear.z = LAND_HOLD_CMD_M - 1.0f;
                break;
            }

            if (k_uptime_get_32() - mission_t0_ms > (uint32_t)MISSION_DURATION_S * 1000u) {
                LOG_INF("id=%u mission duration elapsed — landing in place at "
                        "(%.2f, %.2f)", (unsigned)element_id,
                        (double)own_pos_m.x, (double)own_pos_m.y);
                state = FLIGHT_LANDING;
                land_reason = LAND_REASON_BACKSTOP;
                land_hold_x = own_pos_m.x;
                land_hold_y = own_pos_m.y;
                land_hold_valid = true;
                sp.linear.z = LAND_HOLD_CMD_M - 1.0f;
                break;
            }

#ifdef CONFIG_DEMO_MODE_CHOREO
            /* Real L5: recompute quorum/role/task_slot/abort from the
             * actual world model. scr_set_quorum_hold_ms() above means
             * scr.quorum_state (and everything scr_tick() derives from
             * it — abort_state, leader, role, task_slot) is already the
             * held view: a LOST -> >=DEGRADED recovery must be SUSTAINED
             * for QUORUM_UP_MS before it's reported, filtering out the
             * single-lucky-gossip-frame flicker that used to run the
             * tracker briefly and ratchet the target toward a (possibly
             * corrupt) position estimate (2026-07-19 flight 2). Quorum
             * LOSS remains immediate regardless — see scr_set_quorum_
             * hold_ms()'s doc. */
            scr_tick(&scr, &wm);

            float nearest_m = -1.0f;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self && !e->is_stale) {
                    float dx = e->state.position.x - own_pos_m.x;
                    float dy = e->state.position.y - own_pos_m.y;
                    float d  = sqrtf(dx * dx + dy * dy);
                    if (nearest_m < 0.0f || d < nearest_m) {
                        nearest_m = d;
                    }
                }
            }
            bool quorum_up = scr.quorum_state != SCR_QUORUM_LOST;
            log_quorum_up = quorum_up;

            /* Station-compatibility check, once per contact: stations
             * closer than ~2× the separation floor mean station-keeping
             * and the exchange will fight the emergency repulsion the
             * whole flight — a placement (or lighthouse-bias) problem no
             * amount of choreography survives. */
            static bool was_up;
            if (quorum_up && !was_up && nearest_m >= 0.0f &&
                nearest_m < 2.0f * DEMO_MIN_SEP_M) {
                LOG_WRN("id=%u peers only %.2f m apart (floor %.2f m) — "
                        "station-keeping will fight separation repulsion; "
                        "place drones >= %.1f m apart (or suspect a biased "
                        "lighthouse frame)",
                        (unsigned)element_id, (double)nearest_m,
                        (double)DEMO_MIN_SEP_M,
                        (double)(2.0f * DEMO_MIN_SEP_M));
            }
            was_up = quorum_up;

#if defined(CONFIG_CF21BL_PM) && defined(CONFIG_PWM)
            /* Battery-triggered preemption (the L6/L7 goal-queue demo):
             * on the low-battery edge, park whatever's running and fly
             * straight home via choreo_preempt_goal(). CONVERGE, not
             * MOVE — MOVE's directive is intent.target displaced by this
             * drone's offset from the participant centroid (see bse.h),
             * which is wrong here: the drone wants literally home, not
             * home + formation offset. CONVERGE collapses straight to
             * the target and is otherwise unused by this demo's script,
             * so it's unambiguous when it appears. cf21bl_stabilizer_get_
             * pos_home() needs CONFIG_PWM for the same reason the other
             * two call sites below guard it that way. */
            static bool was_battery_low;
            bool battery_low =
                (own_state.health_flags & ELEMENT_HEALTH_LOW_BATTERY) != 0;
            if (battery_low && !was_battery_low && !choreo_is_preempted()) {
                float home_x, home_y;
                if (cf21bl_stabilizer_get_pos_home(&home_x, &home_y)) {
                    /* z: own_pos_m.z (the current commanded cruise
                     * altitude), not 0 — cf21bl_stabilizer_get_pos_home()
                     * only returns a lighthouse x/y, and CONVERGE's target
                     * is read verbatim as the altitude setpoint
                     * (z_setpoint_m = dir->target.z below). Leaving z at
                     * its implicit 0 would command an immediate ramp to
                     * ground level while still translating home — this
                     * demo's own landing sequence already lands in place
                     * once quiescent, so RTH only needs to move laterally. */
                    /* Home is where WE took off, which after an EXCHANGE
                     * is where the PARTNER now is.  RTH ends in a landing,
                     * so aiming at an occupied spot is a collision, not a
                     * close pass — and the emergency repulsion cannot save
                     * it, because the goal point itself is the problem.
                     * Nudge the destination clear before submitting it. */
                    float rth_x = home_x, rth_y = home_y;
                    bool  moved = demo_deconflict_point(&wm, &rth_x, &rth_y);

                    choreo_goal_t rth = {
                        .type   = CHOREO_GOAL_CONVERGE,
                        .target = { rth_x, rth_y, own_pos_m.z },
                    };
                    int rc = choreo_preempt_goal(&rth);
                    if (moved) {
                        LOG_WRN("id=%u battery low — preempting to "
                                "return-to-home (%.2f, %.2f), rc=%d "
                                "[deconflicted from home (%.2f, %.2f): a "
                                "peer is parked there]",
                                (unsigned)element_id, (double)rth_x,
                                (double)rth_y, rc, (double)home_x,
                                (double)home_y);
                    } else {
                        LOG_WRN("id=%u battery low — preempting to "
                                "return-to-home (%.2f, %.2f), rc=%d",
                                (unsigned)element_id, (double)rth_x,
                                (double)rth_y, rc);
                    }
                }
            }
            was_battery_low = battery_low;
#endif

#ifdef CONFIG_PWM
            /* Current commanded cruise altitude — see
             * cf21bl_departure_recall_point()'s comment for why a plain
             * callback needs this mirrored instead of reading own_pos_m.z
             * directly. */
            g_departure_recall_z_m = own_pos_m.z;
#endif
            choreo_tick(&wm, &scr);
            choreo_publish_state(&own_state);

            /* HARD_RT gossip on the quorum-loss edge. Not delayed by
             * QUORUM_UP_MS despite scr's held view above: loss is always
             * reported immediately (scr_set_quorum_hold_ms() only ever
             * holds the RECOVERY edge) — the wire signal that this
             * element's quorum just failed goes out the same tick it
             * happens, same as before this was moved into L5. Fires once
             * per outage, not every tick it persists — scr_get_abort_
             * state() is a level held for the whole LOST period (see
             * scr.h), so this checks the NONE/CLEARED -> TRIGGERED edge
             * specifically. Mirrors tapestry-os/subsys/runtime/
             * runtime.c's step 4b. */
            static scr_abort_state_t last_abort_state;
            scr_abort_state_t abort_state = scr_get_abort_state(&scr);

            if (abort_state == SCR_ABORT_TRIGGERED &&
                last_abort_state != SCR_ABORT_TRIGGERED) {
                own_state.update_seq++;
                LOG_WRN("id=%u quorum LOST — sending HARD_RT gossip now",
                        (unsigned)element_id);
                transport_send(&own_state, TAPESTRY_QOS_HARD_RT);
                gossip_accum = 0;
            }
            last_abort_state = abort_state;

            static int last_step = -2;
            if (choreo_script_step() != last_step) {
                last_step = choreo_script_step();
                LOG_INF("id=%u choreo step %d %s", (unsigned)element_id,
                        last_step,
                        choreo_goal_status() == CHOREO_STATE_SUSPENDED
                            ? "(suspended)" : "");
            }

            /* Checked BEFORE choreo_script_complete(): a departure-policy
             * landing (LAND_IN_PLACE, a completed RECALL, or a HOLD that
             * timed out) also sets s_script_done internally
             * (land_for_departure(), choreo.c) — the same quiescence
             * signal a normal finish uses — so choreo_script_complete()
             * would ALSO read true afterward. Checking the more specific
             * flag first is what lets land_reason distinguish the two. */
            if (choreo_departure_triggered()) {
                LOG_INF("id=%u departure policy landed us at (%.2f, %.2f)",
                        (unsigned)element_id,
                        (double)own_pos_m.x, (double)own_pos_m.y);
                state = FLIGHT_LANDING;
                land_reason = LAND_REASON_DEPARTURE;
                land_hold_x = own_pos_m.x;
                land_hold_y = own_pos_m.y;
                land_hold_valid = true;
                sp.linear.z = LAND_HOLD_CMD_M - 1.0f;
                break;
            }

            if (choreo_script_complete()) {
                /* Quiescence: the script is done and the directive is IDLE.
                 * This platform's inactive posture is "on the ground,
                 * disarmed" — land in place.
                 *
                 * "In place" means the STATION the final step commanded,
                 * not necessarily where own_pos_m happens to read this
                 * exact tick. choreo_script_complete() fires the tick
                 * AFTER the last step's own advance condition was met — if
                 * that step was HOLD, bse.c's HOLD-inherits-the-prior-
                 * achieved-goal-point fix (bse.h's HOLD doc) already keeps
                 * this tight, but choreo_terminate() has, by this point,
                 * already submitted the IDLE intent that ends the script
                 * (advance_to() in choreo.c), so choreo_get_directive()
                 * here reads IDLE, not the settled HOLD target — there is
                 * nothing left to read the station back from. Latching
                 * own_pos_m is therefore still correct for a script ENDING
                 * on HOLD's achievement (the whole point of the bse.c fix
                 * is that own_pos_m at this instant already equals the
                 * commanded station, not a coasted-past overshoot of it).
                 * A script ending on a bare MOVE/FORM/CONVERGE/EXCHANGE
                 * step's own achievement (no trailing HOLD) — or on a
                 * TIMEOUT rather than achievement — still lands at
                 * whatever own_pos_m reads, same as before this comment;
                 * that residual case is the achieve_eps/tracker-lag gap
                 * bse.c's HOLD fix cannot reach without a HOLD step to
                 * land the inheritance on. */
                LOG_INF("id=%u choreo complete — resting: landing in place "
                        "at (%.2f, %.2f)", (unsigned)element_id,
                        (double)own_pos_m.x, (double)own_pos_m.y);
                state = FLIGHT_LANDING;
                land_reason = LAND_REASON_COMPLETE;
                land_hold_x = own_pos_m.x;
                land_hold_y = own_pos_m.y;
                land_hold_valid = true;
                sp.linear.z = LAND_HOLD_CMD_M - 1.0f;
                break;
            }

            float      min_dist_m = -1.0f;
            demo_sep_t sep        = { .stale = false, .age_ms = 0 };
            const tapestry_bse_directive_t *dir = choreo_get_directive();
            /* Per-goal quorum at the tracking layer too: a HOLD directive
             * references only this drone's own station, so it is tracked
             * even with quorum lost (a solo drone station-keeps properly);
             * peer-referential directives are frozen while LOST. CONVERGE
             * is included here too: this demo's own script never submits
             * CONVERGE, so it can only mean the battery-preempt RTH goal
             * above, which references only this drone's own home
             * coordinate — an isolated, low-battery drone must still be
             * able to fly home. */
            bool self_referential =
                choreo_current_goal_type() == CHOREO_GOAL_HOLD ||
                choreo_current_goal_type() == CHOREO_GOAL_CONVERGE;
            if ((quorum_up || self_referential) &&
                dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
                min_dist_m = demo_choreo_track(&wm, &own_pos_m, &target,
                                               dir->target.x, dir->target.y,
                                               WM_CYCLE_MS, element_id, &sep);
                z_setpoint_m = dir->target.z;
            } else {
                /* Quorum LOST on a peer-referential goal (choreo SUSPENDED)
                 * or directive HOLD (exchange awaiting its snapshot) —
                 * target frozen where it is; the stabilizer keeps station
                 * on it.  Separation is still measured directly: the
                 * airframe is airborne and a peer can close on it whether
                 * or not this drone is tracking anything, and reporting
                 * UNKNOWN here purely because no drive ran would leave the
                 * same blind spot the drives just had. */
                min_dist_m = demo_min_separation(&wm, &own_pos_m, &sep);
            }

            log_min_dist_m   = min_dist_m;
            log_sep          = sep;
            log_sep_measured = true;   /* both branches above measured */
#else /* DEMO_MODE_SHOWCASE */
            demo_sep_t sep = { .stale = false, .age_ms = 0 };
            float min_dist_m = demo_compute_drive(&wm, &own_pos_m, &target, WM_CYCLE_MS,
                                                  element_id, &sep);
#ifdef CONFIG_DEMO_HOLD_STATION
            /* Pinned member: the drive above still ran (its min_dist_m
             * feeds the separation warning below, and its LOG_DBG keeps
             * this drone's view of the field observable), but the target
             * stays at the first-fix seed — formation forces are observed,
             * not obeyed.  Peers still see this drone and form around it. */
            demo_setpoint_init(&target, seed_x, seed_y);
#endif
#endif /* CONFIG_DEMO_MODE_CHOREO */
            if (min_dist_m >= 0.0f && min_dist_m < DEMO_MIN_SEP_M) {
                static int sep_log_div;
                if (++sep_log_div >= 20) {   /* ~2 Hz at WM_CYCLE_MS=100 */
                    sep_log_div = 0;
                    if (sep.stale) {
                        /* Same severity as a confirmed violation — a
                         * two-second-old position is the best evidence
                         * available and "probably too close" is still too
                         * close — but worded so a log reader can never
                         * mistake it for a fresh measurement.  The forces
                         * did NOT react to this peer (repulsion stays on
                         * fresh peers only), which is exactly why the
                         * warning matters. */
                        LOG_WRN("id=%u separation violation (last known, "
                                "%ums stale): nearest peer %.2f m (min %.2f m)",
                                (unsigned)element_id, (unsigned)sep.age_ms,
                                (double)min_dist_m, (double)DEMO_MIN_SEP_M);
                    } else {
                        LOG_WRN("id=%u separation violation: nearest peer %.2f m "
                                "(min %.2f m)", (unsigned)element_id,
                                (double)min_dist_m, (double)DEMO_MIN_SEP_M);
                    }
                }
            } else if (min_dist_m < 0.0f) {
                /* No ACTIVE peer at all to measure against: separation is
                 * UNKNOWN, not known-safe.  This used to be reached
                 * whenever no peer was FRESH, which was ~57% of flight 25's
                 * status samples (27/45 and 24/45) — more than half the
                 * flight with the check above inert.  Now that stale
                 * entries carry their last-known position into the
                 * measurement, reaching here means every peer has passed
                 * WM_EXPIRE_THRESHOLD_MS (5 s of silence, presumed gone),
                 * which is the end-of-show case this warning was written
                 * for: the first finisher lands and goes gossip-silent (the
                 * FLIGHT_LANDED gate at the send site) while a partner
                 * flies out its remaining seconds (2026-08-24 flight 15:
                 * ~6 s of it).  Rare now, and genuinely blind when it
                 * happens. */
                static int sep_blind_div;
                if (++sep_blind_div >= 50) {   /* ~0.2 Hz at WM_CYCLE_MS=100 */
                    sep_blind_div = 0;
                    LOG_WRN("id=%u separation UNKNOWN — no active peer in view",
                            (unsigned)element_id);
                }
            }

#ifdef CONFIG_PWM
            /* cf21bl_stabilizer.c (and this getter) only exists in this
             * build when CONFIG_PWM=y — see CMakeLists.txt's gossip-only
             * (-DCONFIG_PWM=n, substrate_null) test mode. */
            float home_x, home_y;
            if (cf21bl_stabilizer_get_pos_home(&home_x, &home_y)) {
                float nx = (target.x - home_x) / (float)CONFIG_CF21BL_POS_MAX_M;
                float ny = (target.y - home_y) / (float)CONFIG_CF21BL_POS_MAX_M;
                sp.linear.x = nx < -1.0f ? -1.0f : (nx > 1.0f ? 1.0f : nx);
                sp.linear.y = ny < -1.0f ? -1.0f : (ny > 1.0f ? 1.0f : ny);
            }
#endif
            sp.linear.z = alt_cmd_m - 1.0f;
            break;
        }

        case FLIGHT_LANDING:
#ifdef CONFIG_DEMO_MODE_CHOREO
            /* Descending is not the same as blind: own position is still
             * valid while the fix holds, peers still gossip, and a partner
             * drifting overhead during the descent is exactly the sort of
             * thing the flight log should be able to show afterwards.  No
             * warning is raised from here — that stays a FLYING concern —
             * this only keeps min_d honest instead of reporting -1.00 for
             * the whole descent. */
            if (fix_valid) {
                demo_sep_t land_sep;
                log_min_dist_m   = demo_min_separation(&wm, &own_pos_m,
                                                       &land_sep);
                log_sep          = land_sep;
                log_sep_measured = true;
            }
#endif
#ifdef CONFIG_PWM
            /* Hand the vertical profile to the stabilizer's closed-loop
             * descent on the first LANDING tick.  It walks the target down
             * from the measured altitude and cuts on ground settle, so
             * nothing here needs to (or may) drive linear.z toward the
             * idle sentinel — doing that is what dropped the airframe. */
            if (!land_requested) {
                land_requested = true;
                cf21bl_stabilizer_request_land(true);
            }
#endif
            sp.linear.x = 0.0f;
            sp.linear.y = 0.0f;
#ifdef CONFIG_PWM
            /* Land in place (see land_hold_valid above): hold the latched
             * position instead of translating home during the descent. */
            if (land_hold_valid) {
                float home_x, home_y;
                if (cf21bl_stabilizer_get_pos_home(&home_x, &home_y)) {
                    float nx = (land_hold_x - home_x) / (float)CONFIG_CF21BL_POS_MAX_M;
                    float ny = (land_hold_y - home_y) / (float)CONFIG_CF21BL_POS_MAX_M;
                    sp.linear.x = nx < -1.0f ? -1.0f : (nx > 1.0f ? 1.0f : nx);
                    sp.linear.y = ny < -1.0f ? -1.0f : (ny > 1.0f ? 1.0f : ny);
                }
            }
#endif
            /* Deliberately NOT walked toward -1.0: this must stay clear of
             * the idle sentinel for the whole descent, or the stabilizer
             * reads it as "motors off" and the closed-loop landing above
             * never gets to finish. */
            sp.linear.z = LAND_HOLD_CMD_M - 1.0f;

            {
                /* Primary: the stabilizer settled the airframe on the
                 * ground and latched its motors off. */
                bool settled = false;
#ifdef CONFIG_PWM
                settled = cf21bl_stabilizer_is_landed();
#endif
                /* Backstop (see the LAND_* block comment): an independent
                 * measured-lighthouse touchdown check, bounded by
                 * LAND_FORCE_DISARM_MS so a missing or z-biased fix cannot
                 * hold the drone in LANDING forever.  This is also the
                 * whole gate on builds without CONFIG_CF21BL_ALTITUDE_HOLD,
                 * where there is no closed-loop descent to defer to. */
                static uint32_t land_zero_ms;
                land_zero_ms += WM_CYCLE_MS;

                bool down = true;
                lh2_position_t lz;
                if (cf21bl_lighthouse_is_valid() &&
                    cf21bl_lighthouse_get_position(&lz) == 0) {
                    down = lz.z <= LAND_TOUCHDOWN_Z_M;
                }
                if (down || land_zero_ms >= LAND_FORCE_DISARM_MS) {
                    land_settle_ms += WM_CYCLE_MS;
                } else {
                    land_settle_ms = 0;
                }

                if (settled || land_settle_ms >= LAND_SETTLE_MS) {
                    state = FLIGHT_LANDED;
                    LOG_INF("id=%u landed — disarming (%s)",
                            (unsigned)element_id,
                            settled ? "stabilizer settle"
                                    : (down ? "z gate confirmed"
                                            : "z gate timed out"));
                }
            }
            break;

        case FLIGHT_LANDED:
        default:
            sp.linear.x = 0.0f;
            sp.linear.y = 0.0f;
            sp.linear.z = -1.0f;
            break;
        }

        substrate_move(&sp);
#ifdef CONFIG_DEMO_MODE_CHOREO
        demo_set_leds(&wm, choreo_current_indicator());
#else
        /* Showcase mode links no choreo.c and does not include choreo.h —
         * SUBSTRATE_SIGNAL_NONE is the "no script indicator override"
         * value, i.e. exactly the quorum/freshness heuristic this mode had
         * before per-step indicators existed. */
        demo_set_leds(&wm, SUBSTRATE_SIGNAL_NONE);
#endif

        static uint32_t log_accum;
        log_accum += WM_CYCLE_MS;
        if (log_accum >= 1000) {
            log_accum = 0;
            int fresh = 0, active = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self) {
                    active++;
                    if (!e->is_stale) { fresh++; }
                }
            }
#ifdef CONFIG_DEMO_MODE_CHOREO
            /* q=H/L is the DEBOUNCED quorum; (susp) = choreo SUSPENDED.
             * Flight 2 hid the flicker story because neither was logged. */
            /* min_d provenance, as a one-character suffix:
             *   (none)  fresh measurement against a fresh peer
             *   *       nearest peer was stale — last-known position, no
             *           repulsion acted on it
             *   ?       not measured this tick (own fix lost, or a state
             *           that runs no drive) — separation unknown because
             *           WE are unlocated, which is not the same claim as
             *           min_d=-1.00 with no suffix: that one means the scan
             *           ran and every peer had expired.
             *
             * step=-1 is ambiguous on its own — the script deactivates both
             * when it completes and when choreo_preempt_goal() parks it —
             * so a preempting goal (battery RTH) is called out explicitly.
             * Flight 42 lost its "preempting to return-to-home" WRN to
             * console splicing and the 1 Hz line showed a drone flying
             * 1.5 m across the arena at step=-1 with no stated reason.
             * why= is absent while still flying. */
            const char *sep_mark = !log_sep_measured ? "?"
                                 : (log_sep.stale    ? "*" : "");
            LOG_INF("id=%u %s peers %d/%d pos=(%.2f,%.2f) tgt=(%.2f,%.2f) alt=%.2f "
                    "cmd_z=%.2f min_d=%.2f%s step=%d%s q=%c%s%s%s",
                    (unsigned)element_id, flight_state_name(state), fresh, active,
                    (double)own_pos_m.x, (double)own_pos_m.y,
                    (double)target.x, (double)target.y,
                    (double)alt_cmd_m, (double)sp.linear.z,
                    (double)log_min_dist_m, sep_mark,
                    choreo_script_step(),
                    choreo_is_preempted() ? "(preempt)" : "",
                    log_quorum_up ? 'H' : 'L',
                    choreo_goal_status() == CHOREO_STATE_SUSPENDED ? "(susp)" : "",
                    choreo_goal_achieved() ? " achieved" : "",
                    land_reason_name(land_reason));
#else
            LOG_INF("id=%u %s peers %d/%d pos=(%.2f,%.2f) tgt=(%.2f,%.2f) alt=%.2f "
                    "cmd_z=%.2f",
                    (unsigned)element_id, flight_state_name(state), fresh, active,
                    (double)own_pos_m.x, (double)own_pos_m.y,
                    (double)target.x, (double)target.y,
                    (double)alt_cmd_m, (double)sp.linear.z);
#endif
        }

        gossip_accum += WM_CYCLE_MS;
#ifdef CONFIG_DEMO_MODE_CHOREO
        /* Choreo mode: keep gossiping after landing — see the deadlock this
         * fixes below.  DEMO_MIN_SEP_M repulsion also still sees this
         * drone as a real physical obstacle, which is wanted: a landed
         * airframe is a genuine collision hazard for a still-active
         * partner's beeline. */
        bool keep_gossiping = true;
#else
        /* Showcase mode: a LANDED drone goes gossip-silent — it has left
         * the collective, and peers must be able to expire it
         * (WM_EXPIRE_THRESHOLD_MS) so the formation heals around the
         * survivors (the "member departs" beat).  LANDING (still airborne,
         * descending) keeps gossiping — peers should make room for it
         * until it is actually down. */
        bool keep_gossiping = (state != FLIGHT_LANDED);
#endif
        /*
         * Why choreo mode differs (2026-07-19 flight 12 deadlock): a
         * 2-element script has no "departs and is healed around" beat —
         * both elements are expected to finish.  If the first finisher
         * went silent on landing, its partner's world-model entry for it
         * ages past WM_STALE_THRESHOLD_MS, the debounced quorum drops to
         * LOST (the partner's only peer just vanished), and choreo
         * SUSPENDS.  EXCHANGE is peer-referential, so it freezes on
         * suspension — INCLUDING its own step timeout, which only counts
         * while RUNNING.  With no live peer to ever restore quorum, the
         * partner is stuck hovering, frozen mid-exchange, rescued only by
         * the unconditional MISSION_DURATION_S backstop tens of seconds
         * later.  Keeping the landed element's gossip alive keeps its
         * peer's quorum up, so the exchange step's own (much tighter)
         * timeout governs the wait instead.
         */
        if (keep_gossiping && gossip_accum >= DEMO_GOSSIP_MS) {
            own_state.update_seq++;
            transport_send(&own_state, TAPESTRY_QOS_SOFT_RT);
            gossip_accum = 0;
        }

        if (state == FLIGHT_LANDED) {
            static bool slept;
            if (!slept) {
                slept = true;
                k_msleep(500);
                substrate_set_power(SUBSTRATE_POWER_SLEEP);
            }
        }

        k_msleep(WM_CYCLE_MS);
    }

    return 0;
}
