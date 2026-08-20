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
 *   Altitude:    CONFIG_CF21BL_ALTITUDE_HOLD (baro, closed-loop) — each
 *                drone holds a fixed, ID-staggered cruise altitude.
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
 *   python3 ~/code/tapestry/read_console.py   (CRTP radio — USART3 is taken
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
 *      its ID-staggered cruise height, then joins the formation.
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

/* Per-drone cruise altitude, staggered by element_id to reduce downwash
 * interaction.  Step compressed 0.25→0.20 m (cruises 0.30/0.50/0.70): at
 * 0.80 m the top drone received BS1's light only ~12.6° above its deck
 * horizon and dropped to ok=0 in bursts (grazing incidence — BS1 is
 * mounted low and far in this room); 0.70 m buys back ~3° of arrival
 * angle and every recorded dropout instance was on the highest drone. */
#define ALT_BASE_M           0.30f
#define ALT_STEP_PER_ID_M    0.20f

/* Gentle altitude ramp on takeoff (same convention as altitude-hold-tether:
 * ramp the closed-loop PID's TARGET, don't jump straight to cruise). */
#define ALT_RAMP_START_M     0.15f
#define ALT_RAMP_RATE_MPS    0.10f

/* Individual landing ramp (walk the altitude target down, settle, disarm). */
#define LAND_RATE_MPS        0.30f
#define LAND_SETTLE_MS       2000
/* Touchdown gate: the walked-down TARGET reaching zero says nothing about
 * the airframe (2026-07-19 flight 10: a drone cut its motors mid-air, the
 * long-flagged disarm-on-target bug).  Require the measured lighthouse
 * altitude to agree before the settle timer runs; if the fix is invalid
 * (or z is biased high) LAND_FORCE_DISARM_MS bounds the wait — by then
 * the zero-target thrust has had ample time to put the airframe down. */
#define LAND_TOUCHDOWN_Z_M   0.08f
#define LAND_FORCE_DISARM_MS 10000

/* Sustained lighthouse fix loss before an individual landing — brief blips
 * just zero X/Y and hold; see the flight_state_t machine below. */
#define FIX_LOSS_GRACE_MS    2000

/* Geofence: distance from the lighthouse origin (NOT this drone's home —
 * the origin is the one frame shared by every drone).  TUNE to the room's
 * actual coverage; 2.0 m is conservative relative to CF21BL_POS_MAX_M=3. */
#define GEOFENCE_RADIUS_M    2.0f

/* Every drone lands independently after this long from its own arm time —
 * a pure safety backstop in choreo mode (the script normally ends the
 * flight well before it), and the "coordinated" land in showcase mode.
 * Showcase keeps it per-build (Kconfig) so one drone can leave the
 * formation early: it lands, goes gossip-silent (see the FLIGHT_LANDED
 * gate at the send site), peers expire it after WM_EXPIRE_THRESHOLD_MS,
 * and the field re-forms without it. */
#ifdef CONFIG_DEMO_MODE_CHOREO
/* The script's own total time bound (choreoc requires every step to be
 * time-bounded, so this is a hard ceiling) + margin for ramp and descent.
 * Reached only if the script stalls (e.g. partner lost mid-show →
 * SUSPENDED). */
#define MISSION_DURATION_S   (CHOREO_SCRIPT_TOTAL_TIMEOUT_MS / 1000u + 40u)
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
         * Ground as a SELF-HEALING radio-diagnostic station instead:
         * gossip + listen, re-log the retained window outcome and the
         * duplicate-ID evidence every 2 s, and recover WITHOUT a
         * power-cycle by either
         *   (a) renegotiating at a jittered interval — if the peer is
         *       meanwhile gossiping (grounded like us, or flying), its
         *       claimed ID resolves the collision (run 8: gossip flowed
         *       at ~35% while both sat deaf-mute at id=0), or
         *   (b) seeing a fresh peer directly (IDs already differ).
         * Deliberate solo flights: -DCONFIG_DEMO_ALLOW_SOLO=y. */
        LOG_ERR("id=%u auto-ID heard NO peers — NOT ARMING; grounded "
                "self-healing diagnostic mode (renegotiates until a peer "
                "is found)", (unsigned)element_id);

        element_state_t diag_state = { 0 };
        diag_state.id          = element_id;
        diag_state.orientation = orientation_identity();  /* grounded, on the
                                                            * ground — no
                                                            * attitude to report */
        world_model_t diag_wm;
        wm_init(&diag_wm, element_id, &diag_state, 0.0f);

        uint32_t diag_gossip_ms = DEMO_GOSSIP_MS;
        uint32_t diag_log_ms    = 0;
        uint32_t reneg_in_ms    = 15000u + (k_cycle_get_32() % 30000u);
        substrate_set_signal(SUBSTRATE_SIGNAL_FAILED);

        for (bool recovered = false; !recovered; k_msleep(WM_CYCLE_MS)) {
            transport_drain(&diag_wm, element_id);
            wm_tick(&diag_wm, WM_CYCLE_MS);

            lh2_position_t dp;
            if (cf21bl_lighthouse_is_valid() &&
                cf21bl_lighthouse_get_position(&dp) == 0) {
                diag_state.position.x = dp.x;
                diag_state.position.y = dp.y;
            }
            wm_update_self(&diag_wm, &diag_state);

            int fresh = 0, active = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &diag_wm.entries[i];
                if (e->is_active && !e->is_self) {
                    active++;
                    if (!e->is_stale) { fresh++; }
                }
            }

            if (fresh > 0) {
                /* A distinct-ID peer is visible — radio and identities are
                 * fine; adopt the visible collective size and proceed. */
                n_total = 1 + active;
                LOG_WRN("id=%u recovered: peer VISIBLE (fresh=%d) — "
                        "n_total=%d, proceeding to flight prep",
                        (unsigned)element_id, fresh, n_total);
                substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
                recovered = true;
                continue;
            }

            diag_log_ms += WM_CYCLE_MS;
            if (diag_log_ms >= 2000u) {
                diag_log_ms = 0;
                uint32_t btx, nonces, running;
                transport_get_negotiation_stats(&btx, &nonces, &running);
                uint32_t dup = gossip_own_id_frames();
                if (dup > 0u) {
                    LOG_ERR("id=%u GROUNDED-DIAG: DUPLICATE ID — %u frames "
                            "from another element also claiming id=%u; "
                            "renegotiating in %u s",
                            (unsigned)element_id, dup,
                            (unsigned)element_id, reneg_in_ms / 1000u);
                } else {
                    LOG_INF("id=%u GROUNDED-DIAG: peers fresh=0 active=%d "
                            "window(beacons=%u nonces=%u running=%u) "
                            "own_id_frames=0",
                            (unsigned)element_id, active,
                            btx, nonces, running);
                }
            }

            diag_gossip_ms += WM_CYCLE_MS;
            if (diag_gossip_ms >= DEMO_GOSSIP_MS) {
                diag_gossip_ms = 0;
                diag_state.update_seq++;
                transport_send(&diag_state, TAPESTRY_QOS_SOFT_RT);
            }

            if (reneg_in_ms <= WM_CYCLE_MS) {
                LOG_INF("id=%u re-running auto-ID negotiation ...",
                        (unsigned)element_id);
                int nt;
                element_id_t nid = transport_negotiate_id(&nt);
                if (nt >= 2) {
                    element_id    = nid;
                    n_total       = nt;
                    diag_state.id = nid;
                    LOG_WRN("id=%u recovered via renegotiation: n_total=%d, "
                            "proceeding to flight prep",
                            (unsigned)element_id, n_total);
                    substrate_set_signal(SUBSTRATE_SIGNAL_ACTIVE);
                    recovered = true;
                } else {
                    reneg_in_ms = 15000u + (k_cycle_get_32() % 30000u);
                    LOG_INF("id=%u still alone after renegotiation — next "
                            "retry in %u s", (unsigned)element_id,
                            reneg_in_ms / 1000u);
                }
            } else {
                reneg_in_ms -= WM_CYCLE_MS;
            }
        }
    }
#endif /* !CONFIG_DEMO_ALLOW_SOLO */
#else
    const element_id_t element_id = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;
    const int n_total = CONFIG_TAPESTRY_ELEMENT_COUNT;
#endif
    const float cruise_alt_m = ALT_BASE_M + (float)element_id * ALT_STEP_PER_ID_M;

    LOG_INF("CF21BL formation — element %u  n_total=%d  cruise_alt=%.2fm  "
            "target_spacing=%.2fm",
            (unsigned)element_id, n_total, (double)cruise_alt_m,
            (double)DEMO_TARGET_SPACING_M);

#ifdef CONFIG_DEMO_MODE_CHOREO
    /* Real L5 SCR.  CONFIG_TAPESTRY_QUORUM_MIN/_TARGET (this Kconfig,
     * default 1/1 — this script only ever needs one fresh partner) were
     * previously dead config, unused anywhere in this example; same
     * options tapestry-scr-hw's reference element feeds to scr_init().
     * SCR_CAP_ACTUATOR satisfies the script's CHOREO_CAP_LOCOMOTION
     * requirement, enforced below by choreo_submit_script() now that a
     * real scr is registered. */
    scr_state_t scr;
    scr_init(&scr, element_id,
        (uint8_t)CONFIG_TAPESTRY_QUORUM_MIN,
        (uint8_t)CONFIG_TAPESTRY_QUORUM_TARGET,
        SCR_CAP_ACTUATOR);

    choreo_init(element_id);
    choreo_register_scr(&scr);
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
    float          landing_alt_m = 0.0f;
    uint32_t       mission_t0_ms = k_uptime_get_32();
    uint32_t       fix_lost_since_ms = 0;
    uint32_t       land_settle_ms = 0;
    /* z: the per-ID staggered CRUISE altitude (computed above as
     * cruise_alt_m), not a live baro reading — cf21bl_stabilizer.c exposes
     * no altitude accessor today. Real enough to be useful (this is where
     * the drone is commanded to hold) without fabricating sensor precision
     * that doesn't exist; only x/y are refreshed from the lighthouse fix
     * below, so this value holds for the whole flight. A live reading
     * during ramp/landing is a follow-up, not done here — see this
     * README's "Known limitations". */
    position_t     own_pos_m   = { 0.0f, 0.0f, cruise_alt_m };
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

        if (have_pos) {
            own_state.position = own_pos_m;
        }
#ifdef CONFIG_CF21BL_PM
        float vbat = cf21bl_pm_vbat();
        if (vbat > 0.0f) {
            float pct = (vbat - 3.0f) / (4.2f - 3.0f);
            if (pct < 0.0f) { pct = 0.0f; }
            if (pct > 1.0f) { pct = 1.0f; }
            own_state.energy_level = (uint8_t)(pct * 100.0f);
        }
        own_state.health_flags = cf21bl_pm_battery_low()
                                  ? ELEMENT_HEALTH_LOW_BATTERY
                                  : ELEMENT_HEALTH_OK;
#endif
        wm_update_self(&wm, &own_state);

        substrate_twist_t sp = { 0 };
#ifdef CONFIG_DEMO_MODE_CHOREO
        bool log_quorum_up = false;   /* debounced quorum, set in FLYING */
#endif

        switch (state) {
        case FLIGHT_RAMPING:
            alt_cmd_m += ALT_RAMP_RATE_MPS * LOOP_DT_S;
            if (alt_cmd_m >= cruise_alt_m) {
                alt_cmd_m = cruise_alt_m;
                state = FLIGHT_FLYING;
                LOG_INF("id=%u Cruise altitude reached — formation control active",
                        (unsigned)element_id);
            }
            sp.linear.z = alt_cmd_m - 1.0f;
            break;

        case FLIGHT_FLYING: {
            if (!fix_valid) {
                if (fix_lost_since_ms == 0) {
                    fix_lost_since_ms = k_uptime_get_32();
                    LOG_WRN("id=%u lighthouse fix lost — holding, X/Y zeroed",
                            (unsigned)element_id);
                }
                if (k_uptime_get_32() - fix_lost_since_ms > FIX_LOSS_GRACE_MS) {
                    LOG_ERR("id=%u fix lost > %d ms — landing independently",
                            (unsigned)element_id, FIX_LOSS_GRACE_MS);
                    state = FLIGHT_LANDING;
                    land_hold_valid = false;   /* no fix — no position to hold */
                    landing_alt_m = alt_cmd_m;
                    sp.linear.z = alt_cmd_m - 1.0f;
                    break;
                }
                /* Zero X/Y so the stabilizer's fix-lost fallback (velocity
                 * feedforward) doesn't inherit a stale position-style value. */
                sp.linear.x = 0.0f;
                sp.linear.y = 0.0f;
                sp.linear.z = alt_cmd_m - 1.0f;
                break;
            }
            fix_lost_since_ms = 0;

            /* Deliberately horizontal-only (unlike formation.c's peer
             * separation, which now folds in z per an explicit choice —
             * see position_distance()'s comment in csm.h). own_pos_m.z is
             * a FIXED per-ID constant (cruise_alt_m) for this drone's
             * whole flight, not a varying peer proximity signal; folding
             * a constant into a radius check just shrinks each drone's
             * effective horizontal margin by a fixed, ID-dependent amount
             * (sqrt(R^2 - z^2) if it were 3D) for no safety benefit, and
             * z can be a meaningful fraction of GEOFENCE_RADIUS_M at this
             * scale. Flagged for review rather than silently decided —
             * this was not itself confirmed under "make it 3D now", only
             * peer-separation/collision math was. */
            float origin_dist = sqrtf(own_pos_m.x * own_pos_m.x
                                       + own_pos_m.y * own_pos_m.y);
            if (origin_dist > GEOFENCE_RADIUS_M) {
                LOG_ERR("id=%u geofence breach (%.2f m > %.2f m) — landing independently",
                        (unsigned)element_id,
                        (double)origin_dist, (double)GEOFENCE_RADIUS_M);
                state = FLIGHT_LANDING;
                land_hold_x = own_pos_m.x;
                land_hold_y = own_pos_m.y;
                land_hold_valid = true;
                landing_alt_m = alt_cmd_m;
                sp.linear.z = alt_cmd_m - 1.0f;
                break;
            }

            if (k_uptime_get_32() - mission_t0_ms > (uint32_t)MISSION_DURATION_S * 1000u) {
                LOG_INF("id=%u mission duration elapsed — landing in place at "
                        "(%.2f, %.2f)", (unsigned)element_id,
                        (double)own_pos_m.x, (double)own_pos_m.y);
                state = FLIGHT_LANDING;
                land_hold_x = own_pos_m.x;
                land_hold_y = own_pos_m.y;
                land_hold_valid = true;
                landing_alt_m = alt_cmd_m;
                sp.linear.z = alt_cmd_m - 1.0f;
                break;
            }

#ifdef CONFIG_DEMO_MODE_CHOREO
            /* Real L5: recompute quorum/role/task_slot/abort from the
             * actual world model. */
            scr_tick(&scr, &wm);

            /* Debounced VIEW of quorum for L6/L7, layered on top of the
             * real scr_tick() output: a single lucky gossip frame keeps a
             * peer "fresh" for WM_STALE_THRESHOLD_MS (1500 ms), so gating
             * on instantaneous freshness let lone packets flicker the
             * Choreo awake for a second at a time — each flicker ran the
             * tracker briefly and its leash ratcheted the target toward a
             * (possibly corrupt) position estimate (2026-07-19 flight 2).
             * Requiring ≥ QUORUM_UP_MS of SUSTAINED freshness means at
             * least two consecutive gossip frames: real contact, not a
             * lucky packet.  Loss is immediate.  At this example's
             * thresholds (quorum_min = quorum_target = 1) that is the same
             * verdict scr_tick() computes, plus a confirmation delay on the
             * up-transition that peer counts cannot express.
             *
             * The debounce is applied to a COPY handed to choreo_tick(),
             * never written back into scr: quorum_state is the only field
             * L6/L7 reads, and mutating the live struct would leave
             * scr_get_quorum() reporting the debounced view while
             * _prev_quorum_state and scr_get_abort_state() tracked the real
             * history — a state the L5 contract says cannot happen, and a
             * trap for whoever wires in the abort protocol.
             *
             * This debounce belongs inside L5, not here: deciding whether
             * the collective has quorum is L5's job, and an obligation
             * every element main loop has to remember is one a new element
             * main loop will forget.  It lives here only because moving it
             * makes scr_tick() time-dependent (today it is a pure function
             * of the world model) and delays SCR_ABORT_CLEARED by
             * QUORUM_UP_MS — a change to the L5→L6 contract that wants
             * flight validation, not a refactor.  Deferred past 0.9.0. */
#define QUORUM_UP_MS 2000
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
            static uint32_t fresh_streak_ms;
            if (scr.fresh_count >= 1) {
                if (fresh_streak_ms < QUORUM_UP_MS) {
                    fresh_streak_ms += WM_CYCLE_MS;
                }
            } else {
                fresh_streak_ms = 0;
            }
            bool quorum_up = fresh_streak_ms >= QUORUM_UP_MS;
            log_quorum_up = quorum_up;
            scr_state_t scr_view = scr;
            scr_view.quorum_state = quorum_up ? SCR_QUORUM_HEALTHY
                                              : SCR_QUORUM_LOST;

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
                    choreo_goal_t rth = {
                        .type   = CHOREO_GOAL_CONVERGE,
                        .target = { home_x, home_y },
                    };
                    int rc = choreo_preempt_goal(&rth);
                    LOG_WRN("id=%u battery low — preempting to "
                            "return-to-home (%.2f, %.2f), rc=%d",
                            (unsigned)element_id, (double)home_x,
                            (double)home_y, rc);
                }
            }
            was_battery_low = battery_low;
#endif

            choreo_tick(&wm, &scr_view);
            own_state.goal_achieved = choreo_goal_achieved();

            /* HARD_RT gossip on the REAL (undebounced) quorum-loss edge —
             * scr_view's QUORUM_UP_MS softening above is for L6/L7 only;
             * the wire signal that this element's quorum just failed
             * should not wait on that confirmation delay. Fires once per
             * outage, not every tick it persists — scr_get_abort_state()
             * is a level held for the whole LOST period (see scr.h), so
             * this checks the NONE/CLEARED -> TRIGGERED edge specifically.
             * Mirrors tapestry-os/subsys/runtime/runtime.c's step 4b. */
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

            if (choreo_script_complete()) {
                /* Quiescence: the script is done and the directive is IDLE.
                 * This platform's inactive posture is "on the ground,
                 * disarmed" — land in place. */
                LOG_INF("id=%u choreo complete — resting: landing in place "
                        "at (%.2f, %.2f)", (unsigned)element_id,
                        (double)own_pos_m.x, (double)own_pos_m.y);
                state = FLIGHT_LANDING;
                land_hold_x = own_pos_m.x;
                land_hold_y = own_pos_m.y;
                land_hold_valid = true;
                landing_alt_m = alt_cmd_m;
                sp.linear.z = alt_cmd_m - 1.0f;
                break;
            }

            float min_dist_m = -1.0f;
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
                                               WM_CYCLE_MS, element_id);
            }
            /* else: quorum LOST on a peer-referential goal (choreo
             * SUSPENDED) or directive HOLD (exchange awaiting its
             * snapshot) — target frozen where it is; the stabilizer keeps
             * station on it. */
#else /* DEMO_MODE_SHOWCASE */
            float min_dist_m = demo_compute_drive(&wm, &own_pos_m, &target, WM_CYCLE_MS,
                                                  element_id);
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
                    LOG_WRN("id=%u separation violation: nearest peer %.2f m "
                            "(min %.2f m)", (unsigned)element_id,
                            (double)min_dist_m, (double)DEMO_MIN_SEP_M);
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
            landing_alt_m -= LAND_RATE_MPS * LOOP_DT_S;
            if (landing_alt_m < 0.0f) { landing_alt_m = 0.0f; }
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
            sp.linear.z = landing_alt_m - 1.0f;
            if (landing_alt_m <= 0.02f) {
                /* Touchdown gate (see LAND_TOUCHDOWN_Z_M): only run the
                 * settle timer once the MEASURED altitude agrees the
                 * airframe is down; a lagging altitude loop no longer gets
                 * its motors cut mid-air.  land_zero_ms bounds the wait
                 * when the fix is unavailable or z-biased. */
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
                if (land_settle_ms >= LAND_SETTLE_MS) {
                    state = FLIGHT_LANDED;
                    LOG_INF("id=%u landed — disarming (z gate %s)",
                            (unsigned)element_id,
                            down ? "confirmed" : "timed out");
                }
            } else {
                land_settle_ms = 0;
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
        demo_set_leds(&wm);

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
            LOG_INF("id=%u %s peers %d/%d pos=(%.2f,%.2f) tgt=(%.2f,%.2f) alt=%.2f "
                    "cmd_z=%.2f step=%d q=%c%s%s",
                    (unsigned)element_id, flight_state_name(state), fresh, active,
                    (double)own_pos_m.x, (double)own_pos_m.y,
                    (double)target.x, (double)target.y,
                    (double)alt_cmd_m, (double)sp.linear.z,
                    choreo_script_step(),
                    log_quorum_up ? 'H' : 'L',
                    choreo_goal_status() == CHOREO_STATE_SUSPENDED ? "(susp)" : "",
                    choreo_goal_achieved() ? " achieved" : "");
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
