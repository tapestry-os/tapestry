/*
 * main.c — CF21BL collective formation demo (lighthouse + syslink P2P)
 *
 * Two build modes (Kconfig choice DEMO_MODE):
 *
 *   DEMO_MODE_CHOREO (default) — the L6/L7 path.  ONE BINARY for all
 *     drones: element IDs are negotiated at boot (transport_negotiate_id,
 *     same auto-ID protocol as cutebot-formation).  A declarative L7
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
 *   - Own battery critical: already handled independently inside
 *     cf21bl_stabilizer.c (CONFIG_CF21BL_PM forced-landing path) — no code
 *     needed here, and it does not depend on or affect other drones.
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
#include <tapestry/scr.h>      /* scr_state_t for the synthetic quorum */
#include <tapestry/choreo.h>
/* The show itself.  GENERATED from ../choreo.toml (the file to edit) by
 * sdk/tools/choreoc.py — see the regeneration command in its banner. */
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
    int n_total;
    const element_id_t element_id = transport_negotiate_id(&n_total);
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
    choreo_init(element_id);
    /* k_choreo_script comes from the generated choreo_script.h — the
     * authored script is ../choreo.toml (coordinate-free: hold references
     * each drone's own station, exchange references the partner's; no
     * takeoff/landing/altitude anywhere — quiescence at script end maps
     * to land-in-place below, and altitude staggering is a platform
     * deconfliction rule the Choreo never sees).
     * No L5 SCR on this platform — no scr registered, so the capability
     * check passes by default; the synthetic quorum below still gives the
     * lifecycle machine its RUNNING/SUSPENDED signal. */
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
    scr_state_t scr_synth = { 0 };
    scr_synth.own_id       = element_id;
    scr_synth.quorum_state = SCR_QUORUM_LOST;   /* until peers are fresh */
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
    own_state.id = element_id;

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

    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;

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
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
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
    position_t     own_pos_m   = { 0.0f, 0.0f };
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
            /* Synthetic L5 quorum from L4 freshness (no SCR on this
             * platform): any fresh peer → HEALTHY; none → LOST, which
             * suspends the Choreo (frozen script timers, frozen BSE) and
             * freezes the target below — the same hold-on-stale discipline
             * the spring field uses, expressed through the L7 lifecycle. */
            int fresh_peers = 0;
            for (int i = 0; i < MAX_ELEMENTS; i++) {
                const wm_entry_t *e = &wm.entries[i];
                if (e->is_active && !e->is_self && !e->is_stale) {
                    fresh_peers++;
                }
            }
            scr_synth.fresh_count  = (uint8_t)fresh_peers;
            scr_synth.quorum_state = fresh_peers >= 1 ? SCR_QUORUM_HEALTHY
                                                      : SCR_QUORUM_LOST;

            choreo_tick(&wm, &scr_synth);

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
            if (scr_synth.quorum_state != SCR_QUORUM_LOST &&
                dir->type == TAPESTRY_BSE_DIRECTIVE_MOVE_TO_POINT) {
                min_dist_m = demo_choreo_track(&wm, &own_pos_m, &target,
                                               dir->target.x, dir->target.y,
                                               WM_CYCLE_MS, element_id);
            }
            /* else: quorum LOST (choreo SUSPENDED) or directive HOLD
             * (exchange awaiting its snapshot) — target frozen where it
             * is; the stabilizer keeps station on it. */
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
                land_settle_ms += WM_CYCLE_MS;
                if (land_settle_ms >= LAND_SETTLE_MS) {
                    state = FLIGHT_LANDED;
                    LOG_INF("id=%u landed — disarming", (unsigned)element_id);
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
            LOG_INF("id=%u %s peers %d/%d pos=(%.2f,%.2f) tgt=(%.2f,%.2f) alt=%.2f "
                    "cmd_z=%.2f step=%d%s",
                    (unsigned)element_id, flight_state_name(state), fresh, active,
                    (double)own_pos_m.x, (double)own_pos_m.y,
                    (double)target.x, (double)target.y,
                    (double)alt_cmd_m, (double)sp.linear.z,
                    choreo_script_step(),
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
        /* A LANDED drone goes gossip-silent: it has left the collective, and
         * peers must be able to expire it (WM_EXPIRE_THRESHOLD_MS) so the
         * formation heals around the survivors.  Without this gate a landed
         * drone broadcast its ground position forever and the others held
         * formation around a parked airframe.  LANDING (still airborne,
         * descending) keeps gossiping — peers should make room for it until
         * it is actually down. */
        if (state != FLIGHT_LANDED && gossip_accum >= GOSSIP_INTERVAL_MS) {
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
