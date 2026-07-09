/*
 * main.c — CF21BL collective formation demo (L4 CSM + lighthouse + syslink P2P)
 *
 * Three Crazyflie 2.1 brushless drones flying a spring-field formation using
 * REAL lighthouse position (not dead reckoning) for both peer gossip and
 * the stabilizer's own X/Y position hold.  See formation.h for the metres
 * unit convention and the shared-calibration requirement.
 *
 * Architecture:
 *   Attitude:    BMI088 rate + angle loops (cf21bl_stabilizer.c, unchanged)
 *   Yaw heading: CONFIG_CF21BL_YAW_HOLD — locked to boot orientation so
 *                world-frame corrections stay meaningful (see lh2-hover's
 *                placement-requirement note; the same requirement applies
 *                here: place each drone with its nose along lighthouse
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

#include "cf21bl_lighthouse.h"
#include "cf21bl_stabilizer.h"
#ifdef CONFIG_CF21BL_PM
#include "cf21bl_pm.h"
#endif

#include "formation.h"

LOG_MODULE_REGISTER(cf21bl_formation, LOG_LEVEL_INF);

/* ── Calibrated BS poses + OOTX calibration ──────────────────────────────── */
/* Poses from lighthouse_cal_office_260706.yaml (2026-07-06, second recalibration same
 * day — BS1 suspected partially occluded, tilted further and re-run).
 * Supersedes both the July-4 poses and the first 2026-07-06 poses.  OOTX
 * sweep calibration below is unchanged (same uid per physical BS across
 * all three calibrations — factory calibration doesn't change when a base
 * station is moved or tilted, only its geometry pose does).  MUST match
 * the physical base station placement at flight time and MUST be
 * identical across all three drones (gossiped positions are only
 * comparable in a shared frame).  If the room is recalibrated again,
 * update this AND examples/lh2-hover/src/main.c. */
static const lh2_bs_pose_t BS0 = {
    .origin = {-0.6803646087646484f, 0.6335355639457703f, 1.615210771560669f},
    .rot    = {0.8344101905822754f,  -0.08563866466283798f, 0.5444498062133789f,
               0.13026301562786102f,  0.9905099272727966f, -0.04383661970496178f,
              -0.535528838634491f,    0.1074993908405304f,  0.8376471400260925f}
};
static const lh2_bs_pose_t BS1 = {
    .origin = {0.09399518370628357f, -2.2131965160369873f, 1.4227608442306519f},
    .rot    = {0.049453821033239365f, -0.9982976317405701f,  0.03092208132147789f,
               0.9098809957504272f,    0.057799000293016434f, 0.41082337498664856f,
              -0.4119112491607666f,    0.00781862810254097f,  0.911190390586853f}
};

static const lh2_bs_calib_t BS0_CALIB = {
    .sweep = {
        { .phase = 0.0f,                  .tilt = -0.0482177734375f,
          .curve = -0.139892578125f,      .gibphase = 2.232421875f,
          .gibmag = -0.001861572265625f,  .ogeephase = 1.1142578125f,
          .ogeemag = -0.1802978515625f },
        { .phase = -0.0070343017578125f,  .tilt = 0.038848876953125f,
          .curve = -0.047149658203125f,   .gibphase = 1.4541015625f,
          .gibmag = -0.0013513565063476562f, .ogeephase = 2.359375f,
          .ogeemag = -0.25439453125f },
    },
    .uid = 3438823989u
};
static const lh2_bs_calib_t BS1_CALIB = {
    .sweep = {
        { .phase = 0.0f,                  .tilt = -0.047393798828125f,
          .curve = -0.3046875f,           .gibphase = 1.1494140625f,
          .gibmag = -0.004795074462890625f, .ogeephase = 0.0887451171875f,
          .ogeemag = 0.09014892578125f },
        { .phase = -0.0010623931884765625f, .tilt = 0.051727294921875f,
          .curve = -0.1802978515625f,     .gibphase = 1.525390625f,
          .gibmag = -0.007568359375f,     .ogeephase = 0.97998046875f,
          .ogeemag = 0.24072265625f },
    },
    .uid = 3211055830u
};

#define BS0_CHANNEL  0
#define BS1_CHANNEL  1

/* ── Mission parameters ───────────────────────────────────────────────────── */

/* Per-drone cruise altitude, staggered by element_id to reduce downwash
 * interaction — TUNE ON HARDWARE, 0.25 m is a starting guess, not validated. */
#define ALT_BASE_M           0.30f
#define ALT_STEP_PER_ID_M    0.25f

/* Gentle altitude ramp on takeoff (same convention as altitude-hold-tether:
 * ramp the closed-loop PID's TARGET, don't jump straight to cruise). */
#define ALT_RAMP_START_M     0.15f
#define ALT_RAMP_RATE_MPS    0.10f

/* Individual landing ramp (same convention as lh2-hover Stage 7). */
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
 * the closest thing to a "coordinated" land without a wireless uplink. */
#define MISSION_DURATION_S   60

#define LOOP_DT_S  ((float)WM_CYCLE_MS * 0.001f)

/* ── Flight state machine ─────────────────────────────────────────────────── */

typedef enum {
    FLIGHT_RAMPING,   /* gentle climb from ALT_RAMP_START_M to cruise altitude */
    FLIGHT_FLYING,    /* formation control active                              */
    FLIGHT_LANDING,   /* ramping altitude target down to 0, X/Y held at home    */
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
        LOG_ERR("lighthouse init failed — cannot fly without position");
        return -1;
    }

    /* substrate_init() -> cf21bl_init(): ESC arming + gyro cal + stabilizer
     * start (baro home average).  Motors silent throughout. */
    if (substrate_init() != 0) {
        LOG_WRN("substrate_init failed — actuation disabled");
    }

    if (transport_init() != 0) {
        LOG_WRN("transport_init failed — no peer awareness");
    }

    const element_id_t element_id = (element_id_t)CONFIG_TAPESTRY_ELEMENT_ID;
    const int n_total = CONFIG_TAPESTRY_ELEMENT_COUNT;
    const float cruise_alt_m = ALT_BASE_M + (float)element_id * ALT_STEP_PER_ID_M;

    LOG_INF("CF21BL formation — element %u  n_total=%d  cruise_alt=%.2fm  "
            "target_spacing=%.2fm",
            (unsigned)element_id, n_total, (double)cruise_alt_m,
            (double)DEMO_TARGET_SPACING_M);

    LOG_INF("Waiting for lighthouse fix (up to 30 s) ...");
    {
        uint32_t deadline = k_uptime_get_32() + 30000u;
        while (!cf21bl_lighthouse_is_valid()) {
            if (k_uptime_get_32() > deadline) {
                LOG_ERR("No lighthouse fix — base stations on? poses correct? "
                        "staying grounded.");
                substrate_set_power(SUBSTRATE_POWER_SLEEP);
                return -1;
            }
            k_msleep(200);
        }
    }
    LOG_INF("Fix acquired");

    LOG_INF("PLACE ON GROUND (nose along lighthouse world +X) AND STAND CLEAR "
            "— arming in 5 s ...");
    for (int i = 5; i > 0; i--) { LOG_INF("  %d ...", i); k_msleep(1000); }

    element_state_t own_state = { 0 };
    own_state.id = element_id;

    world_model_t wm;
    wm_init(&wm, element_id, &own_state, 0.0f);

    /* Target is absolute world-frame, metres. Initialized from this drone's
     * own first fix (below, in the flight loop) rather than world (0,0) —
     * with zero fresh peers demo_compute_drive() never moves the target, so
     * seeding it at the origin would command a translation to wherever
     * lighthouse (0,0) physically is instead of holding at the takeoff spot
     * (unlike lh2-hover, which always commands zero offset from home). */
    demo_setpoint_t target = { 0 };
    bool            target_init = false;

    uint32_t gossip_accum = GOSSIP_INTERVAL_MS;

    /* Convergence hold: wait until all expected peers are fresh (proceeds
     * anyway after the grace period — matches the old demo's behaviour). */
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

    LOG_INF("Arming and entering flight loop");
    substrate_set_power(SUBSTRATE_POWER_ACTIVE);

    flight_state_t state       = FLIGHT_RAMPING;
    float          alt_cmd_m   = ALT_RAMP_START_M;
    float          landing_alt_m = 0.0f;
    uint32_t       mission_t0_ms = k_uptime_get_32();
    uint32_t       fix_lost_since_ms = 0;
    uint32_t       land_settle_ms = 0;
    position_t     own_pos_m   = { 0.0f, 0.0f };
    bool           have_pos    = false;

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
                LOG_INF("Cruise altitude reached — formation control active");
            }
            sp.linear.z = alt_cmd_m - 1.0f;
            break;

        case FLIGHT_FLYING: {
            if (!fix_valid) {
                if (fix_lost_since_ms == 0) {
                    fix_lost_since_ms = k_uptime_get_32();
                    LOG_WRN("lighthouse fix lost — holding, X/Y zeroed");
                }
                if (k_uptime_get_32() - fix_lost_since_ms > FIX_LOSS_GRACE_MS) {
                    LOG_ERR("fix lost > %d ms — landing independently",
                            FIX_LOSS_GRACE_MS);
                    state = FLIGHT_LANDING;
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
                LOG_ERR("geofence breach (%.2f m > %.2f m) — landing independently",
                        (double)origin_dist, (double)GEOFENCE_RADIUS_M);
                state = FLIGHT_LANDING;
                landing_alt_m = alt_cmd_m;
                sp.linear.z = alt_cmd_m - 1.0f;
                break;
            }

            if (k_uptime_get_32() - mission_t0_ms > (uint32_t)MISSION_DURATION_S * 1000u) {
                LOG_INF("mission duration elapsed — landing");
                state = FLIGHT_LANDING;
                landing_alt_m = alt_cmd_m;
                sp.linear.z = alt_cmd_m - 1.0f;
                break;
            }

            float min_dist_m = demo_compute_drive(&wm, &own_pos_m, &target, WM_CYCLE_MS);
            if (min_dist_m >= 0.0f && min_dist_m < DEMO_MIN_SEP_M) {
                static int sep_log_div;
                if (++sep_log_div >= 20) {   /* ~2 Hz at WM_CYCLE_MS=100 */
                    sep_log_div = 0;
                    LOG_WRN("separation violation: nearest peer %.2f m "
                            "(min %.2f m)", (double)min_dist_m, (double)DEMO_MIN_SEP_M);
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
            sp.linear.z = landing_alt_m - 1.0f;
            if (landing_alt_m <= 0.02f) {
                land_settle_ms += WM_CYCLE_MS;
                if (land_settle_ms >= LAND_SETTLE_MS) {
                    state = FLIGHT_LANDED;
                    LOG_INF("landed — disarming");
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
            LOG_INF("%s peers %d/%d pos=(%.2f,%.2f) tgt=(%.2f,%.2f) alt=%.2f "
                    "cmd_z=%.2f",
                    flight_state_name(state), fresh, active,
                    (double)own_pos_m.x, (double)own_pos_m.y,
                    (double)target.x, (double)target.y,
                    (double)alt_cmd_m, (double)sp.linear.z);
        }

        gossip_accum += WM_CYCLE_MS;
        if (gossip_accum >= GOSSIP_INTERVAL_MS) {
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
