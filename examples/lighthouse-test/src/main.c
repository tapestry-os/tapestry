/*
 * lighthouse-test — LH2 position readout, no motors.
 *
 * Reads position from two SteamVR 2.0 base stations and logs at 5 Hz.
 * Move the drone by hand; verify X/Y/Z track correctly.
 *
 * CONSOLE: USART3 is taken by the lighthouse deck (230400 baud).
 * Read log output via CRTP radio:
 *   python3 ~/code/tapestry/read_console.py
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/examples/lighthouse-test
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu  (activate .venv)
 *
 * Before running:
 *   1. Attach the Lighthouse deck to the CF21BL expansion connector.
 *   2. Power on two SteamVR 2.0 base stations; wait ~30 s for spin-up.
 *   3. Get calibrated BS poses from cfclient (Lighthouse → Manage Geometry
 *      → Estimate geometry), then paste the JSON values into BS0/BS1 below.
 *   4. Confirm the channel numbers match your SteamVR channel assignments.
 *      The channel in the frame is 0-indexed (SteamVR channel − 1).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "cf21bl_lighthouse.h"

LOG_MODULE_REGISTER(lh2_test, LOG_LEVEL_INF);

/* ── Calibration — paste from cfclient geometry estimator ─────────────────── */

/*
 * JSON from cfclient "Manage geometry → Estimate":
 *   "rotation_matrix": [[r00,r01,r02],[r10,r11,r12],[r20,r21,r22]]
 *   "origin": [ox, oy, oz]
 *
 * Paste as:  .origin = {ox, oy, oz}
 *            .rot    = {r00,r01,r02, r10,r11,r12, r20,r21,r22}
 *
 * For initial SPI-link testing with no geometry estimated yet, leave as
 * identity / placeholder — position will be wrong but the log will show
 * whether frames are arriving (fix status changes from "no fix" to numbers).
 */
/* office_shade_bookcase_July4_2026.yaml (same values as lh2-hover) */
static const lh2_bs_pose_t BS0 = {
    .origin = {1.7085734605789185, -0.43685105443000793, 1.8661978244781494},
    .rot    = {-0.8205968737602234, 0.055611010640859604, -0.568795382976532,
               -0.08727049082517624, -0.9957755208015442, 0.02854771353304386,
               -0.5648049712181091, 0.07306522130966187, 0.8219834566116333}
};

static const lh2_bs_pose_t BS1 = {
    .origin = {0.9165827035903931, 2.814929246902466, 1.7187764644622803},
    .rot    = {0.09428899735212326, 0.9954821467399597, -0.011177653446793556,
               -0.8471673130989075, 0.0743338093161583, -0.526100754737854,
               -0.5228930711746216, 0.05907485634088516, 0.8503487706184387}
};

/* OOTX sweep calibration — same YAML, "calibs:" section */
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

/*
 * SteamVR channel assignments (0-indexed = SteamVR channel number − 1).
 * Check cfclient Lighthouse tab to see which channel each BS is assigned.
 * Default: BS0 on channel 0, BS1 on channel 1.
 */
#define BS0_CHANNEL  0
#define BS1_CHANNEL  1

/* ── Main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    LOG_INF("=== LH2 lighthouse-test (no motors) ===");
    LOG_INF("Console: CRTP radio (wired USART3 taken by lighthouse deck)");

    /* Load calibration */
    cf21bl_lighthouse_set_bs_pose(0, &BS0);
    cf21bl_lighthouse_set_bs_pose(1, &BS1);
    cf21bl_lighthouse_set_bs_calib(0, &BS0_CALIB);
    cf21bl_lighthouse_set_bs_calib(1, &BS1_CALIB);
    cf21bl_lighthouse_set_bs_channel(0, BS0_CHANNEL);
    cf21bl_lighthouse_set_bs_channel(1, BS1_CHANNEL);

    /* Start UART reader thread (blocks until synchronization on 0xFF frame) */
    int ret = cf21bl_lighthouse_init();
    if (ret) {
        LOG_ERR("cf21bl_lighthouse_init failed: %d", ret);
        return ret;
    }

    LOG_INF("Waiting for UART sync + fix from both base stations...");

    int no_fix_count = 0;
    while (true) {
        lh2_position_t pos;
        if (cf21bl_lighthouse_get_position(&pos) == 0) {
            no_fix_count = 0;
            LOG_INF("pos  x=%+.3f  y=%+.3f  z=%+.3f  m",
                    (double)pos.x, (double)pos.y, (double)pos.z);
        } else {
            no_fix_count++;
            if (no_fix_count % 10 == 1) {
                LOG_INF("no fix (check base stations, deck connection, channel config)");
            }
        }
        k_msleep(200);   /* 5 Hz */
    }

    return 0;
}
