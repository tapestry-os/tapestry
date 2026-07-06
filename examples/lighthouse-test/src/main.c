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
/* lighthouse_cal_office_260706.yaml — SECOND 2026-07-06 recalibration (BS1 suspected
 * partially occluded, tilted further and re-run); supersedes both the
 * July-4 poses and the first 2026-07-06 poses that used to live here. */
static const lh2_bs_pose_t BS0 = {
    .origin = {-0.6803646087646484, 0.6335355639457703, 1.615210771560669},
    .rot    = {0.8344101905822754, -0.08563866466283798, 0.5444498062133789,
               0.13026301562786102, 0.9905099272727966, -0.04383661970496178,
               -0.535528838634491, 0.1074993908405304, 0.8376471400260925}
};

static const lh2_bs_pose_t BS1 = {
    .origin = {0.09399518370628357, -2.2131965160369873, 1.4227608442306519},
    .rot    = {0.049453821033239365, -0.9982976317405701, 0.03092208132147789,
               0.9098809957504272, 0.057799000293016434, 0.41082337498664856,
               -0.4119112491607666, 0.00781862810254097, 0.911190390586853}
};

/* OOTX sweep calibration — from the same YAML's "calibs:" section, unchanged
 * from July 4 (factory calibration is per-physical-BS, tied to uid, not to
 * where the base station is placed in the room). */
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
