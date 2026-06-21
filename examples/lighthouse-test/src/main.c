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
 * Build:  west build -p always -b crazyflie21bl tapestry/tapestry/examples/lighthouse-test
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
static const lh2_bs_pose_t BS0 = {
    .origin = {-0.1968589723110199f, 2.560563087463379f, 
                1.2248899936676025f}, /* meters, world frame */ 
    .rot = {0.24485638737678528, 0.9695349335670471, 
                -0.006882685702294111,
            -0.9386824369430542, 0.23527540266513824,
                -0.25203338265419006,
            -0.2427358329296112, 0.0681726410984993,
                0.9676940441131592}
};

static const lh2_bs_pose_t BS1 = {
    .origin = {1.066727876663208f, -1.6772024631500244f,
                0.9965450167655945f}, /* meters, world frame */ 
    .rot = {-0.25859639048576355, -0.9628901481628418,
                -0.07726871222257614,
            0.9137280583381653, -0.2697758674621582,
                0.30384543538093567,
            -0.3134150207042694, 0.007970745675265789,
                0.9495828151702881}
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
