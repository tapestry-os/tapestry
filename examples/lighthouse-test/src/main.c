/*
 * lighthouse-test — LH2 position readout, no motors.
 *
 * Reads position from two SteamVR 2.0 base stations and logs at 5 Hz.
 * Move the drone by hand; verify X/Y/Z track correctly.
 *
 * CONSOLE: USART3 is taken by the lighthouse deck (230400 baud).
 * Read log output via CRTP radio:
 *   python3 tapestry/tapestry-os/tools/crazyflie_console.py
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
/* Single shared copy of poses + OOTX calibration + channel assignments —
 * see that header's comment for the recalibration procedure.  This example
 * is the verification tool for it: after any recalibration, a drone on the
 * floor at the calibration spot must read ≈ (0, 0, 0). */
#include "../../lighthouse_cal.h"

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
