/*
 * imu-test — BMI088 IMU bring-up (sensor-only, no actuation)
 *
 * Initializes the BMI088 accel+gyro over I2C3 (cf21bl_imu), runs a
 * complementary filter on the gyro/accel data, and logs raw readings,
 * roll/pitch estimates, and the measured gyro INT3 (PC14) interrupt rate.
 *
 * Does NOT call substrate_init() / link the motor driver — props can stay
 * on, ESCs are never armed.
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/examples/imu-test
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu  (activate ~/code/tapestry/.venv)
 * Read:   minicom -D /dev/ttyUSB0 -b 115200   (USART3)
 *         python3 ~/code/tapestry/read_console.py   (CRTP radio)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "cf21bl_imu.h"

LOG_MODULE_REGISTER(imu_test, LOG_LEVEL_INF);

#define REPORT_INTERVAL_MS 1000
#define GYRO_ODR_HZ        1000.0f

int main(void)
{
    LOG_INF("=== CF21bl IMU bring-up (sensor-only, no actuation) ===");

    if (cf21bl_imu_init() != 0) {
        LOG_ERR("cf21bl_imu_init failed - aborting");
        return -1;
    }

    cf21bl_imu_filter_init();

    int64_t last_report_ms = k_uptime_get();
    uint32_t last_drdy = cf21bl_imu_get_drdy_count();
    uint32_t sample_count = 0;

    while (1) {
        cf21bl_imu_sample_t sample;
        int ret = cf21bl_imu_read(&sample);
        if (ret) {
            LOG_WRN("cf21bl_imu_read failed: %d", ret);
            continue;
        }
        sample_count++;

        cf21bl_imu_attitude_t att;
        cf21bl_imu_filter_update(&sample, 1.0f / GYRO_ODR_HZ, &att);

        int64_t now_ms = k_uptime_get();
        if (now_ms - last_report_ms >= REPORT_INTERVAL_MS) {
            uint32_t drdy = cf21bl_imu_get_drdy_count();
            uint32_t drdy_rate =
                (uint32_t)((uint64_t)(drdy - last_drdy) * 1000U / (now_ms - last_report_ms));

            LOG_INF("gyro[rad/s] x=%.3f y=%.3f z=%.3f  accel[g] x=%.3f y=%.3f z=%.3f",
                    (double)sample.gyro_rps[0], (double)sample.gyro_rps[1],
                    (double)sample.gyro_rps[2], (double)sample.accel_g[0],
                    (double)sample.accel_g[1], (double)sample.accel_g[2]);
            LOG_INF("roll=%.2f deg pitch=%.2f deg  INT3=%u Hz  reads/s=%u",
                    (double)att.roll_deg, (double)att.pitch_deg, drdy_rate, sample_count);

            last_report_ms = now_ms;
            last_drdy = drdy;
            sample_count = 0;
        }
    }

    return 0;
}
