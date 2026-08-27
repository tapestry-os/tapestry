/*
 * baro-test — BMP388 barometer bring-up (sensor-only, no actuation)
 *
 * Reads BMP388 pressure via I2C3, converts to altitude relative to boot,
 * and logs at 5 Hz.  Lift the board to see altitude increase.
 *
 * No IMU, no motors, no CRTP — console is USB CDC ACM on PA11/PA12.
 *
 * Build:  west build -p always -b crazyflie21bl tapestry/tapestry/examples/baro-test
 * Flash:  cfloader flash build/zephyr/zephyr.bin stm32-dfu  (activate .venv)
 * Read:   minicom -D /dev/ttyUSB0 -b 115200       (wired USART3 on PC10/PC11)
 *   or:   python3 tapestry/tapestry-os/tools/crazyflie_console.py  (CRTP radio via crazyradio2)
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(baro_test, LOG_LEVEL_INF);

/* 11.77 Pa per meter at sea level (standard atmosphere linear approximation).
 * Accurate to ±1% for altitude changes up to a few hundred meters. */
#define PA_PER_M   11.77f

/* Number of readings averaged to establish the home baseline. */
#define BASELINE_SAMPLES  20

static const struct device *const baro = DEVICE_DT_GET(DT_NODELABEL(bmp388_baro));

int main(void)
{
    if (!device_is_ready(baro)) {
        LOG_ERR("BMP388 not ready — check overlay address (0x76?) and I2C3 wiring");
        return -ENODEV;
    }

    LOG_INF("=== BMP388 baro test (sensor-only, no actuation) ===");

    /* Establish home baseline from first BASELINE_SAMPLES readings. */
    float p_home = 0.0f;
    for (int n = 0; n < BASELINE_SAMPLES; n++) {
        sensor_sample_fetch(baro);
        struct sensor_value pres;
        sensor_channel_get(baro, SENSOR_CHAN_PRESS, &pres);
        p_home += sensor_value_to_float(&pres) * 1000.0f;  /* kPa → Pa */
        k_msleep(20);   /* BMP388 at 50 Hz ODR: new sample every 20 ms */
    }
    p_home /= (float)BASELINE_SAMPLES;
    LOG_INF("Home baseline: %.1f Pa  (lift board to see altitude increase)", (double)p_home);

    while (true) {
        sensor_sample_fetch(baro);

        struct sensor_value pres, temp;
        sensor_channel_get(baro, SENSOR_CHAN_PRESS, &pres);
        sensor_channel_get(baro, SENSOR_CHAN_AMBIENT_TEMP, &temp);

        float p_Pa  = sensor_value_to_float(&pres) * 1000.0f;  /* kPa → Pa */
        float alt_m = (p_home - p_Pa) / PA_PER_M;
        float t_c   = sensor_value_to_float(&temp);

        LOG_INF("press=%.1f Pa  alt=%+.3f m  temp=%.1f C",
                (double)p_Pa, (double)alt_m, (double)t_c);

        k_msleep(200);   /* 5 Hz — readable on console */
    }

    return 0;
}
