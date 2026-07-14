/*
 * cf21bl_pm.h — battery monitoring for the Crazyflie 2.1 brushless
 *
 * The STM32 has no VBAT ADC path on this board; battery voltage is measured
 * by the nRF51 power-management MCU and delivered over the syslink UART
 * (USART6) as SYSLINK_PM_BATTERY_STATE (0x13) packets:
 *
 *   payload = [flags u8][vBat float32 LE][chargeCurrent float32 LE]
 *   flags bit0 = isCharging, bit1 = usbPluggedIn, bit2 = canCharge
 *
 * (Wire format from Bitcraze crazyflie-firmware pm_stm32f4.c PmSyslinkInfo.)
 *
 * Two operating modes, selected automatically at build time:
 *
 *   CONFIG_TAPESTRY_TRANSCEIVER_SYSLINK enabled — the transport subsystem
 *   owns USART6; its RX parser forwards battery packets to
 *   cf21bl_pm_syslink_input().  cf21bl_pm_init() does no UART setup.
 *
 *   Otherwise — this module owns USART6 with its own minimal syslink RX
 *   parser, and periodically sends SYSLINK_PM_BATTERY_AUTOUPDATE to wake the
 *   nRF51 TX path (the nRF51 stays silent until it receives any packet)
 *   until the first battery packet arrives.
 *
 * Enable with CONFIG_CF21BL_PM=y and add cf21bl_pm.c to the application
 * CMakeLists.  CONFIG_CF21BL_PM_COMPENSATE additionally scales the motor
 * commands in cf21bl_stabilizer.c by VREF/vbat so thrust stays consistent
 * as the pack sags (stock firmware: CONFIG_ENABLE_THRUST_BAT_COMPENSATED).
 */

#ifndef TAPESTRY_CF21BL_PM_H
#define TAPESTRY_CF21BL_PM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * cf21bl_pm_init — start battery monitoring.  Call from cf21bl_init() before
 * the stabilizer starts.  Returns 0 on success (including "waiting for first
 * packet"), negative errno if the UART is unavailable in standalone mode.
 */
int cf21bl_pm_init(void);

/* Filtered battery voltage in volts; 0.0f until the first packet arrives. */
float cf21bl_pm_vbat(void);

/*
 * Motor-command scale factor VREF/vbat (clamped to [0.8, 1.3]) that keeps
 * delivered motor voltage — and thus thrust and loop gain — approximately
 * constant as the pack discharges.  Returns 1.0f when no measurement is
 * available or the reading is implausible (< 2 V, per stock sanity check).
 */
float cf21bl_pm_thrust_scale(void);

/* True when vbat has been below 3.35 V continuously for 5 s (warning level,
 * matches stock DEFAULT_BAT_LOW_VOLTAGE / _LOW_DURATION_TO_TRIGGER_SEC). */
bool cf21bl_pm_battery_low(void);

/* Latched true after vbat stays below 3.0 V continuously for 2 s (stock
 * DEFAULT_BAT_CRITICAL_LOW_VOLTAGE).  The stabilizer responds by forcing a
 * controlled landing.  Only a power-cycle clears the latch. */
bool cf21bl_pm_battery_critical(void);

/*
 * cf21bl_pm_syslink_input — feed one SYSLINK_PM_BATTERY_STATE payload (without
 * the syslink header/checksum).  Called by the transport subsystem's syslink
 * RX parser, or by this module's own parser in standalone mode.  ISR-safe.
 */
void cf21bl_pm_syslink_input(const uint8_t *payload, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* TAPESTRY_CF21BL_PM_H */
