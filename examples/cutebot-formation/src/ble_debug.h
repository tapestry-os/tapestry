/*
 * ble_debug.h — wireless debug telemetry for untethered runs
 * (CONFIG_DEMO_BLE_DEBUG; build with -DEXTRA_CONF_FILE=ble_debug.conf).
 *
 * A second BLE advertising set, in LEGACY (31-byte) format so any BLE
 * scanner can hear it, including ones without LE Extended Advertising
 * support, to which the gossip set's extended PDUs are invisible.
 *
 * Every packet is one manufacturer-specific AD structure, 27 bytes after
 * the Tapestry company ID (0x08D7):
 *
 *   [0] kind   0x10 status / 0x11 text
 *   [1] id     element id (0xFF until negotiated)
 *   [2-3] seq  u16 LE, +1 per NEW packet (each is advertised ~2-3 times)
 *
 *   status [4..26]: ble_debug_status_t, packed as documented in
 *                   ble_debug.c's pack_status() (23 bytes)
 *   text   [4]:     tseq, u8, +1 per text chunk (the receiver's gap check)
 *          [5..26]: 22 bytes of a '\n'-separated log stream, 0-padded
 *
 * Robots ignore it for free: transceiver_ble.c hands any Tapestry AD that
 * is neither a gossip nor a fact frame to its weak hook, a no-op on robots.
 *
 * Packets go out from a system-workqueue item every 120 ms, independent
 * of main() — so a robot stuck in ID negotiation is still visible (as
 * id 0xFF, step -1).  The functions below may be called from any thread.
 */
#ifndef BLE_DEBUG_H
#define BLE_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int8_t  step;          /* choreo_script_step(), -1 before the script   */
    uint8_t track;
    uint8_t quorum;        /* scr_quorum_state_t (0 LOST .. 2 HEALTHY)     */
    uint8_t fresh_peers;
    bool    suspended;     /* Choreo frozen by quorum loss                 */
    bool    discovered;    /* own (local) discovered bit                   */
    bool    point_valid;   /* a TARGET shared fact is held (rank 0)        */
    bool    target_valid;  /* directive is MOVE_TO_POINT                   */
    float   x, y, heading; /* own estimate, world units / radians          */
    float   tx, ty;        /* directive target                             */
    float   px, py;        /* that target landmark's position              */
    uint8_t gray_max;      /* darkest line-sensor reading this tick        */
} ble_debug_status_t;

/* Create and start the advertising set.  Call after transport_init()
 * (which enables Bluetooth).  Returns 0 or a negative errno; on failure
 * every other call is a no-op. */
int  ble_debug_init(void);

void ble_debug_set_id(uint8_t id);

/* Append one line (a '\n' is added) to the text stream.  Lines that don't
 * fit the 2 KB queue are dropped and counted in the status packet. */
void ble_debug_printf(const char *fmt, ...);

/* Latest status; sent at ~2 Hz, more often while no text is queued. */
void ble_debug_set_status(const ble_debug_status_t *st);

#endif /* BLE_DEBUG_H */
