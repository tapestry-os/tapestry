/*
 * crazyflie21br_mix.h — Crazyflie 2.1 motor mixing math
 *
 * Pure C99 header; no OS or Zephyr dependencies.
 * Included by substrate_crazyflie21br.c and the tapestry-cf21-sim test harness.
 *
 * Motor layout (view from above, front = +x body frame):
 *
 *       Front (+x)
 *   M4(CW)   M1(CCW)
 *      \    /
 *     [fuselage]
 *      /    \
 *   M3(CCW)  M2(CW)
 *       Back
 *
 * Spin-direction convention (Crazyflie 2.1 brushless, per Bitcraze documentation):
 *   Diagonal motors spin the same direction; adjacent motors spin opposite.
 *   M1 (front-right, CCW prop) → CW  reaction torque on body (−Z)
 *   M2 (back-right,  CW  prop) → CCW reaction torque on body (+Z)
 *   M3 (back-left,   CCW prop) → CW  reaction torque on body (−Z)
 *   M4 (front-left,  CW  prop) → CCW reaction torque on body (+Z)
 *
 * Verify on hardware before closed-loop flight: spin M2(BR)+M4(FL) at low
 * throttle — frame should rotate CCW (yaw left, +Z). If CW, negate all
 * four Y terms in cf21_mix().
 *
 * Tapestry substrate_twist_t axes consumed (substrate.h conventions):
 *   linear.z   collective thrust : -1.0 = idle,        +1.0 = full throttle
 *   angular.x  roll  rate        : +1.0 = right-side-down
 *   angular.y  pitch rate        : +1.0 = nose-up
 *   angular.z  yaw   rate        : +1.0 = counterclockwise (turn left)
 *   linear.x   forward velocity  : +1.0 = forward  → nose-down pitch
 *   linear.y   lateral velocity  : +1.0 = left     → right-side-up roll
 *
 * Intermediate mixing terms:
 *   T = (linear.z + 1.0) / 2.0   maps [-1, 1] → [0, 1]  (collective)
 *   R = angular.x - linear.y      roll  command (right-side-down positive)
 *   P = angular.y - linear.x      pitch command (nose-up positive)
 *   Y = angular.z                  yaw   command (CCW positive)
 *
 * Motor equations (before clamping):
 *   M1 (FR, CCW) = T − R + P − Y    CCW prop → CW  torque → decrease for CCW yaw
 *   M2 (BR, CW)  = T − R − P + Y    CW  prop → CCW torque → increase for CCW yaw
 *   M3 (BL, CCW) = T + R − P − Y    CCW prop → CW  torque → decrease for CCW yaw
 *   M4 (FL, CW)  = T + R + P + Y    CW  prop → CCW torque → increase for CCW yaw
 *
 * Verification:
 *   Roll RSD (R > 0): M1↓ M2↓ (right) M3↑ M4↑ (left) → right-side-down ✓
 *   Pitch nose-up (P > 0): M1↑ M4↑ (front) M2↓ M3↓ (back) → nose-up ✓
 *   Yaw CCW (Y > 0): M2↑ M4↑ (diagonal CW props → CCW torque) M1↓ M3↓ → net CCW ✓
 *
 * All outputs clamped to [0.0, 1.0] before returning.
 */

#ifndef TAPESTRY_CRAZYFLIE21BR_MIX_H
#define TAPESTRY_CRAZYFLIE21BR_MIX_H

#include <tapestry/substrate.h>

/* ── Output type ─────────────────────────────────────────────────────────── */

typedef struct {
    float m1;  /* front-right, CCW [0.0, 1.0] */
    float m2;  /* back-right,  CW  [0.0, 1.0] */
    float m3;  /* back-left,   CCW [0.0, 1.0] */
    float m4;  /* front-left,  CW  [0.0, 1.0] */
} cf21_motors_t;

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static inline float cf21_clampf(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/* ── Motor mixing ────────────────────────────────────────────────────────── */

static inline void cf21_mix(const substrate_twist_t *twist, cf21_motors_t *out)
{
    float T = (twist->linear.z + 1.0f) * 0.5f;   /* collective [0, 1]  */
    float R = twist->angular.x - twist->linear.y; /* roll  cmd          */
    float P = twist->angular.y - twist->linear.x; /* pitch cmd          */
    float Y = twist->angular.z;                   /* yaw   cmd          */

    out->m1 = cf21_clampf(T - R + P - Y);   /* front-right, CCW */
    out->m2 = cf21_clampf(T - R - P + Y);   /* back-right,  CW  */
    out->m3 = cf21_clampf(T + R - P - Y);   /* back-left,   CCW */
    out->m4 = cf21_clampf(T + R + P + Y);   /* front-left,  CW  */
}

#endif /* TAPESTRY_CRAZYFLIE21BR_MIX_H */
