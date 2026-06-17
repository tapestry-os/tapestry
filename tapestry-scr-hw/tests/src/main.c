/*
 * main.c — Crazyflie 2.1 motor mixing unit tests
 *
 * Tests cf21_mix() in crazyflie21br_mix.h without any Zephyr hardware drivers.
 * All test inputs are exact float fractions so results are reproducible
 * across platforms; epsilon comparisons guard against rounding.
 *
 * Motor layout reminder (view from above):
 *   M4(CW)   M1(CCW)   ← front
 *   M3(CCW)  M2(CW)    ← back
 *
 * Mixing equations:
 *   T = (linear.z + 1.0) / 2.0
 *   R = angular.x - linear.y
 *   P = angular.y - linear.x
 *   Y = angular.z
 *
 *   M1 (FR, CCW) = T − R + P − Y
 *   M2 (BR, CW)  = T − R − P + Y
 *   M3 (BL, CCW) = T + R − P − Y
 *   M4 (FL, CW)  = T + R + P + Y
 *
 * Build:  west build -b native_sim tapestry-cf21-sim/tests
 * Run:    ./build/zephyr/zephyr.exe
 */

#include <zephyr/ztest.h>
#include <math.h>
#include "crazyflie21br_mix.h"

/* ── Helpers ─────────────────────────────────────────────────────────────── */

#define FLOAT_EPS  1e-5f

#define ASSERT_NEAR(val, expected, msg) \
    zassert_true(fabsf((val) - (expected)) < FLOAT_EPS, \
                 msg ": got %.6f expected %.6f", (double)(val), (double)(expected))

static cf21_motors_t mix(float lx, float ly, float lz,
                         float ax, float ay, float az)
{
    substrate_twist_t t = {
        .linear  = { .x = lx, .y = ly, .z = lz },
        .angular = { .x = ax, .y = ay, .z = az  },
    };
    cf21_motors_t out;
    cf21_mix(&t, &out);
    return out;
}

/* ── Test suite ──────────────────────────────────────────────────────────── */

ZTEST_SUITE(cf21_mix, NULL, NULL, NULL, NULL, NULL);

/*
 * test_zero_twist
 *
 * All twist components zero → T = 0.5, R = P = Y = 0.
 * Every motor must output exactly 0.5 (half throttle / hover point).
 */
ZTEST(cf21_mix, test_zero_twist)
{
    cf21_motors_t m = mix(0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f);

    ASSERT_NEAR(m.m1, 0.5f, "M1 zero-twist");
    ASSERT_NEAR(m.m2, 0.5f, "M2 zero-twist");
    ASSERT_NEAR(m.m3, 0.5f, "M3 zero-twist");
    ASSERT_NEAR(m.m4, 0.5f, "M4 zero-twist");
}

/*
 * test_full_throttle
 *
 * linear.z = +1.0 → T = 1.0, no roll/pitch/yaw.
 * All motors must reach full power.
 */
ZTEST(cf21_mix, test_full_throttle)
{
    cf21_motors_t m = mix(0.0f, 0.0f, 1.0f,  0.0f, 0.0f, 0.0f);

    ASSERT_NEAR(m.m1, 1.0f, "M1 full throttle");
    ASSERT_NEAR(m.m2, 1.0f, "M2 full throttle");
    ASSERT_NEAR(m.m3, 1.0f, "M3 full throttle");
    ASSERT_NEAR(m.m4, 1.0f, "M4 full throttle");
}

/*
 * test_idle_throttle
 *
 * linear.z = -1.0 → T = 0.0.
 * All motors must be at idle (zero output).
 */
ZTEST(cf21_mix, test_idle_throttle)
{
    cf21_motors_t m = mix(0.0f, 0.0f, -1.0f,  0.0f, 0.0f, 0.0f);

    ASSERT_NEAR(m.m1, 0.0f, "M1 idle");
    ASSERT_NEAR(m.m2, 0.0f, "M2 idle");
    ASSERT_NEAR(m.m3, 0.0f, "M3 idle");
    ASSERT_NEAR(m.m4, 0.0f, "M4 idle");
}

/*
 * test_yaw_ccw
 *
 * angular.z = +0.25 (CCW, turn left), linear.z = 0 → T = 0.5.
 * CW props (M2, M4) must increase; CCW props (M1, M3) must decrease.
 * Expected:
 *   Y = 0.25, R = P = 0
 *   M1 = 0.5 − 0.25 = 0.25   M2 = 0.5 + 0.25 = 0.75
 *   M3 = 0.5 − 0.25 = 0.25   M4 = 0.5 + 0.25 = 0.75
 */
ZTEST(cf21_mix, test_yaw_ccw)
{
    cf21_motors_t m = mix(0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.25f);

    ASSERT_NEAR(m.m1, 0.25f, "M1 yaw-CCW");
    ASSERT_NEAR(m.m2, 0.75f, "M2 yaw-CCW");
    ASSERT_NEAR(m.m3, 0.25f, "M3 yaw-CCW");
    ASSERT_NEAR(m.m4, 0.75f, "M4 yaw-CCW");

    /* CW prop pair must always be higher than CCW pair during CCW yaw */
    zassert_true(m.m2 > m.m1, "M2 (CW) > M1 (CCW) for CCW yaw");
    zassert_true(m.m4 > m.m3, "M4 (CW) > M3 (CCW) for CCW yaw");
}

/*
 * test_yaw_cw
 *
 * angular.z = -0.25 (CW, turn right).
 * CCW props (M1, M3) must increase; CW props (M2, M4) must decrease.
 * Expected (signs flip vs. CCW case):
 *   M1 = 0.75   M2 = 0.25   M3 = 0.75   M4 = 0.25
 */
ZTEST(cf21_mix, test_yaw_cw)
{
    cf21_motors_t m = mix(0.0f, 0.0f, 0.0f,  0.0f, 0.0f, -0.25f);

    ASSERT_NEAR(m.m1, 0.75f, "M1 yaw-CW");
    ASSERT_NEAR(m.m2, 0.25f, "M2 yaw-CW");
    ASSERT_NEAR(m.m3, 0.75f, "M3 yaw-CW");
    ASSERT_NEAR(m.m4, 0.25f, "M4 yaw-CW");
}

/*
 * test_roll_right_side_down
 *
 * angular.x = +0.25 (roll right, right side goes down).
 * Right motors (M1, M2) must decrease; left motors (M3, M4) must increase.
 * Expected:
 *   R = 0.25, T = 0.5, P = Y = 0
 *   M1 = 0.5 − 0.25 = 0.25 (FR, right side)
 *   M2 = 0.5 − 0.25 = 0.25 (BR, right side)
 *   M3 = 0.5 + 0.25 = 0.75 (BL, left side)
 *   M4 = 0.5 + 0.25 = 0.75 (FL, left side)
 */
ZTEST(cf21_mix, test_roll_right_side_down)
{
    cf21_motors_t m = mix(0.0f, 0.0f, 0.0f,  0.25f, 0.0f, 0.0f);

    ASSERT_NEAR(m.m1, 0.25f, "M1 roll-RSD");
    ASSERT_NEAR(m.m2, 0.25f, "M2 roll-RSD");
    ASSERT_NEAR(m.m3, 0.75f, "M3 roll-RSD");
    ASSERT_NEAR(m.m4, 0.75f, "M4 roll-RSD");

    /* Right pair (M1, M2) always lower than left pair (M3, M4) on RSD roll */
    zassert_true(m.m1 < m.m3, "right < left: M1 < M3");
    zassert_true(m.m2 < m.m4, "right < left: M2 < M4");
    /* Front-back symmetry: same roll on both sides */
    ASSERT_NEAR(m.m1, m.m2, "M1 == M2 (pure roll, no pitch/yaw)");
    ASSERT_NEAR(m.m3, m.m4, "M3 == M4 (pure roll, no pitch/yaw)");
}

/*
 * test_pitch_nose_up
 *
 * angular.y = +0.25 (pitch nose-up).
 * Front motors (M1, M4) must increase; back motors (M2, M3) must decrease.
 * Expected:
 *   P = 0.25, T = 0.5, R = Y = 0
 *   M1 = 0.5 + 0.25 = 0.75 (FR, front)
 *   M2 = 0.5 − 0.25 = 0.25 (BR, back)
 *   M3 = 0.5 − 0.25 = 0.25 (BL, back)
 *   M4 = 0.5 + 0.25 = 0.75 (FL, front)
 */
ZTEST(cf21_mix, test_pitch_nose_up)
{
    cf21_motors_t m = mix(0.0f, 0.0f, 0.0f,  0.0f, 0.25f, 0.0f);

    ASSERT_NEAR(m.m1, 0.75f, "M1 pitch-up");
    ASSERT_NEAR(m.m2, 0.25f, "M2 pitch-up");
    ASSERT_NEAR(m.m3, 0.25f, "M3 pitch-up");
    ASSERT_NEAR(m.m4, 0.75f, "M4 pitch-up");

    /* Front pair always higher than back pair on nose-up pitch */
    zassert_true(m.m1 > m.m2, "front > back: M1 > M2");
    zassert_true(m.m4 > m.m3, "front > back: M4 > M3");
    /* Left-right symmetry: same pitch contribution on both sides */
    ASSERT_NEAR(m.m1, m.m4, "M1 == M4 (pure pitch, no roll/yaw)");
    ASSERT_NEAR(m.m2, m.m3, "M2 == M3 (pure pitch, no roll/yaw)");
}

/*
 * test_forward
 *
 * linear.x = +0.25 (forward velocity command).
 * Contributes a nose-down pitch (P = −0.25), which tilts the drone forward.
 * Front motors (M1, M4) must decrease; back motors (M2, M3) must increase.
 * Expected:
 *   P = 0 − 0.25 = −0.25, T = 0.5, R = Y = 0
 *   M1 = 0.5 + (−0.25) = 0.25   M4 = 0.25
 *   M2 = 0.5 − (−0.25) = 0.75   M3 = 0.75
 */
ZTEST(cf21_mix, test_forward)
{
    cf21_motors_t m = mix(0.25f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f);

    ASSERT_NEAR(m.m1, 0.25f, "M1 forward");
    ASSERT_NEAR(m.m2, 0.75f, "M2 forward");
    ASSERT_NEAR(m.m3, 0.75f, "M3 forward");
    ASSERT_NEAR(m.m4, 0.25f, "M4 forward");

    /* Nose-down: back motors higher than front motors */
    zassert_true(m.m2 > m.m1, "back > front for forward: M2 > M1");
    zassert_true(m.m3 > m.m4, "back > front for forward: M3 > M4");
}

/*
 * test_lateral_left
 *
 * linear.y = +0.25 (left lateral velocity command).
 * Contributes a right-side-up roll (R = −0.25), tilting the drone left.
 * Right motors (M1, M2) must increase; left motors (M3, M4) must decrease.
 * Expected:
 *   R = 0 − 0.25 = −0.25, T = 0.5, P = Y = 0
 *   M1 = 0.5 − (−0.25) = 0.75   M2 = 0.75
 *   M3 = 0.5 + (−0.25) = 0.25   M4 = 0.25
 */
ZTEST(cf21_mix, test_lateral_left)
{
    cf21_motors_t m = mix(0.0f, 0.25f, 0.0f,  0.0f, 0.0f, 0.0f);

    ASSERT_NEAR(m.m1, 0.75f, "M1 left");
    ASSERT_NEAR(m.m2, 0.75f, "M2 left");
    ASSERT_NEAR(m.m3, 0.25f, "M3 left");
    ASSERT_NEAR(m.m4, 0.25f, "M4 left");

    /* Right-side-up: right motors higher to push right side up and tilt left */
    zassert_true(m.m1 > m.m3, "right > left for leftward lateral: M1 > M3");
    zassert_true(m.m2 > m.m4, "right > left for leftward lateral: M2 > M4");
}

/*
 * test_clamp_high
 *
 * Full throttle + full CCW yaw: unclamped M2/M4 would exceed 1.0.
 * Clamped outputs must be exactly 1.0; the lower pair must be in range.
 *
 * linear.z = 1.0 → T = 1.0; angular.z = 0.5 → Y = 0.5
 *   M2 (unclamped) = 1.0 + 0.5 = 1.5 → clamped to 1.0
 *   M4 (unclamped) = 1.0 + 0.5 = 1.5 → clamped to 1.0
 *   M1 (unclamped) = 1.0 − 0.5 = 0.5 → not clamped
 *   M3 (unclamped) = 1.0 − 0.5 = 0.5 → not clamped
 */
ZTEST(cf21_mix, test_clamp_high)
{
    cf21_motors_t m = mix(0.0f, 0.0f, 1.0f,  0.0f, 0.0f, 0.5f);

    zassert_true(m.m2 <= 1.0f, "M2 must not exceed 1.0");
    zassert_true(m.m4 <= 1.0f, "M4 must not exceed 1.0");
    ASSERT_NEAR(m.m2, 1.0f, "M2 clamped to 1.0");
    ASSERT_NEAR(m.m4, 1.0f, "M4 clamped to 1.0");
    ASSERT_NEAR(m.m1, 0.5f, "M1 within range");
    ASSERT_NEAR(m.m3, 0.5f, "M3 within range");
}

/*
 * test_clamp_low
 *
 * Idle throttle + CCW yaw: unclamped M1/M3 would go negative.
 * Clamped outputs must be exactly 0.0; the higher pair must be in range.
 *
 * linear.z = -1.0 → T = 0.0; angular.z = 0.5 → Y = 0.5
 *   M1 (unclamped) = 0.0 − 0.5 = −0.5 → clamped to 0.0
 *   M3 (unclamped) = 0.0 − 0.5 = −0.5 → clamped to 0.0
 *   M2 (unclamped) = 0.0 + 0.5 =  0.5 → not clamped
 *   M4 (unclamped) = 0.0 + 0.5 =  0.5 → not clamped
 */
ZTEST(cf21_mix, test_clamp_low)
{
    cf21_motors_t m = mix(0.0f, 0.0f, -1.0f,  0.0f, 0.0f, 0.5f);

    zassert_true(m.m1 >= 0.0f, "M1 must not go below 0.0");
    zassert_true(m.m3 >= 0.0f, "M3 must not go below 0.0");
    ASSERT_NEAR(m.m1, 0.0f, "M1 clamped to 0.0");
    ASSERT_NEAR(m.m3, 0.0f, "M3 clamped to 0.0");
    ASSERT_NEAR(m.m2, 0.5f, "M2 within range");
    ASSERT_NEAR(m.m4, 0.5f, "M4 within range");
}

/*
 * test_symmetry_yaw
 *
 * Opposite yaw commands must produce mirror-image outputs.
 * M1(CCW) and M2(CW) swap roles under yaw reversal; M3(CCW) and M4(CW) likewise.
 * With the same throttle and yaw magnitude ±0.2:
 *   m_ccw.m1 == m_cw.m2   (CCW: M1(CCW)↓ mirrors CW: M2(CW)↓)
 *   m_ccw.m2 == m_cw.m1
 *   m_ccw.m3 == m_cw.m4
 *   m_ccw.m4 == m_cw.m3
 */
ZTEST(cf21_mix, test_symmetry_yaw)
{
    cf21_motors_t ccw = mix(0.0f, 0.0f, 0.0f,  0.0f, 0.0f,  0.2f);
    cf21_motors_t cw  = mix(0.0f, 0.0f, 0.0f,  0.0f, 0.0f, -0.2f);

    ASSERT_NEAR(ccw.m1, cw.m2, "CCW.M1 == CW.M2");
    ASSERT_NEAR(ccw.m2, cw.m1, "CCW.M2 == CW.M1");
    ASSERT_NEAR(ccw.m3, cw.m4, "CCW.M3 == CW.M4");
    ASSERT_NEAR(ccw.m4, cw.m3, "CCW.M4 == CW.M3");
}

/*
 * test_symmetry_roll
 *
 * Opposite roll commands must produce mirror-image front-back outputs.
 * With angular.x = ±0.2 (right-side-down vs left-side-down):
 *   m_rsd.m1 == m_lsd.m3   right motor in RSD == left motor in LSD
 *   m_rsd.m3 == m_lsd.m1
 */
ZTEST(cf21_mix, test_symmetry_roll)
{
    cf21_motors_t rsd = mix(0.0f, 0.0f, 0.0f,  0.2f, 0.0f, 0.0f);
    cf21_motors_t lsd = mix(0.0f, 0.0f, 0.0f, -0.2f, 0.0f, 0.0f);

    ASSERT_NEAR(rsd.m1, lsd.m3, "RSD.M1 == LSD.M3 (right mirror of left)");
    ASSERT_NEAR(rsd.m3, lsd.m1, "RSD.M3 == LSD.M1");
    ASSERT_NEAR(rsd.m2, lsd.m4, "RSD.M2 == LSD.M4");
    ASSERT_NEAR(rsd.m4, lsd.m2, "RSD.M4 == LSD.M2");
}

/*
 * test_symmetry_pitch
 *
 * Opposite pitch commands must produce mirror-image left-right outputs.
 * With angular.y = ±0.2 (nose-up vs nose-down):
 *   m_up.m1 == m_dn.m2   front motor in nose-up == back motor in nose-down
 *   m_up.m2 == m_dn.m1
 */
ZTEST(cf21_mix, test_symmetry_pitch)
{
    cf21_motors_t up = mix(0.0f, 0.0f, 0.0f,  0.0f,  0.2f, 0.0f);
    cf21_motors_t dn = mix(0.0f, 0.0f, 0.0f,  0.0f, -0.2f, 0.0f);

    ASSERT_NEAR(up.m1, dn.m2, "nose-up.M1 == nose-down.M2 (front mirror of back)");
    ASSERT_NEAR(up.m2, dn.m1, "nose-up.M2 == nose-down.M1");
    ASSERT_NEAR(up.m4, dn.m3, "nose-up.M4 == nose-down.M3");
    ASSERT_NEAR(up.m3, dn.m4, "nose-up.M3 == nose-down.M4");
}

/*
 * test_superposition
 *
 * Roll and pitch applied simultaneously must equal individual contributions
 * added to the base thrust (superposition holds because mixing is linear).
 *
 * Expected:
 *   T = 0.5, R = 0.1, P = 0.15, Y = 0
 *   M1 = 0.5 − 0.1 + 0.15 = 0.55
 *   M2 = 0.5 − 0.1 − 0.15 = 0.25
 *   M3 = 0.5 + 0.1 − 0.15 = 0.45
 *   M4 = 0.5 + 0.1 + 0.15 = 0.75
 */
ZTEST(cf21_mix, test_superposition)
{
    cf21_motors_t m = mix(0.0f, 0.0f, 0.0f,  0.1f, 0.15f, 0.0f);

    ASSERT_NEAR(m.m1, 0.55f, "M1 roll+pitch superposition");
    ASSERT_NEAR(m.m2, 0.25f, "M2 roll+pitch superposition");
    ASSERT_NEAR(m.m3, 0.45f, "M3 roll+pitch superposition");
    ASSERT_NEAR(m.m4, 0.75f, "M4 roll+pitch superposition");
}

/*
 * test_throttle_scale
 *
 * Varying only linear.z must shift all four motors identically
 * (collective thrust does not disturb roll/pitch/yaw balance).
 *
 * At T = 0.25 and T = 0.75, all four outputs must be equal to T.
 */
ZTEST(cf21_mix, test_throttle_scale)
{
    cf21_motors_t lo = mix(0.0f, 0.0f, -0.5f,  0.0f, 0.0f, 0.0f);
    cf21_motors_t hi = mix(0.0f, 0.0f,  0.5f,  0.0f, 0.0f, 0.0f);

    /* T = (-0.5 + 1) / 2 = 0.25 */
    ASSERT_NEAR(lo.m1, 0.25f, "lo M1");
    ASSERT_NEAR(lo.m2, 0.25f, "lo M2");
    ASSERT_NEAR(lo.m3, 0.25f, "lo M3");
    ASSERT_NEAR(lo.m4, 0.25f, "lo M4");

    /* T = ( 0.5 + 1) / 2 = 0.75 */
    ASSERT_NEAR(hi.m1, 0.75f, "hi M1");
    ASSERT_NEAR(hi.m2, 0.75f, "hi M2");
    ASSERT_NEAR(hi.m3, 0.75f, "hi M3");
    ASSERT_NEAR(hi.m4, 0.75f, "hi M4");
}
