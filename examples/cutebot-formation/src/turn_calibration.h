/*
 * turn_calibration.h — per-robot turn-scale calibration for the Cutebot
 * Pro's odometry.
 *
 * Each robot's encoders mis-measure rotation by a few percent, CCW and CW
 * differently, and heading error is what makes a dead-reckoning estimate
 * drift. DEMO_MODE_TURN_CALIBRATE spins a robot a known number of encoder
 * turns; measure how far past or short of its starting heading it ends:
 *
 *   scale = (360 x turns + offset_deg) / encoder_total_deg
 *
 * and add a line here. Keyed by the micro:bit's hardware id (the first
 * four bytes of its device id, logged as "turn cal hw 0x..."), so one
 * firmware image serves every robot; a robot with
 * no entry uses 1.0. In firmware rather than flash because flashing a
 * micro:bit over USB erases its whole flash. Re-measure when a micro:bit
 * moves to another chassis.
 *
 * ccw / cw: true rotation = encoder-measured rotation x scale.
 */
#ifndef TURN_CALIBRATION_H
#define TURN_CALIBRATION_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t hw_id;
    float    ccw;
    float    cw;
} demo_turn_cal_t;

static const demo_turn_cal_t demo_turn_cal[] = {
    /* { 0x00000000, 1.0000f, 1.0000f },  hw id, CCW scale, CW scale.
     * Baseline 1.0000 while re-measuring (2026-10-05 evening). "id" is the
     * swarm id each robot had that day; ids are renegotiated at boot. */
    { 0x1e22785c, 0.9892f, 0.9871f },   /* id 0: 1 session of 15 turns each way, CCW -50, CW -60 (evening) */
    { 0x3cba0acb, 0.9974f, 0.9980f },   /* id 1: 1 session of 15 turns each way, 0 deg both (evening) */
    { 0xdd4e88f3, 0.9912f, 0.9941f },   /* id 2: fits over 3 CCW / 2 CW sessions of 15 turns (2026-10-05 evening) */
    { 0xe0a1225b, 0.9872f, 0.9869f },   /* id 3: 1 session of 15 turns each way, -60 both (evening) */
};

/* This robot's entry, or NULL. */
static inline const demo_turn_cal_t *demo_turn_cal_lookup(uint32_t hw_id)
{
    for (size_t i = 0; i < sizeof(demo_turn_cal) / sizeof(demo_turn_cal[0]); i++) {
        if (demo_turn_cal[i].hw_id != 0u && demo_turn_cal[i].hw_id == hw_id) {
            return &demo_turn_cal[i];
        }
    }
    return NULL;
}

#endif /* TURN_CALIBRATION_H */
