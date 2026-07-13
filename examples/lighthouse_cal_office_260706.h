/*
 * lighthouse_cal_office_260706.h — the ONE in-code copy of the base-station
 * geometry + OOTX calibration, generated from lighthouse_cal_office_260706.yaml
 * (same directory; cfclient export, 2026-07-06 second recalibration — BS1
 * suspected partially occluded, tilted further and re-run).
 *
 * Every executable that consumes lighthouse position includes THIS file —
 * the constants used to be duplicated per-example, and a partial update
 * (2 of 4 copies) once shipped stale geometry to a flight.  If the room is
 * recalibrated: re-run cfclient's Estimate Geometry, export the YAML, and
 * update this ONE file (poses from "geos:", calib from "calibs:").
 *
 * - Poses ("geos:") change whenever a base station is MOVED or TILTED.
 * - OOTX sweep calibration ("calibs:") is factory data tied to each
 *   physical base station's uid — it does NOT change with placement, only
 *   if a base station unit is swapped.
 * - The world frame is defined by the drone's pose during Estimate
 *   Geometry.  All drones in a formation MUST fly the same values —
 *   gossiped positions are only comparable in a shared frame.
 *
 * Sanity check after any recalibration (see examples/lighthouse-test):
 * a drone on the floor at the calibration spot must read ≈ (0, 0, 0).
 */

#ifndef TAPESTRY_EXAMPLES_LIGHTHOUSE_CAL_OFFICE_260706_H
#define TAPESTRY_EXAMPLES_LIGHTHOUSE_CAL_OFFICE_260706_H

#include "cf21bl_lighthouse.h"

static const lh2_bs_pose_t BS0 = {
    .origin = {-0.6803646087646484f, 0.6335355639457703f, 1.615210771560669f},
    .rot    = {0.8344101905822754f,  -0.08563866466283798f, 0.5444498062133789f,
               0.13026301562786102f,  0.9905099272727966f, -0.04383661970496178f,
              -0.535528838634491f,    0.1074993908405304f,  0.8376471400260925f}
};
static const lh2_bs_pose_t BS1 = {
    .origin = {0.09399518370628357f, -2.2131965160369873f, 1.4227608442306519f},
    .rot    = {0.049453821033239365f, -0.9982976317405701f,  0.03092208132147789f,
               0.9098809957504272f,    0.057799000293016434f, 0.41082337498664856f,
              -0.4119112491607666f,    0.00781862810254097f,  0.911190390586853f}
};

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

/* SteamVR channel assignments (0-indexed = SteamVR channel number − 1).
 * Check cfclient's Lighthouse tab for which channel each BS is assigned. */
#define BS0_CHANNEL  0
#define BS1_CHANNEL  1

#endif /* TAPESTRY_EXAMPLES_LIGHTHOUSE_CAL_OFFICE_260706_H */
