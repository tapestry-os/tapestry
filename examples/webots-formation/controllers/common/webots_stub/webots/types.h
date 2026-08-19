/* webots_stub/ — CI-only stand-ins for the slice of the real Webots C API
 * (github.com/cyberbotics/webots, include/controller/c/webots/) that this
 * example's controllers use. Not shipped by Webots, not a substitute for
 * building against the real SDK — see ../../../../ci-check/Makefile for
 * why this exists and what it does and doesn't verify.
 *
 * This tree holds only the substrate-agnostic wb_robot_* declarations —
 * identical for any Webots controller regardless of which devices it
 * drives, which is why it lives in controllers/common/ rather than
 * ../../../ci-check/webots_stub/ (that holds cf21bl's own device-specific
 * stubs — motor/gps/gyro/inertial_unit — the same split the real
 * common/cf21bl code already uses). */

#ifndef WEBOTS_CI_STUB_TYPES_H
#define WEBOTS_CI_STUB_TYPES_H

typedef int WbDeviceTag;

#endif
