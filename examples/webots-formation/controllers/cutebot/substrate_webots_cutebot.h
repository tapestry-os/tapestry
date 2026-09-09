/*
 * substrate_webots_cutebot.h — Tapestry L1 Physical Substrate Interface,
 * Webots differential-drive Cutebot backend
 *
 * Implements tapestry/substrate.h against a simulated Cutebot in Webots:
 * two independently-driven wheel RotationalMotors ("left_wheel_motor"/
 * "right_wheel_motor"), a GPS device ("gps"), and an InertialUnit
 * ("inertial_unit") for yaw — see ../../worlds/ring_cutebot.wbt for the
 * robot definition these device names must match.
 *
 * Unlike cf21bl's substrate_webots.c (PID attitude cascade run at the
 * Webots physics rate, decoupled from the L4-L7 coordination tick), a
 * ground rover under simple velocity control needs no inner loop:
 * substrate_move() sets wheel angular velocities directly, and Webots
 * applies them continuously between coordination ticks on its own — so
 * there is no substrate_webots_step() here, and position/yaw getters
 * read the GPS/InertialUnit devices live rather than a cached sample.
 *
 * Motion model — same differential-drive convention as
 * tapestry-os/boards/bbc_microbit_v2/substrate_cutebot.c (real hardware):
 *   twist.linear.x  = forward velocity, normalized [-1, 1]
 *   twist.angular.z = yaw rate,          normalized [-1, 1], positive = CCW
 * scaled here to MAX_SPEED_MPS / MAX_YAW_RATE_RADPS and converted to
 * per-wheel angular velocity via WHEEL_RADIUS_M/WHEEL_TRACK_M, instead of
 * hardware's I2C percent-speed command — see substrate_webots_cutebot.c.
 */

#ifndef TAPESTRY_SUBSTRATE_WEBOTS_CUTEBOT_H
#define TAPESTRY_SUBSTRATE_WEBOTS_CUTEBOT_H

/* Ground-truth pose, read live from the GPS/InertialUnit devices — not a
 * Tapestry-level abstraction. main.c uses these to populate gossip state
 * and the demo_odometry_t fed to demo_track_target(). Call only after
 * substrate_init(). */
void substrate_webots_cutebot_get_position(float *x, float *y, float *z);
float substrate_webots_cutebot_get_yaw(void);

#endif /* TAPESTRY_SUBSTRATE_WEBOTS_CUTEBOT_H */
