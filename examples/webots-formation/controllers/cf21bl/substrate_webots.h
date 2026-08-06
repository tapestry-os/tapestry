/*
 * substrate_webots.h — Tapestry L1 Physical Substrate Interface, Webots backend
 *
 * Implements tapestry/substrate.h (tapestry-os/include/tapestry/substrate.h)
 * against a simulated Crazyflie in Webots, using the Webots C controller
 * API (webots/motor.h, webots/gps.h, webots/gyro.h, webots/inertial_unit.h)
 * and the vendored Bitcraze PID controller (pid_controller.c) for the
 * attitude/velocity/altitude loop — same devices and gains as Webots'
 * bundled crazyflie.c reference controller.
 *
 * substrate_twist_t here uses the *documented* substrate.h convention: a
 * body-frame rate command. This is a cleaner fit than the real cf21bl
 * hardware's substrate implementation, which reinterprets linear.x/y as an
 * absolute home-relative position for its own stabilizer's reasons — no
 * equivalent trick is needed here since Webots' GPS is already absolute
 * ground truth. See ../../README.md.
 *
 * Two-rate design:
 *   substrate_move()        — called at the L4/L5/L7 coordination cadence
 *                              (WM_CYCLE_MS, 100 ms) by main.c. Only
 *                              latches the desired twist; does no I/O.
 *   substrate_webots_step()  — called every Webots physics step (the
 *                              simulation's basic time step, typically
 *                              8-32 ms) by main.c's outer loop. Reads
 *                              sensors, runs the PID cascade against the
 *                              latched twist, and writes the four motors.
 * This decouples Tapestry's coordination tick rate from the physics rate
 * flight control actually needs — the same physical decoupling real
 * hardware has between cf21bl_stabilizer.c's fast attitude loop and the
 * 100 ms main loop that only updates its target.
 */

#ifndef TAPESTRY_SUBSTRATE_WEBOTS_H
#define TAPESTRY_SUBSTRATE_WEBOTS_H

/* Advance the physics-rate control loop by dt seconds (the Webots basic
 * time step, in seconds). Call once per wb_robot_step(), after
 * substrate_init(). */
void substrate_webots_step(double dt);

/* Ground-truth pose from the last substrate_webots_step() call — Webots
 * GPS/IMU readings, not a Tapestry-level abstraction. main.c uses these
 * to populate gossip state and drive the choreo target tracker. */
void substrate_webots_get_position(float *x, float *y, float *z);
float substrate_webots_get_yaw(void);

#endif /* TAPESTRY_SUBSTRATE_WEBOTS_H */
