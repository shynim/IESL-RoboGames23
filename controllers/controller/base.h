#ifndef BASE_H
#define BASE_H

#include <webots/types.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

void base_init(double);
void compass_init(double);
void gps_init(double);
void base_reset();
void base_move(double, double, double);

#define MAX_SPEED 0.465
#define SPEED_INCREMENT 0.05
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

// robot geometry
#define WHEEL_RADIUS 0.05
#define LX 0.228  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.158  // lateral distance from robot's COM to wheel [m].

static WbDeviceTag wheels[4];
static WbDeviceTag gps;
static WbDeviceTag compass;

static double robot_vx = 0.0;
static double robot_vy = 0.0;
static double robot_omega = 0.0;

static void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  wb_motor_set_position(t, INFINITY);
  wb_motor_set_velocity(t, velocity);
}

static void base_set_wheel_speeds_helper(double speeds[4]) {
  int i;
  for (i = 0; i < 4; i++)
    base_set_wheel_velocity(wheels[i], speeds[i]);
}

void compass_init(double time_step){
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, time_step);

}

void gps_init(double time_step){
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, time_step);

}

void base_init(double time_step){
  int i;
  char wheel_name[16];
  for (i = 0; i < 4; i++) {
    sprintf(wheel_name, "wheel%d", (i + 1));
    wheels[i] = wb_robot_get_device(wheel_name);
  }

  compass_init(time_step);
  gps_init(time_step);
}

void base_reset() {
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);
  robot_vx = 0.0;
  robot_vy = 0.0;
  robot_omega = 0.0;
}

void base_move(double vx, double vy, double omega) {
  double speeds[4];
  speeds[0] = 1 / WHEEL_RADIUS * (vx + vy + (LX + LY) * omega);
  speeds[1] = 1 / WHEEL_RADIUS * (vx - vy - (LX + LY) * omega);
  speeds[2] = 1 / WHEEL_RADIUS * (vx - vy + (LX + LY) * omega);
  speeds[3] = 1 / WHEEL_RADIUS * (vx + vy - (LX + LY) * omega);
  base_set_wheel_speeds_helper(speeds);
  //printf("Speeds: vx=%.2f[m/s] vy=%.2f[m/s] ω=%.2f[rad/s]\n", vx, vy, omega);
}

#ifdef __cplusplus
}
#endif

#endif
