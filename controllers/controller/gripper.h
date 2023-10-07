#ifndef GRIPPER_H
#define GRIPPER_H

#include <webots/motor.h>
#include <webots/robot.h>

#ifdef __cplusplus
extern "C" {
#endif

void gripper_init();

void gripper_grip();
void gripper_release();

#define LEFT 0
#define RIGHT 1

#define MIN_POS 0.0
#define MAX_POS 0.015

static WbDeviceTag fingers;

void gripper_init() {
  fingers = wb_robot_get_device("finger::left");

  wb_motor_set_velocity(fingers, 0.03);
}

void gripper_grip() {
  wb_motor_set_position(fingers, MIN_POS);
}

void gripper_release() {
  wb_motor_set_position(fingers, MAX_POS);
}

#ifdef __cplusplus
}
#endif

#endif
