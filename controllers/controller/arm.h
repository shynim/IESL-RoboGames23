#ifndef ARM_H
#define ARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

void arm_init();

enum Height {
  ARM_PLATE,
  ARM_FLOOR,
  ARM_RESET,
  ARM_DOWN

};

void arm_set_height(enum Height height);

enum Orientation {
  ARM_RIGHT,
  ARM_LEFT,
  ARM_FRONT
};

void arm_set_orientation(enum Orientation orientation);

enum Arm {ARM1, ARM2, ARM3, ARM4, ARM5};

static WbDeviceTag arm_elements[5];

static enum Height current_height = ARM_RESET;
static enum Orientation current_orientation = ARM_FRONT;

enum Height new_height;
enum Orientation new_orientation;

void arm_init() {
  arm_elements[ARM1] = wb_robot_get_device("arm1");
  arm_elements[ARM2] = wb_robot_get_device("arm2");
  arm_elements[ARM3] = wb_robot_get_device("arm3");
  arm_elements[ARM4] = wb_robot_get_device("arm4");
  arm_elements[ARM5] = wb_robot_get_device("arm5");

}

void arm_set_height(enum Height height) {
  switch (height) {
    case ARM_FLOOR:
      wb_motor_set_position(arm_elements[ARM2], -0.97);
      wb_motor_set_position(arm_elements[ARM3], -1.55);
      wb_motor_set_position(arm_elements[ARM4], -0.61);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_PLATE:
      wb_motor_set_position(arm_elements[ARM2], -0.62);
      wb_motor_set_position(arm_elements[ARM3], -0.98);
      wb_motor_set_position(arm_elements[ARM4], 1.1);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
      break;
    case ARM_DOWN:
      wb_motor_set_position(arm_elements[ARM2], -1.13446);
      wb_motor_set_position(arm_elements[ARM3], -1.55);
      wb_motor_set_position(arm_elements[ARM4], 1.1);
      wb_motor_set_position(arm_elements[ARM5], 0.0);
    default:
      //fprintf(stderr, "arm_height() called with a wrong argument\n");
      return;
  }
  current_height = height;
}

void arm_set_orientation(enum Orientation orientation) {
  switch (orientation) {
    case ARM_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -M_PI_2);
      break;
    case ARM_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], M_PI_2);
      break;
    case ARM_FRONT:
      wb_motor_set_position(arm_elements[ARM1], 0);
      break;
    default:
      //fprintf(stderr, "arm_set_side() called with a wrong argument\n");
      return;
  }
  current_orientation = orientation;
}

#ifdef __cplusplus
}
#endif

#endif




