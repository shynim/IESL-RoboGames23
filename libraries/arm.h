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
void arm_reset();

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

void arm_set_sub_arm_rotation(enum Arm arm, double radian);
double arm_get_sub_arm_length(enum Arm arm);

void arm_ik(double x, double y, double z);

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

  wb_motor_set_velocity(arm_elements[ARM2], 0.5);

  arm_set_height(ARM_RESET);
  arm_set_orientation(ARM_FRONT);
}

void arm_reset() {
  arm_set_height(ARM_RESET);
  arm_set_orientation(ARM_FRONT);
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
      fprintf(stderr, "arm_height() called with a wrong argument\n");
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
      fprintf(stderr, "arm_set_side() called with a wrong argument\n");
      return;
  }
  current_orientation = orientation;
}

void arm_set_sub_arm_rotation(enum Arm arm, double radian) {
  wb_motor_set_position(arm_elements[arm], radian);
}

double arm_get_sub_arm_length(enum Arm arm) {
  switch (arm) {
    case ARM1:
      return 0.253;
    case ARM2:
      return 0.155;
    case ARM3:
      return 0.135;
    case ARM4:
      return 0.081;
    case ARM5:
      return 0.105;
  }
  return 0.0;
}

void arm_ik(double x, double y, double z) {
  double y1 = sqrt(x * x + y * y);
  double z1 = z + arm_get_sub_arm_length(ARM4) + arm_get_sub_arm_length(ARM5) - arm_get_sub_arm_length(ARM1);

  double a = arm_get_sub_arm_length(ARM2);
  double b = arm_get_sub_arm_length(ARM3);
  double c = sqrt(y1 * y1 + z1 * z1);

  double alpha = -asin(x / y1);
  double beta = -(M_PI_2 - acos((a * a + c * c - b * b) / (2.0 * a * c)) - atan(z1 / y1));
  double gamma = -(M_PI - acos((a * a + b * b - c * c) / (2.0 * a * b)));
  double delta = -(M_PI + (beta + gamma));
  double epsilon = M_PI_2 + alpha;

  wb_motor_set_position(arm_elements[ARM1], alpha);
  wb_motor_set_position(arm_elements[ARM2], beta);
  wb_motor_set_position(arm_elements[ARM3], gamma);
  wb_motor_set_position(arm_elements[ARM4], delta);
  wb_motor_set_position(arm_elements[ARM5], epsilon);
}

#ifdef __cplusplus
}
#endif

#endif




