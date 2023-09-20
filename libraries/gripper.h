/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Allows to handle the gipper
 */

#ifndef GRIPPER_H
#define GRIPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <webots/motor.h>
#include <webots/robot.h>

void gripper_init();

void gripper_grip();  // dangerous to grip an object with this function -> creates a lot of internal constraints
void gripper_release();
void gripper_set_gap(double gap);

#define LEFT 0
#define RIGHT 1

#define MIN_POS 0.0
#define MAX_POS 0.015
#define OFFSET_WHEN_LOCKED 0.021

static WbDeviceTag fingers;

double bound(double v, double a, double b) {
  return (v > b) ? b : (v < a) ? a : v;
}

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

void gripper_set_gap(double gap) {
  double v = bound(0.5 * (gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
  wb_motor_set_position(fingers, v);
}


#ifdef __cplusplus
}
#endif

#endif
