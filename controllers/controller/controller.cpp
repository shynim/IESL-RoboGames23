#include <iostream>
using namespace std;

#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/lidar.h>
#include <webots/compass.h>
#include <webots/gps.h>

#include <arm.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SPEED 0.3 // m/s
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

// robot geometry
#define WHEEL_RADIUS 0.05
#define LX 0.228  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.158  // lateral distance from robot's COM to wheel [m].

#define TIME_STEP 32

static WbDeviceTag lidars[3];
static WbDeviceTag wheels[4];
static WbDeviceTag compass;
static WbDeviceTag gps;

static double robot_vx = 0.0;
static double robot_vy = 0.0;
static double robot_omega = 0.0;

bool go_to_reached = false;
bool turn_to_reached = false;

static double kx = 10;
static double ky = 10;
static double kz = 10;

static double setPoint_x = 0.0;
static double setPoint_y = 0.0;

struct Vector{
  double u;
  double v;
};

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  wb_motor_set_position(t, INFINITY);
  wb_motor_set_velocity(t, velocity);
}

static void base_set_wheel_speeds_helper(double speeds[4]) {
  int i;
  for (i = 0; i < 4; i++)
    base_set_wheel_velocity(wheels[i], speeds[i]);
}

void base_reset() {
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);
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

void pid_x(float setPoint, float currentPos){
  float err = setPoint - currentPos;
  float correction = (err * -1) * kx;

  if(correction > MAX_SPEED){
    correction = MAX_SPEED;
  }else if(correction < -MAX_SPEED){
    correction = -MAX_SPEED;
  }

  robot_vx = correction;
  
}

void pid_y(float setPoint, float currentPos){
  float err = setPoint - currentPos;
  float correction = (err) * ky;

  if(correction > MAX_SPEED){
    correction = MAX_SPEED;
  }else if(correction < -MAX_SPEED){
    correction = -MAX_SPEED;
  }

  robot_vy = correction;
  
}

void pid_z(double setAng, double currentAng){
  double err = setAng - currentAng;
  if(abs(err) > 3){
    err = ((2 * M_PI) - abs(err)) * ((err * -1) / abs(err));
  }
  double correction = (err) * kz;

  if(correction > MAX_SPEED){
    correction = MAX_SPEED;
  }else if(correction < -MAX_SPEED){
    correction = -MAX_SPEED;
  }

  robot_omega = correction;
}

bool are_same(float a, double b, double tolerance = 1e-6) {
    return std::fabs(a - b) < tolerance;
}

double angle(const Vector *v1, const Vector *v2) {
  return atan2(v2->v, v2->u) - atan2(v1->v, v1->u);
}

void turn_to(double z){
  while(!turn_to_reached){
    step();

    const double *compass_raw_values = wb_compass_get_values(compass);

    Vector front = {compass_raw_values[0], compass_raw_values[1]};
    Vector north = {1.0, 0.0};
    double theta = angle(&front, &north);

    turn_to_reached = (are_same(z, theta));
    cout << turn_to_reached << endl;

    pid_z(z, theta);

    base_move(robot_vx, robot_vy, robot_omega);

  }

}

void go_to(double x, double y){

  while(!go_to_reached){

    step();

    // const float *range_image_x= wb_lidar_get_range_image(lidars[1]);
    // cout << (*range_image_y) << " ";
    // const float *range_image_y= wb_lidar_get_range_image(lidars[2]);
    // cout << (*range_image_x) << endl;

    const double *range_image = wb_gps_get_values(gps);
    
    const double *range_image_y = &range_image[1];
    //cout << (*range_image_y) << " ";
    const double *range_image_x = &range_image[0];
    //cout << (*range_image_x) << endl;

    go_to_reached = (are_same(*range_image_y, y) && are_same(*range_image_x, x));

    if(abs(*range_image_y - y) > abs(*range_image_x - x)){
      pid_x(y, *range_image_y);
      double ratio = (*range_image_x - x) / (*range_image_y - y);
      robot_vy = ratio * robot_vx;

      if(robot_vy > MAX_SPEED){
        robot_vy = MAX_SPEED;
      }else if(robot_vy < -MAX_SPEED){
        robot_vy = -MAX_SPEED;
      }
      base_move(robot_vx, -robot_vy, robot_omega);
    }else{
      pid_y(x, *range_image_x);
      double ratio = (*range_image_y - y) / (*range_image_x - x);
      robot_vx = ratio * robot_vy;
      
      if(robot_vx > MAX_SPEED){
        robot_vx = MAX_SPEED;
      }else if(robot_vx < -MAX_SPEED){
        robot_vx = -MAX_SPEED;
      }
      base_move(-robot_vx, robot_vy, robot_omega);
    } 

  }
  
}

void lidar_init(){
  int i;
  char lidar_name[16];
  for (i = 0; i < 3; i++) {
    sprintf(lidar_name, "lidar%d", (i + 1));
    lidars[i] = wb_robot_get_device(lidar_name);
    wb_lidar_enable(lidars[i], TIME_STEP);
    wb_lidar_enable_point_cloud(lidars[i]);
  }

}

void base_init(){
  int i;
  char wheel_name[16];
  for (i = 0; i < 4; i++) {
    sprintf(wheel_name, "wheel%d", (i + 1));
    wheels[i] = wb_robot_get_device(wheel_name);
  }
}

void compass_init(){
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);

}

void gps_init(){
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

}

int main(int argc, char **argv) {
  wb_robot_init();

  compass_init();
  gps_init();
  base_init();
  lidar_init();
  arm_init();
  gripper_init();
  passive_wait(2.0);

  //main loop
  while(true){
  
    step();
    // go_to_reached = false;
    // go_to(setPoint_x, setPoint_y);

    // go_to_reached = false;
    // go_to(-1.0, -1.0);
     
    // go_to_reached = false;
    // go_to(1.0, -1.0);

    // go_to_reached = false;
    // go_to(1.0, -1.0);

    // const double *compass_raw_values = wb_compass_get_values(compass);

    // Vector front = {compass_raw_values[0], compass_raw_values[1]};
    // Vector north = {1.0, 0.0};

    // double theta = angle(&front, &north);
    // cout << theta << endl;
    // base_move(robot_vx, robot_vy, 0.5);

    turn_to_reached = false;
    turn_to(M_PI / 2);

    turn_to_reached = false;
    turn_to(-(M_PI / 2));
  }

  wb_robot_cleanup();

  return 0;
}
