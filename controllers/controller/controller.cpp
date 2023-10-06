#include <iostream>
using namespace std;

#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/lidar.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arm.h>
#include <gripper.h>

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

static double kx = 50;
static double ky = 50;
static double kz = 10;

int pc = 0;
double m = 0.001;
double x = -1.725;
double y = 0.6;

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

double angle(const  Vector *v1, const  Vector *v2) {
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

void base_control(){
  int c = wb_keyboard_get_key();
    
    if ((c >= 0) && c != pc) {
      switch (c) {
        case WB_KEYBOARD_UP:
          x += m;
          break;
        case WB_KEYBOARD_DOWN:
          x -= m;
          break;
        case WB_KEYBOARD_LEFT:
          y += m;
          break;
        case WB_KEYBOARD_RIGHT:
          y -= m;
          break;
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          m *= 10;
          break;
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          m /= 10;
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
      go_to_reached = false;
      go_to(x, y);
      cout << x << " " << y << " " << m <<  endl;
    }
    pc = c;

}

void place_double(const Vector &pickup, const Vector &place){
  go_to_reached = false;
  go_to(pickup.u, 0.5);    
  
  go_to_reached = false;
  go_to(pickup.u, pickup.v);

  gripper_grip();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pickup.u, 0.5);

  arm_set_height(ARM_PLATE);
  passive_wait(0.25);

  arm_set_height(ARM_DOWN);

  arm_set_orientation(ARM_RIGHT);

  go_to_reached = false;
  go_to(place.u, place.v);

  gripper_release();
  passive_wait(0.2);

  arm_set_height(ARM_FLOOR);

  arm_set_orientation(ARM_FRONT);
 
}

void place_single(const Vector (&pos_arr)[8]){

  int i = 0;

  go_to_reached = false; //
  go_to(pos_arr[i++].u, pos_arr[i].v);    
  
  go_to_reached = false;
  go_to(pos_arr[i++].u, pos_arr[i].v);    

  gripper_grip();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pos_arr[i++].u, pos_arr[i].v);    

  go_to_reached = false;
  go_to(pos_arr[i++].u, pos_arr[i].v);    

  gripper_release();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pos_arr[i++].u, pos_arr[i].v);    

  gripper_grip();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pos_arr[i++].u, pos_arr[i].v);    

  arm_set_height(ARM_PLATE);
  passive_wait(0.5);
  
  arm_set_height(ARM_DOWN);
  passive_wait(0.5);

  arm_set_orientation(ARM_RIGHT);
  passive_wait(1.3);

  gripper_release();
  passive_wait(0.5);

  arm_set_height(ARM_FLOOR);
  passive_wait(0.5);

  arm_set_orientation(ARM_FRONT);
  passive_wait(0.5);

  gripper_release();
  passive_wait(0.2);

  go_to_reached = false;
  go_to(pos_arr[i++].u, pos_arr[i].v);    

  gripper_grip();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pos_arr[i++].u, pos_arr[i].v);    

  arm_set_height(ARM_PLATE);
  passive_wait(0.5);
  
  arm_set_height(ARM_DOWN);
  passive_wait(0.5);

  arm_set_orientation(ARM_RIGHT);
  passive_wait(1.3);

  gripper_release();
  passive_wait(0.2);

  arm_set_height(ARM_FLOOR);

  arm_set_orientation(ARM_FRONT);
  
  
}

int main(int argc, char **argv) {

  wb_robot_init();

  compass_init();
  gps_init();
  base_init();
  arm_init();
  gripper_init();
  passive_wait(0.2);

  //main loop
  while(true){
  
    step();

    gripper_release();

    arm_set_height(ARM_FLOOR);

    //align with first set
    go_to_reached = false;
    go_to(x, y);

    kx = 10;
    ky = 10;

    //pushing first set
    go_to_reached = false;
    go_to(-1.725, 0.441);

    kx = 50;
    ky = 50;
     
    //align with first set
    go_to_reached = false;
    go_to(x, y);

    //align with second set
    go_to_reached = false;
    go_to(-1.625, 0.6);
 
    kx = 10;
    ky = 10;

    //pushing second set
    go_to_reached = false;
    go_to(-1.625, 0.441);

    kx = 50;
    ky = 50;
    
    place_double((Vector){-1.75, 0.409}, (Vector){1.256, 1.14});
    place_double((Vector){-1.7, 0.409}, (Vector){1.256, 0.64});
    place_double((Vector){-1.65, 0.409}, (Vector){1.256, 0.14});
    place_double((Vector){-1.6, 0.409}, (Vector){1.256, -0.36});
    place_double((Vector){-1.75, 0.359}, (Vector){1.256, -0.86});

    place_single({{-1.7, 0.5}, {-1.7, 0.359}, {-1.7, 0.5}, {1.256, 1.0365}, {1.256, 1.049}, {1.256, 1.14}, {1.256, 1.0240}, {1.256, 0.64}});
    place_single({{-1.65, 0.5}, {-1.65, 0.359}, {-1.65, 0.5}, {1.256, 0.0365}, {1.256, 0.049}, {1.256, 0.14}, {1.256, 0.024}, {1.256, -0.36}});

    place_double((Vector){-1.6, 0.359}, (Vector){1.256, -0.86});

    break;

  }

  cout << wb_robot_get_time() / 60 << endl;
  wb_robot_cleanup();

  return 0;
}


