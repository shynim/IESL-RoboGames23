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
#include <base.h>
#include <arm.h>
#include <gripper.h>

#define TIME_STEP 32

bool go_to_reached = false;

static double kx = 50;
static double ky = 50;

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

bool are_same(float a, double b, double tolerance = 1e-6) {
  return std::fabs(a - b) < tolerance;
}

void go_to(double x, double y){

  while(!go_to_reached){

    step();

    const double *range_image = wb_gps_get_values(gps);
    
    const double *range_image_y = &range_image[1];
    const double *range_image_x = &range_image[0];

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

void start(){
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
  passive_wait(0.45);

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
  go_to(pos_arr[i].u, pos_arr[i].v);i++;    
  
  go_to_reached = false;
  go_to(pos_arr[i].u, pos_arr[i].v);i++;   

  gripper_grip();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pos_arr[i].u, pos_arr[i].v);i++;  

  go_to_reached = false;
  go_to(pos_arr[i].u, pos_arr[i].v);i++;  

  gripper_release();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pos_arr[i].u, pos_arr[i].v);i++;    

  gripper_grip();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pos_arr[i].u, pos_arr[i].v);i++;    

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
  go_to(pos_arr[i].u, pos_arr[i].v);i++;    

  gripper_grip();
  passive_wait(0.5);

  go_to_reached = false;
  go_to(pos_arr[i].u, pos_arr[i].v);i++;    

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

  base_init(TIME_STEP);
  arm_init();
  gripper_init();
  passive_wait(0.2);

  //main loop
  while(true){
  
    step();

    start();

    place_double((Vector){-1.75, 0.409}, (Vector){1.256, 1.14});
    place_double((Vector){-1.7, 0.409}, (Vector){1.256, 0.64});
    place_double((Vector){-1.65, 0.409}, (Vector){1.256, 0.14});
    place_double((Vector){-1.6, 0.409}, (Vector){1.256, -0.36});
    place_double((Vector){-1.75, 0.359}, (Vector){1.256, -0.86});

    place_single((Vector[]){{-1.7, 0.5}, {-1.7, 0.359}, {-1.7, 0.5}, {1.256, 1.0365}, {1.256, 1.049}, {1.256, 1.14}, {1.256, 1.0240}, {1.256, 0.64}});
    place_single((Vector[]){{-1.65, 0.5}, {-1.65, 0.359}, {-1.65, 0.5}, {1.256, 0.0365}, {1.256, 0.049}, {1.256, 0.14}, {1.256, 0.024}, {1.256, -0.36}});

    place_double((Vector){-1.6, 0.3706}, (Vector){1.256, -0.86});
    
    base_reset();
    break;

  }

  cout << wb_robot_get_time() / 60 << endl;
  wb_robot_cleanup();

  return 0;
}


