#include "main.h"
#include "config.hpp"
#include <math.h>
#include <vector>
#include "auton_function.h"
/*
namespace WheelTracker{
  float wheelrad = 2.75;

  float lencval = 0;
  float rencval = 0;
  float bencval = 0;

  float prevl = 0;
  float prevr = 0;
  float prevb = 0;
  float thetar = 180;//constant
  float theta1;
  float theta0 = 0;
  float thetam;

  float rdis = 4.72440945;
  float ldis = 4.72440945;
  float bdis= 4.33070866;

  float dltheta = 0; //change in encoder angle
  float drtheta = 0;
  float dbtheta = 0;

  float dl; //change in distance
  float dr;
  float ds;

  float dtheta;


  std::vector<float> d (3);
  std::vector<float> dtemp (3);
  std::vector <float> d1 (3);

  float rad;
  float angle;
  float currentl;
  float currentr;
  float currentb;

std::vector<float> track(){



    currentl = LeftEncoder.get_value();
    currentr = RightEncoder.get_value();
    currentb = BackEncoder.get_value();

    dltheta = currentl - prevl;
    drtheta = currentr - prevr;
    dbtheta = currentb - prevb;

    //distance traveled by each wheel
    dl = ((dltheta)/360) * 2 *M_PI * wheelrad;
    dr = ((drtheta)/360) * 2 *M_PI * wheelrad;
    ds = ((dbtheta)/360) * 2 *M_PI * wheelrad;

    //update values
    prevl = currentl;
    prevr = currentr;
    prevb = currentb;

    theta1 = thetar + 180*((dl-dr)/(rdis+ldis))/M_PI;//convert angle to degrees and add to initial angle to find new angle

    dtheta = theta1 - theta0; //change in angle

    if(dtheta == 0.0){//if only vertical movement add vertical component dr

      dtemp.at(0) = ds;
      dtemp.at(1) = dr;
    }else{
      dtemp.at(0) = 2 * sin(dtheta/2) * (ds/dtheta + bdis);
      dtemp.at(1) = 2 * sin(dtheta/2) * (dr/dtheta + rdis);


    //calculate average orientation
    thetam = theta0 + dtheta/2;

    //convert position to polar
    angle = atan(dtemp[1] / dtemp[0]);
    rad = sqrt(dtemp[0] * dtemp[0] + dtemp[1] * dtemp[1]);

    angle += -1 * thetam;

    //convert back
    dtemp[0] = rad * cos(angle);
    dtemp[1] = rad * sin(angle);
}
    //add position vector to old one
    for(int i = 0; i < 2; i++){
      d1[i] = d[i] + dtemp[i];
    }

      theta0 = theta1;

      d[0] = d1[0];
      d[1] = d1[1];
      d1[2] = theta0;
      //return new position
      return d1;
    }

}

void measure_jerk(){
  float vel_max = 200*4/60; //inches per s
  float pos = 0;
  float d_pos;
  float vel;
  float vel_prev;
  float a_prev;
  float a;
  float d_a;
  float j_max = 5.0 ;//inch / s^3
  while (true){
    d_pos = RobotPosition[0] - pos;

    a += 0.01*j_max;
    vel += a_prev*0.01;
    pos += vel_prev*0.01;
    vel_prev = vel;
    a_prev = a;
    left_wheel.move_velocity(vel);
    right_wheel.move_velocity(vel);
    left_chain.move_velocity(vel);
    right_chain.move_velocity(vel);

    pros::delay(10);
  }
}
*/
/**
void curvyboi(){//should be task but idk how
  double VMAX = 200;
  double pl;//power
  double pr;//power
  double vl;//read from csv
  double vr;//read from csv
  double kp;//Kp
  double sl;//read from csv
  double sr;//read from csv
  double el;//encoder value
  double er;//encoder value
  double ktheta;//Ktheta
  double dtheta;//heading - gyro reading

  dtheta = heading - actual;//heading read from csv

  pl = vl/VMAX + kp*(sl-el) - ktheta * dtheta;

  pr = vr/VMAX + kp*(sr-er) + ktheta * dtheta;

  left_wheel.movevelocity(VMAX*pl);
  left_chain.movevelocity(VMAX*pl);
  right_wheel.movevelocity(pr*VMAX);
  right_chain.movevelocity(pr*VMAX);

}
**/
//test
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 void position_task(void* param){

 	leftenc.reset();
 	rightenc.reset();
 	backenc.reset();
   while(true){
     trackPos(mainPosition);
 		if ((int)pros::millis() % 50 == 0){
 			printf("Xpos: %f\r\n",mainPosition.x);
 			printf("Ypos: %f\r\n",mainPosition.y);
 			printf("Angle: %f\r\n",mainPosition.angle);
 			printf("l: %f\r\n",leftenc.get());
 			printf("r: %f\r\n",rightenc.get());
 			printf("b: %f\r\n",backenc.get());

 		}
 		pros::delay(10);
   }
 }
void test2(){
  std::string text("wheelTrack");
  pros::Task main_pos(position_task,&text);
  move_straight_rel_test(0, 10.0);
}
void test(){
  right_wheel.move_velocity(200);
  pros::delay(500);
}



void autonomous() {

switch(selectedAuton){
  case 10: test();
    break;
  case 11: test();
    break;
  case 12: test();
    break;
  case 13: test();
    break;
  case 20: test();
    break;
  case 21: test();
    break;
  case 22: test();
    break;
  case 23: test();
    break;
  case 30: test();
    break;
  case 31: test();
    break;
  case 32: test();
    break;
  case 33: test();
    break;
  default: test2();
    break;

}


}
