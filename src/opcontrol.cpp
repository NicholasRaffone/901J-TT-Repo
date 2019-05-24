#include "main.h"
#include "config.hpp"
#include <math.h>
#include <vector>
#include "okapi/api.hpp"

using namespace okapi;

ThreeEncoderSkidSteerModel myChassis = ChassisModelFactory::create(
  okapi::MotorGroup group3 ({1,2}),
  //left_wheel, // Left motors
  //okapi::MotorGroup right (1, 2),
    //{RIGHT_WHEEL_PORT, RIGHT_CHAIN_PORT},// Right motors
    right_wheel,
  LeftEncoder,
  RightEncoder,
  BackEncoder,
  200.0, // 4 inch wheels, 12.5 inch wheelbase width
  2000.0
);
/**
ThreeEncoderSkidSteerModel profileController = AsyncControllerFactory::motionProfile(
  1.0,  // Maximum linear velocity of the Chassis in m/s
  2.0,  // Maximum linear acceleration of the Chassis in m/s/s
  10.0, // Maximum linear jerk of the Chassis in m/s/s/s
  myChassis // Chassis Controller
);**/

  float wheelrad = 2.75;

  float lencval = 0;
  float rencval = 0;
  float bencval = 0;

  float prevl = 0;
  float prevr = 0;
  float prevb = 0;
  float thetar = 90;//constant
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

    //printf("Error: %f \n", theta1);
    //printf("dtheta: %f \n", dtheta);

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
    //if(dtheta == 0.0){//if only vertical movement add vertical component dr

    //  dtemp.at(0) = ds;
  //    dtemp.at(1) = dr;
    //}
      dtemp.at(0) = 2 * sin(dtheta/2) * (ds/dtheta + bdis);
      dtemp.at(1) = 2 * sin(dtheta/2) * (dr/dtheta + rdis);


    //calculate average orientation
    thetam = (theta0 + dtheta)/2;

    //convert position to polar
    angle = atan(dtemp[1] / dtemp[0]);
    rad = sqrt(dtemp[0] * dtemp[0] + dtemp[1] * dtemp[1]);

    angle += -1 * thetam;

    //convert back
    dtemp[0] = rad * cos(angle);
    dtemp[1] = rad * sin(angle);
//}
    //add position vector to old one
    for(int i = 0; i < 2; i++){
      d1[i] = d[i] + dtemp[i];
    }

      theta0 = theta1;

      d[0] = d1[0];
      d[1] = d1[1];
      d1[2] = thetam;
      //return new position


      return d1;
    }



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {


		while (true) {
			double power = 600*master.get_analog(ANALOG_LEFT_Y)/127;
			double turn = 600*master.get_analog(ANALOG_RIGHT_X)/137;
			//int left = (int)(pow(((power + turn)/600.0),2.0)*600.0);
			//int right = (int) (pow(((power - turn)/600.0),2.0)*600.0);
			int left = power+turn;
			int right = power-turn;
			left_wheel.move_velocity(left);
			left_chain.move_velocity(left);
			right_wheel.move_velocity(right);
			right_chain.move_velocity(right);


			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) != 0){
				left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				left_wheel.move_velocity(0);
				left_chain.move_velocity(0);
				right_wheel.move_velocity(0);
				right_chain.move_velocity(0);
			}else{
				left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			}

			  RobotPosition = track();
			  printf("X: %f\r\n",RobotPosition[0]);
			  printf("Y: %f\r\n",RobotPosition[1]);
			  printf("THETA: %f\r\n",RobotPosition[2]);
			  printf("LEFT ENCODER %d\r\n",LeftEncoder.get_value());
			  printf("RIGHT ENCODER %d\r\n",RightEncoder.get_value());
			  printf("Back ENCODER %d\r\n",BackEncoder.get_value());

			pros::delay(10);
		}
}
