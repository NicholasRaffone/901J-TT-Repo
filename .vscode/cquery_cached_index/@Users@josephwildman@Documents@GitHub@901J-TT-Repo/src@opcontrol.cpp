#include "main.h"
#include "config.hpp"
#include <math.h>
#include <vector>
#include "okapi/api.hpp"

/*TimeUtil profiledUtil = TimeUtilFactory::withSettledUtilParams(50, 5, 100_ms);

AsyncPosIntegratedController leftController(std::shared_ptr<Motor>(&driveL1), chassisUtil);
AsyncPosIntegratedController rightController(std::shared_ptr<Motor>(&driveR1), chassisUtil);

SkidSteerModel chassisModel = ChassisModelFactory::create({DRIVE_PORT_L1, DRIVE_PORT_L2}, {-DRIVE_PORT_R1, -DRIVE_PORT_R2}, 200);

ChassisControllerIntegrated chassisController(
    chassisUtil,
    std::shared_ptr<SkidSteerModel>(&chassisModel),
    std::unique_ptr<AsyncPosIntegratedController>(&leftController),
    std::unique_ptr<AsyncPosIntegratedController>(&rightController),
    AbstractMotor::gearset::green, {4.125_in, 13.078_in});*/

using namespace okapi;

TimeUtil chassisUtil = TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms);


okapi::MotorGroup group1 ({Motor(17,true,AbstractMotor::gearset::green),Motor(18,false,AbstractMotor::gearset::green)});
okapi::MotorGroup group2 ({Motor(13,false,AbstractMotor::gearset::green),Motor(15,true,AbstractMotor::gearset::green)});

okapi::ADIEncoder leftenc ('A','B');
okapi::ADIEncoder rightenc ('C','D');
okapi::ADIEncoder backenc ('E','F');

  okapi::ChassisScales scales ({4_in, 16.4_in});

ThreeEncoderSkidSteerModel myChassis = ChassisModelFactory::create(
  group1,
  //left_wheel, // Left motors
  //okapi::MotorGroup right (1, 2),
    //{RIGHT_WHEEL_PORT, RIGHT_CHAIN_PORT},// Right motors
  group2,
  leftenc,
  rightenc,
  backenc,
  200.0, // 4 inch wheels, 12.5 inch wheelbase width
  12000.0
);

auto profileController = AsyncControllerFactory::motionProfile(
  0.75,  // Maximum linear velocity of the Chassis in m/s
  1.5,  // Maximum linear acceleration of the Chassis in m/s/s
  7.5, // Maximum linear jerk of the Chassis in m/s/s/s
  std::shared_ptr<ThreeEncoderSkidSteerModel>(&myChassis),
  scales,
AbstractMotor::gearset::green,
chassisUtil
);


std::vector<float> d (3);

void WheelTrack2 (void* param){

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



  std::vector<float> dtemp (3);
  std::vector <float> d1 (3);

  float rad;
  float angle;
  float currentl;
  float currentr;
  float currentb;


  while (true){
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
      pros::delay(5);
    }
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
  //std::string text("wheelTrack");
  //pros::Task punchTask(WheelTrack2,&text);

  profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{5_ft, 2_ft, -90_deg}}, // The next point in the profile, 3 feet forward
    "A" // Profile name
  );
  profileController.generatePath({
    Point{-5_ft, -2_ft, 90_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{0_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
    "B" // Profile name
  );

  profileController.setTarget("A");
  profileController.waitUntilSettled();

  /*profileController.generatePath({
    Point{5_ft, 2_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{6_ft, 1_ft, 45_deg}}, // The next point in the profile, 3 feet forward
    "B" // Profile name
  );*/
  /*profileController.generatePath({
    Point{6_ft, 1_ft, 45_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{7_ft, 0_ft, 90_deg}}, // The next point in the profile, 3 feet forward
    "C" // Profile name
  );*/
  profileController.setTarget("B",true);
  profileController.waitUntilSettled();


  //profileController.setTarget("C");
  //profileController.waitUntilSettled();
  //profileController.setTarget("B");

  //profileController.waitUntilSettled();

    //profileController.setTarget("C");

    //profileController.waitUntilSettled();
		while (true) {
			double power = 500*master.get_analog(ANALOG_LEFT_Y)/127;
			double turn = 500*master.get_analog(ANALOG_RIGHT_X)/127;
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

      printf("THETA: %f",d[2]);

			pros::delay(10);
		}
}
