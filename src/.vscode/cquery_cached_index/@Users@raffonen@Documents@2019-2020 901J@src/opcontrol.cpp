#include "main.h"
#include "config.hpp"
#include <math.h>
#include <vector>
#include "okapi/api.hpp"
#include "auton_function.h"
const float WHEELDIAM = 2.75;
const float L_DIS_IN = 4.72440945;
const float R_DIS_IN = 4.72440945;
const float B_DIS_IN = 4.33070866;
const float TICKS_PER_ROTATION =  360.0;
const float  SPIN_TO_IN_LR = (WHEELDIAM * M_PI / TICKS_PER_ROTATION);
const float  SPIN_TO_IN_S = (WHEELDIAM * M_PI / TICKS_PER_ROTATION);
//___int_least16_t_definedconst int DEFAULTSLEWRATEINCREMENT = 10;



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

/*using namespace okapi;

TimeUtil chassisUtil = TimeUtilFactory::withSettledUtilParams(50, 5, 250_ms);


okapi::MotorGroup group1 ({Motor(17,true,AbstractMotor::gearset::green),Motor(18,false,AbstractMotor::gearset::green)});
okapi::MotorGroup group2 ({Motor(13,false,AbstractMotor::gearset::green),Motor(15,true,AbstractMotor::gearset::green)});

okapi::ChassisScales scales ({4_in, 14.9606_in});

ThreeEncoderSkidSteerModel myChassis = ChassisModelFactory::create(
  group1,
  group2,
  leftenc,
  rightenc,
  backenc,
  200.0, // 4 inch wheels, 12.5 inch wheelbase width
  12000.0
);

TimeUtil chassisUtil2 = TimeUtilFactory::withSettledUtilParams(50, 5, 100_ms);*/
/*
chassisUtil,
std::shared_ptr<ThreeEncoderSkidSteerModel>(&myChassis),
okapi::IterativePosPIDController::Gains{0.1,0.0001,0.001},
okapi::IterativePosPIDController::Gains{0.1,0.0001,0.001},
okapi::IterativePosPIDController::Gains{0.1,0.0001,0.001},
AbstractMotor::gearset::green,
scales
std::unique_ptr<okapi::IterativePosPIDController>
*/
/*
auto bruh = new IterativePosPIDController(0.01, 0.01, 0.01, 0,
                          chassisUtil);
std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>();



auto profileController = AsyncControllerFactory::motionProfile(
  0.5,  // Maximum linear velocity of the Chassis in m/s
  1.0,  // Maximum linear acceleration of the Chassis in m/s/s
  4.0, // Maximum linear jerk of the Chassis in m/s/s/s
  std::shared_ptr<ThreeEncoderSkidSteerModel>(&myChassis),
  scales,
AbstractMotor::gearset::green,
chassisUtil
);


*/
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






void lift_task(void* param){

  while(true){

  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      lift_PID(-343,90,200,0);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)&&master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    lift_PID(-308,100,200,0);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)&& master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      lift.move_velocity(-200);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)&&master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    lift.move_velocity(200);
  } else {
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift.move_velocity(0);
  }
  pros::delay(8);
}
}

void tilter_task(void* param){
  while (true){
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)&& master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        tilter_PID(135,100,(double)0.5,0);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)&& master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      tilter_PID(135,100,(double)0.5,0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
    tilter.move_velocity(25);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
    tilter.move_velocity(-200);
    } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
    } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            tilter_PID(330,80,(double)0.06,0);
    }
    else{
    tilter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    tilter.move_velocity(0);
    }


    pros::delay(8);

}
}

void opcontrol() {
  //std::string text("wheelTrack");
  //pros::Task punchTask(WheelTrack2,&text);
  //deploy();

  std::string text("tilter");
  std::string texttwo("lift");
  pros::Task task(lift_task,&texttwo);
  pros::Task task2(tilter_task,&text);
  //___int_least8_t_definedturn_PID(90.0);
  //move_test(12.0);
  /**profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{4_ft, 2_ft, 0_deg}}, // The next point in the profile, 3 feet forward
    "A" // Profile name
  );
  profileController.generatePath({
    Point{-5_ft, -2_ft, 90_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{0_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
    "B" // Profile name
  );**/

  //profileController.setTarget("A",false);
  //profileController.waitUntilSettled();
  //profileController.setTarget("B",true);
  //profileController.waitUntilSettled();

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



  //profileController.setTarget("C");
  //profileController.waitUntilSettled();
  //profileController.setTarget("B");

  //profileController.waitUntilSettled();

    //profileController.setTarget("C");

    //profileController.waitUntilSettled();
		while (true) {
			double power = 200*master.get_analog(ANALOG_LEFT_Y)/127;
			double turn = 200*master.get_analog(ANALOG_RIGHT_X)/127;
			int left = (int)(pow(((power + turn)/200.0),2.0)*200.0);
			int right = (int) (pow(((power - turn)/200.0),2.0)*200.0);
			if(power+turn < 0){
        left *=-1;
      }
      if(power - turn < 0){
        right*=-1;
      }
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

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake1.move_velocity(-100);
        intake2.move_velocity(100);
        intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)&& !master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake1.move_velocity(75);
        intake2.move_velocity(-75);
        intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      } else {

        intake1.move_velocity(0);
        intake2.move_velocity(0);
      }





			pros::delay(10);
		}
}
