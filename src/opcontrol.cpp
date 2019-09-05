#include "main.h"
#include "config.hpp"
#include <math.h>
#include <vector>
#include "okapi/api.hpp"
const float WHEELDIAM = 2.75;
const float L_DIS_IN = 4.72440945;
const float R_DIS_IN = 4.72440945;
const float B_DIS_IN = 4.33070866;
const float TICKS_PER_ROTATION =  360.0;
const float  SPIN_TO_IN_LR = (WHEELDIAM * M_PI / TICKS_PER_ROTATION);
const float  SPIN_TO_IN_S = (WHEELDIAM * M_PI / TICKS_PER_ROTATION);
const int DEFAULTSLEWRATEINCREMENT = 10;




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



  okapi::ChassisScales scales ({4_in, 16.4_in});

/* ThreeEncoderSkidSteerModel myChassis = ChassisModelFactory::create(
  group1,
  group2,
  leftenc,
  rightenc,
  backenc,
  200.0, // 4 inch wheels, 12.5 inch wheelbase width
  12000.0
);

TimeUtil chassisUtil2 = TimeUtilFactory::withSettledUtilParams(50, 5, 100_ms);

chassisUtil,
std::shared_ptr<ThreeEncoderSkidSteerModel>(&myChassis),
okapi::IterativePosPIDController::Gains{0.1,0.0001,0.001},
okapi::IterativePosPIDController::Gains{0.1,0.0001,0.001},
okapi::IterativePosPIDController::Gains{0.1,0.0001,0.001},
AbstractMotor::gearset::green,
scales
std::unique_ptr<okapi::IterativePosPIDController>

auto bruh = new IterativePosPIDController(0.01, 0.01, 0.01, 0,
                          chassisUtil);
std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>();



auto profileController = AsyncControllerFactory::motionProfile(
  0.75,  // Maximum linear velocity of the Chassis in m/s
  1.5,  // Maximum linear acceleration of the Chassis in m/s/s
  7.5, // Maximum linear jerk of the Chassis in m/s/s/s
  std::shared_ptr<ThreeEncoderSkidSteerModel>(&myChassis),
  scales,
AbstractMotor::gearset::green,
chassisUtil
); */


void trackPos(rPos& position) //Based off of 5225a E-bots Pilons APS code, https://github.com/nickmertin/5225A-2017-2018/blob/master/src/auto.c
{
  int currentL = leftenc.get();
  int currentR = rightenc.get();
  int currentB = backenc.get();


  float deltaL = (currentL - position.leftLast) * SPIN_TO_IN_LR;
  float deltaR = (currentR - position.rightLast) * SPIN_TO_IN_LR;
  float deltaB = (currentB - position.backLast) * SPIN_TO_IN_S;

  position.leftLast = currentL;
  position.rightLast = currentR;
  position.backLast = currentB;

  float h; // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
	float i; // Half on the angle that I've traveled
	float h2; // The same as h but using the back instead of the side wheels

	float angle = (deltaL - deltaR) / (L_DIS_IN + R_DIS_IN); // The angle that I've traveled
  if (angle)
	{
		float r = deltaR / angle; // The radius of the circle the robot travel's around with the right side of the robot
		i = angle / 2.0;
		float sinI = sin(i);
		h = ((r + R_DIS_IN) * sinI) * 2.0;

		float r2 = deltaB / angle; // The radius of the circle the robot travel's around with the back of the robot
		h2 = ((r2 + B_DIS_IN) * sinI) * 2.0;
	}
	else
	{
		h = deltaR;
		i = 0;

		h2 = deltaB;
	}
  float p = i + position.angle; // The global ending angle of the robot
	float cosP = cos(p);
	float sinP = sin(p);

	// Update the global position
	position.y += h * cosP;
	position.x += h * sinP;

	position.y += h2 * -sinP; // -sin(x) = sin(-x)
	position.x += h2 * cosP; // cos(x) = cos(-x)

	position.angle += angle;
}

void position_task(void* param){

	//leftenc.reset();
	//rightenc.reset();
	//backenc.reset();
  while(true){
    trackPos(mainPosition);
		//if ((int)pros::millis() % 50 == 0){
			/*printf("Xpos: %f\r\n",mainPosition.x);
			printf("Ypos: %f\r\n",mainPosition.y);
			printf("Angle: %f\r\n",mainPosition.angle);
			printf("l: %f\r\n",leftenc.get());
			printf("r: %f\r\n",rightenc.get());
			printf("b: %f\r\n",backenc.get());*/

		//}
		pros::delay(10);
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
 void brakeMotors(){//brake the base motors
   left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
   left_wheel.move_velocity(0);
   left_chain.move_velocity(0);
   right_wheel.move_velocity(0);
   right_chain.move_velocity(0);
 }
 void unBrakeMotors(){
   left_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   right_wheel.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   left_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   right_chain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
 }

 void slewRateControl(pros::Motor *motor, int targetVelocity, int increment){
   int currentVelocity = motor->get_target_velocity();
   if (targetVelocity != 0){
     if (currentVelocity != targetVelocity){
       if (targetVelocity > currentVelocity){
         currentVelocity += increment;
       } else if (targetVelocity < currentVelocity){
         currentVelocity -= increment;
       }
       if (std::abs(currentVelocity) > std::abs(targetVelocity)){
         currentVelocity = targetVelocity;
       }
     }
   } else {
     currentVelocity = targetVelocity;
   }
   motor->move_velocity(currentVelocity);
 }

 void move_test(double yCoord){

   int maxVelocity = 100;
  const double goal = yCoord-mainPosition.y;
  bool goalMet = false; bool oneTime = true;
  int targetVelocity = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = goal;
  double kP = 4;
  double kI = 0.0004;
  double kD = 0.01;
  double integral = 0;
  double derivative = 0;

  if (yCoord < 0) {maxVelocity *= -1;}




  while(!goalMet){


    currentPosition = mainPosition.y;
    error = goal - currentPosition;

    if (std::abs(error) < 600){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (targetVelocity > maxVelocity){
      targetVelocity = maxVelocity;
    }

    slewRateControl(&left_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&left_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT);

    if (std::abs(error) < 0.1){
      goalMet = true;
    }




    pros::delay(10);
  }

  brakeMotors();
}
void turn_PID(float targetDegree){
  int maxVelocity = 20;
  const double degreeGoal = targetDegree;
  bool goalMet = false;
  int targetVelocity = 0;
  int leftTarget = 0;
  int rightTarget = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 1;
  double kI = 0.01;
  double kD = 0.00;
  double integral = 0;
  double derivative = 0;
  if(targetDegree<0){maxVelocity *= -1;}



  while(!goalMet){
    currentPosition = mainPosition.angle*180/M_PI;
    error = degreeGoal - currentPosition;
    printf("%f\r\n",currentPosition);
    if (std::abs(error) < 1000){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (std::abs(targetVelocity) > std::abs(maxVelocity)){
      targetVelocity = maxVelocity;
    }


      leftTarget = targetVelocity;
      rightTarget = -1*targetVelocity;


    slewRateControl(&left_wheel, leftTarget, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&left_chain, leftTarget, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_wheel, rightTarget, DEFAULTSLEWRATEINCREMENT);
    slewRateControl(&right_chain, rightTarget, DEFAULTSLEWRATEINCREMENT);

    if (std::abs(error) < 6){
      goalMet = true;
    }

    pros::delay(10);
  }
  brakeMotors();
}
void opcontrol() {
  //std::string text("wheelTrack");
  //pros::Task punchTask(WheelTrack2,&text);
  std::string text("position");
  pros::Task punchTask(position_task,&text);
  //___int_least8_t_definedturn_PID(90.0);
  //move_test(12.0);
  /*profileController.generatePath({
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
  profileController.setTarget("B",true);
  profileController.waitUntilSettled();
*/
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
			//int left = (int)(pow(((power + turn)/600.0),2.0)*600.0);
			//int right = (int) (pow(((power - turn)/600.0),2.0)*600.0);
			int left = power+turn;
			int right = power-turn;
			left_wheel.move_velocity(left);
			left_chain.move_velocity(left);
			right_wheel.move_velocity(right);
			right_chain.move_velocity(right);
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      //test
      right_lift.move_velocity(-70);
      left_lift.move_velocity(-70);

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      right_lift.move_velocity(70);
      left_lift.move_velocity(70);
    } else {
      left_lift.move_velocity(0);
      right_lift.move_velocity(0);
    }

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

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intake.move_velocity(200);
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        intake.move_velocity(-200);

      } else {
        intake.move_velocity(0);
      }


			pros::delay(10);
		}
}
