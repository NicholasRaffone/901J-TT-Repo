#include "main.h"
#include "config.hpp"
#include <math.h>
#include <vector>
#include "auton_function.h"
#include "okapi/api.hpp"


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {

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
 void enconder_task(void* param){
	 okapi::ADIEncoder leftencoder ({'D', 'C'});
	 	okapi::ADIEncoder rightencoder ({'F', 'E',true});
	 int i = 0;
	 float l = leftencoder.get();
	 float r = rightencoder.get();
	 while(true){
		 if(i%10000==0){
			 //printf("Left: %f\n", (leftencoder.get()-l));
			 //printf("Right: %f\n", (rightencoder.get()-r));
		 }
		 i++;
	 }
 }
void opcontrol() {
	std::string texttwo("enc");
	pros::Task task(enconder_task,&texttwo,"");
	using namespace okapi;
	//okapi::AbstractTimer timer({100_ms});
	//okapi::TimeUtil chassisUtil(std::unique_ptr<AbstractTimer>({100_ms}));

  okapi::MotorGroup group1 ({Motor(1,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees),Motor(11,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees)});
  okapi::MotorGroup group2 ({Motor(10,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees),Motor(20,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees)});

  okapi::ADIEncoder leftencoder ({'D', 'C'});
	okapi::ADIEncoder rightencoder ({'F', 'E',true});

  auto leftautoenc = std::make_shared<ADIEncoder>('D', 'C');
  auto rightautoenc = std::make_shared<ADIEncoder>('F', 'E',true);

  okapi::Motor moto1 (1,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees);
  okapi::Motor moto2(11,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees);
  okapi::Motor moto3(10,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees);
  okapi::Motor moto4(20,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees);

  auto leftmoment = {moto1,moto2};
  auto rightmoment = {moto3,moto4};

  auto groupleft = std::make_shared<MotorGroup>(leftmoment);
  auto groupright = std::make_shared<MotorGroup>(rightmoment);

  //PathfinderPoint a = {12_in,12_in,90_deg};

  //auto pointA PathfinderPoint(a);

  //auto groupleft = std::make_shared<MotorGroup>({testm3,testm4});

	/**auto chassis = ChassisControllerBuilder().withMotors(group1, group2) // left motor is 1, right motor is 2 (reversed)
	    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
	    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 13.85_in}, imev5GreenTPR})
			.withMaxVelocity(150.0)
	    // left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
	    .withSensors(leftencoder, rightencoder)
	    // specify the tracking wheels diameter (3 in), track (7 in), and TPR (360)
	    .withOdometry({{2.75_in, 14.4_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
			//.withOdometry()
	    .buildOdometry();

			**/



auto bruh = std::make_shared<SkidSteerModel>(
		groupleft,
		groupright,
		leftautoenc,
		rightautoenc,
		127.0,
		200.0);

		//max v, a, j- logger pro
	okapi::PathfinderLimits limits({0.9,5.8,7.5});

	okapi::ChassisScales scales({3.25_in, 13.85_in},imev5GreenTPR);

	okapi::AbstractMotor::GearsetRatioPair pair(AbstractMotor::gearset::green,1);

	auto profileController = AsyncMotionProfileControllerBuilder()
	.withOutput(bruh,scales,pair)
	.withLimits(limits)
	.buildMotionProfileController()
	;

  profileController->moveTo(    {
        {0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
        {1_ft, 1_ft, 0_deg}
      });





	// set the state to zero
	//chassis->setState({0_in, 0_in, 0_deg});
	//chassis->turnToAngle(90_deg);
	/**while (true) {
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
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
    tilter.move_velocity(25);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
    tilter.move_velocity(-25);
  } else{
    tilter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    tilter.move_velocity(0);
  }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      intake1.move_velocity(200);
      intake2.move_velocity(-200);
      intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)&& !master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      intake1.move_velocity(-200);
      intake2.move_velocity(200);
      intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    } else {

      intake1.move_velocity(0);
      intake2.move_velocity(0);
    }
  }**/
	// turn 45 degrees and drive approximately 1.4 ft
	//chassis->moveDistance(12_in);
	//chassis->turnAngle(90_deg);
	//chassis->turnAngle(-45_deg);

	//chassis->driveToPoint({4_in, 4_in});
	//chassis->driveToPoint({4_in, 8_in});
	//chassis->driveToPoint({0_in, 0_in});
	// turn approximately 45 degrees to end up at 90 degrees
	//chassis->turnToAngle(90_deg);
	// turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot
	//chassis->turnToPoint({0_ft, 5_ft});

}
