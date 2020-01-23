#include "main.h"
#include "config.hpp"
#include <math.h>
#include <vector>
#include "auton_function.h"
#include "okapi/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
	 int i = 0;
	 float l = leftenc.get_value();
	 float r = leftenc.get_value();
	 while(true){
		 if(i%10000==0){
			 printf("Left: %f\n", (l-leftenc.get_value()));
			 printf("Right: %f\n", (r-rightenc.get_value()));
		 }
		 i++;
	 }
 }
void opcontrol() {
	std::string texttwo("enc");
	pros::Task task(enconder_task,&texttwo,"");
	using namespace okapi;
	okapi::MotorGroup group1 ({Motor(1,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees),Motor(11,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees)});
 okapi::MotorGroup group2 ({Motor(10,true,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees),Motor(20,true,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::degrees)});


	auto chassis = ChassisControllerBuilder().withMotors(group1, group2) // left motor is 1, right motor is 2 (reversed)
	    // green gearset, 4 inch wheel diameter, 11.5 inch wheelbase
	    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 15_in}, imev5GreenTPR})
			.withMaxVelocity(80.0)
	    // left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
	    .withSensors(ADIEncoder{'D', 'C'}, ADIEncoder{'G', 'H'})
	    // specify the tracking wheels diameter (3 in), track (7 in), and TPR (360)
	    .withOdometry({{2.75_in, 7.204724_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
	    .buildOdometry();

	// set the state to zero
	chassis->setState({0_in, 0_in, 0_deg});
	// turn 45 degrees and drive approximately 1.4 ft
	//chassis->moveDistance(12_in);
	//chassis->turnToAngle(90_deg);
	chassis->driveToPoint({4_in, 4_in});
	pros::delay(1000);
	//chassis->driveToPoint({4_in, 8_in});
	//chassis->driveToPoint({0_in, 0_in});
	// turn approximately 45 degrees to end up at 90 degrees
	chassis->turnToAngle(90_deg);
	// turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot
	//chassis->turnToPoint({5_ft, 0_ft});
}
