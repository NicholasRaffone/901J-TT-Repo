#include "main.h"
#include "config.hpp"

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
			double turn = 600*master.get_analog(ANALOG_RIGHT_X)/127;
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

			pros::delay(2);
		}
}
