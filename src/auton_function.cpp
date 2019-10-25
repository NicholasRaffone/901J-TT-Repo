#include "main.h"
#include "config.hpp"
#include "auton_function.h"

const int LIFTGEARRATIO = 7;
const int DEFAULTSLEWRATEINCREMENT = 10;
/*
void liftpid(int targetDegree, int maxvel){
  left_lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  const double degreeGoal = (targetDegree*LIFTGEARRATIO);

  bool goalMet = false;
  int targetVelocity = 0;ns
  double currentPosition = 0;
  double followPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.8;
  double kI = 0.0025;
  double kD = 0.001;
  double diff = 0;
  double integral = 0;
  double derivative = 0;
  if (targetDegree < 0) {maxvel *= -1;}
  right_lift.tare_position();
  left_lift.tare_position();


  while(!goalMet){
    currentPosition = left_lift.get_positon();
    followPosition = right_lift.get_position();
    diff = currentPosition - followPosition;
    error = degreeGoal - currentPosition;

    if (std::abs(error) < 100){
      integral += error;
    }

    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (targetVelocity > maxVelocity){
      targetVelocity = maxVelocity;
    }

    left_lift.move_velocity(targetVelocity);
    right_lift.move_velocity(targetVelocity + diff);

    if (std::abs(error) < 4){
      goalMet = true;
  }

  pros::delay(10);
  }
  right_lift.move_velocity(0);
  left_lift.move_velocity(0);
}
*/
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

void lift_PID(float targetDegree, int maxVelocity, int delay)
{
  const double degreeGoal = (targetDegree*7);
  bool goalMet = false;
  bool limitStart = false;
  int targetVelocity = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.15;
  double kI = 0.0025;
  double kD = 0.003;
  double integral = 0;
  double derivative = 0;


  if (targetDegree < 0) {maxVelocity *= -1;}

  lift.tare_position();
  pros::delay(delay);

  while(!goalMet){
    currentPosition = lift.get_position();
    error = degreeGoal - currentPosition;

    if (std::abs(error) < 100){
      integral += error;
    }
    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (targetVelocity > maxVelocity){
      targetVelocity = maxVelocity;
    }

    slewRateControl(&lift, targetVelocity, DEFAULTSLEWRATEINCREMENT);

    if (std::abs(error) < 12){
      goalMet = true;
    }

    pros::delay(10);
  }
}

void tilter_PID(float targetDegree, int maxVelocity, double kp,int delay){
  intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  tilter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  const double degreeGoal = (targetDegree*7);
  bool goalMet = false;
  bool limitStart = false;
  int targetVelocity = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = kp;
  double kI = 0.003;
  double kD = 0.005;
  double integral = 0;
  double derivative = 0;

  deg = 0;

  if (targetDegree < 0) {maxVelocity *= -1;}

  tilter.tare_position();

pros::delay(delay);
  while(!goalMet){
    currentPosition = tilter.get_position();
    error = degreeGoal - currentPosition;

    if (std::abs(error) < 100){
      integral += error;
    }
      if (std::abs(error) < 1100){
    intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }
    derivative = error - previous_error;
    previous_error = error;

    targetVelocity = kP*error + kI*integral + kD*derivative;

    if (targetVelocity > maxVelocity){
      targetVelocity = maxVelocity;
    }

    slewRateControl(&tilter, targetVelocity, DEFAULTSLEWRATEINCREMENT);

    if (std::abs(error) < 4){
      goalMet = true;
    }
    deg = tilter.get_position();

    pros::delay(10);
  }
  tilter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  tilter.move_velocity(0);
  intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

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

void deploy(){
  tilter_PID(75,100,(double)0.2,0);
  printf("bruh");
  lift_PID(-280,80,0);
  tilter_PID(30,100,(double)0.2,0);
  pros::delay(400);
  lift_PID(200,80,0);

  //lift_PID(500,70,0);
  //tilter_PID(-175,200,(double)0.1,0);
}
