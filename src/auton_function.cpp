#include "main.h"
#include "config.hpp"
#include "auton_function.h"

const int LIFTGEARRATIO = 7;
const int DEFAULTSLEWRATEINCREMENT = 10;
const float WHEELDIAM = 2.75;
const float L_DIS_IN = 4.72440945;
const float R_DIS_IN = 4.72440945;
const float B_DIS_IN = 4.33070866;
const float TICKS_PER_ROTATION =  360.0;
const float  SPIN_TO_IN_LR = (WHEELDIAM * M_PI / TICKS_PER_ROTATION);
const float  SPIN_TO_IN_S = (WHEELDIAM * M_PI / TICKS_PER_ROTATION);
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

void lift_PID(float targetDegree, int maxVelocity, int delay, int multi)
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
    if(multi == 1){
      intake1.move_velocity(-100);
      intake2.move_velocity(100);
    } else{
      intake1.move_velocity(0);
      intake2.move_velocity(0);
    }
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
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
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

void move_straight_rel_test(double xCoord, int maxVel, int multi){
  leftenc.reset();
  rightenc.reset();

  if(multi == 1){
    intake1.move_velocity(-100);
    intake2.move_velocity(100);
  } else{
    intake1.move_velocity(0);
    intake2.move_velocity(0);
  }
 const double degreeGoal = (xCoord/(1.375*2*M_PI))*TICKS_PER_ROTATION;
 int maxVelocity = maxVel;
 bool isX = false;
 double target = 0.0;
 double goal = 0.0;
 bool goalMet = false; bool oneTime = true;
 int targetVelocity = 0;
 double currentPosition = 0;
 double error = 0;
 double previous_error = goal;
 double kP = 0.3;
 double kI = 0.0005;
 double kD = 0.005;
 double integral = 0;
 double derivative = 0;

 target = degreeGoal;

 if (target < 0) {maxVelocity *= -1;}

 while(!goalMet){

   currentPosition = (leftenc.get()+rightenc.get())/2;
   error = target - currentPosition;

   if (std::abs(error) < 600){
     integral += error;
   }

   derivative = error - previous_error;
   previous_error = error;

   targetVelocity = kP*error + kI*integral + kD*derivative;

   if (abs(targetVelocity) > abs(maxVelocity)){
     targetVelocity = maxVelocity;
   }

   slewRateControl(&left_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT);
   slewRateControl(&left_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT);
   slewRateControl(&right_wheel, targetVelocity, DEFAULTSLEWRATEINCREMENT);
   slewRateControl(&right_chain, targetVelocity, DEFAULTSLEWRATEINCREMENT);

   if (std::abs(error) < 2){
     goalMet = true;
   }

   pros::delay(10);
 }

 brakeMotors();
 intake1.move_velocity(0);
 intake2.move_velocity(0);
}

void turn_PID(float targetDegree){
  leftenc.reset();
  float turn_constant = 2.4;
  int maxVelocity = 70;
  const double degreeGoal = targetDegree*turn_constant;
  bool goalMet = false;
  int targetVelocity = 0;
  int leftTarget = 0;
  int rightTarget = 0;
  double currentPosition = 0;
  double error = 0;
  double previous_error = degreeGoal;
  double kP = 0.75;
  double kI = 0.001;
  double kD = 0.001;
  double integral = 0;
  double derivative = 0;
  if(targetDegree<0){maxVelocity *= -1;}


  while(!goalMet){

      currentPosition = leftenc.get();

    error = degreeGoal - currentPosition;
    printf("%f\r\n",currentPosition);
    if (std::abs(error) < 100){
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

    if (std::abs(error) < 4){
      goalMet = true;
    }

    pros::delay(10);
  }
  brakeMotors();
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
void move_align(float targetDistance, int velocity){
   const double degreeGoal = (targetDistance/(2*2*M_PI))*TICKS_PER_ROTATION;
   left_wheel.tare_position();
   if (targetDistance < 0){
     velocity *= -1;
   }
   left_wheel.move_velocity(velocity);
   left_chain.move_velocity(velocity);
   right_wheel.move_velocity(velocity);
   right_chain.move_velocity(velocity);

  while (std::abs(left_wheel.get_position()) < degreeGoal) {
    pros::delay(5);
  }
}
void deploy_task(void* param){
  lift_PID(-75,90,0,200);

  //tilter_PID(10,100,(double)0.2,0);

  intake1.move_velocity(100);
  intake2.move_velocity(-100);
  pros::delay(200);
  lift.move_velocity(200);
  pros::delay(500);
  lift.move_velocity(0);
  intake1.move_velocity(0);
  intake2.move_velocity(-0);
}
void deploy(){
  std::string text("deploy");
  pros::Task task4(deploy_task,&text);

  tilter_PID(85,100,(double)0.2,0);




  //lift_PID(500,70,0);
  //tilter_PID(-175,200,(double)0.1,0);
}
