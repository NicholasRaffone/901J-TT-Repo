 #include "main.h"
#include "config.hpp"
#include <math.h>
#include <vector>
#include "auton_function.h"
#include "okapi/api.hpp"
const float WHEELDIAM = 2.75;
const float L_DIS_IN = 4.72440945;
const float R_DIS_IN = 4.72440945;
const float B_DIS_IN = 4.33070866;
const float TICKS_PER_ROTATION =  360.0;
const float  SPIN_TO_IN_LR = (WHEELDIAM * M_PI / TICKS_PER_ROTATION);
const float  SPIN_TO_IN_S = (WHEELDIAM * M_PI / TICKS_PER_ROTATION);




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

 using namespace okapi;

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

 TimeUtil chassisUtil2 = TimeUtilFactory::withSettledUtilParams(50, 5, 100_ms);
 auto bruh = new IterativePosPIDController(0.01, 0.01, 0.01, 0,
                           chassisUtil);
 std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>();

 auto profileController = AsyncControllerFactory::motionProfile(
   1.0,  // Maximum linear velocity of the Chassis in m/s
   1.2,  // Maximum linear acceleration of the Chassis in m/s/s
   5.0, // Maximum linear jerk of the Chassis in m/s/s/s
   std::shared_ptr<ThreeEncoderSkidSteerModel>(&myChassis),
   scales,
 AbstractMotor::gearset::green,
 chassisUtil
 );
void intake_task(void* param){
  intake1.move_velocity(51);
  intake2.move_velocity(-51);
  pros::delay(620);
  intake1.move_velocity(0);
  intake2.move_velocity(0);
}
void turn_task(void* param){
intake1.move_velocity(50);
intake2.move_velocity(-50);
pros::delay(700);
intake1.move_velocity(0);
intake2.move_velocity(0);
}
void intake_task2(void* param){
  pros::delay(200);
  tilter_PID(403,75,(double)0.07,0);


}
void deploy_task(void* param){
  move_align(-6.5,80);
}

void blue_unproc(){
  //std::string text("wheelTrack");
  //pros::Task main_pos(position_task,&text);
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
  profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{2.3_ft, -2_ft, 66_deg}}, // The next point in the profile, 3 feet forward
    "A" // Profile name
  );
  //turn_PID(90);
  //turn_PID(-90);

  move_align(4,80);

  std::string text("deploy");
  pros::Task task3(deploy_task,&text);
  deploy();
  tilter.move_velocity(-20);
  pros::delay(300);
  move_straight_rel_test(45.5,71, 1);
  tilter.move_velocity(0);
  //intake1.move_velocity(-100);
  //intake2.move_velocity(100);
  move_straight_rel_test(-25.5, 130, 1);
  pros::delay(50);
  std::string textsmth("intake");
  pros::Task task2(turn_task,&textsmth);
  turn_PID(-142.5);
  std::string texttwo("intake");
  pros::Task task(intake_task2,&texttwo);
  move_straight_rel_test(11.5, 100, 0);
  pros::delay(400);
  left_wheel.move_velocity(29);
  left_chain.move_velocity(29);
  right_wheel.move_velocity(29);
  right_chain.move_velocity(29);
  pros::delay(500);
  brakeMotors();
  unBrakeMotors();
  move_straight_rel_test(-20.5, 200, 0);
}

void red_unproc_test(){
  profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{2.3_ft, -2_ft, 66_deg}}, // The next point in the profile, 3 feet forward
    "A" // Profile name
  );
  //turn_PID(90);
  //turn_PID(-90);

  move_align(4,80);

  std::string text("deploy");
  pros::Task task3(deploy_task,&text);
  deploy();
  tilter.move_velocity(-20);
  pros::delay(300);
  move_straight_rel_test(45.5,71, 1);
  tilter.move_velocity(0);
  //intake1.move_velocity(-100);
  //intake2.move_velocity(100);
  move_straight_rel_test(-25.5, 130, 1);
  pros::delay(50);
  std::string textsmth("intake");
  pros::Task task2(turn_task,&textsmth);
  turn_PID(142.5);
  std::string texttwo("intake");
  pros::Task task(intake_task2,&texttwo);
  move_straight_rel_test(11.5, 100, 0);
  pros::delay(400);
  left_wheel.move_velocity(29);
  left_chain.move_velocity(29);
  right_wheel.move_velocity(29);
  right_chain.move_velocity(29);
  pros::delay(500);
  brakeMotors();
  unBrakeMotors();
  move_straight_rel_test(-20.5, 200, 0);
}
void test(){
  right_wheel.move_velocity(200);
  pros::delay(500);
}
void skills_auton(){


  profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{2.3_ft, -2_ft, 66_deg}}, // The next point in the profile, 3 feet forward
    "A" // Profile name
  );
  //turn_PID(90);
  //turn_PID(-90);

  move_align(4,80);

  std::string text("deploy");
  pros::Task task3(deploy_task,&text);
  deploy();
  tilter.move_velocity(-30);
  pros::delay(300);
  move_straight_rel_test(44, 50, 1);
  tilter.move_velocity(0);
  intake1.move_velocity(-100);
  intake2.move_velocity(100);
  profileController.setTarget("A",true);
  profileController.waitUntilSettled();

  intake1.move_velocity(0);
  intake2.move_velocity(0);
  std::string texttwo("intake");
  pros::Task task(intake_task,&texttwo);
  move_straight_rel_test(28.2, 150, 0);

  intake1.move_velocity(0);
  intake2.move_velocity(0);
  lift.move_velocity(0);
    tilter_PID(370,50,(double)0.15,0);
    move_align(1.5,30);


    brakeMotors();

    unBrakeMotors();

    move_straight_rel_test(-10, 150, 0);

}
void red_unproc(){
  profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{2_ft, 2_ft, -65_deg}}, // The next point in the profile, 3 feet forward
    "A" // Profile name
  );
  //turn_PID(90);
  //turn_PID(-90);

  move_align(4,80);

  std::string text("deploy");
  pros::Task task4(deploy_task,&text);
  deploy();
  tilter.move_velocity(-30);
  move_straight_rel_test(44, 80, 1);
  tilter.move_velocity(0);
  intake1.move_velocity(-100);
  intake2.move_velocity(100);
  profileController.setTarget("A",true);
  profileController.waitUntilSettled();

  intake1.move_velocity(0);
  intake2.move_velocity(0);
  std::string texttwo("intake");
  pros::Task task(intake_task,&texttwo);
  move_straight_rel_test(28.3, 170, 0);

  lift.move_velocity(-40);
  pros::delay(150);
  lift.move_velocity(0);
  intake1.move_velocity(0);
  intake2.move_velocity(0);

    tilter_PID(365,50,(double)0.15,0);


    move_align(.5,30);
    move_straight_rel_test(-10, 170, 0);
}

void blue_proc(){
  profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{0.8_ft, 2.5_ft, -90_deg}}, // The next point in the profile, 3 feet forward
    "A" // Profile name
  );
  //turn_PID(90);
  //turn_PID(-90);

  move_align(4,80);

  std::string text("deploy");
  pros::Task task(deploy_task,&text);
  deploy();


  unBrakeMotors();
  tilter.move_velocity(-30);
  move_straight_rel_test(36, 80, 1);
  tilter.move_velocity(0);
  intake1.move_velocity(-100);
  intake2.move_velocity(100);
  //profileController.setTarget("A",true);
  //profileController.waitUntilSettled();
  //move_straight_rel_test(-30, 180, 1);
  pros::delay(200);
  move_straight_rel_test(-28, 150, 1);
  pros::delay(200);
  intake1.move_velocity(0);
  intake2.move_velocity(0);
  turn_PID(89);
  std::string texttwo("intake");
  pros::Task task2(intake_task,&texttwo);
  move_straight_rel_test(22.5, 150, 1);
  /*intake1.move_velocity(51);
  intake2.move_velocity(-51);
  pros::delay(500);
  intake1.move_velocity(0);
  intake2.move_velocity(0);*/

    tilter_PID(365,50,(double)0.15,0);


    //move_align(1.5,-50);

    move_align(1,30);


    move_straight_rel_test(-10, 150, 0);
}
void red_proc(){
  profileController.generatePath({
    Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    Point{0.8_ft, 2.5_ft, -90_deg}}, // The next point in the profile, 3 feet forward
    "A" // Profile name
  );
  //turn_PID(90);
  //turn_PID(-90);

  move_align(4,80);

  std::string text("deploy");
  pros::Task task4(deploy_task,&text);
  deploy();


  unBrakeMotors();
  tilter.move_velocity(-30);
  move_straight_rel_test(36, 80, 1);
  tilter.move_velocity(0);
  intake1.move_velocity(-100);
  intake2.move_velocity(100);
  //profileController.setTarget("A",true);
  //profileController.waitUntilSettled();
  move_straight_rel_test(-29.7, 170, 1);
  pros::delay(200);
  turn_PID(-96);
  pros::delay(200);
  intake1.move_velocity(0);
  intake2.move_velocity(0);
  std::string texttwo("intake");
  pros::Task task2(intake_task,&texttwo);
  move_straight_rel_test(22.5, 150, 0);

  intake1.move_velocity(0);
  intake2.move_velocity(0);

    tilter_PID(360,50,(double)0.15,0);

    move_align(1,30);

    move_straight_rel_test(-10, 100, 0);
}


void blue_proc_test(){
  move_align(4,80);

  std::string text("deploy");
  pros::Task task(deploy_task,&text);
  deploy();


  unBrakeMotors();
  tilter.move_velocity(-30);
  move_straight_rel_test(36, 80, 1);
  tilter.move_velocity(0);
  intake1.move_velocity(-100);
  intake2.move_velocity(100);
  //profileController.setTarget("A",true);
  //profileController.waitUntilSettled();
  //move_straight_rel_test(-30, 180, 1);
  pros::delay(200);
  turn_PID(121);
  //std::string textsmth("intake");
  //pros::Task task2(turn_task,&textsmth);
  pros::delay(200);
  move_straight_rel_test(27, 80, 1);
  pros::delay(200);
  std::string textsmth("intake");
  pros::Task task2(turn_task,&textsmth);
  turn_PID(10);
  pros::delay(200);
  std::string texttwo("intake");
  pros::Task taskthree(intake_task2,&texttwo);
  move_straight_rel_test(8, 100, 0);
  pros::delay(400);
  left_wheel.move_velocity(29);
  left_chain.move_velocity(29);
  right_wheel.move_velocity(29);
  right_chain.move_velocity(29);
  pros::delay(400);
  brakeMotors();
  unBrakeMotors();
  move_straight_rel_test(-10.5, 200, 0);

}

void red_proc_test(){
  move_align(4,80);

  std::string text("deploy");
  pros::Task task(deploy_task,&text);
  deploy();


  unBrakeMotors();
  tilter.move_velocity(-30);
  move_straight_rel_test(36, 80, 1);
  tilter.move_velocity(0);
  intake1.move_velocity(-100);
  intake2.move_velocity(100);
  //profileController.setTarget("A",true);
  //profileController.waitUntilSettled();
  //move_straight_rel_test(-30, 180, 1);
  pros::delay(200);
  turn_PID(-124);
  //std::string textsmth("intake");
  //pros::Task task2(turn_task,&textsmth);
  pros::delay(200);
  move_straight_rel_test(27, 80, 1);
  pros::delay(200);
  std::string textsmth("intake");
  pros::Task task2(turn_task,&textsmth);
  turn_PID(-10);
  pros::delay(200);
  std::string texttwo("intake");
  pros::Task taskthree(intake_task2,&texttwo);
  move_straight_rel_test(8, 100, 0);
  pros::delay(400);
  left_wheel.move_velocity(29);
  left_chain.move_velocity(29);
  right_wheel.move_velocity(29);
  right_chain.move_velocity(29);
  pros::delay(400);
  brakeMotors();
  unBrakeMotors();
  move_straight_rel_test(-10.5, 200, 0);

}

void autonomous() {

switch(selectedAuton){
  case 10: red_unproc_test();
    break;
  case 11: red_proc_test();
    break;
  case 12: test();
    break;
  case 13: red_proc_test();
    break;
  case 20: blue_unproc();
    break;
  case 21: blue_proc_test();
    break;
  case 22: test();
    break;
  case 23: blue_proc();
    break;
  case 30: skills_auton();
    break;
  case 31: test();
    break;
  case 32: test();
    break;
  case 33: test();
    break;
  default: red_unproc_test();
    break;

}


}
