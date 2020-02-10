#include "config.hpp"
#include "pos_struct.h"
#include "okapi/api.hpp"
using namespace okapi;


const int LEFT_WHEEL_PORT = 19; //17
const int LEFT_CHAIN_PORT = 20; //18
const int RIGHT_CHAIN_PORT = 12; //13
const int RIGHT_WHEEL_PORT = 11; //15
const int LIFT_PORT = 16;
const int INTAKE1 = 13;
const int INTAKE2 = 14;
const int TILTERPORT = 10;

const char LENC_TOP_PORT = 'D';
const char LENC_BOT_PORT = 'C';
const char RENC_TOP_PORT = 'F';
const char RENC_BOT_PORT = 'E';
const char BENC_TOP_PORT = 'P';
const char BENC_BOT_PORT = 'O';

pros::Motor left_wheel (LEFT_WHEEL_PORT, pros::E_MOTOR_GEARSET_18,true);//17
pros::Motor right_wheel (RIGHT_WHEEL_PORT, pros::E_MOTOR_GEARSET_18, false);//15
pros::Motor left_chain (LEFT_CHAIN_PORT, pros::E_MOTOR_GEARSET_18, true);//18
pros::Motor right_chain (RIGHT_CHAIN_PORT, pros::E_MOTOR_GEARSET_18, false);//13
pros::Motor lift (LIFT_PORT,pros::E_MOTOR_GEARSET_36, false);
pros::Controller master (CONTROLLER_MASTER);
pros::Motor intake1 (INTAKE1, pros::E_MOTOR_GEARSET_18, true);//13
pros::Motor intake2 (INTAKE2, pros::E_MOTOR_GEARSET_18, true);//13
pros::Motor tilter(TILTERPORT, pros::E_MOTOR_GEARSET_36, false);//13

//pros::ADIEncoder LeftEncoder(LENC_TOP_PORT,LENC_BOT_PORT,true);
//pros::ADIEncoder RightEncoder(RENC_TOP_PORT,RENC_BOT_PORT,true);
//pros::ADIEncoder BackEncoder(BENC_TOP_PORT,BENC_BOT_PORT,true);

pros::ADIEncoder leftenc (LENC_TOP_PORT,LENC_BOT_PORT,false);
pros::ADIEncoder rightenc (RENC_TOP_PORT,RENC_BOT_PORT,true);
pros::ADIEncoder backenc ('D','F');

int selectedAuton = 0;

float deg=0;


rPos mainPosition {0.0,0.0,0.0,0,0,0};
