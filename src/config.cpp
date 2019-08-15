#include "config.hpp"
#include "pos_struct.h"
#include "okapi/api.hpp"
using namespace okapi;

const int LEFT_WHEEL_PORT = 17; //17
const int LEFT_CHAIN_PORT = 18; //18
const int RIGHT_CHAIN_PORT = 13; //13
const int RIGHT_WHEEL_PORT = 15; //15

const char LENC_TOP_PORT = 'A';
const char LENC_BOT_PORT = 'B';
const char RENC_TOP_PORT = 'G';
const char RENC_BOT_PORT = 'H';
const char BENC_TOP_PORT = 'E';
const char BENC_BOT_PORT = 'F';

pros::Motor left_wheel (LEFT_WHEEL_PORT, pros::E_MOTOR_GEARSET_18,true);//17
pros::Motor right_wheel (RIGHT_WHEEL_PORT, pros::E_MOTOR_GEARSET_18, true);//15
pros::Motor left_chain (LEFT_CHAIN_PORT, pros::E_MOTOR_GEARSET_18, false);//18
pros::Motor right_chain (RIGHT_CHAIN_PORT, pros::E_MOTOR_GEARSET_18, false);//13
pros::Controller master (CONTROLLER_MASTER);
//pros::ADIEncoder LeftEncoder(LENC_TOP_PORT,LENC_BOT_PORT,true);
//pros::ADIEncoder RightEncoder(RENC_TOP_PORT,RENC_BOT_PORT,true);
//pros::ADIEncoder BackEncoder(BENC_TOP_PORT,BENC_BOT_PORT,true);

okapi::ADIEncoder leftenc ('A','B',true);
okapi::ADIEncoder rightenc ('C','D',true);
okapi::ADIEncoder backenc ('E','F');

int selectedAuton = 0;


rPos mainPosition {0.0,0.0,0.0,0,0,0};
