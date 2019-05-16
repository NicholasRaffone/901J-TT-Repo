#include "config.hpp"
const int LEFT_WHEEL_PORT = 17; //17
const int LEFT_CHAIN_PORT = 18; //18
const int RIGHT_CHAIN_PORT = 14; //14
const int RIGHT_WHEEL_PORT = 15; //15
const char LENC_TOP_PORT = 'A';
const char LENC_BOT_PORT = 'B';
const char RENC_TOP_PORT = 'C';
const char RENC_BOT_PORT = 'D';
const char BENC_TOP_PORT = 'E';
const char BENC_BOT_PORT = 'F';

pros::Motor left_wheel (LEFT_WHEEL_PORT);
pros::Motor right_wheel (RIGHT_WHEEL_PORT, true);
pros::Motor left_chain (LEFT_CHAIN_PORT);
pros::Motor right_chain (RIGHT_CHAIN_PORT, true);
pros::Controller master (CONTROLLER_MASTER);
pros::ADIEncoder LeftEncoder(LENC_TOP_PORT,LENC_BOT_PORT,true);
pros::ADIEncoder RightEncoder(RENC_TOP_PORT,RENC_BOT_PORT,true);
pros::ADIEncoder BackEncoder(BENC_TOP_PORT,BENC_BOT_PORT,true);



bool blueSide = false; // 1
bool farSide = false; // 2
bool park = true; // 4
std::vector<float> RobotPosition (3);
