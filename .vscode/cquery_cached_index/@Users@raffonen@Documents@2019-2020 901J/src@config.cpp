#include "config.hpp"
const int LEFT_WHEEL_PORT = 17; //17
const int LEFT_CHAIN_PORT = 18; //18
const int RIGHT_CHAIN_PORT = 14; //14
const int RIGHT_WHEEL_PORT = 15; //15



pros::Motor left_wheel (LEFT_WHEEL_PORT);
pros::Motor right_wheel (RIGHT_WHEEL_PORT, true);
pros::Motor left_chain (LEFT_CHAIN_PORT);
pros::Motor right_chain (RIGHT_CHAIN_PORT, true);
pros::Controller master (CONTROLLER_MASTER);


bool blueSide = false; // 1
bool farSide = false; // 2
bool park = true; // 4
