#include "main.h"
#include <vector>
#include "pos_struct.h"
#include "okapi/api.hpp"
using namespace okapi;


extern const int LEFT_WHEEL_PORT; //17
extern const int LEFT_CHAIN_PORT; //18
extern const int RIGHT_CHAIN_PORT; //13
extern const int RIGHT_WHEEL_PORT; //15
extern const int LIFT_PORT;
extern const int INTAKE1;
extern const int INTAKE2;
extern const int TILERPORT;
extern const char LENC_TOP_PORT;
extern const char LENC_BOT_PORT;
extern const char RENC_TOP_PORT;
extern const char RENC_BOT_PORT;
extern const char BENC_TOP_PORT;
extern const char BENC_BOT_PORT;


extern pros::Motor left_wheel;
extern pros::Motor right_wheel;
extern pros::Motor left_chain;
extern pros::Motor right_chain;
extern pros::Motor lift;
extern pros::Motor intake1;
extern pros::Motor intake2;
extern pros::Motor tilter;


extern pros::Controller master;
extern pros::Motor left_lift;
extern pros::Motor right_lift;
extern okapi::ADIEncoder leftenc;
extern okapi::ADIEncoder rightenc;
extern okapi::ADIEncoder backenc;

extern int selectedAuton;

extern float deg;

extern rPos mainPosition;
