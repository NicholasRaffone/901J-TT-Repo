#include "main.h"
#include <vector>

extern const int LEFT_WHEEL_PORT; //17
extern const int LEFT_CHAIN_PORT; //18
extern const int RIGHT_CHAIN_PORT; //14
extern const int RIGHT_WHEEL_PORT; //15
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
extern pros::Controller master;
extern pros::ADIEncoder LeftEncoder;
extern pros::ADIEncoder RightEncoder;
extern pros::ADIEncoder BackEncoder;
extern std::vector<float> RobotPosition;
