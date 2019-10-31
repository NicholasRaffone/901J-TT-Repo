#include "main.h"
#include "config.hpp"

extern const int LIFTGEARRATIO;
extern const int DEFAULTSLEWRATEINCREMENT;

void liftpid(int targetDegree, int maxvel);
void slewRateControl(pros::Motor *motor, int targetVelocity, int increment);
void tilter_PID(float targetDegree, int maxVelocity, double kp, int delay);
void lift_PID(float targetDegree, int maxVelocity, int delay,int multi);
void brakeMotors();
void unBrakeMotors();
void deploy();
