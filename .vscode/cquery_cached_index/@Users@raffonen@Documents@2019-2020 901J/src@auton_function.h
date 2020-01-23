#include "main.h"
#include "config.hpp"

extern const int LIFTGEARRATIO;
extern const int DEFAULTSLEWRATEINCREMENT;

void liftpid(int targetDegree, int maxvel);
void slewRateControl(pros::Motor *motor, int targetVelocity, int increment);
void tilter_PID(float targetDegree, int maxVelocity, double kp, int delay);
void lift_PID(float targetDegree, int maxVelocity, int delay,int multi);
void trackPos(rPos& position);
void move_straight_rel_test(double xCoord, int maxVel, int multi);
void move_align(float targetDistance, int velocity);
void brakeMotors();
void unBrakeMotors();
void deploy();
void turn_PID(float targetDegree);
