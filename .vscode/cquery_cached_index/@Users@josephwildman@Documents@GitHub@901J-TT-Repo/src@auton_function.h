#include "main.h"
#include "config.hpp"

extern const int LIFTGEARRATIO;

void liftpid(int targetDegree, int maxvel);
void slewRateControl(pros::Motor *motor, int targetVelocity, int increment);
