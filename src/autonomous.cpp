#include "main.h"
#include "config.hpp"
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
void autonomous() {}

float[] track(){
  float wheelrad = 5;

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

  float rdis = 10;
  float ldis = 10;
  float bdis=10;

  float dltheta = 0; //change in encoder angle
  float drtheta = 0;
  float dbtheta = 0;

  float dl; //change in distance
  float dr;
  float ds;

  float dtheta;

  float[] d = new float[2];
  float[] dtemp = new float[2];
  float d1 = new float[2];

  float rad;
  float angle;
  while(true){
    dltheta = currentl - prevl;
    drtheta = currentr - prevr;
    dbtheta = currentb - prevb;

    //distance traveled by each wheel
    dl = ((dltheta)/360) * 2 *pi * wheelrad;
    dr = ((drtheta)/360) * 2 *pi * wheelrad;
    ds = ((dbtheta)/360) * 2 *pi * wheelrad;

    //update values
    prevl = currentl;
    prevr = currentr;
    prevb = currentb;

    theta1 = thetar + 180*((dl-dr)/(rdis+ldis))/pi;//convert angle to degrees and add to initial angle to find new angle

    dtheta = theta1 - theta0; //change in angle

    if(dtheta == 0.0){//if only vertical movement add vertical component dr
      dtemp = [ds,dr];
    }else{
      dtemp = 2 * sin(dtheta/2) * [ds/dtheta + bdis,dr/dtheta + rdis];
    }

    //calculate average orientation
    thetam = theta0 + dtheta/2;

    //convert position to polar
    angle = arctan(dtemp[1] / dtemp[0]);
    rad = sqrt(dtemp[0] * dtemp[0] + dtemp[1] * dtemp[1]);

    angle += -1 * thetam;

    //convert back
    dtemp[0] = rad * cos(angle);
    dtemp[1] = rad * sin(angle);

    //add position vector to old one
    for(int i = 0; i < dtemp.length; i++){
      d1[i] = d[i] + dtemp[i];
    }

      theta0 = theta1;

      d = d1;

      //return new position
      return d1;
  }

}
