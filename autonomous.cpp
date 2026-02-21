#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>

#include "../include/autonomous.h"
#include "motor-control.h"

// IMPORTANT: Remember to add respective function declarations to custom/include/autonomous.h
// Call these functions from custom/include/user.cpp
// Format: returnType functionName() { code }
//color: 0 = default, 1 = red, 2 = blue

void PID_test() {
 // Use this for tuning linear
 //driveTo(24, 2000);
 //wait(2, sec);
 //driveTo(-24, 2000);
 //Use this for tuning turn
 
 turnToAngle(90, 2000);
 wait(1, sec);
 turnToAngle(180, 2000);
 wait(1, sec);
 turnToAngle(270, 2000);
 wait(1, sec);
 turnToAngle(360, 2000);
 wait(1, sec);
 turnToAngle(0, 2000);
 wait(1, sec);
}

void SAWP2() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  //wing.set(true);

  //move, loader-r2
  intake(-12, 12, 12);
  driveTo(24.5, 2000, true, 11.0);
  fd.set(true);
  turnToAngle(90, 500);

  //collect 3B, loader-r2
  driveTo(15, 700, true, 5.0);
  wait(200, msec);

  //score 4B, long-r2
  driveTo(-22, 1000, true, 10.0);
  hood.set(true);
  intake(-12, -12, -12);
  fd.set(false);
  wait(100, msec);
  intake(-12, 12, 12);
  wait(1000, msec);

  //collect 3B, loader-r2
  turnToAngle(182, 600);
  hood.set(false);
  driveTo(3, 400, false);
  moveToPoint(3.2, 8.5, 1, 1200, true, 3.0);
  swing(-132, 1, 600, true, 10.0);
  intake(0, 0, 0);

  //score 3B, middle-low
  driveTo(12.5, 800);
  intake(8, -12, -12.0);
  wait(1100, msec);
  intake(0, 0, 0);

  //collect 3B, loader-r1
  driveTo(-10, 900);
  turnToAngle(-180, 800);
  intake(-12, 12, 12);
  moveToPoint(4, -18, 1, 1800);
  driveTo(7, 500, true, 9.0);
  driveTo(-2, 200);
  //driveTo(30, 1000, false, 8.5);
  //fd.set(true);
  //wait(100, msec);
  //fd.set(false);
  //driveTo(-2.5, 300, false, 10.0);

  //score 3B, middle-high
  turnToAngle(135, 600);
  driveTo(-14.5, 1000);
  intake(12, -12, 12);
  wait(100, msec);
  intake(-12, 12, -8);
  wait(1000, msec);
}

void SAWP1() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  //wing.set(true);

  //move, loader-r2
  intake(-12, 12, 12);
  driveTo(24.5, 2000, true, 11.0);
  fd.set(true);
  turnToAngle(90, 1000);

  //collect 3B, loader-r2
  driveTo(15, 700, true, 5.5);
  wait(200, msec);

  //score 4B, long-r2
  driveTo(-22, 1000, true, 10.0);
  hood.set(true);
  intake(-12, -12, -12);
  fd.set(false);
  wait(100, msec);
  intake(-12, 12, 12);
  wait(1100, msec);

  //collect 6B, middles
  turnToAngle(180, 600);
  hood.set(false);
  moveToPoint(6, -1, 1, 1500);
  driveTo(2, 750, true, 9.0);
  moveToPoint(6, -18, 1, 2000);
  driveTo(2, 750, true, 9.0);

  //score 4B, middle-high
  driveTo(4, 1000);
  turnToAngle(135, 1000);
  driveTo(-14, 1000);
  intake(-12, -12, 12);
  wait(200, msec);
  intake(-12, 12, -12);
  wait(300, msec);

  //move, loader-r1
  //moveToPoint(0, )
  /*driveTo(30, 5000, true, 7);
  fd.set(true);
  correct_angle = normalizeTarget(90); //updates heading

  //collect 3B, loader-r1
  driveTo(4, 2000, true, 9.0);
  intake(-12, 12, 12);
  wait(200, msec);

  //score 6B, long-r1
  driveTo(-22, 1000, true, 10.0);
  hood.set(true);
  intake(-12, -12, -12);
  fd.set(false);
  wait(100, msec);
  intake(-12, 12, 12);
  */
}

void Skills() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  //wing.set(true);

  //move, loader-r2
  intake(-12, 12, 12);

  driveTo(24.5, 2000);
  fd.set(true);
  turnToAngle(90, 1000);

  //collect 3B, loader-r2
  driveTo(15, 700, true, 5.5);
  wait(900, msec);
  driveTo(-6, 1000);
  intake(0, 0, 0);
  fd.set(false);

  //move, long-b1
  inertial_sensor.setRotation(90, degrees);
  correct_angle = 90;
  x_pos = 0;
  y_pos = 0;
  moveToPoint(-5, 7, -1, 2000);
  turnToAngle(90, 1000);
  moveToPoint(-54, 7, -1, 2000);
  turnToAngle(45, 1000, false);
  driveTo(-2, 1000, false);
  turnToAngle(-90, 1000, false);
  driveTo(-4, 1000);

  //score 7B, long-b1
  //hood.set(true);

  //intake(-12, -12, -12);
  //wait(100, msec);
  //intake(-12, 12, 12);
  //wait(1500, msec);
  //intake(0, 0, 0);

  //driveTo(2, 1000);
  //hood.set(false);
  //driveTo(-2, 1000, true, 6.0);

  //move, loader-b1
  //fd.set(true);
  //driveTo(15, 1000);
  //driveTo(15, 700, true, 5.5);
  //wait(900, msec);

  //score 6B, loader-b1
  //driveTo(-22, 1000, true, 10.0);
  //hood.set(true);

  //intake(-12, -12, -12);
  //fd.set(false);
  //wait(100, msec);
  //intake(-12, 12, 12);
  //wait(1500, msec);

  //move, park-red
  //driveTo(4, 1000);
  //turnToAngle(-135, 1000);
  //driveTo(-2, 1000);
  //turnToAngle(-90, 1000);
  //moveToPoint(-54, 7, -1, 2000);
  //moveToPoint(-5, 7, -1, 2000);

  //turnToAngle(180, 1000);
  //inertial_sensor.setRotation(180, degrees);
  //correct_angle = 180;
  //x_pos = 0;
  //y_pos = 0;
  //moveToPoint(10, 10, 1, 1500);
  //fd.set(true);
  //wait(500, msec);
  //driveTo(-4, 2000);
  //driveChassis(9, 9);
  //wait(800, msec);
  //driveChassis(0, 0);
  //fd.set(false);
  //intake(12, -12, -12);
}

/*
//color sorting?
void colorGood(){
 optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
 optical_sensor.setLightPower(100);          // Set light power to max
 int color1 = 0;

 while(true) {
   if (optical_sensor.color() == red) {
     color1 = 1;
   } else if (optical_sensor.color() == blue) {
     color1 = 2;
   } else {
     color1 = 0;
   }
   wait(10, msec);
 }
}
*/
