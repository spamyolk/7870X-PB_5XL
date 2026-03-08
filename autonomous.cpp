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

void intakeThread() {
  optical_sensor.setLight(ledState::on);
  optical_sensor.setLightPower(100);
  while(!optical_sensor.isNearObject()) {
    wait(10, msec);
  }
  intake(-8, -12, -12);
  optical_sensor.setLight(ledState::off);
}

void Test() {
  /*
  intake(-12, 12, 12);
  thread it = thread(intakeThread);
  wait(3, sec);
  it.interrupt();
  intake(12, -12, -12);
  */
}

void Seven_R() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);
  
  //move, middle-high -> collect+score 2B, middle-high
  intake(-12, 12, 12);
  moveToPoint(-10, 12, 1, 1500, false);
  turnToAngle(-135, 1000);
  intake(-12, -4, -4);
  driveTo(-14.5, 1500);
  intake(-12, 7, -5);
  wait(600, msec);

  //move, loader-r1 -> collect 6B
  moveToPoint(-12.5, -5, 1, 2000, false);
  fd.set(true);
  turnToAngle(-180, 1000);
  intake(-12, 12, 12);
  driveTo(11.5, 1000, true, 5.5);
  wait(350, msec);

  //score 7B, long-r1
  driveTo(-22, 1000, true, 10.0);
  hood.set(true);
  fd.set(false);
  wait(1000, msec);

  //wing control
  driveTo(4, 1000, false);
  wing.set(false);
  turnToAngle(135, 1000);
  driveTo(-4.75, 1000, false);
  turnToAngle(180, 1000);
  driveTo(-14, 1300);
  intake(0, 0, 0);
  stopChassis(hold);
}

void SAWP() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);

  //move, loader-r2
  intake(-12, 12, 12);
  driveTo(24.5, 2000);
  fd.set(true);
  turnToAngle(90, 500);

  //collect 3B, loader-r2
  driveTo(15, 700, true, 5.75);
  wait(250, msec);

  //score 4B, long-r2
  turnToAngle(90, 1000);
  driveTo(-21, 1000, true, 10.0);
  hood.set(true);
  intake(-12, -12, -12);
  fd.set(false);
  wait(100, msec);
  intake(-12, 12, 12);
  wait(1000, msec);

  //collect 3B, loader-r2
  intake(-12, 12, 12);
  thread it = thread(intakeThread);
  turnToAngle(200, 750);
  hood.set(false);
  driveTo(4, 1000, false, 6.0);
  fd.set(true);
  driveTo(10, 1000);
  fd.set(false);
  turnToAngle(-135, 500);

  //score 3B, middle-low
  it.interrupt();
  intake(4, -10, -12);
  driveTo(10, 1000);
  intake(7, -7, 0);
  wait(750, msec);
  intake(0, 0, 0);

  //collect 3B, loader-r1
  driveTo(-6, 1000, false);
  turnToAngle(-180, 1000);
  intake(-12, 12, 12);
  driveTo(22, 2000, false);
  fd.set(true);
  driveTo(10, 1000);
  turnToAngle(135, 1000);

  //score 3B, middle-high
  driveTo(-15.5, 1000);
  intake(12, -12, 12);
  wait(100, msec);
  intake(-12, 8, -6);
}

void LeftS() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);

  //drive to loader
  intake(-12, 12, 12);
  driveTo(24.5, 2000);
  fd.set(true);
  turnToAngle(-90, 1000);

  //collect 3B, loader-r2
  driveTo(15, 700, true, 5.5);
  wait(225, msec);

  //score 4B, long-r2
  turnToAngle(-90, 1000);
  driveTo(-21, 1000, true, 10.0);
  hood.set(true);
  intake(-12, -12, -12);
  fd.set(false);
  wait(50, msec);
  intake(-12, 12, 12);
  wait(1000, msec);

  //collect 3B, loader-r2
  intake(-12, 12, 12);
  turnToAngle(-200, 750);
  hood.set(false);
  driveTo(4, 1000, false, 6.0);
  fd.set(true);
  driveTo(10, 1000);
  fd.set(false);
  turnToAngle(-45, 500);

  //score 3B, middle-high
  driveTo(-14.5, 3000, true, 8.0);
  intake(-8, 8, -8);
  wait(750, msec);
  intake(0, 0, 0);

  //wing control
  driveTo(12.5, 1000);
  wing.set(false);
  swing(90, -1, 1000);
  driveTo(5.5, 1000, false, 4.0);
  turnToAngle(75, 1000);
  stopChassis(hold);
}

void LeftF() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);

  //drive to loader
  intake(-12, 12, 12);
  driveTo(24.5, 2000);
  fd.set(true);
  turnToAngle(-90, 1000);

  //collect 3B, loader-r2
  driveTo(15, 700, true, 5.5);
  wait(225, msec);

  //score 4B, long-r2
  turnToAngle(-90, 1000);
  driveTo(-21, 1000, true, 10.0);
  hood.set(true);
  intake(-12, -12, -12);
  fd.set(false);
  wait(50, msec);
  intake(-12, 12, 12);
  wait(1000, msec);

  //collect 3B, loader-r2
  intake(-12, 12, 12);
  turnToAngle(-200, 750);
  hood.set(false);
  driveTo(4, 1000, false, 6.0);
  fd.set(true);
  driveTo(10, 1000);
  fd.set(false);
  turnToAngle(-45, 500);

  //score 3B, middle-high
  driveTo(-14.5, 3000, true, 8.0);
  intake(-8, 8, -8);
  wait(750, msec);
  intake(0, 0, 0);

  //wing control
  driveTo(12.5, 1000);
  wing.set(false);
  swing(90, -1, 1000);
  driveTo(6, 1000, false, 8.0);
  turnToAngle(75, 1000);
  stopChassis(hold);
}

void RightS() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);

  //move, loader-r2
  intake(-12, 12, 12);
  driveTo(24.5, 2000);
  fd.set(true);
  turnToAngle(90, 500);

  //collect 3B, loader-r2
  driveTo(15, 700, true, 5.5);
  wait(225, msec);

  //score 4B, long-r2
  turnToAngle(90, 1000);
  driveTo(-21, 1000, true, 10.0);
  hood.set(true);
  intake(-12, -12, -12);
  fd.set(false);
  wait(50, msec);
  intake(-12, 12, 12);
  wait(1000, msec);

  //collect 3B, loader-r2
  intake(-12, 12, 12);
  thread it = thread(intakeThread);
  turnToAngle(200, 750);
  hood.set(false);
  driveTo(4, 1000, false, 6.0);
  fd.set(true);
  driveTo(10, 1000);
  fd.set(false);
  turnToAngle(-135, 500);

  //score 3B, middle-low
  it.interrupt();
  intake(4, -10, -12);
  driveTo(10, 1000);
  intake(7, -7, 0);
  wait(750, msec);
  intake(0, 0, 0);

  //wing control
  driveTo(-12.5, 1000);
  wing.set(false);
  swing(90, -1, 1000);
  driveTo(-5.5, 1000, false, 4.0);
  turnToAngle(75, 1000);
  stopChassis(hold);
}

void RightF() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);

  //move, loader-r2
  intake(-12, 12, 12);
  driveTo(24.5, 2000);
  fd.set(true);
  turnToAngle(90, 500);

  //collect 3B, loader-r2
  driveTo(15, 700, true, 5.5);
  wait(240, msec);

  //score 4B, long-r2
  turnToAngle(90, 1000);
  driveTo(-21, 1000, true, 10.0);
  hood.set(true);
  intake(-12, -12, -12);
  fd.set(false);
  wait(50, msec);
  intake(-12, 12, 12);
  wait(1000, msec);

  //collect 3B, loader-r2
  intake(-12, 12, 12);
  thread it = thread(intakeThread);
  turnToAngle(200, 750);
  hood.set(false);
  driveTo(4, 1000, false, 6.0);
  fd.set(true);
  driveTo(10, 1000);
  fd.set(false);
  turnToAngle(-135, 500);

  //score 3B, middle-low
  it.interrupt();
  intake(4, -10, -12);
  driveTo(10, 1000);
  intake(7, -7, 0);
  wait(750, msec);
  intake(0, 0, 0);

  //wing control
  driveTo(-12.5, 1000);
  wing.set(false);
  swing(90, -1, 1000);
  driveTo(-6, 1000, false, 8.0);
  turnToAngle(75, 1000);
  stopChassis(hold);
}

void RightF2() {
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

  //wing control
  driveTo(4, 1000, false);
  wing.set(false);
  turnToAngle(45, 1000);
  driveTo(-4.75, 1000, false);
  turnToAngle(90, 1000);
  driveTo(-14, 1300);
  intake(0, 0, 0);
  stopChassis(hold);
}

void Skills() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);
  
  //move, middle-high -> collect+score 2B, middle-high
  intake(-12, 12, 12);
  moveToPoint(-10, 12, 1, 1500, false);
  turnToAngle(-135, 1000);
  intake(-12, -4, -4);
  driveTo(-14.5, 1500);
  intake(-12, 7, -5);
  wait(600, msec);

  //move, loader-r1 -> collect 6B
  moveToPoint(-12.5, -5, 1, 2000, false);
  fd.set(true);
  turnToAngle(-180, 1000);
  intake(-12, 12, 12);
  driveTo(11.5, 1000, true, 6.0);
  wait(1400, msec);

  //move, loader-b2
  driveTo(-8, 1000);
  intake(0, 0, 0);
  turnToAngle(180, 800);
  fd.set(false);
  moveToPoint(-25, 14, -1, 3000, false);
  turnToAngle(180, 800);

  x_pos = 0;
  y_pos = 0;
  moveToPoint(0, 50, -1, 4000);

  turnToAngle(90, 800);
  driveTo(8.4, 2000);
  turnToAngle(0, 800);
  driveTo(-14, 1500, true, 8.0);

  //score 6B, long-b2
  hood.set(true);
  wait(100, msec);
  intake(-12, 12, 12);
  wait(2200, msec);
  
  //collect 6B, loader-b2
  fd.set(true);
  driveTo(14, 1000);
  hood.set(false);
  intake(-12, 12, 12);
  turnToAngle(0, 1000);
  driveTo(12, 700, true, 6.0);
  wait(1400, msec);

  //score 6B, loader-b1
  intake(0, 0, 0);
  driveTo(-14, 1500);
  turnToAngle(0, 1000);
  driveTo(-8, 750, true, 8.5);
  hood.set(true);
  wait(100, msec);
  intake(-12, 12, 12);
  wait(2200, msec);
  fd.set(false);
  intake(0, 0, 0);
  x_pos = 0;
  y_pos = 0;


  //move, loader-b1
  driveTo(8, 1000);
  hood.set(false);
  turnToAngle(90, 1000);
  driveTo(62.6, 4000, true, 10.0);
  turnToAngle(0, 1000);
  driveTo(-13, 1000);
  wait(250, msec);


  //collect 6B, loader-b1
  fd.set(true);
  driveTo(14, 1000);
  hood.set(false);
  intake(-12, 12, 12);
  turnToAngle(0, 1000);
  driveTo(12, 700, true, 6.0);
  wait(1400, msec);

  //move, loader-r2
  driveTo(-8, 1000);
  intake(0, 0, 0);
  turnToAngle(0, 800);
  fd.set(false);
  turnToAngle(-45, 800);
  driveTo(-12.5, 1000);
  turnToAngle(0, 800);

  x_pos = 0;
  y_pos = 0;
  moveToPoint(0, -50, -1, 4000);

  turnToAngle(-90, 800);
  driveTo(7.75, 2000);
  turnToAngle(180, 800);
  driveTo(-14, 1500, true, 9.0);

  //score 6B, long-b2
  hood.set(true);
  wait(100, msec);
  intake(-12, 12, 12);
  wait(2200, msec);
  
  //collect 6B, loader-b2
  fd.set(true);
  driveTo(14, 1000);
  hood.set(false);
  intake(-12, 12, 12);
  turnToAngle(180, 800);
  driveTo(12, 700, true, 6.0);
  wait(1400, msec);

  //score 6B, loader-b1
  intake(0, 0, 0);
  driveTo(-14, 1500);
  turnToAngle(180, 800);
  driveTo(-8, 750, true, 8.25);
  hood.set(true);
  wait(100, msec);
  intake(-12, 12, 12);
  wait(2200, msec);
  fd.set(false);
  intake(0, 0, 0);

  //move, RP
  wing.set(false);
  curveCircle(-90, 36, 1500);
  hood.set(false);
  intake(12, 12, -12);
  driveTo(-2, 1000);
  driveTo(24, 2000, true, 10.0);
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
