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
 //Use this for tuning turn
 /*
 turnToAngle(90, 2000);
 wait(1, sec);
 turnToAngle(180, 2000);
 wait(1, sec);
 turnToAngle(-360, 2000);
 wait(1, sec);
 turnToAngle(360, 2000);
 wait(1, sec);
 turnToAngle(0, 2000);
 wait(1, sec);
 */
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



//SLOT1
void SAWP() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true); //add later

  //move, loader-r2
  intake(-12, 12, 12);
  driveTo(19.65, 1250, false); //change to true later if time permits
  fd.set(true);
  turnToAngle(90, 500);

  //collect 3B, loader-r2
  driveTo(16.25, 825, true, 6.5);
  wait(350, msec);

  //score 4B, long-r2
  driveTo(-30, 1000, true);
  hood.set(true);
  fd.set(false);
  wait(600, msec); //650

  //collect 3B, loader-r2
  turnToAngle(195, 750); //adjust to false if time needed
  hood.set(false);
  moveToPoint(-3, 16, 1, 1000, false, 11);
  moveToPoint(-2.5, -26, 1, 1000, false);
  fd.set(true);
  driveTo(12, 2000);
  fd.set(false);

  //score 6B, long-r1
  moveToPoint(16, -58.5, 1, 2500);
  turnToAngle(-270, 250, false);
  driveTo(-18, 800);
  turnToAngle(-270, 500);
  hood.set(true);
  wait(750, msec);

  //collect 3B, loader-r1
  fd.set(true);
  hood.set(false);
  driveTo(31.5, 1500);
  wait(250, msec);

  //score 3B, middle-high
  moveToPoint(-15, -23, -1, 1500);
  intake(-12, -12, -12);
  wait(150, msec);
  intake(-12, 12, -12);
}

//SLOT2
void Left4() {

}

//SLOT3
void Right4() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  //wing.set(true);
  
  //
  intake(-12, 12, 12);
  moveToPoint(13, 23, 1, 1500, false);
  fd.set(true);
  moveToPoint(50, 0, 1, 1000);
}

//SLOT4
void Left7() {

}

//SLOT5
void Right7() {

}

//SLOT6
void Left34() {

}

//SLOT7
void Right34() {

}

//SLOT8
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
