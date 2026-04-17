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



//SLOT1, 8/10
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

  //score 6B, long-r1
  moveToPoint(16, -58.5, 1, 2500);
  turnToAngle(-270, 250, false);
  driveTo(-18, 800);
  turnToAngle(-270, 500);
  hood.set(true);
  wait(750, msec);

  //collect 3B, loader-r1
  hood.set(false);
  driveTo(31.5, 1500);
  wait(250, msec);

  //score 3B, middle-high
  moveToPoint(-15, -23, -1, 1500);
  intake(-12, -12, -12);
  wait(150, msec);
  intake(-12, 12, -12);
  stopChassis(hold);
}

//SLOT2
void Left4() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);
  
  //collect 3B, middle-low
  intake(-12, 12, 12);
  moveToPoint(-11, 22.5, 1, 1500, false);
  fd.set(true);
  moveToPoint(-13, 25.5, 1, 750, false);

  //score 4B, long-r2
  moveToPoint(-43 , 6, -1, 1500);
  turnToAngle(180, 1000);
  driveTo(-14, 1000);
  hood.set(true);
  wait(600, msec);
  
  //wing long-r2
  moveToPoint(-32, 8, 1, 1000);
  wing.set(false);
  moveToPoint(-43, 34, 1, 2000, true, 6.0);
  intake(0, 0, 0);
  turnToAngle(45, 1000);
  stopChassis(hold);
}

//SLOT3 10/10
void Right4() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);
  
  //collect 3B, middle-low
  intake(-12, 12, 12);
  moveToPoint(11, 22.5, 1, 1500, false);
  fd.set(true);
  moveToPoint(13, 25.5, 1, 750, false);

  //score 4B, long-r2
  moveToPoint(43 , 6, -1, 1500);
  turnToAngle(-180, 1000);
  driveTo(-14, 1000);
  hood.set(true);
  wait(600, msec);
  
  //wing long-r2
  moveToPoint(32, 8, 1, 1000);
  wing.set(false);
  moveToPoint(43, 34, -1, 2000, true, 6.0);
  intake(0, 0, 0);
  turnToAngle(135, 1000);
  stopChassis(hold);
}

//SLOT4 OK, tuning for wing control**
void Left7() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);
  
  //collect 3B, middle-low
  intake(-12, 12, 12);
  moveToPoint(-11.25, 18.5, 1, 1500, false); //11, 22.5
  fd.set(true);
  moveToPoint(-13.25, 21.5, 1, 750, false); //13, 25.5


  //collect 3B, loader-r2
  moveToPoint(-30, 3, 1, 1000, false); //48, 4
  turnToAngle(-180, 500);
  driveTo(16.25, 825, true, 6.5); //16.25
  wait(225, msec);

  //score 7B, long-r2
  driveTo(-29.5, 1000, true);
  hood.set(true);
  fd.set(false);
  wait(1000, msec);

  //wing, long-r2 EDIT THIS
  moveToPoint(-20, 14, 1, 1000);
  wing.set(false);
  turnToAngle(0, 1000);
  driveTo(28, 1000, true, 8.0);
  intake(0, 0, 0);
  turnToAngle(45, 1000);
  stopChassis(hold);
}

//SLOT5 9/10
void Right7() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);
  
  //collect 3B, middle-low
  intake(-12, 12, 12);
  moveToPoint(11, 22.5, 1, 1500, false);
  fd.set(true);
  moveToPoint(13, 25.5, 1, 750, false);

  //collect 3B, loader-r2
  moveToPoint(48, 4, 1, 1000, false);
  turnToAngle(180, 500);
  driveTo(16.25, 825, true, 6.5);
  wait(175, msec);

  //score 7B, long-r2
  driveTo(-30, 1000, true);
  hood.set(true);
  fd.set(false);
  wait(1000, msec);

  //wing, long-r2
  moveToPoint(31, 8, 1, 1000);
  wing.set(false);
  moveToPoint(39, 32, -1, 2000, true, 6.0);
  intake(0, 0, 0);
  turnToAngle(130, 1000);
  stopChassis(hold);
}

//SLOT6, OK NEED EDIT FOR WING
void Left43() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true); //add later

  //move, loader-r2
  intake(-12, 12, 12);
  driveTo(21.5, 1250, false); //change to true later if time permits
  fd.set(true);
  turnToAngle(-90, 500);

  //collect 3B, loader-r2
  driveTo(16.5, 825, true, 6.5);
  wait(300, msec);

  //score 4B, long-r2
  driveTo(-30, 1000, true);
  hood.set(true);
  fd.set(false);
  wait(1000, msec);

  //collect 3B, loader-l2
  turnToAngle(-195, 750); //adjust angle
  hood.set(false);
  driveTo(4, 1000, false, 6.0);
  fd.set(true);
  driveTo(12, 1000);
  turnToAngle(-42, 500);
  hood.set(false);

  //score 3B, middle-high
  driveTo(-19, 1000);
  intake(12, -12, -12);
  wait(150, msec);
  intake(-12, 12, -12);
  wait(1000, msec);
  fd.set(false);
  intake(0, 0, 0);

  //wing control, long-l2
  driveTo(19, 1500);
  wing.set(false);
  swing(90, 1, 1000);
  driveTo(10, 1000, false, 6.0);
  turnToAngle(135, 1000);
  stopChassis(hold);
}

//SLOT7 8/10
void Right43() {
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
  wait(300, msec);

  //score 4B, long-r2
  driveTo(-30, 1000, true);
  hood.set(true);
  fd.set(false);
  wait(600, msec);
  thread it = thread(intakeThread);

  //collect 3B, loader-r2
  turnToAngle(195, 750); //adjust to false if time needed
  hood.set(false);
  driveTo(4, 1000, false, 6.0);
  fd.set(true);
  driveTo(12, 1000);
  fd.set(false);
  turnToAngle(-135, 500);
  hood.set(false);

  //score 3B, middle-low
  it.interrupt();
  intake(4, -10, -12);
  driveTo(16, 1000);
  intake(12, -12, 0);
  wait(750, msec);
  intake(0, 0, 0);

  //wing control, long-r2
  driveTo(-18, 1500);
  wing.set(false);
  swing(90, -1, 1000);
  driveTo(-10, 1000, false, 6.0);
  turnToAngle(45, 1000);
  stopChassis(hold);
}

//SLOT8
void Skills() {
  //calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);
  
  //move, middle-high -> collect+score 2B, middle-high
  intake(-12, 12, 12);
  moveToPoint(-11.25, 18.5, 1, 1500, false);
  fd.set(true);
  moveToPoint(-14.75, 21.5, 1, 750, false);
  turnToAngle(-135, 1000);
  intake(-12, -4, -4);
  driveTo(-20, 1500);
  intake(-12, 6, -4);
  wait(1000, msec);

  //move, loader-r1 -> collect 6B
  moveToPoint(-26, 3, 1, 1000, false); //48, 4
  turnToAngle(-180, 500);
  driveTo(16.25, 825, true, 6.5);
  intake(-12, 12, 12);
  driveTo(11.5, 1000, true, 5.5);
  wait(1250, msec);

  //move, loader-b2
  driveTo(-8, 1000);
  intake(0, 0, 0);
  turnToAngle(180, 800);
  fd.set(false);
  moveToPoint(-44, 14, -1, 3000, false);
  turnToAngle(180, 800);

  x_pos = 0;
  y_pos = 0;
  moveToPoint(0, 56, -1, 4000);

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


  driveTo(-30, 5000);
  driveTo(-40, 2500);

  turnToAngle(-90, 800);
  driveTo(7.75, 2000);
  turnToAngle(180, 800);
  driveTo(-14, 1500, true, 9.0);
/*
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
  */
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
