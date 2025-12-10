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
 // Use this for tuning linear and turn pid
 driveTo(24, 2000);
 wait(1, sec);
 turnToAngle(90, 800);
 wait(0.5, sec);
 turnToAngle(180, 800);
 wait(0.5, sec);
 turnToAngle(270, 8000);
 wait(0.5, sec);
 turnToAngle(360, 800);
 wait(0.5, sec);
 turnToAngle(0, 2000);
 wait(0.5, sec);
 driveTo(-24, 2000);
}

void Rightside() {
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  hood.set(false); //retracts hood

  driveTo(35, 3000, true, 10.0);
  turnToAngle(90, 2000);
  fd1.set(true);
  fd2.set(true);
  intake(-12, 12, 12);
  wait(500, msec);

  driveTo(13.75, 2500);
  driveTo(-1, 1000, true, 5.0);
  driveTo(1,1000,true,5.0);
  wait(900, msec);

  driveTo(-10, 3000, true, 10.0);
  fd1.set(false);
  fd2.set(false);
  wait(500, msec);
  driveTo(-20, 3000, true, 10.0);
  wait(200, msec);
  hood.set(true);
  wait(2000, msec);

  driveTo(10, 2000, true, 8.0);
  hood.set(false);
  wait(500, msec);
  driveChassis(-12.0, -12.0);
  wait(500, msec);
  stopChassis();
}

void Leftside() {
 correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
 hood.set(false); //retracts hood

 //intake first 3 Blocks
 driveTo(12, 2000, false);
 turnToAngle(-90, 1000, true);
 intake(-12, 12, 12); //runs intake(t)
 driveTo(17, 2000, true, 3.5);
 wait(250, msec);

 //scoring in middle
 correct_angle = normalizeTarget(-135); //updates heading
 driveTo(-17.75, 2000, true, 5.0);
 intake(-12, -12, -12);
 hood.set(true); //opens hood for jams
 wait(600, msec);
 intake(-12, 12, -12); //runs intake(m)
 wait(1750, msec);
 intake(0, 0, 0); //stops intake
 hood.set(false);

 //intakes 3 Loader Blocks
 driveTo(51, 5000, true, 6.5);
 correct_angle = normalizeTarget(-180); //updates heading
 driveTo(4, 2000, true, 6.0);
 fd1.set(true); //drops fd
 fd2.set(true);
 hood.set(true);
 wait(500, msec);
 intake(-12, 12, 12); //runs intake(t)
 driveTo(12.25, 2000, false, 12.0);
 driveTo(-1, 800);
 wait(1000, msec);

 //scores in high + push
 driveTo(-30, 3000, true, 6.0);
 hood.set(true);
 fd1.set(false);
 fd2.set(false);

 
}

void SAWP() {
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  hood.set(false); //retracts hood

  //drive to loader
  driveTo(35, 3000, true, 10.0);
  turnToAngle(90, 2000);
  fd1.set(true);
  fd2.set(true);
  intake(-12, 12, 12);
  wait(400, msec);

  //collect 3B
  driveTo(12.5, 2000);
  driveTo(-1, 1000);
  wait(700, msec); //900

  //score 4B
  driveTo(-30, 3000, true, 10.0);
  fd1.set(false);
  fd2.set(false);
  hood.set(true);
  wait(1900, msec);
  driveTo(12, 3000, true, 12.0);
  hood.set(false);
  driveChassis(-12, -12);
  wait(300, msec);

  //score low goal
  swing(227, 1, 3000);
  driveTo(24, 3000, true, 3.0); //2.0
  wait(700, msec);
  driveTo(10, 3000, true, 3.0);
  intake(12, -12, -12);

  // score mid goal
  //swing(210, -1, 2000);
  //driveTo(-12, 2000, true, 6.0);
}

void Skills() {
 
}


//color sorting?
/*void colorGood(){
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
}*/


//1689V HS code/functions
double arm_pid_target = 0, arm_load_target = 60, arm_store_target = 250, arm_score_target = 470;

/*
* armPID
* Runs a single PID update for the arm motor to reach the specified target position.
* - arm_target: Desired arm position (degrees).
*/
void armPID(double arm_target) {
 PID pidarm = PID(0.1, 0, 0.5); // Initialize PID controller for arm
 pidarm.setTarget(arm_target);   // Set target position
 pidarm.setIntegralMax(0);
 pidarm.setIntegralRange(1);
 pidarm.setSmallBigErrorTolerance(1, 1);
 pidarm.setSmallBigErrorDuration(0, 0);
 pidarm.setDerivativeTolerance(100);
 pidarm.setArrive(true);
 arm_motor.spin(fwd, pidarm.update(arm_motor.position(deg)), volt); // Apply PID output to arm motor
}

/*
* armPIDLoop
* Continuously runs the arm PID control in a separate thread, keeping the arm at the target position.
*/
void armPIDLoop() {
 while(true) {
   armPID(arm_pid_target); // Continuously update arm position
   wait(10, msec);
 }
}

/*
* rushClamp
* Waits until the clamp distance sensor detects an object within 85mm, then closes the claw and lowers the rush arm.
* Used for quickly grabbing a mobile goal at the start of autonomous.
*/
void rushClamp() {
 while(clamp_distance.objectDistance(mm) > 85) { // Wait for object to be close enough
   wait(10, msec);
 }
 claw.set(true);        // Close the claw to grab the goal
 rush_arm.set(false);   // Lower the rush arm
}

/*
* intakeThread
* Runs the intake until an object is detected by the optical or distance sensor, then stops the intake.
* Used for picking up rings or other objects during autonomous.
*/
void intakeThread(){
 optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
 optical_sensor.setLightPower(100);          // Set light power to max
 while(!optical_sensor.isNearObject() && intake_distance.objectDistance(mm) > 50){
   wait(10, msec);                           // Wait until object is detected
 }
 intake_motor.stop(hold);                    // Stop intake motor and hold
}

/*
* redGoalRush
* 2024-2025 World Championship runner-up(1698V) autonomous routine.
* This routine executes a complex sequence to rush, grab, and score mobile goals and rings.
* It uses multiple threads for simultaneous arm, clamp, and intake control.
*/

void redGoalRush() {
 arm_motor.setPosition(arm_load_target, deg);         // Set arm to load position
 correct_angle = inertial_sensor.rotation();          // Sync correct_angle with inertial sensor
 arm_pid_target = arm_store_target;                   // Set arm PID target to store position

 thread al = thread(armPIDLoop);                      // Start arm PID loop in a thread
 thread rc = thread(rushClamp);                       // Start clamp routine in a thread
 intake_motor.spin(fwd, 12, volt);                    // Start intake motor at full speed
 thread it = thread(intakeThread);                    // Start intake sensor thread
 rush_arm.set(true);                                  // Lower rush arm

 driveTo(33, 1100, true);                             // Drive forward to first goal
 moveToPoint(-2, 10, -1, 15000, false);               // Pull the goal back
 stopChassis(hold);                                   // Stop chassis and hold position

 rc.interrupt();                                      // Stop clamp thread (goal should be clamped)
 rush_arm.set(true);                                  // Lower rush arm again (ensure down)
 claw.set(false);                                     // Open claw to release goal
 wait(100, msec);                                     // Brief pause

 correct_angle = normalizeTarget(-20);                // Adjust target heading for next maneuver
 driveTo(3, 800, true, 8);                            // Drive forward slightly
 driveTo(-5, 1000, true);                             // Back up

 rush_arm.set(false);                                 // Raise rush arm
 wait(200, msec);                                     // Wait for arm to raise

 turnToAngle(-90, 800, false);                        // Turn to face the goal backwards
 moveToPoint(0, 26, -1, 2000, false, 6);              // Move backwards into the goal
 driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
 mogo_mech.set(true);                                 // Clamp mobile goal
 wait(100, msec);                                     // Wait for clamp

 it.interrupt();                                      // Stop intake thread (ring should be collected)
 intake_motor.spin(fwd, 12, volt);                    // Restart intake

 moveToPoint(1, 7, 1, 2000, true);                    // Move near corner to drop goal
 turnToAngle(-90, 350, true);                         // Turn to drop goal
 mogo_mech.set(false);                                // Release mobile goal clamp
 driveChassis(-4, 4);                                 // Turn a bit to align with next target
 wait(300, msec);                                     // Wait for spin

 intake_motor.spin(fwd, -12, volt);                   // Reverse intake to push disc in front away
 moveToPoint(-13, -4, 1, 1500, false, 10);            // Move forward to push disc out of the way
 turnToAngle(180, 800, false);                        // Turn to clamp goal
 intake_motor.spin(fwd, 0, volt);                     // Stop intake

 moveToPoint(-31, 26, -1, 2000, false, 6);            // Move backwards into the next goal
 driveChassis(-1.5, -1.5);                            // Slowly drive backward for alignment
 mogo_mech.set(true);                                 // Clamp mobile goal
 wait(100, msec);                                     // Wait for clamp

 turnToAngle(145, 300, true);                         // Turn to face corner
 moveToPoint(-4, -3, 1, 2000, false);                 // Move to corner
 intake_motor.spin(fwd, 12, volt);                    // Start intake

 correct_angle = normalizeTarget(135);                // Update heading for next maneuver
 driveTo(1000, 1500, false, 4);                       // Drive forward infinitely until timeout
 driveTo(-13, 2000, true, 6);                         // Back up
 driveTo(10, 2500, true, 3);                          // Drive forward to intake second corner ring

 wait(200, msec);                                     // Brief wait for intake

 moveToPoint(-11, 6, -1, 2000, false, 10);            // Move backward out of the corner
 turnToAngle(45, 400, true);                          // Turn to align for wallstake

 al.interrupt();                                      // Stop arm PID thread
 arm_pid_target = arm_score_target - 100;             // Set arm to scoring position
 thread al2 = thread(armPIDLoop);                     // Start new arm PID thread

 moveToPoint(12, 34, 1, 1700, true, 8);               // Move forward to final wallstake scoring position

 al2.interrupt();                                     // Stop arm PID thread
 arm_motor.spin(fwd, 1, volt);                        // Spin arm forward slightly

 turnToAngle(40, 200);                                // Final turn for alignment
 driveChassis(1, 1);                                  // Slow drive forward
}
