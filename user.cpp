#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"


// Modify autonomous, driver, or pre-auton code below


void runAutonomous() {
 int auton_selected = 3;
 switch(auton_selected) {
   case 1:
     PID_test();
     break;
   case 2:
     Rightside();
     break; 
   case 3:
     Leftside();
     break;
   case 4:
     SAWP();
     break;
   case 5:
     Skills();
     break;
   case 6:
     break;
   case 7:
     break;
   case 8:
     break;
   case 9:
     redGoalRush();
     break;
 }
}


// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;


int colorGood(){
 optical_sensor.setLight(ledState::on);      // Turn on optical sensor light
 optical_sensor.setLightPower(100);          // Set light power to max


 while(true) {
   if (optical_sensor.color() == red) {
     return 1;
   } else if (optical_sensor.color() == blue) {
     return 2;
   } else {
     return 0;
   }
   wait(10, msec);
 }
}


void runDriver() {
 stopChassis(coast);
 heading_correction = false;
 //other personal vars
 int driver_selected = 3;
 bool x_toggleState = false;
 bool x_lastPressed = false;
 bool a_toggleState = false;
 bool a_lastPressed = false;
 bool b_toggleState = false;
 bool b_lastPressed = false;
 bool y_toggleState = false;
 bool y_lastPressed = false;


 while (true) {
   thread cg = thread(colorGood);
  
   // [-100, 100] for controller stick axis values
   ch1 = controller_1.Axis1.value();
   ch2 = controller_1.Axis2.value();
   ch3 = controller_1.Axis3.value();
   ch4 = controller_1.Axis4.value();


   // true/false for controller button presses
   l1 = controller_1.ButtonL1.pressing();
   l2 = controller_1.ButtonL2.pressing();
   r1 = controller_1.ButtonR1.pressing();
   r2 = controller_1.ButtonR2.pressing();
   button_a = controller_1.ButtonA.pressing();
   button_b = controller_1.ButtonB.pressing();
   button_x = controller_1.ButtonX.pressing();
   button_y = controller_1.ButtonY.pressing();
   button_up_arrow = controller_1.ButtonUp.pressing();
   button_down_arrow = controller_1.ButtonDown.pressing();
   button_left_arrow = controller_1.ButtonLeft.pressing();
   button_right_arrow = controller_1.ButtonRight.pressing();


   // default tank drive or replace it with your preferred driver code here:
   driveChassis(ch3 * 0.12, ch2 * 0.12);


   //wing controls
   if (button_x && !x_lastPressed) {
     x_toggleState = !x_toggleState;
   }
   x_lastPressed = button_x;


   if (x_toggleState) {
     wing.set(true);
   } else {
     wing.set(false);
   }


   //hood controls
   if (button_a && !a_lastPressed) {
     a_toggleState = !a_toggleState;
   }
   a_lastPressed = button_a;


   if (a_toggleState) {
     hood.set(true);
   } else {
     hood.set(false);
   }


   //front dropper controls
   if (button_b && !b_lastPressed) {
     b_toggleState = !b_toggleState;
   }
   b_lastPressed = button_b;


   if (b_toggleState) {
     fd1.set(true);
     fd2.set(true);
   } else {
     fd1.set(false);
     fd2.set(false);
   }


   //double park controls
   if (button_y && !y_lastPressed) {
     y_toggleState = !y_toggleState;
   }
   y_lastPressed = button_y;


   if (y_toggleState) {
     double_park.set(true);
   }/* else {
     double_park.set(false);
   }*/


   switch (driver_selected) {
     case 1:
       //red intake controls
       if (r1 == true) {
         //intake to score tall/capacity
         if (colorGood() == 1) {
           intake_motor1.spin(fwd, -12, volt);
           intake_motor2.spin(fwd, 12, volt);
           intake_motor3.spin(fwd, 12, volt);
         } else {
           intake_motor1.spin(fwd, -12, volt);
           intake_motor2.spin(fwd, 12, volt);
           intake_motor3.spin(fwd, -12, volt);
         }
       } else if (r2 == true) {
         //intake to score middle
         if (colorGood() == 1) {
           intake_motor1.spin(fwd, -12, volt);
           intake_motor2.spin(fwd, 12, volt);
           intake_motor3.spin(fwd, -12, volt);
         } else {
           intake_motor1.spin(fwd, -12, volt);
           intake_motor2.spin(fwd, 12, volt);
           intake_motor3.spin(fwd, 12, volt);
         }
       } else if (l1 == true) {
         //outtake
         intake_motor1.spin(fwd, 12, volt);
         intake_motor2.spin(fwd, -12, volt);
         intake_motor3.spin(fwd, -12, volt);
       } else {
         intake_motor1.stop();
         intake_motor1.setStopping(coast);
         intake_motor2.stop();
         intake_motor2.setStopping(coast);
         intake_motor3.stop();
         intake_motor3.setStopping(coast);
       }


       wait(10, msec);
       break;
    
     case 2:
       //blue intake controls
       if (r1 == true) {
         //intake to score tall/capacity
         if (colorGood() == 2) {
           intake_motor1.spin(fwd, -12, volt);
           intake_motor2.spin(fwd, 12, volt);
           intake_motor3.spin(fwd, 12, volt);
         } else {
           intake_motor1.spin(fwd, -12, volt);
           intake_motor2.spin(fwd, 12, volt);
           intake_motor3.spin(fwd, -12, volt);
         }
       } else if (r2 == true) {
         //intake to score middle
         if (colorGood() == 2) {
           intake(-12, 12, -12);
         } else {
           intake(-12, 12, 12);
         }
       } else if (l1 == true) {
         //outtake
         intake(12, -12, -12);
       } else {
         intake(0, 0, 0); //stop
       }


       wait(10, msec);
       break;


     case 3:
       //normal intake controls
       if (r1 == true) {
         //intake to score tall/capacity
         intake(-12, 12, 12);
       } else if (r2 == true) {
         //intake to score middle
         intake(-12, 12, -12);
       } else if (l1 == true) {
         //outtake
         intake(12, -12, -12);
       } else {
         intake(0, 0, 0); //stop
       }


       wait(10, msec);
       break;
     }
 }
}


void runPreAutonomous() {
   // Initializing Robot Configuration. DO NOT REMOVE!
 vexcodeInit();
  // Calibrate inertial sensor
 inertial_sensor.calibrate();


 // Wait for the Inertial Sensor to calibrate
 while (inertial_sensor.isCalibrating()) {
   wait(10, msec);
 }


 double current_heading = inertial_sensor.heading();
 Brain.Screen.print(current_heading);
  // odom tracking
 resetChassis();
 if(using_horizontal_tracker && using_vertical_tracker) {
   thread odom = thread(trackXYOdomWheel);
 } else if (using_horizontal_tracker) {
   thread odom = thread(trackXOdomWheel);
 } else if (using_vertical_tracker) {
   thread odom = thread(trackYOdomWheel);
 } else {
   thread odom = thread(trackNoOdomWheel);
 }
}
