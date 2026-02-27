//calibrate
  correct_angle = inertial_sensor.rotation(); //correct angle variable to inertial sensor
  wing.set(true);

  //move, middle-high -> collect+score 2B
  intake(-12, 12, 12);
  moveToPoint(-10, 12, 1, 1500, false);
  turnToAngle(-135, 1000);
  intake(-12, -2, -4);
  driveTo(-14.5, 1500);
  intake(-12, 8, -6);
  wait(700, msec); //maybe cut 100msec

  //move, loader-r1 -> collect 6B
  moveToPoint(-12.5, -5, 1, 2000, false);
  fd.set(true);
  turnToAngle(-180, 1000);
  intake(-12, 12, 12);
  driveTo(12, 1000, true, 5.5);
  wait(1200, msec);

  //move, loader-b2
  driveTo(-8, 1000);
  intake(0, 0, 0);
  fd.set(false);
  moveToPoint(-26, 12, -1, 1500, false);
  turnToAngle(180, 1000);
  
  driveTo(-8, 1000);
  turnToAngle(-135, 1000);
  intake(0, 0, 0);
  fd.set(false);
  moveToPoint(-26, 12, -1, 1500);
  //turnToAngle(180, 1000);

  //moveToPoint(-25, 64, -1, 3000); //try false
  //moveToPoint(-13, 70, -1, 1500);

  //score 6B, long-b2
  //turnToAngle(0, 1500);
  //driveTo(-10, 1500);
  //hood.set(true);
  //intake(-12, 12, 12);
  //wait(2250, msec);

  //collect 6B, loader-b2
  //fd.set(true);
  //intake(0, 0, 0);
  //moveToPoint(-12.75, 70, 1, 1500);
  //driveTo(16, 2000);
  //hood.set(false);
  //intake(-12, 12, 12);
  //wait(1200, msec);

  //score 6B, long-b2
  //driveTo(-22, 1500);
  //hood.set(true);
  //fd.set(false);
  //wait(2250, msec);
