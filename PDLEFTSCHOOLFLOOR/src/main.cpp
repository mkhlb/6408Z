/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Me                                                        */
/*    Created:      Fri Nov 29 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RulerL               sonar         A, B
// RulerR               sonar         C, D
// RulerS               sonar         G, H
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of vex::brain used for printing to the V5 brain screen
// A global instance of vex::competition
vex::competition Competition;

// define your global instances of motors and other devices here
controller PuppetMaster;

motor FRDrive(PORT2, gearSetting::ratio18_1, true);
motor FLDrive(PORT6, gearSetting::ratio18_1, false);
motor BRDrive(PORT11, gearSetting::ratio18_1, true);
motor BLDrive(PORT5, gearSetting::ratio18_1, false);

motor ArmR(PORT3, gearSetting::ratio36_1, true);
motor ArmL(PORT7, gearSetting::ratio36_1, false);

motor IntakeOne(PORT13, gearSetting::ratio18_1, true); // right
motor IntakeTwo(PORT16, gearSetting::ratio18_1, false);

// sonar RulerL = sonar(Brain.ThreeWirePort.A); this will break the code, gives
// memory permission error

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
/*
bool suck = false;
bool out = false;
void R2Pressed() {
  out = false;
  if (suck == false) {
    Brain.Screen.clearScreen();
    Brain.Screen.print("i am closed");
    suck = true;
  } else if (suck == true) {
    suck = false;
  }
}

void R1Pressed() {
  suck = false;
  if (out == false) {
    Brain.Screen.clearScreen();
    Brain.Screen.print("i am closed");
    out = true;
  } else if (out == true) {
    out = false;
  }
} */

float minMax(int number, int max) {
  float out;
  if (number > max) {
    out = (float)(max);
  } else if (number < -max) {
    out = (float)(-max);
  } else {
    out = (float)(number);
  }
  return out;
}

int sigmoid_map[255] = {
    -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70,
    -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70, -70,
    -70, -70, -70, -69, -69, -69, -69, -69, -69, -69, -69, -69, -69, -68, -68,
    -68, -68, -68, -67, -67, -67, -66, -66, -65, -65, -64, -64, -63, -62, -62,
    -61, -60, -59, -58, -57, -55, -54, -53, -51, -50, -48, -46, -44, -43, -41,
    -39, -37, -35, -33, -31, -29, -27, -26, -24, -22, -20, -19, -17, -16, -15,
    -13, -12, -11, -10, -9,  -8,  -8,  -7,  -6,  -6,  -5,  -5,  -4,  -4,  -3,
    -3,  -3,  -2,  -2,  -2,  -2,  -2,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
    -1,  -1,  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,
    1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   3,   3,
    3,   4,   4,   5,   5,   6,   6,   7,   8,   8,   9,   10,  11,  12,  13,
    15,  16,  17,  19,  20,  22,  24,  26,  27,  29,  31,  33,  35,  37,  39,
    41,  43,  44,  46,  48,  50,  51,  53,  54,  55,  57,  58,  59,  60,  61,
    62,  62,  63,  64,  64,  65,  65,  66,  66,  67,  67,  67,  68,  68,  68,
    68,  68,  69,  69,  69,  69,  69,  69,  69,  69,  69,  69,  70,  70,  70,
    70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,
    70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70,  70};

void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void NewPID(double TARGET_VALUEX, double TARGET_VALUEZ, double kP, double kD,
            double closeness, double maxSpeed) {
  // initialize PrevError only once
  double PrevLeftError = 0;
  double PrevRightError = 0;
  double PrevSideError = 0;
  double LeftDerivative = 0;
  double RightDerivative = 0;
  double SideDerivative = 0;
  if (TARGET_VALUEX != 0 && TARGET_VALUEZ != 0) {
    while (RulerL.distance(distanceUnits::cm) > TARGET_VALUEZ + closeness ||
           RulerL.distance(distanceUnits::cm) < TARGET_VALUEZ - closeness ||
           RulerR.distance(distanceUnits::cm) > TARGET_VALUEZ + closeness ||
           RulerR.distance(distanceUnits::cm) < TARGET_VALUEZ - closeness ||
           RulerS.distance(distanceUnits::cm) > TARGET_VALUEX + closeness ||
           RulerS.distance(distanceUnits::cm) < TARGET_VALUEX - closeness) {
      // proportional component below
      double LeftPower = 0;
      double RightPower = 0;
      double SidePower = 0;

      double LeftError = TARGET_VALUEZ - RulerL.distance(distanceUnits::cm);
      double RightError = TARGET_VALUEZ - RulerR.distance(distanceUnits::cm);
      if (PrevLeftError == 0) {
        LeftDerivative = 0;
      } else {
        LeftDerivative = LeftError - PrevLeftError;
      }
      PrevLeftError = LeftError;

      if (PrevRightError == 0) {
        RightDerivative = 0;
      } else {
        RightDerivative = RightError - PrevRightError;
      }
      PrevRightError = RightError;

      LeftPower = LeftError * kP + RightDerivative * kD;
      RightPower = RightError * kP + RightDerivative * kD;

      double SideError = TARGET_VALUEX - RulerS.distance(distanceUnits::cm);
      if (PrevSideError == 0) {
        SideDerivative = 0;
      } else {
        SideDerivative = SideError - PrevSideError;
      }
      PrevSideError = SideError;
      SidePower = SideError * kP + SideDerivative * kD;

      // derivative goes here
      PuppetMaster.Screen.clearLine();
      PuppetMaster.Screen.print(SideDerivative * kD);
      //PuppetMaster.Screen.print(RulerS.distance(distanceUnits::cm));
      FRDrive.spin(directionType::fwd, minMax(RightPower - SidePower, maxSpeed),
                   velocityUnits::pct);
      FLDrive.spin(directionType::fwd, minMax(LeftPower + SidePower, maxSpeed),
                   velocityUnits::pct);
      BRDrive.spin(directionType::fwd, minMax(RightPower + SidePower, maxSpeed),
                   velocityUnits::pct);
      BLDrive.spin(directionType::fwd, minMax(LeftPower - SidePower, maxSpeed),
                   velocityUnits::pct);

      task::sleep(50);
    }
  } else if (TARGET_VALUEZ != 0) {
    while (RulerL.distance(distanceUnits::cm) > TARGET_VALUEZ + closeness ||
           RulerL.distance(distanceUnits::cm) < TARGET_VALUEZ - closeness ||
           RulerR.distance(distanceUnits::cm) > TARGET_VALUEZ + closeness ||
           RulerR.distance(distanceUnits::cm) < TARGET_VALUEZ - closeness) {
      // proportional component below
      double LeftPower = 0;
      double RightPower = 0;

      double LeftError = TARGET_VALUEZ - RulerL.distance(distanceUnits::cm);
      double RightError = TARGET_VALUEZ - RulerR.distance(distanceUnits::cm);
      if (PrevLeftError == 0) {
        LeftDerivative = 0;
      } else {
        LeftDerivative = LeftError - PrevLeftError;
      }
      PrevLeftError = LeftError;

      if (PrevRightError == 0) {
        RightDerivative = 0;
      } else {
        RightDerivative = RightError - PrevRightError;
      }
      PrevRightError = RightError;

      LeftPower = LeftError * kP + RightDerivative * kD;
      RightPower = RightError * kP + RightDerivative * kD;

      // derivative goes here

      FRDrive.spin(directionType::fwd, minMax(RightPower, maxSpeed), velocityUnits::pct);
      FLDrive.spin(directionType::fwd, minMax(LeftPower, maxSpeed), velocityUnits::pct);
      BRDrive.spin(directionType::fwd, minMax(RightPower, maxSpeed), velocityUnits::pct);
      BLDrive.spin(directionType::fwd, minMax(LeftPower, maxSpeed), velocityUnits::pct);

      task::sleep(100);
    }
  } else if (TARGET_VALUEX != 0) {
    while (RulerS.distance(distanceUnits::cm) > TARGET_VALUEX + closeness ||
           RulerS.distance(distanceUnits::cm) < TARGET_VALUEX - closeness) {
      // proportional component below
      double SidePower = 0;

      double SideError = TARGET_VALUEX - RulerS.distance(distanceUnits::cm);
      if (PrevSideError == 0) {
        SideDerivative = 0;
      } else {
        SideDerivative = SideError - PrevSideError;
      }
      PrevSideError = SideError;
      SidePower = SideError * kP + SideDerivative * kD;

      // derivative goes here

      FRDrive.spin(directionType::fwd, minMax(SidePower, maxSpeed), velocityUnits::pct);
      FLDrive.spin(directionType::fwd, minMax(-SidePower, maxSpeed), velocityUnits::pct);
      BRDrive.spin(directionType::fwd, minMax(-SidePower, maxSpeed), velocityUnits::pct);
      BLDrive.spin(directionType::fwd, minMax(SidePower, maxSpeed), velocityUnits::pct);

      task::sleep(25);
    }
  }

  FRDrive.stop();
  BRDrive.stop();
  FLDrive.stop();
  BLDrive.stop();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user ode here.
  int direction = 1; //-1 for left

  /*ArmL.spinFor(-0.1,rotationUnits::rev, 50, velocityUnits::pct,false);
  ArmR.spinFor(-0.1,rotationUnits::rev, 50, velocityUnits::pct,true);*/

  /*FRDrive.spinFor(1,rotationUnits::rev, 100, velocityUnits::pct,false);
  BRDrive.spinFor(1,rotationUnits::rev, 100, velocityUnits::pct,false);
  FLDrive.spinFor(1,rotationUnits::rev, 100, velocityUnits::pct,false);
  BLDrive.spinFor(1,rotationUnits::rev, 100, velocityUnits::pct,true);*/

  /*IntakeOne.stop();
  IntakeTwo.stop();*/

  float FIRST_POINT = 18;

  /*ArmL.startSpinFor(directionType::fwd, 1, rotationUnits::rev);
  ArmR.startSpinFor(directionType::fwd, 1, rotationUnits::rev); */

  FRDrive.setStopping(brakeType::coast);
  BRDrive.setStopping(brakeType::coast);
  FLDrive.setStopping(brakeType::coast);
  BLDrive.setStopping(brakeType::coast);

  IntakeOne.setStopping(brakeType::hold);
  IntakeTwo.setStopping(brakeType::hold);

  ArmR.setStopping(brakeType::hold);
  ArmL.setStopping(brakeType::hold);

  IntakeOne.spin(directionType::rev, 100, velocityUnits::pct);
  IntakeTwo.spin(directionType::rev, 100, velocityUnits::pct);

  FRDrive.spinFor(0.693, rotationUnits::rev, 90, velocityUnits::pct, false);
  BRDrive.spinFor(0.693, rotationUnits::rev, 90, velocityUnits::pct, false);
  FLDrive.spinFor(0.693, rotationUnits::rev, 90, velocityUnits::pct, false);
  BLDrive.spinFor(0.693, rotationUnits::rev, 90, velocityUnits::pct, true);

  FRDrive.spinFor(-0.1155, rotationUnits::rev, 70, velocityUnits::pct, false);
  BRDrive.spinFor(-0.1155, rotationUnits::rev, 70, velocityUnits::pct, false);
  FLDrive.spinFor(-0.1155, rotationUnits::rev, 70, velocityUnits::pct, false);
  BLDrive.spinFor(-0.1155, rotationUnits::rev, 70, velocityUnits::pct, true);


  // flips out intake
  PuppetMaster.Screen.print("start lmao");

  

  NewPID(0, 16, 2.96, 0.648, 0.5, 100);
  task::sleep(220);

  IntakeOne.stop();
  IntakeTwo.stop();

  ArmL.startSpinFor(directionType::rev, 0.87, rotationUnits::rev, 80, velocityUnits::pct);
  ArmR.startSpinFor(directionType::rev, 0.87, rotationUnits::rev, 80, velocityUnits::pct);

  task::sleep(100);

  NewPID(0, 65, 2.8, -0.6, 4.5, 58);
  NewPID(94, 77, 2.4, 0, 1, 40);

  IntakeOne.spin(directionType::rev, 100, velocityUnits::pct);
  IntakeTwo.spin(directionType::rev, 100, velocityUnits::pct);

  ArmL.startSpinFor(directionType::fwd, 0.92, rotationUnits::rev, 15, velocityUnits::pct);
  ArmR.spinFor(directionType::fwd, 0.92, rotationUnits::rev, 15, velocityUnits::pct);

  task::sleep(150);

  IntakeOne.stop();
  IntakeTwo.stop();

  ArmL.startSpinFor(directionType::rev, 0.2, rotationUnits::rev, 60, velocityUnits::pct);
  ArmR.startSpinFor(directionType::rev, 0.2, rotationUnits::rev, 60, velocityUnits::pct);

  FRDrive.spinFor(1.155, rotationUnits::rev, 80, velocityUnits::pct, false);
  BRDrive.spinFor(1.155, rotationUnits::rev, 80, velocityUnits::pct, false);
  FLDrive.spinFor(-1.155, rotationUnits::rev, 80, velocityUnits::pct, false);
  BLDrive.spinFor(-1.155, rotationUnits::rev, 80, velocityUnits::pct, true);

  task::sleep(350);

  FRDrive.spinFor(7.854, rotationUnits::rev, 100, velocityUnits::pct, false);
  BRDrive.spinFor(-3.465, rotationUnits::rev, 50, velocityUnits::pct, false);
  FLDrive.spinFor(-3.465, rotationUnits::rev, 50, velocityUnits::pct, false);
  BLDrive.spinFor(7.854, rotationUnits::rev, 100, velocityUnits::pct, true);

  task::sleep(100);

  FRDrive.spinFor(2.31, rotationUnits::rev, 90, velocityUnits::pct, false);
  BRDrive.spinFor(2.31, rotationUnits::rev, 90, velocityUnits::pct, false);
  FLDrive.spinFor(2.31, rotationUnits::rev, 90, velocityUnits::pct, false);
  BLDrive.spinFor(2.31, rotationUnits::rev, 90, velocityUnits::pct, true);

  task::sleep(400);
 
  IntakeOne.spin(directionType::fwd, 100, velocityUnits::pct);
  IntakeTwo.spin(directionType::fwd, 100, velocityUnits::pct);

  task::sleep(300);

  ArmL.startSpinFor(directionType::rev, 1, rotationUnits::rev, 30, velocityUnits::pct);
  ArmR.spinFor(directionType::rev, 1, rotationUnits::rev, 30, velocityUnits::pct);

  IntakeOne.stop();
  IntakeTwo.stop();


 

  PuppetMaster.Screen.clearLine();
  PuppetMaster.Screen.print("end");



  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  int BRVelocity = 0; // initializing velocities
  int FRVelocity = 0;
  int BLVelocity = 0;
  int FLVelocity = 0;

  int MAX_SPEED = 70;

  int preciseSpeedX = 0;
  int preciseSpeedY = 0;

  ArmL.setStopping(brakeType::hold); // setting stoppingtypes for later uses of motor.stop()
  ArmR.setStopping(brakeType::hold);

  IntakeOne.setStopping(brakeType::hold);
  IntakeTwo.setStopping(brakeType::hold);

  /*PuppetMaster.ButtonR2.pressed(R1Pressed); //setting callbacks for button
  presses PuppetMaster.ButtonR1.pressed(R2Pressed);*/

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    preciseSpeedX = 0;
    preciseSpeedY = 0;

    PuppetMaster.Screen.clearLine();
    PuppetMaster.Screen.print(RulerS.distance(distanceUnits::cm));
    PuppetMaster.Screen.print(RulerL.distance(distanceUnits::cm));

    if (PuppetMaster.ButtonDown.pressing() == true) {
      preciseSpeedY -= 10;
    }
    if (PuppetMaster.ButtonUp.pressing() == true) {

      preciseSpeedY += 10;
    }
    if (PuppetMaster.ButtonLeft.pressing() == true) {
      preciseSpeedX -= 10;
    }
    if (PuppetMaster.ButtonRight.pressing() == true) {
      preciseSpeedX += 10;
    }

    FLVelocity = sigmoid_map[(int)minMax(PuppetMaster.Axis3.value() +
                                             PuppetMaster.Axis4.value() +
                                             PuppetMaster.Axis1.value(),
                                         127) +
                             127] +
                 preciseSpeedX + preciseSpeedY;
    FRVelocity = sigmoid_map[(int)minMax(PuppetMaster.Axis3.value() -
                                             PuppetMaster.Axis4.value() -
                                             PuppetMaster.Axis1.value(),
                                         127) +
                             127] -
                 preciseSpeedX + preciseSpeedY;

    BRVelocity = sigmoid_map[(int)minMax(PuppetMaster.Axis3.value() +
                                             PuppetMaster.Axis4.value() -
                                             PuppetMaster.Axis1.value(),
                                         127) +
                             127] +
                 preciseSpeedX + preciseSpeedY;
    BLVelocity = sigmoid_map[(int)minMax(PuppetMaster.Axis3.value() -
                                             PuppetMaster.Axis4.value() +
                                             PuppetMaster.Axis1.value(),
                                         127) +
                             127] -
                 preciseSpeedX + preciseSpeedY;

    FRDrive.spin(directionType::fwd, FRVelocity, velocityUnits::pct);
    BRDrive.spin(directionType::fwd, BRVelocity, velocityUnits::pct);

    FLDrive.spin(directionType::fwd, FLVelocity, velocityUnits::pct);
    BLDrive.spin(directionType::fwd, BLVelocity, velocityUnits::pct);

    if (PuppetMaster.ButtonL2.pressing()) {
      ArmL.spin(directionType::fwd, 60, velocityUnits::pct);
      ArmR.spin(directionType::fwd, 60, velocityUnits::pct);
    } else if (PuppetMaster.ButtonL1.pressing()) {
      ArmL.spin(directionType::rev, 100, velocityUnits::pct);
      ArmR.spin(directionType::rev, 100, velocityUnits::pct);
    } else {
      ArmL.stop();
      ArmR.stop();
    }

    if (PuppetMaster.ButtonR2.pressing()) {
      IntakeOne.spin(directionType::fwd, 75, velocityUnits::pct);
      IntakeTwo.spin(directionType::fwd, 75, velocityUnits::pct);
    } else if (PuppetMaster.ButtonR1.pressing()) {
      IntakeOne.spin(directionType::rev, 80, velocityUnits::pct);
      IntakeTwo.spin(directionType::rev, 80, velocityUnits::pct);
    } else {
      IntakeOne.stop();
      IntakeTwo.stop();
    }

    if (PuppetMaster.ButtonA.pressing()) {
      IntakeOne.spin(directionType::fwd, 97, velocityUnits::pct);
      IntakeTwo.spin(directionType::fwd, 97, velocityUnits::pct);

      ArmL.spin(directionType::rev, 35, velocityUnits::pct);
      ArmR.spin(directionType::rev, 35, velocityUnits::pct);
    }
    if (PuppetMaster.ButtonB.pressing()) {
      IntakeOne.spin(directionType::rev, 97, velocityUnits::pct);
      IntakeTwo.spin(directionType::rev, 97, velocityUnits::pct);

      ArmL.spin(directionType::fwd, 33, velocityUnits::pct);
      ArmR.spin(directionType::fwd, 33, velocityUnits::pct);
    }

    /*
    if (suck == true) {
      IntakeOne.spin(directionType::rev, 50, vex::velocityUnits::pct);
      IntakeTwo.spin(directionType::rev, 50, vex::velocityUnits::pct);
    } else if (out == true) {
      IntakeOne.spin(directionType::fwd, 50, vex::velocityUnits::pct);
      IntakeTwo.spin(directionType::fwd, 50, vex::velocityUnits::pct);
    } else {
      IntakeOne.stop();
      IntakeTwo.stop();
    }
    */

    // ........................................................................

    vex::task::sleep(20); // Sleep the task for a short amount of time to
                          // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (1) {
    vex::task::sleep(100); // Sleep the task for a short amount of time to
                           // prevent wasted resources.
  }
}
