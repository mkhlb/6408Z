// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RulerY               sonar         A, B            
// RulerF               sonar         C, D            
// RulerS               sonar         E, F            
// Inertial2            inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RulerY               sonar         A, B            
// RulerF               sonar         C, D            
// RulerS               sonar         E, F            
// Inertial2            inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RulerY               sonar         A, B            
// RulerF               sonar         C, D            
// RulerS               sonar         G, H            
// Inertial2            inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RulerY               sonar         A, B            
// RulerF               sonar         C, D            
// RulerS               sonar         E, F            
// Inertial2            inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RulerL               sonar         A, B            
// RulerF               sonar         C, D            
// RulerS               sonar         E, F            
// Inertial2            inertial      16              
// ---- END VEXCODE CONFIGURED DEVICES ----
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

motor ArmR(PORT1, gearSetting::ratio36_1, true);
motor ArmL(PORT20, gearSetting::ratio36_1, false);

motor IntakeOne(PORT18, gearSetting::ratio18_1, true); // right
motor IntakeTwo(PORT9, gearSetting::ratio18_1, false);

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
  PuppetMaster.Screen.clearLine();
  //PuppetMaster.Screen.print("calibrating...");

  Inertial2.calibrate();
  wait(3, timeUnits::sec);

  PuppetMaster.Screen.clearLine();
  //PuppetMaster.Screen.print("done calibrating");
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

void PIDTurn(double TARGET_ROTATION, double kP, double kI, double kD,
             double ACCEPTABLE_ERROR, int TARGET_TICKS, double MAX_I,
             double MAX_SPEED) {
  double Error = 0;
  double ClockWiseError = 0;
  double CounterWiseError = 0;
  double Integral;
  int TicksAtTarget = 0;
  while (TARGET_TICKS > TicksAtTarget) {

    if (TARGET_ROTATION < Inertial2.heading()) {
      ClockWiseError = 360 - Inertial2.heading() + TARGET_ROTATION;
      CounterWiseError = Inertial2.heading() - TARGET_ROTATION;
    } else {
      ClockWiseError = TARGET_ROTATION - Inertial2.heading();
      CounterWiseError = Inertial2.heading() + (360 - TARGET_ROTATION);
    }

    if (ClockWiseError < CounterWiseError) {

      Error = ClockWiseError;
      Integral = Integral + Error;
      if (Error <= 0) {
        Integral = 0;
      }
      if (Integral > MAX_I) {
        Integral = MAX_I;
      }

      FRDrive.spin(directionType::fwd, -(Error * kP + Integral * kI),
                   velocityUnits::pct);
      FLDrive.spin(directionType::fwd, Error * kP + Integral * kI,
                   velocityUnits::pct);
      BLDrive.spin(directionType::fwd, Error * kP + Integral * kI,
                   velocityUnits::pct);
      BRDrive.spin(directionType::fwd, -(Error * kP + Integral * kI),
                   velocityUnits::pct);
    } else {
      Error = CounterWiseError;
      Integral = Integral + Error;
      if (Error <= 0) {
        Integral = 0;
      }
      if (Integral > MAX_I) {
        Integral = MAX_I;
      }
      FRDrive.spin(directionType::fwd,
                   minMax(Error * kP + Integral * kI, MAX_SPEED),
                   velocityUnits::pct);
      FLDrive.spin(directionType::fwd,
                   -minMax(Error * kP + Integral * kI, MAX_SPEED),
                   velocityUnits::pct);
      BLDrive.spin(directionType::fwd,
                   -minMax(Error * kP + Integral * kI, MAX_SPEED),
                   velocityUnits::pct);
      BRDrive.spin(directionType::fwd,
                   minMax(Error * kP + Integral * kI, MAX_SPEED),
                   velocityUnits::pct);
    }
    PuppetMaster.Screen.print(Error);
    if (Inertial2.heading() < TARGET_ROTATION + ACCEPTABLE_ERROR &&
        Inertial2.heading() > TARGET_ROTATION - ACCEPTABLE_ERROR) {
      TicksAtTarget++;
      /*BRDrive.stop();
      FRDrive.stop();
      BLDrive.stop();
      FLDrive.stop(); */
    } else {
      TicksAtTarget = 0;
    }

    task::sleep(20);
  }
  BRDrive.stop();
  FRDrive.stop();
  BLDrive.stop();
  FLDrive.stop();
}

void PIDDrive(double TARGET_ROTATION, double TARGET_Y, double kZ, double kP,
              double kI, double kD, double ACCEPTABLE_ERROR, int TARGET_TICKS,
              double MAX_I, double MAX_SPEED) {
  double LeftError = 0;
  double RightError = 0;
  double ClockWiseError = 0;
  double CounterWiseError = 0;
  double YError = 0;
  double LeftIntegral = 0;
  double RightIntegral = 0;
  double LeftDerivative = 0;
  double RightDerivative = 0;
  double PrevLeftError = 0;
  double PrevRightError = 0;
  int TicksAtTarget = 0;

  while (TARGET_TICKS > TicksAtTarget) {
    // figure out clockwise and counterclock error
    if (TARGET_ROTATION < Inertial2.heading()) {
      ClockWiseError = 360 - Inertial2.heading() + TARGET_ROTATION;
      CounterWiseError = Inertial2.heading() - TARGET_ROTATION;
    } else {
      ClockWiseError = TARGET_ROTATION - Inertial2.heading();
      CounterWiseError = Inertial2.heading() + (360 - TARGET_ROTATION);
    }

    // integral for left and right

    LeftIntegral = LeftIntegral + LeftError;
    if (LeftError <= 0) {
      LeftIntegral = 0;
    }
    RightIntegral = RightIntegral + RightError;
    if (RightError <= 0) {
      RightIntegral = 0;
    }
    if (LeftIntegral > MAX_I) {
      LeftIntegral = MAX_I;
    }
    if (RightIntegral > MAX_I) {
      RightIntegral = MAX_I;
    }
    YError = TARGET_Y - RulerY.distance(distanceUnits::cm);
    PuppetMaster.Screen.clearLine();
    PuppetMaster.Screen.print(
        YError); // Decide to move counter clockwise or clockwise

    if (ClockWiseError < CounterWiseError) {
      LeftError = YError + ClockWiseError * kZ;
      RightError = YError - ClockWiseError * kZ;
      PuppetMaster.Screen.clearLine();
      PuppetMaster.Screen.print(ClockWiseError);
    } else {
      RightError = YError + CounterWiseError * kZ;
      LeftError = YError - CounterWiseError * kZ;
      PuppetMaster.Screen.clearLine();
      PuppetMaster.Screen.print(CounterWiseError);
    }

    // derivative

    LeftDerivative = LeftError - PrevLeftError;
    RightDerivative = RightError - PrevRightError;

    PrevLeftError = LeftError;
    PrevRightError = RightError;

    // drive
    FRDrive.spin(
        directionType::fwd,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);
    FLDrive.spin(
        directionType::fwd,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);
    BLDrive.spin(
        directionType::fwd,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);
    BRDrive.spin(
        directionType::fwd,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);

    // check if at target and heightens TicksAtTarget
    if (RulerY.distance(distanceUnits::cm) < TARGET_Y + ACCEPTABLE_ERROR &&
        RulerY.distance(distanceUnits::cm) > TARGET_Y - ACCEPTABLE_ERROR) {
      TicksAtTarget++;
      /*BRDrive.stop();
      FRDrive.stop();
      BLDrive.stop();
      FLDrive.stop();*/
      // LeftDerivative = 0;
      // RightDerivative = 0;
    } else {
      TicksAtTarget = 0;
    }

    task::sleep(20);
  }
  BRDrive.stop();
  FRDrive.stop();
  BLDrive.stop();
  FLDrive.stop();
}

void PIDSide(double TARGET_ROTATION, double TARGET_Y, double kZ, double kP,
             double kI, double kD, double ACCEPTABLE_ERROR, int TARGET_TICKS,
             double MAX_I, double MAX_SPEED) {
  double LeftError = 0;
  double RightError = 0;
  double ClockWiseError = 0;
  double CounterWiseError = 0;
  double YError = 0;
  double LeftIntegral = 0;
  double RightIntegral = 0;
  double LeftDerivative = 0;
  double RightDerivative = 0;
  double PrevLeftError = 0;
  double PrevRightError = 0;
  int TicksAtTarget = 0;
  PuppetMaster.Screen.clearLine();
  PuppetMaster.Screen.print(TARGET_ROTATION + ACCEPTABLE_ERROR);
  while (TARGET_TICKS > TicksAtTarget) {
    // figure out clockwise and counterclock error
    if (TARGET_ROTATION < Inertial2.heading()) {
      ClockWiseError = 360 - Inertial2.heading() + TARGET_ROTATION;
      CounterWiseError = Inertial2.heading() - TARGET_ROTATION;
    } else {
      ClockWiseError = TARGET_ROTATION - Inertial2.heading();
      CounterWiseError = Inertial2.heading() + (360 - TARGET_ROTATION);
    }

    // integral for left and right

    LeftIntegral = LeftIntegral + LeftError;
    if (LeftError <= 0) {
      LeftIntegral = 0;
    }
    RightIntegral = RightIntegral + RightError;
    if (RightError <= 0) {
      RightIntegral = 0;
    }
    if (LeftIntegral > MAX_I) {
      LeftIntegral = MAX_I;
    }
    if (RightIntegral > MAX_I) {
      RightIntegral = MAX_I;
    }

    YError = TARGET_Y - RulerS.distance(distanceUnits::cm);

    if (ClockWiseError < CounterWiseError) {
      LeftError = YError + ClockWiseError * kZ;
      RightError = YError - ClockWiseError * kZ;
      PuppetMaster.Screen.clearLine();
      PuppetMaster.Screen.print("clockwise"); // Decide to move counter clockwise or clockwise

    } else {
      RightError = YError + CounterWiseError * kZ;
      LeftError = YError - CounterWiseError * kZ;
      PuppetMaster.Screen.clearLine();
      PuppetMaster.Screen.print("counterclock"); // Decide to move counter clockwise or clockwise
    }

    // derivative
    if (PrevLeftError == 0 && PrevRightError == 0) {
      RightDerivative = 0;
      LeftDerivative = 0;
    } else {
      LeftDerivative = LeftError - PrevLeftError;
      RightDerivative = RightError - PrevRightError;
    }

    PrevLeftError = LeftError;
    PrevRightError = RightError;

    // drive
    FRDrive.spin(
        directionType::fwd,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
                MAX_SPEED),
        velocityUnits::pct);
    FLDrive.spin(
        directionType::fwd,
        -minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);
    BLDrive.spin(
        directionType::fwd,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
                MAX_SPEED),
        velocityUnits::pct);
    BRDrive.spin(
        directionType::fwd,
        -minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);

    // check if at target and heightens TicksAtTarget
    if (RulerS.distance(distanceUnits::cm) < TARGET_Y + ACCEPTABLE_ERROR &&
        RulerS.distance(distanceUnits::cm) > TARGET_Y - ACCEPTABLE_ERROR) {
      TicksAtTarget++;
      /*BRDrive.stop();
      FRDrive.stop();
      BLDrive.stop();
      FLDrive.stop();*/
    } else {
      TicksAtTarget = 0;
    }

    task::sleep(2);
  }
  BRDrive.stop();
  FRDrive.stop();
  BLDrive.stop();
  FLDrive.stop();
}

void PIDSideDrive(double TARGET_ROTATION, double TARGET_Y, double TARGET_X,
                  double kZ, double kP, double kI, double kD,
                  double ACCEPTABLE_ERROR, int TARGET_TICKS, double MAX_I,
                  double MAX_SPEED) {
  double LeftError = 0;
  double RightError = 0;
  double ClockWiseError = 0;
  double CounterWiseError = 0;
  double YError = 0;
  double LeftIntegral = 0;
  double RightIntegral = 0;
  double LeftDerivative = 0;
  double RightDerivative = 0;
  double PrevLeftError = 0;
  double PrevRightError = 0;
  double SideError = 0;
  double SideIntegral = 0;
  double SideDerivative = 0;
  double PrevSideError = 0;
  double SidePower = 0;
  int TicksAtTarget = 0;
  PuppetMaster.Screen.clearLine();
  PuppetMaster.Screen.print(LeftDerivative);
  while (TARGET_TICKS > TicksAtTarget) {
    // figure out clockwise and counterclock error
    if (TARGET_ROTATION < Inertial2.heading()) {
      ClockWiseError = 360 - Inertial2.heading() + TARGET_ROTATION;
      CounterWiseError = Inertial2.heading() - TARGET_ROTATION;
    } else {
      ClockWiseError = TARGET_ROTATION - Inertial2.heading();
      CounterWiseError = Inertial2.heading() + (360 - TARGET_ROTATION);
    }

    // integral for left and right

    LeftIntegral = LeftIntegral + LeftError;
    if (LeftError <= 0) {
      LeftIntegral = 0;
    }
    RightIntegral = RightIntegral + RightError;
    if (RightError <= 0) {
      RightIntegral = 0;
    }
    SideIntegral = SideIntegral + SideError;
    if (SideError <= 0) {
      SideIntegral = 0;
    }
    if (SideIntegral > MAX_I) {
      SideIntegral = MAX_I;
    }
    if (LeftIntegral > MAX_I) {
      LeftIntegral = MAX_I;
    }
    if (RightIntegral > MAX_I) {
      RightIntegral = MAX_I;
    }
    YError = TARGET_Y - RulerY.distance(distanceUnits::cm);
    SideError = -TARGET_X + RulerS.distance(distanceUnits::cm);
    PuppetMaster.Screen.clearLine();
    PuppetMaster.Screen.print(SideError);
    // Decide to move counter clockwise or clockwise

    if (ClockWiseError < CounterWiseError) {
      LeftError = YError + ClockWiseError * kZ;
      RightError = YError - ClockWiseError * kZ;
    } else {
      RightError = YError + CounterWiseError * kZ;
      LeftError = YError - CounterWiseError * kZ;
    }

    // derivative
    if (PrevLeftError == 0 && PrevRightError == 0) {
      RightDerivative = 0;
      LeftDerivative = 0;
    } else {
      LeftDerivative = LeftError - PrevLeftError;
      RightDerivative = RightError - PrevRightError;
    }
    if (PrevSideError == 0) {
      SideDerivative = 0;
    } else {
      SideDerivative = SideError - PrevSideError;
    }

    PrevLeftError = LeftError;
    PrevRightError = RightError;
    PrevSideError = SideError;

    SidePower =
        minMax(kZ * (SideDerivative * kD + SideError * kP + SideIntegral * kI),
               MAX_SPEED);

    // drive
    FRDrive.spin(
        directionType::fwd,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED) -
            SidePower,
        velocityUnits::pct);
    FLDrive.spin(
        directionType::fwd,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED) +
            SidePower,
        velocityUnits::pct);
    BLDrive.spin(
        directionType::fwd,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED) -
            SidePower,
        velocityUnits::pct);
    BRDrive.spin(
        directionType::fwd,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED) +
            SidePower,
        velocityUnits::pct);

    // check if at target and heightens TicksAtTarget
    if (RulerY.distance(distanceUnits::cm) < TARGET_Y + ACCEPTABLE_ERROR &&
        RulerY.distance(distanceUnits::cm) > TARGET_Y - ACCEPTABLE_ERROR &&
        RulerS.distance(distanceUnits::cm) < TARGET_X + ACCEPTABLE_ERROR &&
        RulerS.distance(distanceUnits::cm) > TARGET_X - ACCEPTABLE_ERROR) {
      TicksAtTarget++;
      /*BRDrive.stop();
      FRDrive.stop();
      BLDrive.stop();
      FLDrive.stop();*/
    } else {
      TicksAtTarget = 0;
    }

    task::sleep(2);
  }
  BRDrive.stop();
  FRDrive.stop();
  BLDrive.stop();
  FLDrive.stop();
}

void PIDDriveForward(double TARGET_ROTATION, double TARGET_Y, double kZ, double kP,
              double kI, double kD, double ACCEPTABLE_ERROR, int TARGET_TICKS,
              double MAX_I, double MAX_SPEED) {
  double LeftError = 0;
  double RightError = 0;
  double ClockWiseError = 0;
  double CounterWiseError = 0;
  double YError = 0;
  double LeftIntegral = 0;
  double RightIntegral = 0;
  double LeftDerivative = 0;
  double RightDerivative = 0;
  double PrevLeftError = 0;
  double PrevRightError = 0;
  int TicksAtTarget = 0;

  while (TARGET_TICKS > TicksAtTarget) {
    // figure out clockwise and counterclock error
    if (TARGET_ROTATION < Inertial2.heading()) {
      ClockWiseError = 360 - Inertial2.heading() + TARGET_ROTATION;
      CounterWiseError = Inertial2.heading() - TARGET_ROTATION;
    } else {
      ClockWiseError = TARGET_ROTATION - Inertial2.heading();
      CounterWiseError = Inertial2.heading() + (360 - TARGET_ROTATION);
    }

    // integral for left and right

    LeftIntegral = LeftIntegral + LeftError;
    if (LeftError <= 0) {
      LeftIntegral = 0;
    }
    RightIntegral = RightIntegral + RightError;
    if (RightError <= 0) {
      RightIntegral = 0;
    }
    if (LeftIntegral > MAX_I) {
      LeftIntegral = MAX_I;
    }
    if (RightIntegral > MAX_I) {
      RightIntegral = MAX_I;
    }
    YError = TARGET_Y - RulerF.distance(distanceUnits::cm);
    PuppetMaster.Screen.clearLine();
    PuppetMaster.Screen.print(
        YError); // Decide to move counter clockwise or clockwise

    if (ClockWiseError < CounterWiseError) {
      LeftError = YError + ClockWiseError * kZ;
      RightError = YError - ClockWiseError * kZ;
      PuppetMaster.Screen.clearLine();
      PuppetMaster.Screen.print(ClockWiseError);
    } else {
      RightError = YError + CounterWiseError * kZ;
      LeftError = YError - CounterWiseError * kZ;
      PuppetMaster.Screen.clearLine();
      PuppetMaster.Screen.print(CounterWiseError);
    }

    // derivative

    LeftDerivative = LeftError - PrevLeftError;
    RightDerivative = RightError - PrevRightError;

    PrevLeftError = LeftError;
    PrevRightError = RightError;

    // drive
    FRDrive.spin(
        directionType::rev,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);
    FLDrive.spin(
        directionType::rev,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);
    BLDrive.spin(
        directionType::rev,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);
    BRDrive.spin(
        directionType::rev,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED),
        velocityUnits::pct);

    // check if at target and heightens TicksAtTarget
    if (RulerF.distance(distanceUnits::cm) < TARGET_Y + ACCEPTABLE_ERROR &&
        RulerF.distance(distanceUnits::cm) > TARGET_Y - ACCEPTABLE_ERROR) {
      TicksAtTarget++;
      BRDrive.stop();
      FRDrive.stop();
      BLDrive.stop();
      FLDrive.stop();
      // LeftDerivative = 0;
      // RightDerivative = 0;
    } else {
      TicksAtTarget = 0;
    }

    task::sleep(2);
  }
  BRDrive.stop();
  FRDrive.stop();
  BLDrive.stop();
  FLDrive.stop();
}
void PIDSideDriveForward(double TARGET_ROTATION, double TARGET_Y, double TARGET_X,
                  double kZ, double kP, double kI, double kD,
                  double ACCEPTABLE_ERROR, int TARGET_TICKS, double MAX_I,
                  double MAX_SPEED) {
  double LeftError = 0;
  double RightError = 0;
  double ClockWiseError = 0;
  double CounterWiseError = 0;
  double YError = 0;
  double LeftIntegral = 0;
  double RightIntegral = 0;
  double LeftDerivative = 0;
  double RightDerivative = 0;
  double PrevLeftError = 0;
  double PrevRightError = 0;
  double SideError = 0;
  double SideIntegral = 0;
  double SideDerivative = 0;
  double PrevSideError = 0;
  double SidePower = 0;
  int TicksAtTarget = 0;
  PuppetMaster.Screen.clearLine();
  PuppetMaster.Screen.print(LeftDerivative);
  while (TARGET_TICKS > TicksAtTarget) {
    // figure out clockwise and counterclock error
    if (TARGET_ROTATION < Inertial2.heading()) {
      ClockWiseError = 360 - Inertial2.heading() + TARGET_ROTATION;
      CounterWiseError = Inertial2.heading() - TARGET_ROTATION;
    } else {
      ClockWiseError = TARGET_ROTATION - Inertial2.heading();
      CounterWiseError = Inertial2.heading() + (360 - TARGET_ROTATION);
    }

    // integral for left and right

    LeftIntegral = LeftIntegral + LeftError;
    if (LeftError <= 0) {
      LeftIntegral = 0;
    }
    RightIntegral = RightIntegral + RightError;
    if (RightError <= 0) {
      RightIntegral = 0;
    }
    SideIntegral = SideIntegral + SideError;
    if (SideError <= 0) {
      SideIntegral = 0;
    }
    if (SideIntegral > MAX_I) {
      SideIntegral = MAX_I;
    }
    if (LeftIntegral > MAX_I) {
      LeftIntegral = MAX_I;
    }
    if (RightIntegral > MAX_I) {
      RightIntegral = MAX_I;
    }
    YError = -TARGET_Y + RulerF.distance(distanceUnits::cm);
    SideError = -TARGET_X + RulerS.distance(distanceUnits::cm);
    PuppetMaster.Screen.clearLine();
    PuppetMaster.Screen.print(SideError);
    // Decide to move counter clockwise or clockwise

    if (ClockWiseError < CounterWiseError) {
      LeftError = YError + ClockWiseError * kZ;
      RightError = YError - ClockWiseError * kZ;
    } else {
      RightError = YError + CounterWiseError * kZ;
      LeftError = YError - CounterWiseError * kZ;
    }

    // derivative
    if (PrevLeftError == 0 && PrevRightError == 0) {
      RightDerivative = 0;
      LeftDerivative = 0;
    } else {
      LeftDerivative = LeftError - PrevLeftError;
      RightDerivative = RightError - PrevRightError;
    }
    if (PrevSideError == 0) {
      SideDerivative = 0;
    } else {
      SideDerivative = SideError - PrevSideError;
    }

    PrevLeftError = LeftError;
    PrevRightError = RightError;
    PrevSideError = SideError;

    SidePower =
        minMax(kZ * (SideDerivative * kD + SideError * kP + SideIntegral * kI),
               MAX_SPEED);

    // drive
    FRDrive.spin(
        directionType::fwd,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED) -
            SidePower,
        velocityUnits::pct);
    FLDrive.spin(
        directionType::fwd,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED) +
            SidePower,
        velocityUnits::pct);
    BLDrive.spin(
        directionType::fwd,
        minMax(LeftError * kP + LeftIntegral * kI + LeftDerivative * kD,
               MAX_SPEED) -
            SidePower,
        velocityUnits::pct);
    BRDrive.spin(
        directionType::fwd,
        minMax(RightError * kP + RightIntegral * kI + RightDerivative * kD,
               MAX_SPEED) +
            SidePower,
        velocityUnits::pct);

    // check if at target and heightens TicksAtTarget
    if (RulerF.distance(distanceUnits::cm) < TARGET_Y + ACCEPTABLE_ERROR &&
        RulerF.distance(distanceUnits::cm) > TARGET_Y - ACCEPTABLE_ERROR &&
        RulerS.distance(distanceUnits::cm) < TARGET_X + ACCEPTABLE_ERROR &&
        RulerS.distance(distanceUnits::cm) > TARGET_X - ACCEPTABLE_ERROR) {
      TicksAtTarget++;
      /*BRDrive.stop();
      FRDrive.stop();
      BLDrive.stop();
      FLDrive.stop();*/
    } else {
      TicksAtTarget = 0;
    }

    task::sleep(2);
  }
  BRDrive.stop();
  FRDrive.stop();
  BLDrive.stop();
  FLDrive.stop();
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

  // IntakeOne.stop();
  // IntakeTwo.stop();

  float FIRST_POINT = 18;

  // Inertial2.calibrate();
  task::sleep(3000);

  FRDrive.setStopping(brakeType::hold);
  BRDrive.setStopping(brakeType::hold);
  FLDrive.setStopping(brakeType::hold);
  BLDrive.setStopping(brakeType::hold);

  IntakeOne.setStopping(brakeType::hold);
  IntakeTwo.setStopping(brakeType::hold);

  ArmR.setStopping(brakeType::hold);
  ArmL.setStopping(brakeType::hold);
 
  IntakeOne.spin(directionType::rev, 100, velocityUnits::pct);
  IntakeTwo.spin(directionType::rev, 100, velocityUnits::pct);

  ArmL.startSpinFor(directionType::rev, 0.4, rotationUnits::rev, 80, velocityUnits::pct); 
  ArmR.startSpinFor(directionType::rev, 0.4, rotationUnits::rev, 80, velocityUnits::pct);

  FRDrive.spinFor(0.5, rotationUnits::rev, 90, velocityUnits::pct, false);
  BRDrive.spinFor(0.5, rotationUnits::rev, 90, velocityUnits::pct, false);
  FLDrive.spinFor(0.5, rotationUnits::rev, 90, velocityUnits::pct, false);
  BLDrive.spinFor(0.5, rotationUnits::rev, 90, velocityUnits::pct, true);

  FRDrive.spinFor(-0.1, rotationUnits::rev, 70, velocityUnits::pct, false);
  BRDrive.spinFor(-0.1, rotationUnits::rev, 70, velocityUnits::pct, false);
  FLDrive.spinFor(-0.1, rotationUnits::rev, 70, velocityUnits::pct, false);
  BLDrive.spinFor(-0.1, rotationUnits::rev, 70, velocityUnits::pct, true);

  ArmL.startSpinFor(directionType::fwd, 0.5, rotationUnits::rev, 40, velocityUnits::pct); 
  ArmR.startSpinFor(directionType::fwd, 0.5, rotationUnits::rev, 40, velocityUnits::pct);


  // flips out intake
  PuppetMaster.Screen.print("start lmao");


  PIDDrive(0, 15.5, 0.9, 1, 0, 0.6, 3.5, 4, 40, 100);
  //NewPID(0, 15.5, 2.96, 0.648, 0.8, 100);
  task::sleep(450);

  IntakeOne.stop();
  IntakeTwo.stop();

  ArmL.startSpinFor(directionType::rev, 0.89, rotationUnits::rev, 80, velocityUnits::pct); 
  ArmR.startSpinFor(directionType::rev, 0.89, rotationUnits::rev, 80, velocityUnits::pct);

  task::sleep(30);

  //NewPID(0, 65, 2.8, -0.6, 4.5, 58);
  PIDDrive(0, 65, 0.8, 2, 0, 0, 5, 1, 40, 53);
  //NewPID(94, 76.5, 2.35, -0.3, 1.2, 40);
  //PIDSideDrive(0, 73, 93, 0.59, 0.95, 0.13, 0.9, 4.5, 1, 40, 40);
  PIDSideDrive(0, 76.2, 93, 0.53, 0.75, 0.25, 0.22, 4.9, 2, 15, 40);
  //PIDSideDrive(TARGET_ROTATION, TARGET_Y, TARGET_X, kZ, kP, kI, kD, ACCEPTABLE_ERROR, TARGET_TICKS, MAX_I, MAX_SPEED)

  IntakeOne.spin(directionType::rev, 100, velocityUnits::pct);
  IntakeTwo.spin(directionType::rev, 100, velocityUnits::pct);

  ArmL.startSpinFor(directionType::fwd, 0.92, rotationUnits::rev, 21, velocityUnits::pct); 
  ArmR.spinFor(directionType::fwd, 0.92, rotationUnits::rev, 21, velocityUnits::pct);

  IntakeOne.spin(directionType::rev, 100, velocityUnits::pct);
  IntakeTwo.spin(directionType::rev, 100, velocityUnits::pct);

  task::sleep(150);
 
  IntakeOne.stop();
  IntakeTwo.stop();

  ArmL.startSpinFor(directionType::rev, 0.2, rotationUnits::rev, 60,
                    velocityUnits::pct);
  ArmR.startSpinFor(directionType::rev, 0.2, rotationUnits::rev, 60,
                    velocityUnits::pct);

  PIDDrive(0, 7.5, 0.7, 1.4, 0.1, 0.9, 1.5, 1, 20, 100);  
  
  PIDTurn(90, 1.1, 0, 0.7, 4, 2, 30, 100);
  //PIDTurn(TARGET_ROTATION, kP, kI, kD, ACCEPTABLE_ERROR, TARGET_TICKS, MAX_I, MAX_SPEED)
  IntakeOne.stop();
  IntakeTwo.stop();

  task::sleep(50);

  // PIDSide(270, 3, 2.2, 3.3, 0.1, 0.2, 6, 20, 30, 100);
  PIDSideDriveForward(91, 21.9, 8.5, 0.4, 1.68, 0.1, 0.9, 7.8, 4, 20, 80);
  //PIDSideDriveForward(TARGET_ROTATION, TARGET_Y, TARGET_X, kZ, kP, kI, kD, ACCEPTABLE_ERROR, TARGET_TICKS, MAX_I, MAX_SPEED)
  PuppetMaster.Screen.clearLine();
  PuppetMaster.Screen.print("ayyy");
  PIDTurn(90, 1.4, 0, 0.7, 5.5, 1, 30, 100);
  //PIDDriveForward(TARGET_ROTATION, TARGET_Y, kZ, kP, kI, kD, ACCEPTABLE_ERROR, TARGET_TICKS, MAX_I, MAX_SPEED, ACCEPTABLEROTATION)
  

  //FRDrive.spinFor(1.2, rotationUnits::rev, 100, velocityUnits::pct, false);
  //BRDrive.spinFor(1.2, rotationUnits::rev, 100, velocityUnits::pct, false);
  //FLDrive.spinFor(1.2, rotationUnits::rev, 100, velocityUnits::pct, false);
  //BLDrive.spinFor(1.2, rotationUnits::rev, 100, velocityUnits::pct, true);



  //ArmL.startSpinFor(directionType::fwd, 0.32, rotationUnits::rev, 60,
    //                velocityUnits::pct);
  //ArmR.startSpinFor(directionType::fwd, 0.32, rotationUnits::rev, 60,
  //                  velocityUnits::pct);

  task::sleep(100);

  IntakeOne.spin(directionType::fwd, 100, velocityUnits::pct);
  IntakeTwo.spin(directionType::fwd, 100, velocityUnits::pct);

  task::sleep(500);

  ArmL.startSpinFor(directionType::rev, 0.55, rotationUnits::rev, 33.5,
                        velocityUnits::pct);
  ArmR.spinFor(directionType::rev, 0.55, rotationUnits::rev, 33.5,
                 velocityUnits::pct);
  ArmL.startSpinFor(directionType::rev, 0.49, rotationUnits::rev, 31,
                       velocityUnits::pct);
  ArmR.spinFor(directionType::rev, 0.49, rotationUnits::rev, 31,
                 velocityUnits::pct);
  IntakeOne.stop();
  IntakeTwo.stop();

  PIDTurn(0, 1.1, 0, 0.7, 4, 2, 30, 100);
  
  FRDrive.spinFor(0.5, rotationUnits::rev, 70, velocityUnits::pct, false);
  BRDrive.spinFor(0.5, rotationUnits::rev, 70, velocityUnits::pct, false);
  FLDrive.spinFor(0.5, rotationUnits::rev, 70, velocityUnits::pct, false);
  BLDrive.spinFor(0.5, rotationUnits::rev, 70, velocityUnits::pct, false);

  IntakeOne.spin(directionType::fwd, 100, velocityUnits::pct);
  IntakeTwo.spin(directionType::fwd, 100, velocityUnits::pct);

  ArmL.startSpinFor(directionType::fwd, 1.09, rotationUnits::rev, 60, velocityUnits::pct);
  ArmR.spinFor(directionType::fwd, 1.09, rotationUnits::rev, 60, velocityUnits::pct);
  
  wait(1000, msec);

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

  ArmL.setStopping(
      brakeType::hold); // setting stoppingtypes for later uses of motor.stop()
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
    PuppetMaster.Screen.print(RulerF.distance(distanceUnits::cm));
    //PuppetMaster.Screen.print(RulerY.distance(distanceUnits::cm));

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
                             127] *
                     0.85 +
                 preciseSpeedX + preciseSpeedY;
    FRVelocity = sigmoid_map[(int)minMax(PuppetMaster.Axis3.value() -
                                             PuppetMaster.Axis4.value() -
                                             PuppetMaster.Axis1.value(),
                                         127) +
                             127] *
                     0.85 -
                 preciseSpeedX + preciseSpeedY;

    BRVelocity = sigmoid_map[(int)minMax(PuppetMaster.Axis3.value() +
                                             PuppetMaster.Axis4.value() -
                                             PuppetMaster.Axis1.value(),
                                         127) +
                             127] *
                     0.85 +
                 preciseSpeedX + preciseSpeedY;
    BLVelocity = sigmoid_map[(int)minMax(PuppetMaster.Axis3.value() -
                                             PuppetMaster.Axis4.value() +
                                             PuppetMaster.Axis1.value(),
                                         127) +
                             127] *
                     0.85 -
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

    if (PuppetMaster.ButtonX.pressing()) {
      IntakeOne.spin(directionType::fwd, 100, velocityUnits::pct);
      IntakeTwo.spin(directionType::fwd, 100, velocityUnits::pct);

      ArmL.startSpinFor(directionType::rev, 0.55, rotationUnits::rev, 33.5,
                        velocityUnits::pct);
      ArmR.spinFor(directionType::rev, 0.55, rotationUnits::rev, 33.5,
                 velocityUnits::pct);
      ArmL.startSpinFor(directionType::rev, 0.45, rotationUnits::rev, 32,
                        velocityUnits::pct);
      ArmR.spinFor(directionType::rev, 0.45, rotationUnits::rev, 32,
                 velocityUnits::pct);
      IntakeOne.stop();
      IntakeTwo.stop();

    }
    if (PuppetMaster.ButtonA.pressing()) {
      IntakeOne.spin(directionType::fwd, 97, velocityUnits::pct);
      IntakeTwo.spin(directionType::fwd, 97, velocityUnits::pct);

      ArmL.spin(directionType::rev, 40, velocityUnits::pct);
      ArmR.spin(directionType::rev, 40, velocityUnits::pct);
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
