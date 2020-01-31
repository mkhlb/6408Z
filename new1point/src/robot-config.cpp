#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
sonar RulerL = sonar(Brain.ThreeWirePort.A);
sonar RulerR = sonar(Brain.ThreeWirePort.C);
sonar RulerS = sonar(Brain.ThreeWirePort.G);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}