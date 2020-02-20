using namespace vex;

extern brain Brain;

// VEXcode devices
extern sonar RulerY;
extern sonar RulerF;
extern sonar RulerS;
extern inertial Inertial2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );