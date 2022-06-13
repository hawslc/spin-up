/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Flywheel             motor_group   1, 2            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

int lastPos = 0;

//helper functions
void printController(int i) {
 //prints a number to the controller console (very handy for debugging)
 Controller1.Screen.clearLine(3);
 Controller1.Screen.setCursor(3, 1);
 Controller1.Screen.print(i);
}
 
void printControllerSetup() {
 //sets up the controller to be printed on
 Controller1.Screen.clearLine(3);
 Controller1.Screen.setCursor(3, 1);
}

void autonomous(void) {
  
}

void buttonL1Pressed() {
  
}
 
void buttonL2Pressed() {
  
}
 
void buttonR1Pressed() {
  
}
 
void buttonR2Pressed() {
  
}
 
void buttonUpPressed() {
  
}
 
void buttonDownPressed() {
  
}
 
void buttonLeftPressed() {
  
}
 
void buttonRightPressed() {
  
}
 
void buttonXPressed() {
  Flywheel.spin(forward);
  Flywheel.setVelocity(100, percent);
}

void buttonXReleased() {
  Flywheel.spin(forward);
  Flywheel.setVelocity(0, percent);
}
 
void buttonAPressed() {
 
}
 
void buttonBPressed() {
  //prints out a bunch of useful information on the brain screen--a status check
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Battery voltage: ");
  Brain.Screen.print(Brain.Battery.voltage());
  Brain.Screen.newLine();
  Brain.Screen.print("Battery current: ");
  Brain.Screen.print(Brain.Battery.current());
  Brain.Screen.newLine();
}
 
void buttonYPressed() {
 
  //start autonomous if A is held down while pressing Y
  /*
  if (Controller1.ButtonA.pressing()) {
    autonomousProgram();
  } else {
    autonType++;
    if (autonType > 5) autonType = 0;
    printControllerSetup();
    Controller1.Screen.print("Auton type: ");
    Controller1.Screen.print(autonType);
  }*/
}
 
void init() {
  //initialize settings, variables, and callback functions
  Flywheel.setStopping(coast);
  
  //callbacks for other driver controls
  
  //ring mech
  Controller1.ButtonL1.pressed(buttonL1Pressed);
  Controller1.ButtonL2.pressed(buttonL2Pressed);
  
  //front lift
  Controller1.ButtonR1.pressed(buttonR1Pressed);
  Controller1.ButtonR2.pressed(buttonR2Pressed);
  
  //back lift
  Controller1.ButtonUp.pressed(buttonUpPressed);
  Controller1.ButtonDown.pressed(buttonDownPressed);
  
  //pneumatic claw
  Controller1.ButtonLeft.pressed(buttonLeftPressed);
  Controller1.ButtonRight.pressed(buttonRightPressed);
  
  //other functions
  Controller1.ButtonX.pressed(buttonXPressed);
  Controller1.ButtonX.released(buttonXReleased);
  Controller1.ButtonA.pressed(buttonAPressed);
  Controller1.ButtonB.pressed(buttonBPressed);
  Controller1.ButtonY.pressed(buttonYPressed);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  init();
}

void usercontrol(void) {
  while (1) {
    int change = Flywheel.position(degrees) - lastPos;
    lastPos = Flywheel.position(degrees);
    //the variable change has units of degrees / 20 ms
    //which need to be converted to revolutions / minute
    //so, divide X degrees / 20 ms by 20 to get X degrees / ms
    //multiply that by 60000ms / 1 min to get degrees / minute
    //multiply by 1 revolution / 360 degrees to get RPM.
    printController((float) change / 20 * 60000 / 360);
    wait(20, msec); 
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
