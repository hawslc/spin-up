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
// LeftMotors           motor_group   3, 4            
// RightMotors          motor_group   5, 6            
// RightEncoder         encoder       A, B            
// LeftEncoder          encoder       C, D            
// ---- END VEXCODE CONFIGURED DEVICES ----

//neccessary stuff
#include "vex.h"
#include <iostream>  
#include <cmath>  

using namespace vex;
competition Competition;

//global variables
int flywheelLastPos = 0; //the last position of the flywheel in degrees, based on the internal motor encoders
int flywheelTargetRPM = 2900; //the target RPM of the flywheel
bool flywheelOn = false; //true or false, whether or not the flywheel is on right now
bool auton = false; //whether or not the robot is being controlled autonomously
float driveSpeed = 1.0f; //driving (forward and backward) speed of the robot
float turnSpeed = .7f; //turning speed of the robot

//odometry constants
float pos[2] = {0, 0}; //x and y global position of the robot
float angle = 0; //the angle of the robot relative to the starting angle, in radians

float sideWheelRadius = 4.8; //in inches, the distance between each side wheel and the tracking center
int prevLeftEncoder = 0;//the previous reading of the left encoder, in degrees
int prevRightEncoder = 0;//the previous reading of the right encoder, in degrees


//helper functions

void printController(float i) {
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

//graphing data, can be used for flywheel velocity control or PID tuning
void graphData(int target, int data[], int size, int totalTime, int powerHistory[], int finalError) {
 //setup: draw x-axis and y-axis, as well as the target line
 Brain.Screen.clearScreen();
 Brain.Screen.setPenWidth(5);
 Brain.Screen.setPenColor(white);
 Brain.Screen.setFillColor(transparent);
 Brain.Screen.setCursor(1, 1);
 Brain.Screen.print("Time: ");
 Brain.Screen.print(totalTime);
 Brain.Screen.print("   Final error: ");
 Brain.Screen.print(finalError);
 //draw the axises
 Brain.Screen.drawLine(20, 220, 460, 220);
 Brain.Screen.drawLine(20, 220, 20, 20);
 Brain.Screen.setPenWidth(2);
 Brain.Screen.setPenColor(orange);
 Brain.Screen.drawLine(20, 60, 460, 60);
 Brain.Screen.setPenWidth(4);
 Brain.Screen.setPenColor(green);
 //y positions
 //bottom (0) is 215
 //top (100) is 60
 //above (110) (overshoot) is <60
 int minY = 60;
 int maxY = 210;
 //for x, start at 30 and end at 450
 int minX = 30;
 int maxX = 450;
 float horizontalDisplacement = (float)(maxX - minX) / size;
 for (int i = 0; i < size - 1; i++) {
   //error is the difference between where the robot is and where it needs to be
   //each error value in the array, graph it
 
   //error graph (green) (position)
   int x = minX + horizontalDisplacement * i;
   int y = ((float)data[i] / target) * (maxY - minY) + minY;
   Brain.Screen.setFillColor(green);
   Brain.Screen.setPenColor(green);
   Brain.Screen.drawCircle(x, y, 4);
 
   //motor power graph (cyan) (change in position)
   Brain.Screen.setFillColor(cyan);
   Brain.Screen.setPenColor(cyan);
   y = (1 - ((float)powerHistory[i] / 100)) * (maxY - minY) + minY;
   Brain.Screen.drawCircle(x, y, 4);
 }
}


/*template for a thread/task (task is a higher level abstraction of thread):
int myTaskCallback() {
  while (true) {
    //do something
    wait(25, msec);
  }
  // A task's callback must return an int, even though the code will never get
  // here. You must return an int here. Tasks can exit, but this one does not.
  return 0;
}

in some other function like main():

task myTask = task(myTaskCallback);

*/

int updatePosition() {
  while (true) {
    //odometry, yay!
    //caclulate the robot's absolute position

    //first calculate the change since last time
    int changeLeft = LeftEncoder.position(degrees) - prevLeftEncoder;
    int changeRight = RightEncoder.position(degrees) - prevRightEncoder;
    prevLeftEncoder = LeftEncoder.position(degrees);
    prevRightEncoder = RightEncoder.position(degrees);

    //convert the changes to inches
    //degrees * (1 revolution / 360 degrees) * (pi * 2.75" (diameter of wheel) / 1 revolution) = inches
    float distLeft = (float)changeLeft / 360 * M_PI * 2.75;
    float distRight = (float)changeRight / 360 * M_PI * 2.75;

    //calculates the change in angle according to an equation derived in the notebook
    float changeInAngle = (distLeft - distRight) / (sideWheelRadius * 2);

    float newAngle = angle + changeInAngle;

    //now calculate the change in translational offset
    //y is forward, so a changeY of +2.5 means the robot moved directly forward 2.5 inches
    //x is sideways, right is positive.
    float changeX = 0; //we need a third tracking wheel for this
    float changeY = 0; //set to zero initially

    //we avoid a divide by zero error by splitting the code based on whether the change in angle is zero or not
    if (changeInAngle == 0) {
      changeY = distRight; //there was no turning, so the lateral change in position is interepereted simply
    } else {
      //now you have to account for the fact that going forward while turning changes the position differently
      changeY = 2 * sin(changeInAngle / 2) * ((distRight / changeInAngle) + sideWheelRadius);
    }

    //now convert local position change to global position change
    //the local coordinate system is offset from the global one by (angle + newAngle) / 2
    //first convert to polar coordinates
    float radius = sqrt(changeY * changeY + changeX * changeX);
    float globalAngle = (angle + newAngle) / 2 * -1; //times negative one because we rotate the local coordinates "back" into the global coordinates
    //now convert back to local coordinates
    float changeGlobalX = cos(globalAngle) * radius;
    float changeGlobalY = sin(globalAngle) * radius;

    pos[0] += changeGlobalX;
    pos[1] += changeGlobalY;
    angle += changeInAngle;
    //controller stuff for debugging
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(pos[0]);
    Controller1.Screen.newLine();
    Controller1.Screen.print(pos[1]);
    Controller1.Screen.newLine();
    Controller1.Screen.print(angle * 180 / M_PI);

    wait(10, msec);
  }
  return 0;
}

int flywheelVelocityControl(){
  int flywheelLastPos = 0; //the last position of the flywheel in degrees, based on the internal motor encoders
  int prevError = flywheelTargetRPM;
  double gain = 0.00001; //think of this as kP in a PID loop
  double motorPower = 0; //between 0 and 1, the ratio of motor power applied

  //this task keeps track of the velocity of the flywheel and manages the motor power input
  while (true) {
    //first, calculate the current velocity of the flywheel
    int change = Flywheel.position(degrees) - flywheelLastPos;
    flywheelLastPos = Flywheel.position(degrees);
    //the variable change has units of degrees / 20 ms
    //which need to be converted to revolutions / minute
    //so, divide X degrees / 20 ms by 20 to get X degrees / ms
    //multiply that by 60000ms / 1 min to get degrees / minute
    //multiply by 1 revolution / 360 degrees to get RPM.
    int speed = (int)((float) change / 20 * 60000 / 360);

    printController(speed);

    //now for the tbh (take back half) algorithm
    //similar to PID but minimizes overshoot

    int error = flywheelTargetRPM - speed;
    motorPower += gain * error;

    ///keep motor power variable in proper range
    if (motorPower > 1) motorPower = 1;
    if (motorPower < 0) motorPower = 0;
    

    

    wait(20, msec);
  }

  return 0;
}

void driveCode() {
  //drives the robot around based on controller input, double arcade controls

  //don't drive if the robot is currently being controlled autonomously
  if (auton) return;

  LeftMotors.spin(forward);
  RightMotors.spin(forward);
  
  int forward = Controller1.Axis3.value();
  int turn = Controller1.Axis1.value();

  //fix drift, or not
  //if (std::abs(forward) < 7) forward = 0;
  //if (std::abs(turn) < 7) turn = 0;
  
  //calculate proper motor powers
  int left = forward * driveSpeed + turn * turnSpeed;
  int right = forward * driveSpeed - turn * turnSpeed;

  //set velocity of drive motors
  LeftMotors.setVelocity(left, percent);
  RightMotors.setVelocity(right, percent);
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
  Flywheel.spin(forward, 12, volt);
  flywheelOn = !flywheelOn;
}

void buttonXReleased() {
  Flywheel.stop();
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

  //other
  task myTask = task(updatePosition);
}

void pre_auton(void) {
  init();
}

void usercontrol(void) {
  while (1) {
    driveCode();

    //testing with encoders
    /*
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(RightEncoder.position(degrees));
    Controller1.Screen.newLine();
    Controller1.Screen.print(LeftEncoder.position(degrees));
    */
    wait(20, msec); 
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function, basically initialize
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(20, msec);
  }
}
