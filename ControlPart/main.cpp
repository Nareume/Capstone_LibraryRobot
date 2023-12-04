#include "ROS2toPID.h"  

Robot myRobot;

void setup() {
  myRobot.setupMicroROS();
  myRobot.setupPIDControllers();
}

void loop() {
  myRobot.updateOdometry();
  myRobot.updateMotorSpeeds();
  myRobot.printDebugInfo();
}
