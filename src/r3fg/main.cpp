#include <ros/ros.h>
#include <iostream>

#include "r3fgripper.h"

// rostopic pub --once /r3fg/control irl_robots/r3fgControl '{activate: true, go: true, mode: 0, autorelease: false, glove: false, individualfinger: false, individualscissor: false, finger: [{speed: 0, force: 0, position: 255}, {speed: 0, force: 0, position: 255}, {speed: 0, force: 0, position: 255}], scissor: {speed: 0, force: 0, position: 255}}'

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irl_r3fg");

  R3FGripper* gripper = new R3FGripper();
  gripper->run();

  delete gripper;
  return 0;
}
