#include <ros/ros.h>
#include <iostream>

#include "r2fgripper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irl_r2fg");

  R2FGripper* gripper = new R2FGripper();
  gripper->connectGripper();

  delete gripper;
  return 0;
}
