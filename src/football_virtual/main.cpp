#include <ros/ros.h>
#include <iostream>

#include "football.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irl_football_virtual");

  ros::NodeHandle handle;

  Football ball = Football(handle);
  ball.run();

  return 0;
}
