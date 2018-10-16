#include <ros/ros.h>
#include <iostream>

#include "ur5controller.h"

// rostopic pub --once /ur5/control irl_robots/ur5Control '{command: movej, values: [-0.439, -0.396, 0.325, 0, 0, 0], acceleration: 1.0, velocity: 1.0, time: 10, blend: 0, jointcontrol: true}'

int main(int argc, char **argv)
{
  ros::init(argc, argv, "irl_ur5");

  UR5Controller* ur5 = new UR5Controller();
  ur5->run();

  delete ur5;
  return 0;
}
