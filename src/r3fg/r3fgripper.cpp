#include "r3fgripper.h"

#include "r3fgnetwork.h"
#include "r3fgros.h"

R3FGripper::R3FGripper()
{
  p_3fg_net = new R3FGNetwork(&p_ros_node);
  p_3fg_ros = new R3FGRos(&p_ros_node, p_3fg_net);
}

void R3FGripper::run()
{
  p_3fg_ros->waitForRos();
  p_3fg_net->waitForNet();
}
