#include "ur5controller.h"

#include "ur5ros.h"
#include "ur5network.h"

UR5Controller::UR5Controller()
{
  p_ur5_net = new UR5Network(&p_ros_node);
  p_ur5_ros = new UR5Ros(&p_ros_node, p_ur5_net);
}

void UR5Controller::run()
{
  p_ur5_ros->waitForRos();
  p_ur5_net->waitForNet();
}
