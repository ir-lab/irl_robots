#include "ur5ros.h"

#include "ur5network.h"

UR5Ros::UR5Ros(ros::NodeHandle* n, UR5Network* urn) :
  p_ros_node(n), p_ur5_com(urn)
{
  p_ur5_subscriber = p_ros_node->subscribe("/ur5/control", 1, &UR5Ros::callbackRobotControl, this);
  p_srv_control = p_ros_node->advertiseService("/ur5/ur5SrvControl", &UR5Ros::callbackControlService, this);

  p_ros_thread = new std::thread(&UR5Ros::rosMainLoop, this);
}

void UR5Ros::waitForRos()
{
  p_ros_thread->join();
}

void UR5Ros::callbackRobotControl(irl_robots::ur5Control c)
{
  p_ur5_com->sendCommand(c);
}

void UR5Ros::rosMainLoop()
{
  ROS_INFO("Launching ROS main loop");

  ros::spin();
}

bool UR5Ros::callbackControlService(irl_robots::ur5SrvControlRequest& req, irl_robots::ur5SrvControlResponse& res)
{
  irl_robots::ur5Control c;
  c.acceleration = 0;
  c.blend = 0;
  c.command = req.command;
  c.gain = 0;
  c.jointcontrol = req.jointcontrol;
  c.lookahead = 0;
  c.time = req.time;
  c.values = req.values;
  c.velocity = 0;
  p_ur5_com->sendCommand(c);

  int rate = 2;
  ros::Rate loop_rate(rate);
  for(int i = 0; i < ceil(req.time * rate); i++)
    loop_rate.sleep();
  return true;
}

