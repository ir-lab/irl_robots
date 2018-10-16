#include "r3fgros.h"

#include "r3fgnetwork.h"

R3FGRos::R3FGRos(ros::NodeHandle* _rn, R3FGNetwork* _n) :
  m_ros_node(_rn),
  m_gripper_net(_n)
{
  m_sub_control = m_ros_node->subscribe("/r3fg/control", 1, &R3FGRos::callbackGripperControl, this);
  m_sub_control2 = m_ros_node->subscribe("/r3fg/simplecontrol", 1, &R3FGRos::callbackSimpleControl, this);
  m_ros_thread = new std::thread(&R3FGRos::rosMainLoop, this);
}

void R3FGRos::waitForRos()
{
  m_ros_thread->join();
}

void R3FGRos::callbackGripperControl(irl_robots::r3fgControl c)
{
  m_gripper_net->sendCommand(c);
}

void R3FGRos::callbackSimpleControl(irl_robots::gSimpleControl c)
{
  m_gripper_net->sendCommand(c);
}

void R3FGRos::rosMainLoop()
{
  ROS_INFO("Launching ROS main loop");

  ros::spin();
}
