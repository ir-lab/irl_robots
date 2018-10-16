#ifndef UR5CONTROLLER_H
#define UR5CONTROLLER_H

#include <ros/ros.h>

class UR5Network;
class UR5Ros;
class UR5Controller
{
  public:
    UR5Controller();
    void run();

  private:
    UR5Network* p_ur5_net;
    UR5Ros* p_ur5_ros;
    ros::NodeHandle p_ros_node;
};

#endif // UR5CONTROLLER_H
