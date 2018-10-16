#ifndef R3FGRIPPER_H
#define R3FGRIPPER_H

#include <ros/ros.h>

class R3FGRos;
class R3FGNetwork;
class R3FGripper
{
  public:
    R3FGripper();
    void run();

  private:
    R3FGRos* p_3fg_ros;
    R3FGNetwork* p_3fg_net;
    ros::NodeHandle p_ros_node;
};

#endif // R3FGRIPPER_H
