#ifndef R3FGROS_H
#define R3FGROS_H

#include <thread>
#include <ros/ros.h>
#include <irl_robots/r3fgControl.h>
#include <irl_robots/gSimpleControl.h>

class R3FGNetwork;
class R3FGRos
{
  public:
    R3FGRos(ros::NodeHandle* _rn, R3FGNetwork* _n);
    void waitForRos();

  private:
    void callbackGripperControl(irl_robots::r3fgControl c);
    void callbackSimpleControl(irl_robots::gSimpleControl c);
    void rosMainLoop();

    std::thread* m_ros_thread;
    ros::Subscriber m_sub_control;
    ros::Subscriber m_sub_control2;
    ros::NodeHandle* m_ros_node;
    R3FGNetwork* m_gripper_net;
};

#endif // R3FGROS_H
