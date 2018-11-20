#ifndef UR5ROS_H
#define UR5ROS_H

#include <thread>
#include <ros/ros.h>
#include <irl_robots/ur5Control.h>
#include <irl_robots/ur5SrvControl.h>

class UR5Network;
class UR5Ros
{
  public:
    UR5Ros(ros::NodeHandle* n, UR5Network* urn);
    void waitForRos();

  private:
    void callbackRobotControl(irl_robots::ur5Control c);
    void rosMainLoop();
    bool callbackControlService(irl_robots::ur5SrvControlRequest& req, irl_robots::ur5SrvControlResponse& res);

    std::thread* p_ros_thread;
    ros::NodeHandle* p_ros_node;
    ros::Subscriber p_ur5_subscriber;
    ros::ServiceServer p_srv_control;
    UR5Network* p_ur5_com;
};

#endif // UR5ROS_H
