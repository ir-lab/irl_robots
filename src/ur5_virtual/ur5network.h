#ifndef UR5NETWORK_H
#define UR5NETWORK_H

#include <thread>
#include <ros/ros.h>
#include <queue>
#include <mutex>
#include <irl_robots/ur5Control.h>
#include <irl_robots/ur5Joints.h>
#include <algorithm>

class UR5Network
{
  public:
    UR5Network(ros::NodeHandle* n);
    ~UR5Network();

    void waitForNet();
    void sendCommand(irl_robots::ur5Control com);

  private:
    void netMainLoop();
    void sendNextCommand();
    void readStatus();

    std::thread* p_net_thread;
    ros::NodeHandle* p_ros_node;
    ros::Publisher p_pub_joint;
    ros::Publisher p_pub_tool;
    ros::Publisher p_pub_status;

    std::queue<irl_robots::ur5Control> p_com_queue;
    std::mutex p_com_mutex;
    double p_ur5_mode;
    irl_robots::ur5Joints p_ur5_currjoints;

    int    m_ur5_joint_handles[6];
    int    m_ur5_tool_handle;
    int    m_ur5_handle;
    int    m_client_id;

    double m_p_gain;
    double m_i_gain;
    double m_d_gain;
    double m_max_velocity;
    double m_i_max;
    double m_i_min;
    double m_max_accel;
    double m_joint_offsets[6];

    double p_integral_state[6];
    double p_differential_state[6];
};

#endif // UR5NETWORK_H
