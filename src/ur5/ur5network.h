#ifndef UR5NETWORK_H
#define UR5NETWORK_H

#include <thread>
#include <ros/ros.h>
#include <arpa/inet.h>
#include <queue>
#include <mutex>
#include <irl_robots/ur5Control.h>
#include <irl_robots/ur5Joints.h>
#include <algorithm>

class UR5Network
{
  public:
    UR5Network(ros::NodeHandle* n);
    void waitForNet();
    void sendCommand(irl_robots::ur5Control com);

  private:
    template <typename t>
    void swapByteorder(t* value)
    {
      char* cp = (char*)value;
      std::reverse(cp, cp + sizeof(t));
    }

    void netMainLoop();
    void connectUR5();
    void sendNextCommand();
    void readStatus();
    void parseStatus(char* buffer);

    std::thread* p_net_thread;
    ros::NodeHandle* p_ros_node;
    ros::Publisher p_pub_joint;
    ros::Publisher p_pub_tool;
    ros::Publisher p_pub_status;

    std::string p_ur5_ip;
    int p_ur5_port;
    int p_net_socket;
    struct sockaddr_in p_ur5_socket;
    std::queue<irl_robots::ur5Control> p_com_queue;
    std::mutex p_com_mutex;
    double p_ur5_mode;
    irl_robots::ur5Joints p_ur5_currjoints;

    double m_p_gain;
    double m_i_gain;
    double m_d_gain;
    double m_max_velocity;
    double m_i_max;
    double m_i_min;
    double m_max_accel;

    double p_integral_state[6];
    double p_differential_state[6];
};

#endif // UR5NETWORK_H
