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
    static const double PROPORTIONAL_GAIN;
    static const double INTEGRAL_GAIN;
    static const double DIFFERENTIAL_GAIN;
    static const double MAX_VELOCITY;
    static const double INTEGRAL_MAX;
    static const double INTEGRAL_MIN;
    static const double MAX_ACCELERATION;

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

    double p_integral_state[6];
    double p_differential_state[6];
};

#endif // UR5NETWORK_H
