#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class Football
{
public:
    Football(ros::NodeHandle handle);
    ~Football();

    void run();

private:
    ros::Subscriber    m_stateSubscriber;
    int                m_football_handle;
    int                m_client_id;

    void callback_positionUpdate(const geometry_msgs::PoseStamped::ConstPtr pose);
};
