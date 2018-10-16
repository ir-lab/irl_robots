#pragma once

#include "ros/ros.h"

#include <mutex>

#include <irl_robots/matrix.h>
#include <irl_robots/ur5Control.h>
#include <irl_robots/ur5Joints.h>

// Design control so that it receives a trajectory over a topic its listening to.
// The message will be the irl matrix message or whatever is returned by generating a probable trajectory.
// It will push this trajectory into its state and maintain what step it is currently on.
// It will also subscribe to the UR5 joint states and maintain what teh current state is.

// Start at trajectory step 0.
// Push a speedj to ur5control with that position.
// Next loop: if the current state is within a certain threshold of the trajectory step, then update to next step and push that.
// Otherwise, re-push current trajectory step.

// speedj pushes joint velocities. it's calculated of difference between current and target / time.
// time is the frequency of these message updates.
// but the max joint velocity is capped.
// use a smaller acceleration for smoother movement.

class Ur5ContinuousController
{
public:
    Ur5ContinuousController(ros::NodeHandle handle, unsigned int control_frequency);
    ~Ur5ContinuousController();

    void control_robot();

private:
    const static float                  JOINT_THRESHOLD;
    const static float                  CONTROL_TIME_BUFFER;

    ros::Subscriber                     m_ur5ControllerListener;
    ros::Subscriber                     m_ur5JointListener;
    ros::Publisher                      m_ur5Talker;

    irl_robots::ur5Joints::ConstPtr     m_currentState;
    irl_robots::matrix::ConstPtr        m_currentTrajectory;

    irl_robots::ur5Control              m_message;

    std::mutex                          m_currentStateMutex;
    std::mutex                          m_currentTrajectoryMutex;

    unsigned int                        m_trajectoryIndex;
    unsigned int                        m_controlFrequency;
    float                               m_messageTime;

    void ur5_control_callback(const irl_robots::matrix::ConstPtr& message);
    void ur5_state_callback(const irl_robots::ur5Joints::ConstPtr& message);
};
