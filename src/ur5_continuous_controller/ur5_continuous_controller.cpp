#include "ur5_continuous_controller.h"

// If we want the arm to move faster, we need to increase the joint threshold.
// Otherwise, damping will cause the system to slow down as it approaches each intermediate
// goal to avoid overshooting.
const float Ur5ContinuousController::JOINT_THRESHOLD = 0.08f;
const float Ur5ContinuousController::CONTROL_TIME_BUFFER = 1.2f;

Ur5ContinuousController::Ur5ContinuousController(ros::NodeHandle handle, unsigned int control_frequency) :
    m_ur5ControllerListener(),
    m_ur5JointListener(),
    m_ur5Talker(),
    m_currentState(),
    m_currentTrajectory(),
    m_message(),
    m_trajectoryIndex(0),
    m_controlFrequency(control_frequency),
    m_messageTime((1.0 / control_frequency) * CONTROL_TIME_BUFFER)
{

    m_ur5ControllerListener = handle.subscribe("/ur5/continuous_controller", 1, &Ur5ContinuousController::ur5_control_callback, this);
    m_ur5JointListener = handle.subscribe("/ur5/joints", 1, &Ur5ContinuousController::ur5_state_callback, this);
    m_ur5Talker = handle.advertise<irl_robots::ur5Control>("/ur5/control", 1);

    m_message.acceleration = 0.1;
    m_message.blend = 0;
    m_message.command = "speedj";
    m_message.gain = 0;
    m_message.jointcontrol = true;
    m_message.lookahead = 0;
    m_message.time = m_messageTime;
    m_message.velocity = 0;
}

Ur5ContinuousController::~Ur5ContinuousController()
{

}

void Ur5ContinuousController::ur5_control_callback(const irl_robots::matrix::ConstPtr& message)
{
    std::lock_guard<std::mutex> lock(m_currentTrajectoryMutex);

    m_currentTrajectory = message;
    m_trajectoryIndex = 0;
}

void Ur5ContinuousController::ur5_state_callback(const irl_robots::ur5Joints::ConstPtr& message)
{
    std::lock_guard<std::mutex> lock(m_currentStateMutex);

    m_currentState = message;
}

void Ur5ContinuousController::control_robot()
{
    ros::WallRate rate(m_controlFrequency);

    while(ros::ok())
    {
        if(m_currentTrajectory)
        {
            // Separate scope to minimize time required for locks.
            {
                std::lock_guard<std::mutex> lock_trajectory(m_currentTrajectoryMutex);
                std::lock_guard<std::mutex> lock_state(m_currentStateMutex);

                bool update_index = true;
                for(auto j = 0; j < 6; j++)
                {
                    if(std::abs(m_currentTrajectory->rows[j].values[m_trajectoryIndex] - m_currentState->positions[j]) >= JOINT_THRESHOLD)
                    {
                        update_index = false;
                        break;
                    }
                }

                if(update_index && m_trajectoryIndex + 1 < m_currentTrajectory->rows[0].values.size())
                {
                    m_trajectoryIndex += 1;
                }

                for(auto j = 0; j < 6; j++)
                {
                    m_message.values[j] = m_currentTrajectory->rows[j].values[m_trajectoryIndex];
                }
            }

            m_ur5Talker.publish(m_message);
        }

        ros::spinOnce();
        rate.sleep();
    }
}
