#include "ur5network.h"

#include <cmath>
#include <cstring>

#include <irl_robots/ur5Tool.h>
#include <irl_robots/ur5Status.h>

extern "C" {
    #define NON_MATLAB_PARSING
    #include "extApi.h"
}

//const double UR5Network::PROPORTIONAL_GAIN = 2.5;
const double UR5Network::PROPORTIONAL_GAIN = 5.0;
const double UR5Network::INTEGRAL_GAIN = 0.01;
const double UR5Network::DIFFERENTIAL_GAIN = 10.0;
const double UR5Network::MAX_VELOCITY = 2.0; // this
const double UR5Network::INTEGRAL_MAX = 2.0;
const double UR5Network::INTEGRAL_MIN = -2.0;
const double UR5Network::MAX_ACCELERATION = 2.0;

UR5Network::UR5Network(ros::NodeHandle* n) :
    p_ros_node(n),
    m_ur5_joint_handles(),
    m_ur5_tool_handle(-1),
    m_ur5_handle(-1),
    m_client_id(-1),
    p_integral_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    p_differential_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
    p_pub_joint  = p_ros_node->advertise<irl_robots::ur5Joints>("/ur5/joints", 1);
    p_pub_tool   = p_ros_node->advertise<irl_robots::ur5Tool>  ("/ur5/tool",   1);
    p_pub_status = p_ros_node->advertise<irl_robots::ur5Status>("/ur5/status", 1);

    simxFinish(-1);

    m_client_id = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 2000, 5);

    if(m_client_id != -1)
    {
        std::cout << "Connected to V-REP remote API server..." << std::endl;

        simxInt status = simxGetObjectHandle(m_client_id, (simxChar*)"UR5", &m_ur5_handle, simx_opmode_blocking);

        if(status != simx_return_ok)
        {
            std::cout << "Failed to retrieve the UR5 virtual object." << std::endl;
            std::cout << "Status: " << status << std::endl;
            std::cout << "Handle: " << m_ur5_handle << std::endl;
        }
        else
        {
            for(std::size_t index = 0; index < 6; ++index)
            {
                std::string joint_name = "UR5_joint" + std::to_string(index + 1);
                status = simxGetObjectHandle(m_client_id, joint_name.c_str(), &m_ur5_joint_handles[index], simx_opmode_blocking);

                if(status != simx_return_ok)
                {
                    std::cout << "Failed to retrieve UR5 virtual joint handle." << std::endl;
                }
            }

            status = simxGetObjectHandle(m_client_id, "ik_tip", &m_ur5_tool_handle, simx_opmode_blocking);

            if(status != simx_return_ok)
            {
                std::cout << "Failed to retrieve UR5 virtual tool handle." << std::endl;
            }
        }
    }

    p_net_thread = new std::thread(&UR5Network::netMainLoop, this);
}

void UR5Network::waitForNet()
{
    p_net_thread->join();
}

void UR5Network::sendCommand(irl_robots::ur5Control com)
{
    std::lock_guard<std::mutex> lock(p_com_mutex);
    p_com_queue.push(com);
}

void UR5Network::netMainLoop()
{
    ROS_INFO("Launching Network main loop");

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        sendNextCommand();
        readStatus();
        loop_rate.sleep();
    }
}

void UR5Network::sendNextCommand()
{
    std::lock_guard<std::mutex> lock(p_com_mutex);

    if(p_com_queue.size() == 0)
    {
        return;
    }

    for(auto i = 0; i < 6; i ++)
    {
        if(p_com_queue.front().command == "speedj")
        {
            double error = p_com_queue.front().values[i] - p_ur5_currjoints.positions[i];

            p_integral_state[i] += error;

            // Min/max bounds prevent integral windup.
            if(p_integral_state[i] > INTEGRAL_MAX)
            {
                p_integral_state[i] = INTEGRAL_MAX;
            }
            else if(p_integral_state[i] < INTEGRAL_MIN)
            {
                p_integral_state[i] = INTEGRAL_MIN;
            }

            double velocity =
                (error * PROPORTIONAL_GAIN) +
                (p_integral_state[i] * INTEGRAL_GAIN) +
                ((error - p_differential_state[i]) * DIFFERENTIAL_GAIN);

            p_differential_state[i] = error;

            if(velocity > 0.0)
            {
                velocity = std::min(MAX_VELOCITY, velocity);
            }
            else
            {
                velocity = std::max(-MAX_VELOCITY, velocity);
            }

            simxSetJointTargetVelocity(m_client_id, m_ur5_joint_handles[i], velocity, simx_opmode_streaming);
            std::cout << "Setting velocity " << std::endl;
        }
        else
        {
            simxSetJointTargetPosition(m_client_id, m_ur5_joint_handles[i], p_com_queue.front().values[i], simx_opmode_streaming);
        }
    }

    p_com_queue.pop();
}

void UR5Network::readStatus()
{
    irl_robots::ur5Joints jmsg;
    irl_robots::ur5Tool tmsg;
    irl_robots::ur5Status smsg;

    float position = 0.0;
    for(auto i = 0; i < 6; i ++)
    {
        int status = simxGetJointPosition(m_client_id, m_ur5_joint_handles[i], &position, simx_opmode_streaming);
        if(status == simx_return_ok)
        {
            jmsg.positions[i] = std::fmod(position, 3.14f);
        }
        else
        {
            std::cout << "Failed to retrieve UR5 joint position." << std::endl;
        }

        status = simxGetObjectFloatParameter(m_client_id, m_ur5_joint_handles[i], 2012, &position, simx_opmode_streaming);
        if(status == simx_return_ok)
        {
            jmsg.velocities[i] = position;
        }
        else
        {
            std::cout << "Failed to retrieve UR5 joint velocity." << std::endl;
        }
    }

    // Ignoring the tool status message for now. Don't think anybody is using it.

    smsg.mode = 1.0;

    p_ur5_mode = smsg.mode;
    p_ur5_currjoints = jmsg;

    p_pub_status.publish(smsg);
    p_pub_joint.publish(jmsg);
    p_pub_tool.publish(tmsg);
}
