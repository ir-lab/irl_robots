#include "ur5network.h"

#include <cmath>
#include <cstring>

#include <irl_robots/ur5Tool.h>
#include <irl_robots/ur5Status.h>

extern "C" {
    #define NON_MATLAB_PARSING
    #include "extApi.h"
}

UR5Network::UR5Network(ros::NodeHandle* n) :
    p_ros_node(n),
    m_ur5_joint_handles(),
    m_ur5_tool_handle(-1),
    m_ur5_handle(-1),
    m_client_id(-1),
    p_integral_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    p_differential_state{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
    XmlRpc::XmlRpcValue param_list;
    if(n->getParam("control", param_list))
    {
        m_p_gain = param_list["ur5"]["p_gain"];
        m_i_gain = param_list["ur5"]["i_gain"];
        m_d_gain = param_list["ur5"]["d_gain"];
        m_max_velocity = param_list["ur5"]["max_velocity"];
        m_i_max = param_list["ur5"]["max_i"];
        m_i_min = param_list["ur5"]["min_i"];
        m_max_accel = param_list["ur5"]["max_acceleration"];

        for(std::size_t joint_index = 0; joint_index < 6; joint_index++)
        {
            m_joint_offsets[joint_index] = param_list["ur5"]["virtual_joint_offset"][joint_index];
        }
    }
    else
    {
        throw std::invalid_argument("Missing \"control\" ROS parameters for UR5. Please set them and re-run.");
    }

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

UR5Network::~UR5Network()
{
    simxFinish(m_client_id);

    m_client_id = -1;
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
            if(p_integral_state[i] > m_i_max)
            {
                p_integral_state[i] = m_i_max;
            }
            else if(p_integral_state[i] < m_i_min)
            {
                p_integral_state[i] = m_i_min;
            }

            double velocity =
                (error * m_p_gain) +
                (p_integral_state[i] * m_i_gain) +
                ((error - p_differential_state[i]) * m_d_gain);

            p_differential_state[i] = error;

            if(velocity > 0.0)
            {
                velocity = std::min(m_max_velocity, velocity);
            }
            else
            {
                velocity = std::max(-m_max_velocity, velocity);
            }

            simxSetJointTargetVelocity(m_client_id, m_ur5_joint_handles[i], velocity, simx_opmode_streaming);
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
            jmsg.positions[i] = position + m_joint_offsets[i];
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

    float world_position[3] = {0.0, 0.0, 0.0};
    int status = simxGetObjectPosition(m_client_id, m_ur5_tool_handle, -1, world_position, simx_opmode_streaming);
    if(status == simx_return_ok)
    {
        tmsg.position[0] = world_position[0];
        tmsg.position[1] = world_position[1];
        tmsg.position[2] = world_position[2];
    }

    smsg.mode = 1.0;

    p_ur5_mode = smsg.mode;
    p_ur5_currjoints = jmsg;

    p_pub_status.publish(smsg);
    p_pub_joint.publish(jmsg);
    p_pub_tool.publish(tmsg);
}
