#include "football.h"

extern "C" {
    #define NON_MATLAB_PARSING
    #include "extApi.h"
}

Football::Football(ros::NodeHandle handle) :
    m_football_handle(-1),
    m_client_id(-1)
{
    simxFinish(-1);

    m_client_id = simxStart((simxChar*)"127.0.0.1", 19996, true, true, 2000, 5);

    if(m_client_id != -1)
    {
        std::cout << "Connected to V-REP remote API server..." << std::endl;

        simxInt status = simxGetObjectHandle(m_client_id, (simxChar*)"Football", &m_football_handle, simx_opmode_blocking);

        if(status != simx_return_ok)
        {
            std::cout << "Failed to retrieve the Football virtual object." << std::endl;
            std::cout << "Status: " << status << std::endl;
            std::cout << "Handle: " << m_football_handle << std::endl;
        }
        else
        {
            m_stateSubscriber = handle.subscribe("/vrpn_client_node/Football/pose", 1, &Football::callback_positionUpdate, this);
        }
    }
}

Football::~Football()
{
    simxFinish(m_client_id);

    m_client_id = -1;
    m_football_handle = -1;
}

void Football::run()
{
    ros::spin();
}

void Football::callback_positionUpdate(const geometry_msgs::PoseStamped::ConstPtr pose)
{
    float position[3] = {pose->pose.position.x, pose->pose.position.y, pose->pose.position.z};
    simxSetObjectPosition(m_client_id, m_football_handle, -1, position, simx_opmode_oneshot);
}
