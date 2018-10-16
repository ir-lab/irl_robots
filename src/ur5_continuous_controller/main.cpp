#include "ros/ros.h"
#include "ur5_continuous_controller.h"

#include <iostream>
#include <thread>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_continuous_controller");

    ros::NodeHandle handle;

    Ur5ContinuousController controller(handle, 20);

    controller.control_robot();
}
