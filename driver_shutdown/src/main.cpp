
#include <ros/ros.h>
#include "driver_shutdown.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "driver_shutdown");
    driver_shutdown::DriverShutdown node;
    node.run();
    return 0;
};