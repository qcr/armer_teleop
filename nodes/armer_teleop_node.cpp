#include <ros/ros.h>
#include "armer_teleop.h"

/**
 * @brief Main function of node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "armer_teleop");
    ArmerTeleop armer_teleop;

    ros::spin();
}