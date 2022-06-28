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

    // Set Rate of Functionality
    ros::Rate rate(100);
    while (ros::ok())
    {
        // Run Callbacks (Get Joy Data)
        ros::spinOnce();

        // Run Tele-Operation Function
        armer_teleop.AltRun();

        // Overall Sleep
        rate.sleep();
    }
}