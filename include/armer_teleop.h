#ifndef ARMER_TELEOP_H
#define ARMER_TELEOP_H

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

/**
 * @brief STATE enumeration
 * 
 */
typedef enum 
{
    IDLE = 0,
    ENABLED = 1,
    HOMING = 2
} TELEOP_STATES;

/**
 * @brief Frame (TF) control states
 * 
 */
typedef enum
{
    EE_FRAME = 0,
    BASE_FRAME = 1
} FRAME_CONTROL;

/**
 * @brief Typical button states
 * 
 */
typedef enum
{
    OFF = 0,
    ON = 1
} BUTTON_STATES;

/**
 * @brief Definition of the ArmerTeleop class TODO: move to its own header
 * 
 */
class ArmerTeleop
{
public:
    ArmerTeleop();
    ~ArmerTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    //Control buttons/axis
    int linear_z_, linear_y_, linear_x_pos_, linear_x_neg_;
    int angular_roll_, angular_pitch_, angular_yaw_pos_, angular_yaw_neg_;

    //Additional Functionality Buttons
    int deadman_btn_, home_btn_, toggle_frame_btn_;
    int frame_toggle_count_;

    //Controller Scale4s
    double l_scale_, a_scale_;

    //Other Variables
    std::string frame_id_;
    std::string base_frame_;
    int button_state_;
    TELEOP_STATES teleop_state_;
    FRAME_CONTROL frame_control_;
    BUTTON_STATES frame_btn_states_;

    //ROS Specific
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient home_srv_;
};

#endif