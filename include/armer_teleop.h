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
 * @brief Definition of the ArmerTeleop class
 * 
 */
class ArmerTeleop
{
public:
    // Constructors and Destructors
    ArmerTeleop();
    ~ArmerTeleop();

    // Get and Set Methods
    FRAME_CONTROL GetCurrentFrame( void ) { return _frame_control; }
    BUTTON_STATES GetFrameBtnState( void ) { return _frame_btn_states; }

    // gTest Friendly Methods
    bool ClassConstructionSuccess( void ) { return _class_construction; }
    int TestUpdateFrame( std::vector<int> buttons ) { return UpdateFrame(buttons); }

private:
    // Private Functions
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    int UpdateFrame( std::vector<int> buttons );
    int ConfigureTwist( 
        geometry_msgs::Twist &twist,
        std::vector<float> axes,
        std::vector<int> buttons 
    );

    ros::NodeHandle nh_;

    //Control buttons/axis
    int _linear_z, _linear_y, _linear_x_pos, _linear_x_neg;
    int _angular_roll, _angular_pitch, _angular_yaw_pos, _angular_yaw_neg;

    //Additional Functionality Buttons
    int _deadman_btn, _home_btn, _toggle_frame_btn;
    int _frame_toggle_count;

    //Controller Scales
    double _l_scale, _a_scale;

    //ROS Specific
    ros::Publisher _vel_pub;
    ros::Subscriber _joy_sub;
    ros::ServiceClient _home_srv;

    //Test Variables
    bool _class_construction;

    //Other Variables
    std::string _frame_id;
    std::string _base_frame;
    int _button_state;
    TELEOP_STATES _teleop_state;
    FRAME_CONTROL _frame_control;
    BUTTON_STATES _frame_btn_states;
};

#endif