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
 * @brief joy node params for teleop package as struct
 * 
 */
struct joy_params
{
    int linear_z; 
    int linear_y;
    int linear_x_pos;
    int linear_x_neg;
    double linear_scale; 
    int angular_roll; 
    int angular_pitch; 
    int angular_yaw_pos;
    int angular_yaw_neg;
    double angular_scale; 
    int deadman_btn;
    int home_btn;
    int toggle_frame_btn;
};

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

    // --------- Get and Set Methods
    FRAME_CONTROL GetCurrentFrame( void ) { return _frame_control; }
    BUTTON_STATES GetFrameBtnState( void ) { return _frame_btn_states; }
    joy_params GetJoyParams( void ) { return _joy_params; }
    void SetJoyParams( joy_params params ) { _joy_params = params; }
    void SetFrame( FRAME_CONTROL frame ) { _frame_control = frame; }

    // gTest Friendly Methods
    bool ClassConstructionSuccess( void ) { return _class_construction; }
    int TestUpdateFrame( std::vector<int> buttons ) { return UpdateFrame(buttons); }
    int TestConfigureTwist( 
        geometry_msgs::Twist &twist,
        std::vector<float> axes,
        std::vector<int> buttons  
    ) { return ConfigureTwist(twist, axes, buttons); }

private:
    // Private Functions
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    int UpdateFrame( std::vector<int> buttons );
    int ConfigureTwist( 
        geometry_msgs::Twist &twist,
        std::vector<float> axes,
        std::vector<int> buttons 
    );

    // ROS Specific Variables 
    ros::NodeHandle nh_;
    ros::Publisher _vel_pub;
    ros::Subscriber _joy_sub;
    ros::ServiceClient _home_srv;

    //Test Variables
    bool _class_construction;

    //Other Variables
    joy_params _joy_params;
    int _frame_toggle_count;
    std::string _frame_id;
    std::string _base_frame;
    int _button_state;
    TELEOP_STATES _teleop_state;
    FRAME_CONTROL _frame_control;
    BUTTON_STATES _frame_btn_states;
};

#endif