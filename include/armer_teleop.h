#ifndef ARMER_TELEOP_H
#define ARMER_TELEOP_H

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <actionlib/client/simple_action_client.h>
#include <armer_msgs/HomeAction.h>

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
    int max_axes_size;
    int max_btns_size;
};

/**
 * @brief Action Clients defined here
 * 
 */
typedef actionlib::SimpleActionClient<armer_msgs::HomeAction> Client;

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
    std::vector<float> GetAxesVect ( void ) { return _axes; }
    std::vector<int> GetBtnsVect ( void ) { return _buttons; }
    double GetLeftTriggerVal ( void ) { return _trigger_left; }
    double GetRightTriggerVal ( void ) { return _trigger_right; }
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
    void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    int UpdateFrame( std::vector<int> buttons );
    int ConfigureTwist( 
        geometry_msgs::Twist &twist,
        std::vector<float> axes,
        std::vector<int> buttons 
    );

    template <typename T>
    void ClampInput(
        std::vector<T>& input_vect, 
        T min_value, 
        T max_value
    );

    // ROS Specific Variables 
    ros::NodeHandle nh_;
    ros::Publisher _vel_pub;
    ros::Subscriber _joy_sub;
    Client _home_client;

    //Test Variables
    bool _class_construction;
    std::vector<float> _axes;
    std::vector<int> _buttons;

    //Other Variables
    joy_params _joy_params;
    int _frame_toggle_count;
    std::string _frame_id;
    std::string _base_frame;
    int _button_state;
    TELEOP_STATES _teleop_state;
    FRAME_CONTROL _frame_control;
    BUTTON_STATES _frame_btn_states;
    double _trigger_left, _trigger_right;
    int _left_stick_pressed, _right_stick_pressed;
    double _homing_speed;
};

#endif