#include "armer_teleop.h"

/**
 * @brief Construct a new Armer Teleop:: Armer Teleop object
 *       - NOTE: default arguments based on logitech controller configuration
 */
ArmerTeleop::ArmerTeleop(): 
    _linear_z(1), 
    _linear_y(0),
    _linear_x_pos(5),
    _linear_x_neg(2), 
    _angular_roll(3), 
    _angular_pitch(4), 
    _angular_yaw_pos(10),
    _angular_yaw_neg(9),
    _a_scale(0.1), 
    _l_scale(0.2),
    _deadman_btn(4),
    _home_btn(8),
    _frame_id("tool0"),
    _base_frame("base_link"),
    _toggle_frame_btn(0)
{
    // Not constructed yet
    _class_construction = false;

    // ------- Update class variables from ROS param server (loaded by launch file) ---------
    nh_.param("axis_linear_z", _linear_z, _linear_z);
    nh_.param("axis_linear_y", _linear_y, _linear_y);
    nh_.param("axis_linear_x_positive", _linear_x_pos, _linear_x_pos);
    nh_.param("axis_linear_x_negative", _linear_x_neg, _linear_x_neg);
    nh_.param("axis_angular_roll", _angular_roll, _angular_roll);
    nh_.param("axis_angular_pitch", _angular_pitch, _angular_pitch);
    nh_.param("axis_angular_yaw_positive", _angular_yaw_pos, _angular_yaw_pos);
    nh_.param("axis_angular_yaw_negative", _angular_yaw_neg, _angular_yaw_neg);
    nh_.param("scale_angular", _a_scale, _a_scale);
    nh_.param("scale_linear", _l_scale, _l_scale);
    nh_.param("enable_button", _deadman_btn, _deadman_btn);
    nh_.param("enable_home", _home_btn, _home_btn);
    nh_.param("toggle_frame", _toggle_frame_btn, _toggle_frame_btn);

    // ------- Get node specific params (loaded by launch file)
    nh_.getParam("/armer_teleop/frame_id", _frame_id);
    nh_.getParam("/armer_teleop/base_frame", _base_frame);

    // Defined internal states for telop: [0: idle, 1: enabled; 2: homed; 3: transition]
    _teleop_state = IDLE;
    // Defined state for toggling (using the A button) between base and ee frame - defaults to EE
    _frame_control = EE_FRAME;
    // Debouncing the button input for toggling of state
    _frame_btn_states = OFF; 
    _frame_toggle_count = 0;

    // ------- Debugging Outputs
    ROS_INFO_STREAM("linear scale: " << _l_scale << " and angular scale: " << _a_scale);
    ROS_INFO_STREAM("Frame ID of robot: " << _frame_id);

    // ------- Required publishers and subscribers
    _vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("arm/cartesian/velocity", 1);
    _joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArmerTeleop::joyCallback, this);

    // ------- Required Services (Home)
    _home_srv = nh_.serviceClient<std_srvs::Empty>("/arm/home");

    // Successfully constructed class and setup ROS specific functionality
    _class_construction = true;
}

/**
 * @brief Destroy the Armer Teleop:: Armer Teleop object
 * 
 */
ArmerTeleop::~ArmerTeleop()
{
    //Do nothing
}

/**
 * @brief Configures a geometry_msgs::Twist value from inputted axes and button values
 * 
 * @param twist     geometry_msgs::Twist type
 * @param axes      vector of floats
 * @param buttons   vector of ints
 * @return int      status of method [0, 1, 2]
 */
int ArmerTeleop::ConfigureTwist
( 
    geometry_msgs::Twist &twist,
    std::vector<float> axes,
    std::vector<int> buttons
)
{
    // output status
    int output = 0;

    // Prepare linear twists
    twist.linear.z = _l_scale * axes[_linear_z];
    twist.linear.y = (-1.0) * _l_scale * axes[_linear_y]; //To make sense of axis (positive of end-effector is right, but joy is left)

    //Get both axes (trigger left and right buttons)
    // --> if one is greater than 0 (triggered) apply this value to the twist
    double trigger_left = ((axes[_linear_x_pos] - 1.0) / -2.0);
    double trigger_right = ((axes[_linear_x_neg] - 1.0) / -2.0);
    if(trigger_right > 0 && trigger_left > 0)
    {
        //Do nothing, as both triggers are active
        output = 1;
    }
    else
    {
        // Apply expected behaviour (left or right) directional movement
        if(trigger_left > 0)
        {
            twist.linear.x = _l_scale * trigger_left;
        }
        else if(trigger_right > 0)
        {
            twist.linear.x = (-1.0) * _l_scale * trigger_right;
        }
        else
        {
            twist.linear.x = 0.0;
        }
    }
    
    // Prepare angular twists
    twist.angular.z = (-1.0) * _a_scale * axes[_angular_roll];
    twist.angular.y = (-1.0) * _a_scale * axes[_angular_pitch];

    //Handle angular x (yaw) axis using analogue buttons
    int left_stick_pressed = buttons[_angular_yaw_pos];
    int right_stick_pressed = buttons[_angular_yaw_neg];
    if(left_stick_pressed && right_stick_pressed)
    {
        //Do nothing, as both sticks are pressed
        output = 2;
    }
    else
    {
        // Apply expected behaviour (left or right) directional movement
        if(left_stick_pressed)
        {
            twist.angular.x = _a_scale;
        }
        else if(right_stick_pressed)
        {
            twist.angular.x = (-1.0) * _a_scale;
        }
        else
        {
            twist.angular.x = 0.0;
        }
    }

    return output;
}

/**
 * @brief Updates the robot frame based on button press 
 *        - Switches between base_frame and tool_frame. 
 *        - NOTE: these names are configured from launch for specific robots
 * 
 * @param buttons   vector of ints
 * @return output   int status output (using for testing) [0, 1, 2, 3, 4]
 */
int ArmerTeleop::UpdateFrame( std::vector<int> buttons )
{
    // output for status
    int output = 0;

    // This block of logic sets the correct state of the button press: OFF-ON-OFF
    // ON state is only triggered if the button has been pressed and
    // it was previously OFF. On release of button, and the state was set to ON, 
    // state is set back to OFF and a toggle counter is incremented. If this toggle
    // counter is non-zero, then the functionality of the button is run.
    if(buttons[_toggle_frame_btn] && _frame_btn_states == OFF) 
    {
        _frame_btn_states = ON;
        output = 1;
    }
    else if(!buttons[_toggle_frame_btn] && _frame_btn_states == ON) 
    {
        _frame_toggle_count++;
        _frame_btn_states = OFF;
        output = 2;
    }

    // Checks if the toggle count is non-zero and switches the frame
    // based on the currently configured frame. NOTE: the default frame
    // is the BASE_FRAME on class construction. The toggle count is reset
    // once a frame has been changed
    if(_frame_toggle_count && _frame_control == EE_FRAME)
    {
        ROS_INFO_STREAM("SWITCHED TO BASE_FRAME");
        _frame_control = BASE_FRAME;
        _frame_toggle_count = 0;
        output = 3;
    }
    else if(_frame_toggle_count && _frame_control == BASE_FRAME)
    {
        ROS_INFO_STREAM("SWITCHED TO EE_FRAME");
        _frame_control = EE_FRAME;
        _frame_toggle_count = 0;
        output = 4;
    }

    return output;
}

/**
 * @brief callback function within the ArmerTeleop class
 * 
 * @param joy (message type)
 */
void ArmerTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Declare twist message for configuration
    geometry_msgs::Twist twist;

    // Get the axes and button vectors from the joy message
    std::vector<float> axes = joy->axes;
    std::vector<int> buttons = joy->buttons;

    // twist type updated from axes and button vectors
    ConfigureTwist(twist, axes, buttons);
    
    // Update frame from configured button press
    UpdateFrame(buttons);
    
    // Define the twist stamped message to publish modifed twist from above
    geometry_msgs::TwistStamped twist_s;
    twist_s.twist = twist;

    // Update the frame based on what was configured in the UpdateFrame method
    if(_frame_control == EE_FRAME)
    {
        twist_s.header.frame_id = _frame_id;
    }
    else
    {
        twist_s.header.frame_id = _base_frame;
    }

    // Publish twist stamped message ONLY if deadman is pressed
    // and we are NOT homing
    if (buttons[_deadman_btn] && _teleop_state != HOMING)
    {
        //Trigger publish only if DEADMAN is pressed
        _vel_pub.publish(twist_s);

        //Update teleop state
        _teleop_state = ENABLED;
    }
    // Execute the Armer Driver Homing functionality via ROS action server
    else if ( buttons[_home_btn] )
    {
        //Set state to HOMING
        _teleop_state = HOMING; 

        //Send robot arm to home position (defined by Armer)
        //Only runs if the home service exists
        std_srvs::Empty srv;
        if(_home_srv.exists()) 
        {
            _home_srv.call(srv);
        }
        else 
        {
            ROS_WARN_STREAM("Armer Home Service Unavailable...");
        }
        //Set state to IDLE
        _teleop_state = IDLE; 
    }

}

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