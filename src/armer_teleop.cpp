#include "armer_teleop.h"

/**
 * @brief Construct a new Armer Teleop:: Armer Teleop object
 *       - NOTE: default arguments based on logitech controller configuration
 */
ArmerTeleop::ArmerTeleop(): _home_client("arm/home", true)
{
    // Not constructed yet
    _class_construction = false;

    // ------ Apply Defaults (based on logitech)
    _joy_params.linear_z = 1; 
    _joy_params.linear_y = 0;
    _joy_params.linear_x_pos = 2;
    _joy_params.linear_x_neg = 5; 
    _joy_params.angular_roll = 3; 
    _joy_params.angular_pitch = 4; 
    _joy_params.angular_yaw_pos = 9;
    _joy_params.angular_yaw_neg = 10;
    _joy_params.angular_scale = 0.4; 
    _joy_params.linear_scale = 0.2;
    _joy_params.deadman_btn = 4;
    _joy_params.home_btn = 8;
    _joy_params.toggle_frame_btn = 0;
    _joy_params.max_axes_size = 8;
    _joy_params.max_btns_size = 11;
    _use_joy = true;
    _frame_id = "tool0"; //Default to end-effector name tool0
    _base_frame = "base_link"; //Default base frame name

    // Current Version (FROM PACKAGE XML)
    // See CMakeLists.txt for definition
    ROS_INFO_STREAM("Driver Version: " << armer_teleop_VERSION);

    // ------- Update (if needed) class variables from ROS param server (loaded by launch file) ---------
    nh_.param("axis_linear_z", _joy_params.linear_z, _joy_params.linear_z);
    nh_.param("axis_linear_y", _joy_params.linear_y, _joy_params.linear_y);
    nh_.param("axis_linear_x_positive", _joy_params.linear_x_pos, _joy_params.linear_x_pos);
    nh_.param("axis_linear_x_negative", _joy_params.linear_x_neg, _joy_params.linear_x_neg);
    nh_.param("axis_angular_roll", _joy_params.angular_roll, _joy_params.angular_roll);
    nh_.param("axis_angular_pitch", _joy_params.angular_pitch, _joy_params.angular_pitch);
    nh_.param("axis_angular_yaw_positive", _joy_params.angular_yaw_pos, _joy_params.angular_yaw_pos);
    nh_.param("axis_angular_yaw_negative", _joy_params.angular_yaw_neg, _joy_params.angular_yaw_neg);
    nh_.param("scale_angular", _joy_params.angular_scale, _joy_params.angular_scale);
    nh_.param("scale_linear", _joy_params.linear_scale, _joy_params.linear_scale);
    nh_.param("enable_button", _joy_params.deadman_btn, _joy_params.deadman_btn);
    nh_.param("enable_home", _joy_params.home_btn, _joy_params.home_btn);
    nh_.param("toggle_frame", _joy_params.toggle_frame_btn, _joy_params.toggle_frame_btn);
    nh_.param("max_axes_size", _joy_params.max_axes_size, _joy_params.max_axes_size);
    nh_.param("max_btns_size", _joy_params.max_btns_size, _joy_params.max_btns_size);

    // ------- Get node specific params (loaded by launch file)
    nh_.getParam("/armer_teleop_node/frame_id", _frame_id);
    nh_.getParam("/armer_teleop_node/base_frame", _base_frame);
    ROS_INFO_STREAM("array size: [" << _joy_params.max_axes_size << "]");
    ROS_INFO_STREAM("Loaded Base Frame: [" << _base_frame << "] | Loaded EE Frame: [" << _frame_id << "]");

    // Defined internal states for teleop: [0: idle, 1: enabled_vel_cntl; 2: enabled_home; 3: transition (NOT IMPLEMENTED)]
    _teleop_state = IDLE;
    // Defined state for toggling (using the A button) between base and ee frame - defaults to EE
    _frame_control = EE_FRAME;
    // Debouncing the button input for toggling of state
    _frame_btn_states = OFF; 
    _frame_toggle_count = 0;
    // Setup trigger and analogue stick (pressed) initial states
    _trigger_left = 1;          //Joy node idle value for triggers (remapped to 0 by class)
    _trigger_right = 1;         //Joy node idle value for triggers (remapped to 0 by class)
    _left_stick_pressed = 0;    //Joy node idle value for buttons
    _right_stick_pressed = 0;   //Joy node idle value for buttons
    _homing_speed = 0.2;        //Armer home client speed default

    // Initialise class axes and button arrays with sizes based on controller used
    _axes.assign(_joy_params.max_axes_size, 0);
    _buttons.assign(_joy_params.max_btns_size, 0);

    // ------- Debugging Outputs
    ROS_INFO_STREAM("linear scale: " << _joy_params.linear_scale << " and angular scale: " << _joy_params.angular_scale);

    // ------- Required publishers and subscribers
    _vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("arm/cartesian/velocity", 1);
    _jv_pub = nh_.advertise<armer_msgs::JointVelocity>("arm/joint/velocity", 1);
    _joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArmerTeleop::JoyCallback, this);
    _arr_sub = nh_.subscribe<armer_teleop::FloatArr>("server/array", 10, &ArmerTeleop::CustomCallback, this);

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

    // clamp check (-1.0 or 1.0) in axes input
    ClampInput(axes, (float)-1.0, (float)1.0);
    // clamp check (0 or 1) in button input
    ClampInput(buttons, 0, 1);

    // Update class axes and buttons - Used for Testing
    _axes = axes;
    _buttons = buttons;

    // Prepare linear twists (z and y)
    twist.linear.z = _joy_params.linear_scale * axes[_joy_params.linear_z]; //UP/DOWN in Base Frame
    twist.linear.y = _joy_params.linear_scale * axes[_joy_params.linear_y]; //LEFT/RIGHT in Base Frame

    //Get both axes (trigger left and right buttons) for x axis twist
    // Trigger ranges (default) to 1.0 --> -1.0. The following changes this input
    // to between 0 --> 1.0, where 0 is the idle state and pressed in fully is 1.0
    _trigger_left = ((axes[_joy_params.linear_x_pos] - 1.0) / -2.0);
    _trigger_right = ((axes[_joy_params.linear_x_neg] - 1.0) / -2.0);

    // Resolve for both triggers pressed at once
    if(_trigger_right > 0 && _trigger_left > 0)
    {
        //Do nothing, as both triggers are active
        output = 1;
    }
    else
    {
        // Apply expected behaviour (left or right) directional movement
        if(_trigger_left > 0)
        {
            twist.linear.x = _joy_params.linear_scale * _trigger_left;
        }
        else if(_trigger_right > 0)
        {
            output = 4;
            twist.linear.x = (-1.0) * _joy_params.linear_scale * _trigger_right;
        }
        else
        {
            output = 3;
            twist.linear.x = 0.0;
        }
    }
    
    // Prepare angular twists - z (pitch) and y (roll)
    twist.angular.z = _joy_params.angular_scale * axes[_joy_params.angular_pitch]; //Around z-axis in Base Frame
    twist.angular.y = _joy_params.angular_scale * axes[_joy_params.angular_roll];  //Around y-axis in Base Frame

    //Handle angular x (yaw) axis using analogue buttons
    _left_stick_pressed = buttons[_joy_params.angular_yaw_pos];
    _right_stick_pressed = buttons[_joy_params.angular_yaw_neg];

    //Resolve for both sticks pressed twice
    if(_left_stick_pressed && _right_stick_pressed)
    {
        //Do nothing, as both sticks are pressed
        output = 2;
    }
    else
    {
        // Apply expected behaviour (left or right) directional movement
        if(_left_stick_pressed)
        {
            twist.angular.x = _joy_params.angular_scale;
        }
        else if(_right_stick_pressed)
        {
            twist.angular.x = (-1.0) * _joy_params.angular_scale;
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
    if(buttons[_joy_params.toggle_frame_btn] && _frame_btn_states == OFF) 
    {
        _frame_btn_states = ON;
        output = 1;
    }
    else if(!buttons[_joy_params.toggle_frame_btn] && _frame_btn_states == ON) 
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
void ArmerTeleop::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // Update the axes and button vectors from the joy message
    _axes = joy->axes;
    _buttons = joy->buttons;
}

/**
 * @brief callback function within the ArmerTeleop class
 * 
 * @param FloatArr (message type)
 */
void ArmerTeleop::CustomCallback(const armer_teleop::FloatArr::ConstPtr& arr)
{
    // Got new values from server
    _axes[0] = arr->arr[0]; //Rotation at base
    _axes[1] = arr->arr[1]; //Forward/Backward EE
    _axes[2] = arr->arr[2]; //Up/Down EE

    // if (_axes[0] != 0)
    // {
    //     ROS_INFO_STREAM("Rotation in Base!!!!");

    //     armer_msgs::JointVelocity vel;
    //     std::vector<double> jv_arr;
    //     jv_arr.assign(6, 0);
    //     jv_arr[0] = _axes[0];
    //     vel.joints = jv_arr;

    //     _jv_pub.publish(vel);

    // }
    // else
    // {
    // Declare twist message for configuration
    geometry_msgs::Twist twist;
    twist.linear.z = -_joy_params.linear_scale * _axes[1];
    twist.linear.y = _joy_params.linear_scale * _axes[2];

    // Define the twist stamped message to publish modifed twist from above
    geometry_msgs::TwistStamped twist_s;
    twist_s.twist = twist;
    twist_s.header.frame_id = _frame_id; //Set to EE frame

    _vel_pub.publish(twist_s);
    // _jv_pub.publish(vel);
    // }

    ROS_INFO_STREAM("Received: [" << _axes[0] << ", " << _axes[1] << ", " << _axes[2] << "]");
}

/**
 * @brief Evaluates if a movement has occurred (true if yes)
 * 
 * @param twist (geometry_msgs) giving a float velocity in linear and angular direction
 * @return true 
 * @return false 
 */
bool ArmerTeleop::JoyMovementAck( geometry_msgs::Twist twist )
{
    bool output = true;

    // Check given twist for any movement
    float lin_x = twist.linear.x;
    float lin_y = twist.linear.y;
    float lin_z = twist.linear.z;
    float ang_x = twist.angular.x;
    float ang_y = twist.angular.y;
    float ang_z = twist.angular.z;

    if (lin_x == 0
        && lin_y == 0
        && lin_z == 0
        && ang_x == 0
        && ang_y == 0
        && ang_z == 0)
    {
        // No Movement from Controller
        output = false;
    }

    return output;
}

/**
 * @brief Main method of Teleoperation
 * 
 */
void ArmerTeleop::AltRun( bool test )
{
    // Declare twist message for configuration
    // geometry_msgs::Twist twist;

    // ROS_INFO_STREAM("In Alt Run: [" << _axes[0] << ", " << _axes[1] << ", " << _axes[2] << "]");
}

/**
 * @brief Main method of Teleoperation
 * 
 */
void ArmerTeleop::Run( bool test )
{
    // Declare twist message for configuration
    geometry_msgs::Twist twist;

    // twist type updated from current axes and button vectors
    ConfigureTwist(twist, _axes, _buttons);
    
    // Update frame from configured button press
    UpdateFrame(_buttons);
    
    // Define the twist stamped message to publish modifed twist from above
    geometry_msgs::TwistStamped twist_s;
    twist_s.twist = twist;

    // Allocate button states
    _deadman_state = _buttons[_joy_params.deadman_btn];
    _home_btn_state = _buttons[_joy_params.home_btn];

    // Update the frame based on what was configured in the UpdateFrame method
    if(_frame_control == EE_FRAME)
    {
        twist_s.header.frame_id = _frame_id;
    }
    else
    {
        twist_s.header.frame_id = _base_frame;
    }

    // Run Movement ONLY if DEADMAN is PRESSED
    if (_deadman_state)
    {
        // Run Homing Action (if no other motion present)
        if (_home_btn_state && !JoyMovementAck(twist))
        {
            // Set state to HOMING
            _teleop_state = ENABLED_HOMING; 

            // Send robot arm to home position (defined by Armer)
            // Only runs if the home server exists/connected
            // NOTE: only run if NOT in test (g_test) mode
            if(!test)
            {
                if(_home_client.isServerConnected())
                {
                    armer_msgs::HomeActionGoal home;
                    home.goal.speed = _homing_speed;                
                    _home_client.sendGoal(home.goal);
                }
                else
                {
                    ROS_ERROR_STREAM("Home is NOT connected!");
                }
            }
        }
        // Run Velocity Control (if motion IS present AND we are not Homing)
        else if (JoyMovementAck(twist) && _teleop_state != ENABLED_HOMING)
        {
            //Update teleop state
            _teleop_state = ENABLED_VEL_CNTRL;

            //Trigger publish only if DEADMAN is pressed
            // NOTE: only run if NOT in test (g_test) mode
            if(!test)
            {
                _vel_pub.publish(twist_s);
            }
        }
        else
        {
            //Waiting state to handle teleop state param
            //On Successful Home, Sets State Back to IDLE to Keep Smooth Motion (NO RESET OF DEADMAN)
            if(_home_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !test)
            {
                _teleop_state = IDLE;
            }
        }
    }
    else
    {
        // Terminate any Actions if Released
        if(_home_client.getState() == actionlib::SimpleClientGoalState::ACTIVE && !test)
        {
            ROS_INFO_STREAM("Terminating Action [HOME]");
            _home_client.cancelAllGoals();
        }

        // ADD Other Actions if Required for Termination HERE

        // Currently in IDLE
        _teleop_state = IDLE; 
    }
}

/**
 * @brief This method (given a std::vector<T> of either int or float type) 
 *          clamps the values to minimum of 0 or maximum of 1
 *  
 *          Reference: https://stackoverflow.com/questions/45541921/c-clamp-function-for-a-stdvector
 * 
 * @tparam T Template of either int, or float
 * @param input_image 
 * @param min_value 
 * @param max_value 
 */
template <typename T>
void ArmerTeleop::ClampInput(
    std::vector<T>& input_image, 
    T min_value, 
    T max_value
)
{
    auto clamp = [min_value, max_value](T x) {
        return std::min(std::max(x, min_value), max_value);
    };

    std::transform(input_image.begin(), input_image.end(),
                   input_image.begin(), clamp);
}