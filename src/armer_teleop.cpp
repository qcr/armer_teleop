#include "armer_teleop.h"

/**
 * @brief Construct a new Armer Teleop:: Armer Teleop object
 *       - NOTE: default arguments based on logitech controller configuration
 */
ArmerTeleop::ArmerTeleop(): 
    linear_z_(1), 
    linear_y_(0),
    linear_x_pos_(5),
    linear_x_neg_(2), 
    angular_roll_(3), 
    angular_pitch_(4), 
    angular_yaw_pos_(10),
    angular_yaw_neg_(9),
    a_scale_(0.1), 
    l_scale_(0.2),
    deadman_btn_(4),
    home_btn_(8),
    frame_id_("tool0"),
    base_frame_("base_link"),
    toggle_frame_btn_(0)
{
    // ------- Update class variables from ROS param server (loaded by launch file) ---------
    nh_.param("axis_linear_z", linear_z_, linear_z_);
    nh_.param("axis_linear_y", linear_y_, linear_y_);
    nh_.param("axis_linear_x_positive", linear_x_pos_, linear_x_pos_);
    nh_.param("axis_linear_x_negative", linear_x_neg_, linear_x_neg_);
    nh_.param("axis_angular_roll", angular_roll_, angular_roll_);
    nh_.param("axis_angular_pitch", angular_pitch_, angular_pitch_);
    nh_.param("axis_angular_yaw_positive", angular_yaw_pos_, angular_yaw_pos_);
    nh_.param("axis_angular_yaw_negative", angular_yaw_neg_, angular_yaw_neg_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("enable_button", deadman_btn_, deadman_btn_);
    nh_.param("enable_home", home_btn_, home_btn_);
    nh_.param("toggle_frame", toggle_frame_btn_, toggle_frame_btn_);

    // ------- Get node specific params (loaded by launch file)
    nh_.getParam("/armer_teleop/frame_id", frame_id_);
    nh_.getParam("/armer_teleop/base_frame", base_frame_);

    // Defined internal states for telop: [0: idle, 1: enabled; 2: homed; 3: transition]
    teleop_state_ = IDLE;
    // Defined state for toggling (using the A button) between base and ee frame - defaults to EE
    frame_control_ = EE_FRAME;
    // Debouncing the button input for toggling of state
    frame_btn_states_ = OFF; 
    frame_toggle_count_ = 0;

    // ------- Debugging Outputs
    ROS_INFO_STREAM("linear scale: " << l_scale_ << " and angular scale: " << a_scale_);
    ROS_INFO_STREAM("Frame ID of robot: " << frame_id_);

    // ------- Required publishers and subscribers
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("arm/cartesian/velocity", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArmerTeleop::joyCallback, this);

    // ------- Required Services (Home)
    home_srv_ = nh_.serviceClient<std_srvs::Empty>("/arm/home");
    home_srv_.waitForExistence(); //Blocks until service is available (this node should be launched after Armer)
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
 * @brief callback function within the ArmerTeleop class
 * 
 * @param joy (message type)
 */
void ArmerTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist        twist;
    
    // Prepare linear twists
    twist.linear.z = l_scale_ * joy->axes[linear_z_];
    twist.linear.y = (-1.0) * l_scale_ * joy->axes[linear_y_]; //To make sense of axis (positive of end-effector is right, but joy is left)

    //Get both axes (trigger left and right buttons)
    // --> if one is greater than 0 (triggered) apply this value to the twist
    double trigger_left = ((joy->axes[linear_x_pos_] - 1.0) / -2.0);
    double trigger_right = ((joy->axes[linear_x_neg_] - 1.0) / -2.0);
    if(trigger_right > 0 && trigger_left > 0)
    {
        //Do nothing, as both triggers are active
    }
    else
    {
        if(trigger_left > 0)
        {
            twist.linear.x = l_scale_ * trigger_left;
        }
        else if(trigger_right > 0)
        {
            twist.linear.x = (-1.0) * l_scale_ * trigger_right;
        }
        else
        {
            twist.linear.x = 0.0;
        }
    }
    
    // Prepare angular twists
    twist.angular.z = (-1.0) * a_scale_ * joy->axes[angular_roll_];
    twist.angular.y = (-1.0) * a_scale_ * joy->axes[angular_pitch_];

    //Handle angular x (yaw) axis using analogue buttons
    int left_stick_pressed = joy->buttons[angular_yaw_pos_];
    int right_stick_pressed = joy->buttons[angular_yaw_neg_];
    if(left_stick_pressed && right_stick_pressed)
    {
        //Do nothing, as both sticks are pressed
    }
    else
    {
        if(left_stick_pressed)
        {
            twist.angular.x = a_scale_;
        }
        else if(right_stick_pressed)
        {
            twist.angular.x = (-1.0) * a_scale_;
        }
        else
        {
            twist.angular.x = 0.0;
        }
    }

    // Update frame from button press - if pressed
    if(joy->buttons[toggle_frame_btn_] && frame_btn_states_ == OFF) frame_btn_states_ = ON;
    else if(!joy->buttons[toggle_frame_btn_] && frame_btn_states_ == ON) 
    {
        frame_toggle_count_++;
        frame_btn_states_ = OFF;
    }

    if(frame_toggle_count_ && frame_control_ == EE_FRAME)
    {
        ROS_INFO_STREAM("SWITCHED TO BASE_FRAME");
        frame_control_ = BASE_FRAME;
        frame_toggle_count_ = 0;
    }
    else if(frame_toggle_count_ && frame_control_ == BASE_FRAME)
    {
        ROS_INFO_STREAM("SWITCHED TO EE_FRAME");
        frame_control_ = EE_FRAME;
        frame_toggle_count_ = 0;
    }
    
    // Define the twist stamped message to publish modifed twist from above
    geometry_msgs::TwistStamped twist_s;
    twist_s.twist = twist;

    // Determine required frame
    if(frame_control_ == EE_FRAME)
        twist_s.header.frame_id = frame_id_;
    else
        twist_s.header.frame_id = base_frame_;

    if (joy->buttons[deadman_btn_] && teleop_state_ != HOMING)
    {
        //Trigger publish only if DEADMAN is pressed
        vel_pub_.publish(twist_s);

        //Update telop state
        teleop_state_ = ENABLED;
    }
    else if ( joy->buttons[home_btn_] )
    {
        //Set state to HOMING
        teleop_state_ = HOMING; 

        //Send robot arm to home position (defined by Armer)
        std_srvs::Empty srv;
        home_srv_.call(srv);

        //Set state to IDLE
        teleop_state_ = IDLE; 
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