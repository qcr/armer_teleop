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

typedef enum
{
    YAW_POS = 0,
    YAW_NEG = 1
} YAW_CONTROL;

/**
 * @brief Definition of the ArmerTeleop class TODO: move to its own header
 * 
 */
class ArmerTeleop
{
public:
    ArmerTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    //Control buttons/axis
    int linear_z_, linear_y_, linear_x_pos_, linear_x_neg_;
    int angular_roll_, angular_pitch_, angular_yaw_pos_, angular_yaw_neg_;

    //Additional Functionality Buttons
    int deadman_btn_, home_btn_;

    //Controller Scale4s
    double l_scale_, a_scale_;

    //Other Variables
    std::string frame_id_;
    int button_state_;
    TELEOP_STATES teleop_state_;
    YAW_CONTROL yaw_control_;

    //ROS Specific
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient home_srv_;
};

/**
 * @brief Construct a new Armer Teleop:: Armer Teleop object
 * 
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
    frame_id_("tool0")
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

    // ------- Get node specific params (loaded by launch file)
    nh_.getParam("/armer_teleop/frame_id", frame_id_);

    // Defined internal states for telop: [0: idle, 1: enabled; 2: homed; 3: transition]
    teleop_state_ = IDLE;
    // NOTE: unused but implemented for if we want to 'toggle' the yaw direction rather than press
    yaw_control_ = YAW_POS;

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
            twist.angular.x = (-1.0) * a_scale_;
        }
        else if(right_stick_pressed)
        {
            twist.angular.x = a_scale_;
        }
        else
        {
            twist.angular.x = 0.0;
        }
    }
    
    geometry_msgs::TwistStamped twist_s;
    twist_s.header.frame_id = frame_id_;
    twist_s.twist = twist;

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