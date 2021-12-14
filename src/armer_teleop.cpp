#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

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

    int linear_z_, linear_y_, angular_roll_, angular_pitch_;
    double l_scale_, a_scale_;
    std::string frame_id_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};

/**
 * @brief Construct a new Armer Teleop:: Armer Teleop object
 * 
 */
ArmerTeleop::ArmerTeleop(): 
    linear_z_(1), 
    linear_y_(0), 
    angular_roll_(3), 
    angular_pitch_(4), 
    a_scale_(0.1), 
    l_scale_(0.2),
    frame_id_("tool0")
{
    // ------- Update class variables from ROS param server (loaded by launch file) ---------
    nh_.param("axis_linear_z", linear_z_, linear_z_);
    nh_.param("axis_linear_y", linear_y_, linear_y_);
    nh_.param("axis_angular_roll", angular_roll_, angular_roll_);
    nh_.param("axis_angular_pitch", angular_pitch_, angular_pitch_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    // Get node specific params (loaded by launch file)
    nh_.getParam("frame_id", frame_id_);

    ROS_INFO_STREAM("linear scale: " << l_scale_ << " and angular scale: " << a_scale_);
    ROS_INFO_STREAM("Frame ID of robot: " << frame_id_);

    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("arm/cartesian/velocity", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ArmerTeleop::joyCallback, this);

}

/**
 * @brief callback function within the ArmerTeleop class
 * 
 * @param joy (message type)
 */
void ArmerTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist        twist;
    twist.linear.z = l_scale_ * joy->axes[linear_z_];
    twist.linear.y = (-1.0) * l_scale_ * joy->axes[linear_y_]; //To make sense of axis (positive of end-effector is right, but joy is left)
    twist.angular.z = (-1.0) * a_scale_ * joy->axes[angular_roll_];
    twist.angular.y = (-1.0) * a_scale_ * joy->axes[angular_pitch_];

    geometry_msgs::TwistStamped twist_s;
    twist_s.header.frame_id = "ee_link";
    twist_s.twist = twist;

    vel_pub_.publish(twist_s);

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