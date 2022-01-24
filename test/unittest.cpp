// Bring in Google Testing Framework and ROS
#include <gtest/gtest.h>
#include <ros/ros.h>
// Bring in the package's API for Testing
#include "armer_teleop.h"

/**
 * @brief Tests the creation of the armer_teleop class
 * 
 */
TEST( ArmerTeleopFunctionality, ClassCreationTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop new_class;
    ASSERT_EQ(new_class.ClassConstructionSuccess(), true);
}

/**
 * @brief Tests the toggle of the frame state and the button states
 * 
 */
TEST( ArmerTeleopFunctionality, FrameChangeTest )
{   
    ros::NodeHandle nh("~");
    BUTTON_STATES btn_state;
    FRAME_CONTROL frame_state;
    ArmerTeleop teleop_class;
    // button array is 11 elements long (logitech) and 13 elements (ps4) - MAX used
    std::vector<int> buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Confirm that the default frame state and button states are EE_FRAME and OFF, respectively
    btn_state = teleop_class.GetFrameBtnState();
    frame_state = teleop_class.GetCurrentFrame();
    EXPECT_EQ(btn_state, OFF);
    EXPECT_EQ(frame_state, EE_FRAME);

    // Test a press of the button
    buttons[0] = 1; // frame button press (same for ps4 and logitech)
    teleop_class.TestUpdateFrame(buttons);
    btn_state = teleop_class.GetFrameBtnState();
    frame_state = teleop_class.GetCurrentFrame();
    EXPECT_EQ(btn_state, ON);           //Button state should be ON after a press event
    EXPECT_EQ(frame_state, EE_FRAME);   //EE_FRAME should remain, as the button hasn't toggled yet

    // Test a release of the button and the subsequent update
    buttons[0] = 0; // frame button off (same for ps4 and logitech)
    teleop_class.TestUpdateFrame(buttons);
    btn_state = teleop_class.GetFrameBtnState();
    frame_state = teleop_class.GetCurrentFrame();
    EXPECT_EQ(btn_state, OFF);           //Button state should be ON after a press event
    EXPECT_EQ(frame_state, BASE_FRAME);  //BASE_FRAME should now be updated on a toggle

    // Test a press of the button -> to reset frame back
    buttons[0] = 1; // frame button press (same for ps4 and logitech)
    teleop_class.TestUpdateFrame(buttons);
    btn_state = teleop_class.GetFrameBtnState();
    frame_state = teleop_class.GetCurrentFrame();
    EXPECT_EQ(btn_state, ON);           //Button state should be ON after a press event
    EXPECT_EQ(frame_state, BASE_FRAME); //BASE_FRAME should remain, as the button hasn't toggled yet

    // Test a release of the button and the subsequent update
    buttons[0] = 0; // frame button off (same for ps4 and logitech)
    teleop_class.TestUpdateFrame(buttons);
    btn_state = teleop_class.GetFrameBtnState();
    frame_state = teleop_class.GetCurrentFrame();
    EXPECT_EQ(btn_state, OFF);          //Button state should be ON after a press event
    EXPECT_EQ(frame_state, EE_FRAME);   //EE_FRAME should now be updated on a toggle
}

/**
 * @brief Tests the clamping of axis and button inputs
 * 
 */
TEST( ArmerTeleopFunctionality, InputClampingTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop teleop_class;
    geometry_msgs::Twist test_twist;
    joy_params test_params = teleop_class.GetJoyParams();
    // axes array is 8 elements long (same for logitech and ps4)
    std::vector<float> test_axes;
    std::vector<float> exp_max_clamped_axes(test_params.max_axes_size, 1.0);
    std::vector<float> exp_min_clamped_axes(test_params.max_axes_size, -1.0);
    std::vector<int> test_btns;
    std::vector<int> exp_max_clamped_btns(test_params.max_btns_size, 1);
    std::vector<int> exp_min_clamped_btns(test_params.max_btns_size, 0);

    // Test Exceeding Clamp
    test_axes.assign(test_params.max_axes_size, 2.0);
    test_btns.assign(test_params.max_btns_size, 2);
    teleop_class.TestConfigureTwist(test_twist, test_axes, test_btns);
    EXPECT_EQ(teleop_class.GetAxesVect(), exp_max_clamped_axes);
    EXPECT_EQ(teleop_class.GetBtnsVect(), exp_max_clamped_btns);

    // Test Below Clamp
    test_axes.assign(test_params.max_axes_size, -2.0);
    test_btns.assign(test_params.max_btns_size, -2);
    teleop_class.TestConfigureTwist(test_twist, test_axes, test_btns);
    EXPECT_EQ(teleop_class.GetAxesVect(), exp_min_clamped_axes);
    EXPECT_EQ(teleop_class.GetBtnsVect(), exp_min_clamped_btns);
}

/**
 * @brief Tests the expected linear twist output from a given input (simulated Logitech controller)
 * 
 */
TEST( ArmerTeleopFunctionality, LogitechConfigLinearTwistTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop teleop_class;
    geometry_msgs::Twist test_twist;

    // Set frame to BASE_LINK for comparing (Z: up/down | Y: left/right | X: forward/back) 
    // ---> note that all axis configurations are pos/neg expected
    // Also get the current params for testing (default)
    FRAME_CONTROL test_frame = BASE_FRAME;
    teleop_class.SetFrame(test_frame);
    joy_params test_params = teleop_class.GetJoyParams();

    // axes array is 8 elements long (same for logitech and ps4)
    std::vector<float> axes(test_params.max_axes_size, 0);
    // button array is 11 elements long (logitech) and 13 elements (ps4)
    std::vector<int> buttons(test_params.max_btns_size, 0);

    //------------ LEFT STICK LEFT/RIGHT (Y-Axis) LINEAR MOVEMENT TESTS -------------------------------
    // Test a simulated left stick pushed directly left (pos) for linear in y axis response
    axes[test_params.linear_y] = 1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive linear y velocity and max linear scale
    EXPECT_GT(test_twist.linear.y, 0);
    EXPECT_NEAR(test_twist.linear.y, test_params.linear_scale, 0.01);

    // Test a simulated left stick pushed directly right (neg) for neg linear in y axis response
    axes[test_params.linear_y] = -1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg linear y velocity and max neg linear scale
    EXPECT_LT(test_twist.linear.y, 0);
    EXPECT_NEAR(test_twist.linear.y, -(1.0) * test_params.linear_scale, 0.01);

    //------------ LEFT STICK UP/DOWN (Z-Axis) LINEAR MOVEMENT TESTS -------------------------------
    // Test a simulated left stick pushed directly left (pos) for linear in y axis response
    axes[test_params.linear_z] = 1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive linear y velocity and max linear scale
    EXPECT_GT(test_twist.linear.z, 0);
    EXPECT_NEAR(test_twist.linear.z, test_params.linear_scale, 0.01);

    // Test a simulated left stick pushed directly right (neg) for neg linear in y axis response
    axes[test_params.linear_z] = -1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg linear y velocity and max neg linear scale
    EXPECT_LT(test_twist.linear.z, 0);
    EXPECT_NEAR(test_twist.linear.z, -(1.0) * test_params.linear_scale, 0.01);

    //------------ LEFT/RIGHT TRIGGERS FORWARD/BACKWARD (X-Axis) LINEAR MOVEMENT TESTS -------------------------------
    // Test trigger idle states (not pressed)
    axes[test_params.linear_x_pos] = 1.0; // this is idle (Left trigger not pressed - remapped to 0 in driver)
    axes[test_params.linear_x_neg] = 1.0; // this is idle (Right trigger not pressed - remapped to 0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    ASSERT_EQ(teleop_class.GetLeftTriggerVal(), 0);     //Test Left Trigger remapping to 0 otherwise discontinue
    ASSERT_EQ(teleop_class.GetRightTriggerVal(), 0);    //Test Right Trigger remapping to 0 otherwise discontinue
    EXPECT_EQ(test_twist.linear.x, 0);                  //X axis velocity should be zero

    // Test a simulated left and right trigger pushed (both pressed should not change output of x axis)
    axes[test_params.linear_x_pos] = -1.0;              // this is fully pressed (remapped to 1.0 in driver)
    axes[test_params.linear_x_neg] = -1.0;              // this is fully pressed (remapped to 1.0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    ASSERT_EQ(teleop_class.GetLeftTriggerVal(), 1);     //Test Left Trigger remapping to 1 otherwise discontinue
    ASSERT_EQ(teleop_class.GetRightTriggerVal(), 1);    //Test Right Trigger remapping to 1 otherwise discontinue
    EXPECT_EQ(test_twist.linear.x, 0);                  //X axis velocity should be zero

    // Test a simulated left pushed 
    axes[test_params.linear_x_pos] = -1.0;              // this is fully pressed (remapped to 1.0 in driver)
    axes[test_params.linear_x_neg] = 1.0;               // this is idle (Right trigger not pressed - remapped to 0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    ASSERT_EQ(teleop_class.GetLeftTriggerVal(), 1);     //Test Left Trigger remapping to 1 otherwise discontinue
    ASSERT_EQ(teleop_class.GetRightTriggerVal(), 0);    //Test Right Trigger remapping to 0 otherwise discontinue
    EXPECT_NEAR(test_twist.linear.x, test_params.linear_scale, 0.01);   //X axis velocity should be max positive scale

    // Test a simulated right pushed 
    axes[test_params.linear_x_pos] = 1.0;               // this is idle (Right trigger not pressed - remapped to 0 in driver)
    axes[test_params.linear_x_neg] = -1.0;              // this is fully pressed (remapped to 1.0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    ASSERT_EQ(teleop_class.GetLeftTriggerVal(), 0);     //Test Left Trigger remapping to 0 otherwise discontinue
    ASSERT_EQ(teleop_class.GetRightTriggerVal(), 1);    //Test Right Trigger remapping to 1 otherwise discontinue
    EXPECT_NEAR(test_twist.linear.x, -(1.0) * test_params.linear_scale, 0.01);   //X axis velocity should be max negative scale
}

/**
 * @brief Tests the expected linear twist output from a given input (simulated Logitech controller)
 * 
 */
TEST( ArmerTeleopFunctionality, LogitechConfigAngularTwistTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop teleop_class;
    geometry_msgs::Twist test_twist;

    // Set frame to BASE_LINK for comparing (Z: up/down | Y: left/right | X: forward/back) 
    // ---> note that all axis configurations are pos/neg expected
    // Also get the current params for testing (default)
    FRAME_CONTROL test_frame = BASE_FRAME;
    teleop_class.SetFrame(test_frame);
    joy_params test_params = teleop_class.GetJoyParams();

    // axes array is 8 elements long (same for logitech and ps4)
    std::vector<float> axes(test_params.max_axes_size, 0);
    // button array is 11 elements long (logitech) and 13 elements (ps4)
    std::vector<int> buttons(test_params.max_btns_size, 0);

    //------------ RIGHT STICK LEFT/RIGHT (Y-Axis) ANGULAR MOVEMENT TESTS -------------------------------
    // Test a simulated right stick pushed directly left (pos) for angular in y axis response
    axes[test_params.angular_roll] = 1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive angular y velocity and max angular scale
    EXPECT_GT(test_twist.angular.y, 0);
    EXPECT_NEAR(test_twist.angular.y, test_params.angular_scale, 0.01);

    // Test a simulated right stick pushed directly right (neg) for neg angular in y axis response
    axes[test_params.angular_roll] = -1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg angular y velocity and max neg angular scale
    EXPECT_LT(test_twist.angular.y, 0);
    EXPECT_NEAR(test_twist.angular.y, -(1.0) * test_params.angular_scale, 0.01);

    //------------ RIGHT STICK UP/DOWN (Z-Axis) ANGULAR MOVEMENT TESTS -------------------------------
    // Test a simulated right stick pushed directly left (pos) for angular in y axis response
    axes[test_params.angular_pitch] = 1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive angular y velocity and max angular scale
    EXPECT_GT(test_twist.angular.z, 0);
    EXPECT_NEAR(test_twist.angular.z, test_params.angular_scale, 0.01);

    // Test a simulated right stick pushed directly right (neg) for neg angular in y axis response
    axes[test_params.angular_pitch] = -1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg linear y velocity and max neg linear scale
    EXPECT_LT(test_twist.angular.z, 0);
    EXPECT_NEAR(test_twist.angular.z, -(1.0) * test_params.angular_scale, 0.01);

    //------------ LEFT/RIGHT ANALOGUE STICK PRESSED FORWARD/BACKWARD (X-Axis) ANGULAR MOVEMENT TESTS -------------------------------
    // Test analogue stick idle states (both not pressed)
    buttons[test_params.angular_yaw_pos] = 0; // this is idle (Left stick not pressed)
    buttons[test_params.angular_yaw_neg] = 0; // this is idle (Right stick not pressed)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    EXPECT_EQ(test_twist.angular.x, 0);                  //X axis velocity should be zero

    // Test both analogue sticks pushed (both pressed should not change output of x axis)
    buttons[test_params.angular_yaw_pos] = 1.0;         // this is fully pressed 
    buttons[test_params.angular_yaw_neg] = 1.0;         // this is fully pressed  
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    EXPECT_EQ(test_twist.linear.x, 0);                  //X axis velocity should be zero

    // Test a simulated left stick pushed 
    buttons[test_params.angular_yaw_pos] = 1.0;         // this is fully pressed
    buttons[test_params.angular_yaw_neg] = 0;           // this is idle (stick not pressed)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    EXPECT_NEAR(test_twist.angular.x, test_params.angular_scale, 0.01);   //X axis velocity should be max positive scale

    // Test a simulated right stick pushed 
    buttons[test_params.angular_yaw_pos] = 0;                 // this is idle (stick not pressed)
    buttons[test_params.angular_yaw_neg] = 1.0;               // this is fully pressed (remapped to 1.0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    EXPECT_NEAR(test_twist.angular.x, -(1.0) * test_params.angular_scale, 0.01);   //X axis velocity should be max negative scale
}

/**
 * @brief Tests the expected linear twist output from a given input (simulated ps4 controller)
 * 
 */
TEST( ArmerTeleopFunctionality, PS4ConfigLinearTwistTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop teleop_class;
    geometry_msgs::Twist test_twist;

    // Update as per PS4 Config
    joy_params test_params;
    test_params.linear_y = 0;
    test_params.linear_z = 1; 
    test_params.linear_x_pos = 2;
    test_params.linear_x_neg = 5; 
    test_params.linear_scale = 0.2;
    test_params.angular_roll = 3; 
    test_params.angular_pitch = 4; 
    test_params.angular_yaw_pos = 11;
    test_params.angular_yaw_neg = 12;
    test_params.angular_scale = 0.4; 
    test_params.deadman_btn = 4;
    test_params.home_btn = 10;
    test_params.toggle_frame_btn = 0;
    test_params.max_axes_size = 8;
    test_params.max_btns_size = 13;
    teleop_class.SetJoyParams(test_params);

    // Set frame to BASE_LINK for comparing (Z: up/down | Y: left/right | X: forward/back) 
    // ---> note that all axis configurations are pos/neg expected
    // Also get the current params for testing (default)
    FRAME_CONTROL test_frame = BASE_FRAME;
    teleop_class.SetFrame(test_frame);

    // axes array is 8 elements long (same for logitech and ps4)
    std::vector<float> axes(test_params.max_axes_size, 0);
    // button array is 11 elements long (logitech) and 13 elements (ps4)
    std::vector<int> buttons(test_params.max_btns_size, 0);

    //------------ LEFT STICK LEFT/RIGHT (Y-Axis) LINEAR MOVEMENT TESTS -------------------------------
    // Test a simulated left stick pushed directly left (pos) for linear in y axis response
    axes[test_params.linear_y] = 1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive linear y velocity and max linear scale
    EXPECT_GT(test_twist.linear.y, 0);
    EXPECT_NEAR(test_twist.linear.y, test_params.linear_scale, 0.01);

    // Test a simulated left stick pushed directly right (neg) for neg linear in y axis response
    axes[test_params.linear_y] = -1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg linear y velocity and max neg linear scale
    EXPECT_LT(test_twist.linear.y, 0);
    EXPECT_NEAR(test_twist.linear.y, -(1.0) * test_params.linear_scale, 0.01);

    //------------ LEFT STICK UP/DOWN (Z-Axis) LINEAR MOVEMENT TESTS -------------------------------
    // Test a simulated left stick pushed directly left (pos) for linear in y axis response
    axes[test_params.linear_z] = 1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive linear y velocity and max linear scale
    EXPECT_GT(test_twist.linear.z, 0);
    EXPECT_NEAR(test_twist.linear.z, test_params.linear_scale, 0.01);

    // Test a simulated left stick pushed directly right (neg) for neg linear in y axis response
    axes[test_params.linear_z] = -1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg linear y velocity and max neg linear scale
    EXPECT_LT(test_twist.linear.z, 0);
    EXPECT_NEAR(test_twist.linear.z, -(1.0) * test_params.linear_scale, 0.01);

    //------------ LEFT/RIGHT TRIGGERS FORWARD/BACKWARD (X-Axis) LINEAR MOVEMENT TESTS -------------------------------
    // Test trigger idle states (not pressed)
    axes[test_params.linear_x_pos] = 1.0; // this is idle (Left trigger not pressed - remapped to 0 in driver)
    axes[test_params.linear_x_neg] = 1.0; // this is idle (Right trigger not pressed - remapped to 0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    ASSERT_EQ(teleop_class.GetLeftTriggerVal(), 0);     //Test Left Trigger remapping to 0 otherwise discontinue
    ASSERT_EQ(teleop_class.GetRightTriggerVal(), 0);    //Test Right Trigger remapping to 0 otherwise discontinue
    EXPECT_EQ(test_twist.linear.x, 0);                  //X axis velocity should be zero

    // Test a simulated left and right trigger pushed (both pressed should not change output of x axis)
    axes[test_params.linear_x_pos] = -1.0;              // this is fully pressed (remapped to 1.0 in driver)
    axes[test_params.linear_x_neg] = -1.0;              // this is fully pressed (remapped to 1.0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    ASSERT_EQ(teleop_class.GetLeftTriggerVal(), 1);     //Test Left Trigger remapping to 1 otherwise discontinue
    ASSERT_EQ(teleop_class.GetRightTriggerVal(), 1);    //Test Right Trigger remapping to 1 otherwise discontinue
    EXPECT_EQ(test_twist.linear.x, 0);                  //X axis velocity should be zero

    // Test a simulated left pushed 
    axes[test_params.linear_x_pos] = -1.0;              // this is fully pressed (remapped to 1.0 in driver)
    axes[test_params.linear_x_neg] = 1.0;               // this is idle (Right trigger not pressed - remapped to 0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    ASSERT_EQ(teleop_class.GetLeftTriggerVal(), 1);     //Test Left Trigger remapping to 1 otherwise discontinue
    ASSERT_EQ(teleop_class.GetRightTriggerVal(), 0);    //Test Right Trigger remapping to 0 otherwise discontinue
    EXPECT_NEAR(test_twist.linear.x, test_params.linear_scale, 0.01);   //X axis velocity should be max positive scale

    // Test a simulated right pushed 
    axes[test_params.linear_x_pos] = 1.0;               // this is idle (Right trigger not pressed - remapped to 0 in driver)
    axes[test_params.linear_x_neg] = -1.0;              // this is fully pressed (remapped to 1.0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    ASSERT_EQ(teleop_class.GetLeftTriggerVal(), 0);     //Test Left Trigger remapping to 0 otherwise discontinue
    ASSERT_EQ(teleop_class.GetRightTriggerVal(), 1);    //Test Right Trigger remapping to 1 otherwise discontinue
    EXPECT_NEAR(test_twist.linear.x, -(1.0) * test_params.linear_scale, 0.01);   //X axis velocity should be max negative scale
}

/**
 * @brief Tests the expected linear twist output from a given input (simulated ps4 controller)
 * 
 */
TEST( ArmerTeleopFunctionality, PS4ConfigAngularTwistTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop teleop_class;
    geometry_msgs::Twist test_twist;

    // Update as per PS4 Config
    joy_params test_params;
    test_params.linear_y = 0;
    test_params.linear_z = 1; 
    test_params.linear_x_pos = 2;
    test_params.linear_x_neg = 5; 
    test_params.linear_scale = 0.2;
    test_params.angular_roll = 3; 
    test_params.angular_pitch = 4; 
    test_params.angular_yaw_pos = 11;
    test_params.angular_yaw_neg = 12;
    test_params.angular_scale = 0.4; 
    test_params.deadman_btn = 4;
    test_params.home_btn = 10;
    test_params.toggle_frame_btn = 0;
    test_params.max_axes_size = 8;
    test_params.max_btns_size = 13;
    teleop_class.SetJoyParams(test_params);

    // Set frame to BASE_LINK for comparing (Z: up/down | Y: left/right | X: forward/back) 
    // ---> note that all axis configurations are pos/neg expected
    // Also get the current params for testing (default)
    FRAME_CONTROL test_frame = BASE_FRAME;
    teleop_class.SetFrame(test_frame);

    // axes array is 8 elements long (same for logitech and ps4)
    std::vector<float> axes(test_params.max_axes_size, 0);
    // button array is 11 elements long (logitech) and 13 elements (ps4)
    std::vector<int> buttons(test_params.max_btns_size, 0);

    //------------ RIGHT STICK LEFT/RIGHT (Y-Axis) ANGULAR MOVEMENT TESTS -------------------------------
    // Test a simulated right stick pushed directly left (pos) for angular in y axis response
    axes[test_params.angular_roll] = 1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive angular y velocity and max angular scale
    EXPECT_GT(test_twist.angular.y, 0);
    EXPECT_NEAR(test_twist.angular.y, test_params.angular_scale, 0.01);

    // Test a simulated right stick pushed directly right (neg) for neg angular in y axis response
    axes[test_params.angular_roll] = -1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg angular y velocity and max neg angular scale
    EXPECT_LT(test_twist.angular.y, 0);
    EXPECT_NEAR(test_twist.angular.y, -(1.0) * test_params.angular_scale, 0.01);

    //------------ RIGHT STICK UP/DOWN (Z-Axis) ANGULAR MOVEMENT TESTS -------------------------------
    // Test a simulated right stick pushed directly left (pos) for angular in y axis response
    axes[test_params.angular_pitch] = 1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive angular y velocity and max angular scale
    EXPECT_GT(test_twist.angular.z, 0);
    EXPECT_NEAR(test_twist.angular.z, test_params.angular_scale, 0.01);

    // Test a simulated right stick pushed directly right (neg) for neg angular in y axis response
    axes[test_params.angular_pitch] = -1.0;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg linear y velocity and max neg linear scale
    EXPECT_LT(test_twist.angular.z, 0);
    EXPECT_NEAR(test_twist.angular.z, -(1.0) * test_params.angular_scale, 0.01);

    //------------ LEFT/RIGHT ANALOGUE STICK PRESSED FORWARD/BACKWARD (X-Axis) ANGULAR MOVEMENT TESTS -------------------------------
    // Test analogue stick idle states (both not pressed)
    buttons[test_params.angular_yaw_pos] = 0; // this is idle (Left stick not pressed)
    buttons[test_params.angular_yaw_neg] = 0; // this is idle (Right stick not pressed)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    EXPECT_EQ(test_twist.angular.x, 0);                  //X axis velocity should be zero

    // Test both analogue sticks pushed (both pressed should not change output of x axis)
    buttons[test_params.angular_yaw_pos] = 1.0;         // this is fully pressed 
    buttons[test_params.angular_yaw_neg] = 1.0;         // this is fully pressed  
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    EXPECT_EQ(test_twist.linear.x, 0);                  //X axis velocity should be zero

    // Test a simulated left stick pushed 
    buttons[test_params.angular_yaw_pos] = 1.0;         // this is fully pressed
    buttons[test_params.angular_yaw_neg] = 0;           // this is idle (stick not pressed)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    EXPECT_NEAR(test_twist.angular.x, test_params.angular_scale, 0.01);   //X axis velocity should be max positive scale

    // Test a simulated right stick pushed 
    buttons[test_params.angular_yaw_pos] = 0;                 // this is idle (stick not pressed)
    buttons[test_params.angular_yaw_neg] = 1.0;               // this is fully pressed (remapped to 1.0 in driver)
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    EXPECT_NEAR(test_twist.angular.x, -(1.0) * test_params.angular_scale, 0.01);   //X axis velocity should be max negative scale
}

/**
 * @brief Main function to run all declared tests above (with TEST())
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main ( int argc, char **argv )
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "armer_teleop_test");


    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();

    ros::shutdown();
    return ret;
}