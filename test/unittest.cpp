// Bring in gtest and ros
#include <gtest/gtest.h>
#include <ros/ros.h>
// Bring in the package's API
#include "armer_teleop.h"

/**
 * @brief Tests the creation of the armer_teleop class
 * 
 */
TEST( ArmerTeleopFunctionality, classCreationTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop new_class;
    ASSERT_EQ(new_class.ClassConstructionSuccess(), true);
}

/**
 * @brief Tests the toggle of the frame state and the button states
 * 
 */
TEST( ArmerTeleopFunctionality, frameChangeTest )
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
 * @brief Tests the expected twist output from a given input (simulated Logitech controller)
 * 
 */
TEST( ArmerTeleopFunctionality, LogitechConfigTwistTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop teleop_class;
    geometry_msgs::Twist test_twist;
    FRAME_CONTROL test_frame = BASE_FRAME;
    joy_params test_params;
    // axes array is 8 elements long (same for logitech and ps4)
    std::vector<float> axes = {0, 0, 0, 0, 0, 0, 0, 0};
    // button array is 11 elements long (logitech) and 13 elements (ps4)
    std::vector<int> buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Set frame to BASE_LINK for comparing (Z: up/down | Y: left/right | X: forward/back) 
    // ---> note that all axis configurations are pos/neg expected
    // Also get the current params for testing (default)
    teleop_class.SetFrame(test_frame);
    test_params = teleop_class.GetJoyParams();

    // Test a simulated left stick pushed directly left (pos) for linear in y axis response
    axes[test_params.linear_y] = 1;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect positive linear y velocity and max linear scale
    EXPECT_GT(test_twist.linear.y, 0);
    EXPECT_NEAR(test_twist.linear.y, test_params.linear_scale, 0.1);

    // Test a simulated left stick pushed directly right (neg) for neg linear in y axis response
    axes[test_params.linear_y] = -1;
    teleop_class.TestConfigureTwist(test_twist, axes, buttons);
    // Expect neg linear y velocity and max neg linear scale
    EXPECT_LT(test_twist.linear.y, 0);
    EXPECT_NEAR(test_twist.linear.y, -(1.0) * test_params.linear_scale, 0.1);

    


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