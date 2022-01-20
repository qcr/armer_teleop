// Bring in gtest and ros
#include <gtest/gtest.h>
#include <ros/ros.h>
// Bring in the package's API
#include "armer_teleop.h"

/**
 * @brief Tests the creation of the armer_teleop class
 * 
 */
TEST( TestSuite, classCreationTest )
{
    ros::NodeHandle nh("~");
    ArmerTeleop new_class;
    ASSERT_EQ(new_class.ClassConstructionSuccess(), true);
}

/**
 * @brief Tests the toggle of the frame state and the button states
 * 
 */
TEST( TestSuite, frameChangeTest )
{   
    ros::NodeHandle nh("~");
    BUTTON_STATES btn_state;
    FRAME_CONTROL frame_state;
    ArmerTeleop teleop_class;
    // button array is 11 elements long (logitech) and 13 elements (ps4)
    std::vector<int> buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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