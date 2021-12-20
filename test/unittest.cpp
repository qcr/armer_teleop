// Bring in the package's API
#include "armer_teleop.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test <TEMPLATE>
TEST( TestSuite, classCreationTest )
{
    EXPECT_TRUE(true);
}

TEST( TestSuite, classCreationTest2 )
{
    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main ( int argc, char **argv )
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "armer_teleop_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}