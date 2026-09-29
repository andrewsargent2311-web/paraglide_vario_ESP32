#include <gtest/gtest.h>

// PlatformIO's googletest framework does not supply main().
// One entry point for every test file in this suite.
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
