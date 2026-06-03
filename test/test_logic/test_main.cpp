#include <gtest/gtest.h>

// Native test suite for the hardware-free logic in lib/logic.
// Run with:  pio test -e native

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
