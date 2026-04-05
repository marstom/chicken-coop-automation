#include <gtest/gtest.h>

#include <stdlib.h>
#include <string>

TEST(tomTest, HandlesZeroInput){
    std::string s;
    s="hello";
    s.append("world");

    EXPECT_EQ(s, "helloworld");

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // if you plan to use GMock, replace the line above with
    // ::testing::InitGoogleMock(&argc, argv);

    if (RUN_ALL_TESTS())
    ;

    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}