#include <gtest/gtest.h>

#include <stdlib.h>
#include <string>

// https://medium.com/engineering-iot/unit-testing-on-esp32-with-platformio-a-step-by-step-guide-d33f3241192b
TEST(tomTest, HandlesZeroInput){
    std::string s;
    s="hello";
    s.append("world");


    std::cout << s << std::endl;
    QueueHandle_t webQueue;
    // webQueue = xQueueCreate(12, sizeof(WebMessage));
    // xQueueSend(wchar_t, s, 0);
    // xQueueReceive();
    std::cout << "------------------------------------------" << std::endl;
    SCOPED_TRACE("Tomekejifajsfesoifjoasiejfoiasjfiojfoesij");
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