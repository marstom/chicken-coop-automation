#include <gtest/gtest.h>

#include <cstring>

#include "bounded_copy.h"

// copyBounded backs MqttMessage::setContent and WebMessage::setContent.
// The buffer sizes used below mirror the real structs in mqtt_comm.h:
//   MqttMessage: topic[32], payload[64]
//   WebMessage:  buffer[64], msgType[12]

TEST(BoundedCopy, CopiesShortStringExactly)
{
    char dst[32];
    logic::copyBounded(dst, sizeof(dst), "coop/status");
    EXPECT_STREQ(dst, "coop/status");
}

TEST(BoundedCopy, ExactFitIsCopiedAndTerminated)
{
    // 11 chars into char[12] - exactly fills it, like "temperature" did
    char dst[12];
    logic::copyBounded(dst, sizeof(dst), "temperature");
    EXPECT_STREQ(dst, "temperature");
    EXPECT_EQ(strlen(dst), 11u);
}

TEST(BoundedCopy, OverlongInputIsTruncatedAndTerminated)
{
    // This is the bug that existed in WebMessage::setContent: plain strncpy
    // left msgType unterminated when the input filled the buffer.
    char dst[12];
    logic::copyBounded(dst, sizeof(dst), "temperature-outside");
    EXPECT_EQ(strlen(dst), 11u);              // never overflows the buffer
    EXPECT_STREQ(dst, "temperature");         // truncated, not garbage
    EXPECT_EQ(dst[sizeof(dst) - 1], '\0');    // always terminated
}

TEST(BoundedCopy, RealWorldMqttTopicTruncation)
{
    // MqttMessage::topic is char[32]; an over-long topic must not corrupt memory
    char topic[32];
    logic::copyBounded(topic, sizeof(topic), "coop/some/very/long/topic/name/that/exceeds/the/buffer");
    EXPECT_EQ(strlen(topic), 31u);
    EXPECT_EQ(topic[31], '\0');
}

TEST(BoundedCopy, NullSourceYieldsEmptyString)
{
    char dst[8] = "junk";
    logic::copyBounded(dst, sizeof(dst), nullptr);
    EXPECT_STREQ(dst, "");
}

TEST(BoundedCopy, SizeOneBufferYieldsEmptyString)
{
    char dst[1] = {'x'};
    logic::copyBounded(dst, sizeof(dst), "anything");
    EXPECT_STREQ(dst, "");
}

TEST(BoundedCopy, NullDestinationIsSafe)
{
    logic::copyBounded(nullptr, 32, "anything"); // must not crash
}
