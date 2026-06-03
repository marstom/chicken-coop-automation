#include <gtest/gtest.h>

#include "door_auth.h"

// Security-critical: every test case here is an attack scenario against the
// basement door TCP endpoint.

namespace
{
    constexpr const char *TOKEN = "secret123";
}

TEST(DoorAuth, AcceptsCorrectToken)
{
    EXPECT_TRUE(logic::isAuthorizedDoorRequest("GET /open?token=secret123 HTTP/1.1", TOKEN));
}

TEST(DoorAuth, AcceptsTrailingCarriageReturn)
{
    // WiFiClient::readStringUntil('\n') keeps the '\r' of the "\r\n" line end
    EXPECT_TRUE(logic::isAuthorizedDoorRequest("GET /open?token=secret123 HTTP/1.1\r", TOKEN));
}

TEST(DoorAuth, AcceptsBareRequestWithoutHttpVersion)
{
    // e.g. someone testing with netcat: no " HTTP/1.1" suffix
    EXPECT_TRUE(logic::isAuthorizedDoorRequest("GET /open?token=secret123", TOKEN));
    EXPECT_TRUE(logic::isAuthorizedDoorRequest("GET /open?token=secret123\r", TOKEN));
}

TEST(DoorAuth, RejectsWrongToken)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET /open?token=wrong HTTP/1.1", TOKEN));
}

TEST(DoorAuth, RejectsTokenThatIsPrefixOfExpected)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET /open?token=secret12 HTTP/1.1", TOKEN));
}

TEST(DoorAuth, RejectsTokenThatExtendsExpected)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET /open?token=secret1234 HTTP/1.1", TOKEN));
}

TEST(DoorAuth, RejectsEmptyToken)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET /open?token= HTTP/1.1", TOKEN));
}

TEST(DoorAuth, RejectsMissingTokenParameter)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET /open HTTP/1.1", TOKEN));
}

TEST(DoorAuth, RejectsRootPath)
{
    // the original pre-fix behavior: ANY request opened the door
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET / HTTP/1.1", TOKEN));
}

TEST(DoorAuth, RejectsOtherMethods)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("POST /open?token=secret123 HTTP/1.1", TOKEN));
}

TEST(DoorAuth, RejectsGarbage)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("", TOKEN));
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("\r\n", TOKEN));
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("xx", TOKEN));
}

TEST(DoorAuth, UnsetExpectedTokenDisablesEndpoint)
{
    // secrets.h defaults DOOR_TOKEN to "unset" - the endpoint must be dead
    // then, even for a request that literally sends "unset"
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET /open?token=unset HTTP/1.1", "unset"));
}

TEST(DoorAuth, EmptyExpectedTokenDisablesEndpoint)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET /open?token= HTTP/1.1", ""));
}

TEST(DoorAuth, NullArgumentsAreSafe)
{
    EXPECT_FALSE(logic::isAuthorizedDoorRequest(nullptr, TOKEN));
    EXPECT_FALSE(logic::isAuthorizedDoorRequest("GET /open?token=x HTTP/1.1", nullptr));
    EXPECT_FALSE(logic::isAuthorizedDoorRequest(nullptr, nullptr));
}
