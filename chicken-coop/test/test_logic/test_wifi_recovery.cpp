#include <gtest/gtest.h>

#include "wifi_recovery.h"

// WifiRecovery is the decision core of common::wifi_event::onWiFiEvent - the
// policy whose first version (restart on EVERY disconnect) caused the outages
// this project suffered from.

using logic::WifiAction;
using logic::WifiRecovery;

TEST(WifiRecovery, FirstDisconnectTriesReconnectNotRestart)
{
    WifiRecovery recovery;
    EXPECT_EQ(recovery.onDisconnected(5), WifiAction::Reconnect);
    EXPECT_EQ(recovery.failures(), 1);
}

TEST(WifiRecovery, RestartsExactlyAtThreshold)
{
    WifiRecovery recovery;
    for (int i = 0; i < 4; i++)
    {
        EXPECT_EQ(recovery.onDisconnected(5), WifiAction::Reconnect) << "attempt " << i + 1;
    }
    EXPECT_EQ(recovery.onDisconnected(5), WifiAction::Restart);
}

TEST(WifiRecovery, ThresholdZeroNeverRestarts)
{
    // relay controller policy: physical access is easy, never self-restart
    WifiRecovery recovery;
    for (int i = 0; i < 1000; i++)
    {
        EXPECT_EQ(recovery.onDisconnected(0), WifiAction::Reconnect);
    }
}

TEST(WifiRecovery, GotIpResetsTheCounter)
{
    // The original bug class: transient AP glitches must not accumulate
    // towards a restart once the connection recovers.
    WifiRecovery recovery;
    for (int i = 0; i < 4; i++)
    {
        recovery.onDisconnected(5);
    }
    EXPECT_EQ(recovery.failures(), 4);

    EXPECT_EQ(recovery.onGotIp(), WifiAction::None);
    EXPECT_EQ(recovery.failures(), 0);

    // four more failures after recovery: still below threshold
    for (int i = 0; i < 4; i++)
    {
        EXPECT_EQ(recovery.onDisconnected(5), WifiAction::Reconnect);
    }
}

TEST(WifiRecovery, FlappingConnectionNeverRestarts)
{
    // disconnect/reconnect cycles (e.g. nightly router hiccup) forever:
    // as long as GOT_IP arrives in between, no restart should happen
    WifiRecovery recovery;
    for (int cycle = 0; cycle < 100; cycle++)
    {
        EXPECT_EQ(recovery.onDisconnected(5), WifiAction::Reconnect);
        recovery.onGotIp();
    }
}

TEST(WifiRecovery, ThresholdOneRestartsImmediately)
{
    // threshold 1 reproduces the old restart-on-first-disconnect behavior
    WifiRecovery recovery;
    EXPECT_EQ(recovery.onDisconnected(1), WifiAction::Restart);
}
