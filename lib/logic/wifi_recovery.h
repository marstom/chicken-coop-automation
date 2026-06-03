#pragma once

// Pure logic - no Arduino/FreeRTOS includes, so it is natively testable.

namespace logic
{
    enum class WifiAction
    {
        None,      // nothing to do
        Reconnect, // call WiFi.reconnect()
        Restart,   // give up, ESP.restart()
    };

    /// Decision core of the WiFi recovery policy: try to reconnect on every
    /// disconnect, hard-restart only after N consecutive failures. Every
    /// failed reconnect attempt fires another DISCONNECTED event, so the
    /// counter keeps growing until a GOT_IP event resets it.
    /// The caller (the WiFi event handler) owns the actual side effects.
    class WifiRecovery
    {
    public:
        WifiAction onGotIp()
        {
            failureCount = 0;
            return WifiAction::None;
        }

        /// `restartAfterFailures`: 0 = never restart, keep reconnecting forever.
        WifiAction onDisconnected(int restartAfterFailures)
        {
            failureCount++;
            if (restartAfterFailures > 0 && failureCount >= restartAfterFailures)
            {
                return WifiAction::Restart;
            }
            return WifiAction::Reconnect;
        }

        int failures() const
        {
            return failureCount;
        }

    private:
        int failureCount = 0;
    };
}
