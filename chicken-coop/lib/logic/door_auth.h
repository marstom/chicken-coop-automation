#pragma once

// Pure logic - no Arduino/FreeRTOS includes, so it is natively testable.

namespace logic
{
    /// The door TCP endpoint only answers "GET /open?token=<expectedToken>".
    /// An expected token of "unset" (the secrets.h default) or empty disables
    /// the endpoint entirely.
    /// `requestLine` is the raw first HTTP request line, possibly with a
    /// trailing '\r' (WiFiClient::readStringUntil('\n') keeps it).
    bool isAuthorizedDoorRequest(const char *requestLine, const char *expectedToken);
}
