#pragma once

#include <functional>

#include <Arduino.h>
#include <PubSubClient.h>

namespace common::mqtt
{
    using ConnectFn = bool (*)();
    using MessageCallback = std::function<void(char *, uint8_t *, unsigned int)>;

    void setupAndConnect(
        PubSubClient &client,
        const char *host,
        uint16_t port,
        MessageCallback callback,
        ConnectFn connectToBroker,
        const char *subscribeTopic,
        const char *statusTopic,
        const char *statusPayload);
}
