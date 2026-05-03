#pragma once

#include <functional>

#include <Arduino.h>
#include <PubSubClient.h>

namespace common::mqtt
{
    using ConnectFn = std::function<bool()>;
    using MessageCallback = std::function<void(char *, uint8_t *, unsigned int)>;

    class Mqtt
    {
    private:
        PubSubClient &client;

    public:
        Mqtt(PubSubClient &mqttClient);
        ~Mqtt();
        void addCallback(MessageCallback callback);
        void setupAndConnect(
            ConnectFn connectToBroker,
            const char *host,
            uint16_t port,
            const char *subscribeTopic,
            const char *statusTopic,
            const char *statusPayload);
        bool connectToMqttBroker(const char *mqttUser, const char *mqttPass, const char *thingName);
    };

}
