#pragma once

#include <Arduino.h>
#include <PubSubClient.h>

namespace common::mqtt
{
    /// Everything the shared MQTT task needs to serve one board.
    /// Must have static lifetime - the task keeps a pointer to it.
    struct TaskConfig
    {
        PubSubClient *client;
        const char *user;           // "" or nullptr = connect without credentials
        const char *pass;
        const char *thingName;
        const char *subscribeTopic; // nullptr = no subscription
        const char *statusTopic;    // nullptr = no status message on connect
        const char *statusPayload;
    };

    bool connectToBroker(PubSubClient &client, const char *user, const char *pass, const char *thingName);

    /// Shared MQTT service task: connects and reconnects to the broker
    /// (re-subscribing after every reconnect), publishes messages queued in
    /// `communication::mqttQueue`, and forces a WiFi reconnect if the link
    /// stays down with no event-driven recovery.
    /// `pvParameters` must point to a `TaskConfig` with static lifetime.
    void taskMQTT(void *pvParameters);
}
