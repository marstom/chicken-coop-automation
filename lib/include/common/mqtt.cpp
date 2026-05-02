#include "common/mqtt.h"

#include "debug_tools/debug_tools.h"
#include "mqtt_comm.h"

namespace common::mqtt
{
    void setupAndConnect(
        PubSubClient &client,
        const char *host,
        uint16_t port,
        MessageCallback callback,
        ConnectFn connectToBroker,
        const char *subscribeTopic,
        const char *statusTopic,
        const char *statusPayload)
    {
        client.setServer(host, port);
        client.setCallback(callback);

        while (!client.connected())
        {
            if (connectToBroker())
            {
                debug_tools::logMessage("☑ Connected to MQTT broker!");
                client.subscribe(subscribeTopic);

                communication::MqttMessage msg;
                msg.setContent(statusTopic, statusPayload);
                msg.sendToQueue();
            }
            else
            {
                debug_tools::logMessage("✖ Failed to connect, try again in 2 seconds, rc=");
                debug_tools::logMessage("%d", client.state());
                delay(2000);
            }
        }
    }
}
