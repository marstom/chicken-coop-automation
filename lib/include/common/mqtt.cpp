#include "common/mqtt.h"

#include "debug_tools/debug_tools.h"
#include "mqtt_comm.h"

namespace common::mqtt
{

    Mqtt::Mqtt(PubSubClient &mqttClient) : client(mqttClient)
    {
        debug_tools::logMessage("Mqtt constructor");
    }

    Mqtt::~Mqtt()
    {
        debug_tools::logMessage("Mqtt destructor");
    }

    void Mqtt::addCallback(MessageCallback callback)
    {
        client.setCallback(callback);
    }

    void Mqtt::setupAndConnect(
        ConnectFn connectToBroker,
        const char *host,
        uint16_t port,
        const char *subscribeTopic,
        const char *statusTopic,
        const char *statusPayload)
    {

        client.setServer(host, port);

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

    bool Mqtt::connectToMqttBroker(const char *mqttUser, const char *mqttPass, const char *thingName)
    {
        if (mqttUser[0] != '\0')
        {
            return client.connect(thingName, mqttUser, mqttPass);
        }

        return client.connect(thingName);
    }
}
