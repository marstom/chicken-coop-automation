#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

namespace aws
{
  class AWSIotConnector
  {
  private:
    // must be declared before mqttClient - members are initialized in declaration order
    WiFiClientSecure secureClient;
    PubSubClient mqttClient;

  public:
    AWSIotConnector() : mqttClient(secureClient) {}

    void setup_aws_iot(const char* aws_root_ca, const char* device_cert,
                       const char* device_private_key, const char* aws_endpoint, const int aws_port)
    {
      secureClient.setCACert(aws_root_ca);
      secureClient.setCertificate(device_cert);
      secureClient.setPrivateKey(device_private_key);

      mqttClient.setServer(aws_endpoint, aws_port);
      mqttClient.setBufferSize(1024); // default 256 B is smaller than the payload
      mqttClient.setKeepAlive(60);    // AWS IoT accepts 30-1200 s, default is 15
    }

    bool connect_aws_iot()
    {
      if (mqttClient.connected())
      {
        return true;
      }

      Serial.println("Connecting to AWS IoT...");

      bool connected = mqttClient.connect("ventilation_temperature_sensors");

      if (connected)
      {
        Serial.println("Connected to AWS IoT");
      }
      else
      {
        Serial.printf("MQTT error: %d\n", mqttClient.state());
      }

      return connected;
    }
  };

}; // namespace aws
