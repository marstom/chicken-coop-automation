
// mDNS support
#include <ESPmDNS.h>
#include <WiFi.h>

// set hostname for my chickenCOOP server
// @param hostname - name of hot chicken it would be chicken.local
// @param instance_name - "Chicken Coop"
// https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32c3/api-reference/protocols/mdns.html
namespace common
{

    void setupMDNS(const char *hostname, const char *instance_name)
    {
        // GOT_IP fires on every WiFi reconnect; mDNS survives reconnects and
        // re-announces on its own, so only initialize once
        static bool mdnsStarted = false;
        if (mdnsStarted)
        {
            return;
        }

        // initialize mDNS service
        esp_err_t err = mdns_init();
        if (err)
        {
            printf("MDNS Init failed: %d\n", err);
            return;
        }

        // set hostname
        mdns_hostname_set(hostname);
        // set default instance
        mdns_instance_name_set(instance_name);

        Serial.print("mDNS started: http://");
        Serial.print(hostname);
        Serial.println(".local");

        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        // add our services
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        mdns_service_add(NULL, "_ota", "_tcp", 3232, NULL, 0);
        mdns_service_add(NULL, "mqtt", "_tcp", 1883, NULL, 0);
        mdnsStarted = true;
    }

    // HARD reset on WIFi disconnect
    // TODO, try which is better reconnect pattern, or full restart pattern?

    namespace wifi_event
    {
        // cpp way, cpp not allow nested function, like wrapper pattern in python...
        const char *g_mdns_hostname = nullptr;
        const char *g_instance_name = nullptr;
        void onWiFiEvent(WiFiEvent_t event)
        {
            switch (event)
            {
            case ARDUINO_EVENT_WIFI_STA_GOT_IP:
                Serial.print("WiFi connected, IP: ");
                Serial.println(WiFi.localIP());
                setupMDNS(g_mdns_hostname, g_instance_name);
                break;
            case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
                Serial.println("WiFi disconnected. Try reconnect again...");
                WiFi.reconnect();
                break;
            // case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            //     debug_tools::logMessage("WiFi disconnected. Restarting controller...");
            //     delay(3000);
            //     ESP.restart();
            //     break;
            default:
                break;
            }
        }
    }

}
