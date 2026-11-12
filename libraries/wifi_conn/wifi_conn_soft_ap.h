#pragma once
#include <WiFi.h>

namespace wifi
{
  void connectToWifiWithSoftAPProvisioning(const char* hostname, const char* service_name,
                                     const char* pop, bool autoReconnect);
} // namespace wifi
