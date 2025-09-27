/*

How to?

create file in root
private.ini

[secret]
build_flags =
  -DWIFI_SSID=\"mySSID\"
  -DWIFI_PASS=\"pass\"
*/

#ifndef WIFI_SSID
  #define WIFI_SSID "unset"
#endif
#ifndef WIFI_PASS
  #define WIFI_PASS "unset"
#endif


namespace secrets {
    const char* wifiSsid = WIFI_SSID;
    const char* wifiPass = WIFI_PASS;
}