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
#ifndef SSID_OFFICE
  #define SSID_OFFICE "unset"
#endif
#ifndef SSID_KITCHEN
  #define SSID_KITCHEN "unset"
#endif
#ifndef SSID_GARDEN
  #define SSID_GARDEN "unset"
#endif
#ifndef WIFI_PASS
  #define WIFI_PASS "unset"
#endif


/// Alternative - namespace
// namespace secrets {
//     const char* wifiSsidOffice = WIFI_SSID_OFFICE;
//     const char* wifiSsidKitchen = WIFI_SSID;
//     const char* wifiSsidGarden = WIFI_SSID;
//     const char* wifiPass = WIFI_PASS;
// }