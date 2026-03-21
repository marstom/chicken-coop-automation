# Compiler

Platformio plugin.

```sh
 ~/.platformio/penv/bin/python -m pip install intelhex
```

# Home hardware


`src/main_c3_relay_controller.cpp` - door lock with BLE and WiFi control in the basement



# Network setup

The wifi pass and ssid:

```ini
[secret]
build_flags =
  -DSSID_OFFICE=\"T-Mobile-Tom_bi_paulina_tomek\"
  -DSSID_KITCHEN=\"T-Mobile-Tom\"
  -DSSID_GARDEN=\"T-Mobile-Tom_EXT\"
  -DWIFI_SSID=\"T-Mobile-<SSID>\"
  -DWIFI_PASS=\"<PASSWORD>\"
``` 

# MQTT setup
...

