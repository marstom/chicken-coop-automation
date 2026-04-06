# Compiler

Platformio plugin.

```sh
 ~/.platformio/penv/bin/python -m pip install intelhex
```

# Home hardware


`src/main_c3_relay_controller.cpp` - door lock with BLE and WiFi control in the basement



# Network setup

First `touch secret.ini` in project root.

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

5G networks:

```
T-Mobile-Tom-5G_EXT
T-Mobile-Tom-5G
```

# MQTT setup
...

Steps:
- Go to raspberry
- cd services, docker compose up, then enable mqtt plugin using ./enable_mqtt_plugin;
- Now dashboard is available http://raspberrypi.local:15672/#/, creds are: admin admin.

- Now run python helper `./cli.py sub`, this will display you tempereatures, pressure etc.