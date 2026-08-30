# ESP32 Home Automation

Two ESP32-C3 nodes running on my home network:

| Target | Hardware | What it does |
|---|---|---|
| **Basement door lock** | Seeed XIAO ESP32-C3 | Relay-controlled door, TCP + BLE access, token-authenticated HTTP endpoint |
| **Chicken coop monitor** | Seeed XIAO ESP32-C3 | BME280 (temp/humidity/pressure) + ammonia sensor, publishes to MQTT |

Both nodes connect to WiFi, publish diagnostics over MQTT, and support OTA firmware updates.

---

## Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- Python `intelhex` package (Mac only):
  ```sh
  ~/.platformio/penv/bin/python -m pip install intelhex
  ```

---

## Configuration

Create `secret.ini` in the project root (it is gitignored):

```ini
[secret]
build_flags =
  -DWIFI_SSID=\"<your-ssid>\"
  -DWIFI_PASS=\"<your-password>\"
  -DMQTT_HOST=\"<broker-ip>\"
  -DMQTT_PORT=1883
  -DDOOR_TOKEN=\"<secret-token>\"
```

---

## Build targets

| Environment | Command | Notes |
|---|---|---|
| Door lock (USB) | `pio run -e esp32-c3-relay-controller-usb -t upload` | Upload via USB |
| Door lock (OTA) | `pio run -e esp32-c3-relay-controller-remote -t upload` | Upload to `192.168.0.140` |
| Chicken coop | `pio run -e esp32-c3-chicken-coop -t upload` | Upload via USB |
| Chicken coop (OTA) | `pio run -e esp32-c3-chicken-coop-remote -t upload` | Upload to `chicken.local` |

---

## Tests

Logic is kept in `lib/logic/` as pure C++ (no Arduino/FreeRTOS dependencies) so it runs natively on your machine with GoogleTest.

```sh
make test            # run all tests
make test-verbose    # show individual pass/fail
make test-filter FILTER=DoorAuth   # run one suite
```

Or directly with PlatformIO:

```sh
pio test -e native -v
```

Test suites in `test/test_logic/`:

- `DoorAuth` — token auth for the basement TCP endpoint
- `WifiRecovery` — reconnect vs. restart policy (guards against the restart-loop outage)
- `BoundedCopy` — safe string copy used in MQTT/web messages

---

## MQTT / RabbitMQ

The broker runs in Docker on a Raspberry Pi:

```sh
cd services && docker compose up -d
./enable_mqtt_plugin
```

Management UI: `http://raspberrypi.local:15672` (admin / admin)

Watch live sensor data:

```sh
./py/cli.py sub
```

---

## Discovering devices on the network

```sh
dns-sd -B _http._tcp    # find HTTP servers (door-lock, chicken)
dns-sd -B _ota._tcp     # find OTA targets
dns-sd -B _mqtt._tcp    # find MQTT brokers
```

---

## Project structure

```
src/
  main_c3_relay_controller.cpp        # door lock firmware
  main_c3_chicken_coop_...sensor.cpp  # chicken coop firmware

lib/logic/
  door_auth.h/cpp      # token auth logic
  wifi_recovery.h      # reconnect/restart policy
  bounded_copy.h       # safe strncpy wrapper
  mqtt_comm.h/cpp      # MQTT message structs

test/test_logic/       # native GoogleTest suite

docs/                  # MkDocs site
Android/               # BLE Android app (WIP)
```

---

## Full docs

```sh
cd docs && mkdocs serve
```

Then open `http://localhost:8000`.

---

## Hardware

- [2-channel relay module](https://botland.com.pl/moduly-przekaznikow/14266-modul-przekaznikow-iduino-2-kanaly-z-optoizolacja-styki-10a250vac-cewka-5v-5903351242332.html) — 10A/250VAC, optoisolated, 5V coil
- [BME280](https://botland.com.pl/czujniki-cisnienia/16534-bme280-czujnik-wilgotnosci-temperatury-oraz-cisnienia-110kpa-i2c-spi-33v-wlutowane-zlacza-5904422378189.html) — temp, humidity, pressure via I2C (SDA→D4, SCL→D5)
- [Ammonia sensor NH3](https://botland.com.pl/czujniki-gazow/23744-fermion-czujnik-amoniaku-nh3-mems-1-300ppm-dfrobot-sen0567-6959420923809.html) — 1–300 ppm MEMS
- [Seeed XIAO ESP32-C3](https://botland.com.pl/moduly-wifi-i-bt-esp32/21859-seeed-xiao-esp32-c3-wifibluetooth-seeedstudio-113991054.html) — 32-bit RISC-V 160 MHz, Wi-Fi + BT
