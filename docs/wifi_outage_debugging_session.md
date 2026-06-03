# WiFi Outage Debugging, Refactoring & Testing Notes

> Notes from a Claude Code session (2026-06-03). The problem: after adding a
> WiFi reconnect/restart mechanism, devices sometimes went silent and needed a
> manual power-cycle. This document captures the root causes, the fixes, the
> code-smell review, and a lesson on testing embedded code.

---

## Part 1: Why the devices needed manual restarts

### Bug #1 — MQTT never reconnected after WiFi came back (relay controller)

The relay controller's `taskMQTT` only called `client.loop()`:

```cpp
for (;;)
{
    client.loop();  // returns false forever once the connection drops
    ...
}
```

When WiFi drops, the TCP socket to the broker dies. `WiFi.reconnect()` brings
WiFi back, but **PubSubClient never reconnects and never re-subscribes**.
The device looks alive (WiFi OK, ping OK) but is permanently deaf to MQTT
commands — until a manual restart re-runs the blocking connect in `setup()`.

**Lesson:** an event-driven WiFi reconnect is not enough. Every network client
sitting on top of WiFi (MQTT, etc.) needs its own reconnect + re-subscribe
logic, checked periodically from a task.

### Bug #2 — Watchdog starvation in the TCP server task

```cpp
if (client)
{
    esp_task_wdt_add(NULL);   // subscribes to the WDT on the FIRST request...
    ...handle request...
    esp_task_wdt_reset();     // ...but only feeds it WHILE handling requests
}
```

After the first HTTP request the task was registered with the task watchdog,
but idle iterations never called `esp_task_wdt_reset()` → the device
**panicked and rebooted ~30 s (WDT_TIMEOUT) after any TCP request**.

**Lesson:** `esp_task_wdt_add()` belongs *before* the task loop, and
`esp_task_wdt_reset()` must run on **every** iteration — including idle ones.

### Bug #3 — Restart-on-disconnect fired during the initial connection

The event handler was registered *before* `WiFi.begin()`:

```cpp
WiFi.onEvent(onWiFiEvent);          // handler restarts on DISCONNECTED
connect_to_wifi_with_wait(...);     // begin() + wait loop
```

On ESP32, **every failed connect attempt also fires
`ARDUINO_EVENT_WIFI_STA_DISCONNECTED`** (reasons like `NO_AP_FOUND`,
`AUTH_EXPIRE`). If the AP is slow to come up (e.g. after a power blip), the
device loops: boot → disconnect event → `ESP.restart()` → repeat.

**Lesson:** register WiFi event handlers *after* the initial connection, or
ignore disconnect events until you've been connected at least once.

### Bug #4 — Unprotected infinite waits in `setup()`

Two blocking loops ran before the watchdog was armed and before OTA started:

- `while (WiFi.status() != WL_CONNECTED)` — called `WiFi.begin()` **once** and
  spun forever. Status can wedge in `WL_CONNECT_FAILED` / `WL_NO_SSID_AVAIL`
  with no further events → frozen until manual restart.
- `while (!client.connected())` for MQTT — broker down at boot meant **no OTA,
  no web server, no tasks**. Unrecoverable remotely.

**Lessons:**
- Re-issue `WiFi.begin()` every ~15 s while waiting; don't trust one attempt.
- Never block `setup()` on a network service. Start OTA and tasks first, let a
  task own the broker connection.

### Bug #5 — Heavy work inside the WiFi event callback

The event handler runs on the **system event task**. It contained
`delay(3000)` (blocks all network event processing) and re-ran `mdns_init()` +
`mdns_service_add()` on every `GOT_IP` (duplicate services on each reconnect).

**Lessons:**
- Keep WiFi event handlers tiny: set a flag, bump a counter, return.
- Guard one-time initialization (mDNS) with a `static bool` — `GOT_IP` fires
  on *every* reconnect, and mDNS survives reconnects on its own.

### Likely root cause of the disconnects themselves

- **BLE + WiFi coexistence** on a single-radio ESP32-C3 (ArduinoBLE running
  alongside WiFi) — classic source of random drops.
- **Modem sleep**: `WiFi.setSleep(false)` prevents keep-alive timeouts and
  random disconnects, at the cost of some power.

### The recovery pattern that replaced restart-on-disconnect

```text
DISCONNECTED event → counter++ → WiFi.reconnect()
GOT_IP event       → counter = 0
counter >= N       → ESP.restart()        (last resort; N=5 on remote board,
                                           N=0 (never) where physical access is easy)
task-level safety net: if WiFi.status() != WL_CONNECTED for > 30 s
                       with no event-driven recovery → WiFi.reconnect()
```

The safety net matters because sometimes **no disconnect event is ever
delivered** ("zombie" state) — a periodic status check from a task is the only
thing that catches it.

Plus in the connect helper: `WIFI_STA` mode, `setSleep(false)`,
`setAutoReconnect(true)`, retry `begin()` every 15 s, don't print the password.

---

## Part 2: Code smells found in review

### Security
1. **TCP endpoint opened the door for ANY connection** — no path/token check.
   Any port scanner could open a basement door. Fixed: requires
   `GET /open?token=<DOOR_TOKEN>`, 403 otherwise; `"unset"` disables it.
2. **Hardcoded BLE password in source** (`"paulina"` in tasks.cpp). Moved to a
   `BLE_DOOR_PASS` build flag in `secret.ini`.
3. **Real password committed in a comment** in platformio.ini — git history
   remembers; rotate the credential, delete the comment.

### Robustness
4. Stack-local object (`Mqtt mqtt(...)` in `setup()`) that dies at the end of
   setup — worked by accident through a reference to a global.
5. `sendToQueue(timeout_ms)` ignored its parameter and blocked
   `portMAX_DELAY` — a full queue could hang a sensor task forever.
   Drop-on-timeout is the right behavior for telemetry.
6. `strncpy` into an exactly-sized buffer with no explicit null termination —
   `char msgType[12]` + `"temperature"` (11 chars) fit with zero margin.
   Always `dst[sizeof(dst)-1] = '\0'` after `strncpy`.
7. Door-open logic (`LOW → wait 6 s → HIGH`) duplicated in 3 tasks, two of
   which blocked their own task for 6 s. Fixed: **one owner of the pin**
   (`taskRelay`), everyone else posts a command to `relayQueue`.
8. A debug helper that ignored its parameter (`printStackInfo` always printed
   "MQTTTask").
9. Copy-paste leftovers: the coop *sensor* subscribed to a relay topic and
   announced itself as "relay controller".

### Duplication
The `common/` library existed but was only half-adopted. Duplicated across
both boards: `setupMDNS`, `onWiFiEvent`, `taskMQTT` + broker connect (3
copies!), `taskStackMonitor`, OTA setup, and platformio.ini env boilerplate.

**Refactor outcome:** one shared `common::mqtt::taskMQTT` driven by a
`TaskConfig` struct (client, credentials, subscribe topic, status topic), one
`onWiFiEvent` with a configurable `g_restart_after_failures`, one data-driven
stack monitor (`MonitoredTask` watch list), one OTA setup. ~240 lines deleted,
all 4 PlatformIO envs build.

```cpp
// The config-struct pattern for shared tasks (static lifetime required —
// the task keeps a pointer):
static common::mqtt::TaskConfig mqttConfig = {
    &chicken_coop::client, MQTT_USER, MQTT_PASS,
    chicken_coop::THINGNAME,
    nullptr,                       // no subscription on a publish-only board
    chicken_coop::STATUS_TOPIC,
    "{\"message\": \"...\"}",
};
xTaskCreatePinnedToCore(common::mqtt::taskMQTT, "taskMQTT", 2048 * 4, &mqttConfig, 1, &hMQTTTask, 0);
```

### Minor
- `mqttQueue` was 200 × ~96 B ≈ 19 KB of RAM; 30 entries is plenty.
- Magic numbers → named constants (`DOOR_OPEN_MS`).
- Commented-out dead code: delete it, git remembers.
- platformio.ini: use `extends =` (including `extends = env:other-env`) to
  collapse duplicated env sections.

### ⚠️ Outstanding action items
- [ ] Set a real `DOOR_TOKEN` in `secret.ini` (TCP door is disabled until then).
- [ ] Rotate the WiFi password and BLE password (both are in git history).
- [ ] Consider untracking `secret.ini` (`git rm --cached` + `.gitignore` +
      a committed `secret.ini.example`).
- [ ] Hardware sanity-check of the door (BLE + token URL) — relay paths changed.
- [ ] Possible future: directory restructure (`lib/include/common` →
      `lib/common/`), native test suite.

---

## Part 3: How to test embedded code (lesson)

### The core idea

Embedded code is "hard to test" because functions touch hardware
(`digitalWrite`, `WiFi.status()`, `ESP.restart()`). The fix is a *design*
move, not a tooling move: **separate the decision from the action, then test
the decision.** Make the untestable part so dumb it can't be wrong.

### The testing pyramid for this project

```
        ┌──────────────┐
        │  Hardware-in- │   flash real board, poke it from outside
        │  the-loop     │   (slow, few tests)
        ├──────────────┤
        │  On-target    │   pio test runs Unity tests ON the ESP32
        │  tests        │   (real FreeRTOS / I2C — medium cost)
        ├──────────────┤
        │  Native tests │   run on the PC in milliseconds, no board
        │  (pure logic) │   ← 80% of the value lives here
        └──────────────┘
```

The repo already has `[env:native]` with googletest configured and a stub in
`test/test_something.cpp`.

### Level 1 — natively testable today (pure logic)

`isAuthorizedDoorRequest()` is security code whose test cases are the attack
scenarios:

```cpp
TEST(DoorAuth, RejectsWrongToken)   { EXPECT_FALSE(isAuthorizedDoorRequest("GET /open?token=wrong HTTP/1.1")); }
TEST(DoorAuth, RejectsMissingToken) { EXPECT_FALSE(isAuthorizedDoorRequest("GET /open HTTP/1.1")); }
TEST(DoorAuth, RejectsRootPath)     { EXPECT_FALSE(isAuthorizedDoorRequest("GET / HTTP/1.1")); }
TEST(DoorAuth, AcceptsCorrectToken) { EXPECT_TRUE(isAuthorizedDoorRequest("GET /open?token=GOOD HTTP/1.1")); }
```

Same for `MqttMessage`/`WebMessage` truncation — the `msgType`
null-termination bug would have been caught by
`EXPECT_EQ(strlen(msg.getMessageType()), 11)` with an oversized input.

### Level 2 — designing for testability ("seams")

**a) Extract pure functions** — take the decision out, leave the I/O in:

```cpp
bool isDoorPasswordValid(const char *received, const char *expected);  // pure, testable
// the task just wires it to hardware (thin, trivially correct)
```

**b) Dependency injection** — `TaskConfig` already does this for the MQTT
client. Push further with interfaces:

```cpp
struct DoorDriver {                       // the "seam"
    virtual void energize() = 0;
    virtual void release() = 0;
};
class RelayDoor : public DoorDriver { /* digitalWrite */ };   // production
class FakeDoor  : public DoorDriver { bool open = false; };   // tests
```

Now "command `on=true` opens the door and closes it after `DOOR_OPEN_MS`" is
testable without a relay or a basement.

**c) ArduinoFake** — mocks the Arduino API itself so native tests can assert
`digitalWrite(D0, LOW)` happened. Verbose; prefer (a) and (b) first.

### Level 3 — on-target tests

`pio test -e <board-env>` flashes a Unity test firmware and reports over
serial. Use for things only realistic on-chip: FreeRTOS queue behavior, stack
sizes, real I2C sensors. Keep these few — they're slow.

### Level 4 — system tests from the outside

MQTT is a test interface that needs **zero firmware changes**. A Python script
can publish `basement/relay/1/set` and assert the status topic answers, or
subscribe to `coop/bme280/temperature` and assert a reading arrives every ~1 s.

The most valuable test for this project: **turn the AP off for 2 minutes, turn
it back on, assert both devices resume publishing within N seconds.** That's a
regression test for the exact outage this session fixed.

### Priority order for this codebase

| Priority | What | Level | Why |
|---|---|---|---|
| 1 | `isAuthorizedDoorRequest` | native | security-critical, pure already |
| 2 | `MqttMessage`/`WebMessage` truncation | native | had a real bug; buffer logic |
| 3 | WiFi disconnect counter (extract from `onWiFiEvent`) | native | it caused the outages |
| 4 | "AP outage → recovery" script | system | guards the actual fix |
| 5 | Door-open timing via `FakeDoor` | native | the thing that locks you out |

**The pattern to internalize:** every bug fixed in this session lived in logic
that *could* have been a pure function — the restart loop, the missing null
terminator, the token check. None of them needed hardware to be wrong, so none
of them need hardware to be tested.

Step zero of a real test effort: carve out a hardware-free zone (e.g.
`lib/logic/` that includes nothing from Arduino/FreeRTOS) and move decision
code there as it gets touched.
