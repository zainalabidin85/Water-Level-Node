# WaterLevel-Node (ESP32-C3 SuperMini)

A compact **water-level monitor** firmware for **ESP32‑C3 SuperMini** with:

- **3 buttons** UI (LIGHT/MODE, CAL/UP, ENTER/DN)
- **20×4 I2C LCD** (default address `0x27`)
- **ADC water-level input** (0–3.3V on ADC pin; supports 0–5V sensors using a voltage divider)
- **Captive-portal style AP fallback** for Wi‑Fi provisioning
- **LittleFS web UI** served from `/www` (optional)
- **JSON APIs** for status + readings + calibration
- **Optional MQTT telemetry** (configured ONLY via web API)

> Firmware version in code: `FW_VERSION = 1.0.0`

---

## 1) Hardware required

### Core
- ESP32‑C3 SuperMini
- 20×4 I2C LCD (I2C backpack, commonly `0x27`)
- 3 momentary push buttons (wired to GND, using internal pullups)

### Water-level sensor input
You can use:
- **0–3.3V analog sensor** → connect directly to ADC pin  
or
- **0–5V analog sensor** → **must use a voltage divider** before ESP32 ADC.

**Recommended divider for 0–5V → 0–2.5V (safe):**
- `47k` (top) + `47k` (bottom) → ratio 2.0

This firmware assumes a divider ratio constant:

```cpp
static const float VOLTAGE_DIVIDER_RATIO = 2.0f; // 47k/47k
```

If you use a different divider, adjust this value.

---

## 2) Pin map (ESP32‑C3 SuperMini)

| Function | GPIO | Notes |
|---|---:|---|
| I2C SDA | GPIO8 | LCD |
| I2C SCL | GPIO9 | LCD |
| Water level ADC | GPIO0 | **ADC input (0–3.3V max)** |
| LIGHT/MODE button | GPIO2 | to GND, `INPUT_PULLUP` |
| CAL/UP button | GPIO3 | to GND, `INPUT_PULLUP` |
| ENTER/DN button | GPIO4 | to GND, `INPUT_PULLUP` |

---

## 3) Wiring guide

### LCD (I2C)
- LCD **VCC** → 5V (or 3.3V if your LCD supports it; most I2C backpacks expect 5V)
- LCD **GND** → GND
- LCD **SDA** → GPIO8
- LCD **SCL** → GPIO9

### Buttons (all identical)
Each button:
- One leg → **GPIO pin**
- Other leg → **GND**

No external resistors needed (uses internal pullups).

### ADC sensor wiring

#### A) Sensor output 0–3.3V
- Sensor **Vout** → GPIO0 (ADC)
- Sensor **GND** → GND
- Sensor **VCC** → per sensor spec

#### B) Sensor output 0–5V (requires divider)
Use a resistor divider:

```
Sensor Vout ----[ Rtop ]----+----> GPIO0 (ADC)
                            |
                           [ Rbot ]
                            |
                           GND
```

Example:
- Rtop = 47k
- Rbot = 47k

Then keep:
```cpp
VOLTAGE_DIVIDER_RATIO = 2.0f;
```

---

## 4) How it behaves (Wi‑Fi / AP / Captive Portal)

### Normal boot
1. Firmware tries to connect to previously saved Wi‑Fi credentials.
2. If not connected after ~15s, it retries.
3. After 2 failures, it enters **AP Setup Mode**.

### AP Setup Mode
- SSID: `WaterLevel-XXXX` (derived from chip MAC)
- Password: `12345678`
- Device IP: `192.168.4.1`
- Captive portal DNS: redirects unknown URLs to `/setup`

Open:
- `http://192.168.4.1/setup`

### Buttons (global shortcuts)
- **LIGHT short**: toggle LCD backlight
- **LIGHT long**: force AP Setup Mode
- **LIGHT very-long (~9s)**: wipe Wi‑Fi creds + MQTT settings and go to AP Setup Mode

---

## 5) LCD UI (3 buttons)

### UI states
- **Home**: shows level + voltage + Wi‑Fi/MQTT state + IP
- **Main Menu**
- **Setup / Wi‑Fi**
- **Calibration Menu**
- **Calibration Wizard**
- **Unit Settings**
- **Info**

### Navigation summary
From **Home**:
- **CAL/UP long** → Calibration Menu
- **ENTER/DN long** → Main Menu

Inside menus:
- **CAL/UP short**: move selection up (or adjust)
- **ENTER/DN short**: move selection down (or adjust)
- **ENTER/DN long**: select / confirm / next
- **CAL/UP long**: back / exit (context dependent)

---

## 6) Calibration (2-point)

The firmware supports a 2-point linear calibration:
- **Empty**: level value + measured voltage
- **Full**: level value + measured voltage

It computes:
```
level = slope * V + offset
```

Quality heuristic:
- `OK` if level span ≥ 50 and voltage span ≥ 0.50V
- otherwise `WEAK`

### Calibration Wizard steps
1. Choose unit: Percent / Voltage / Raw / Custom
2. Set Empty value
3. Capture Empty voltage (uses 10s rolling average)
4. Set Full value
5. Capture Full voltage
6. Compute + save to NVS

---

## 7) Web UI + LittleFS

### Filesystem
- LittleFS is mounted on boot
- Web assets live in: **`/www/`**
- Expected UI entry: **`/www/index.html`**
- AP setup page (optional): **`/www/ap.html`**

If UI is missing, firmware serves built-in **Factory Page**.

### Useful pages
- `/` → `/www/index.html` if present, else Factory Page
- `/setup` → `/www/ap.html` if present, else built-in setup page
- `/factory` → built-in factory recovery page
- `/files` → file manager page (AP mode only)

---

## 8) JSON API endpoints

### Status
`GET /api/status`

Returns:
- device name, fw, uptime
- wifi mode/ssid/ip/rssi
- calibration state
- mqtt state (if compiled)
- filesystem total/used

### Level reading
`GET /api/level`

Returns:
- `adc`
- `voltage_adc` (voltage at ESP32 ADC pin)
- `voltage_sensor` (estimated original sensor voltage)
- `level_percent` (when available)
- `level_custom` (when available)
- 10s rolling average + stddev

### Calibration info
`GET /api/cal`

### Clear calibration
`POST /api/cal/clear`

### Unit settings
- `GET /api/settings/unit`
- `POST /api/settings/unit` with form fields:
  - `current_unit` = 0..3
  - `custom_max` (float)

### Wi‑Fi settings
`POST /api/settings/wifi` (form fields)
- `ssid`
- `pass` (optional)

Device reboots after saving.

---

## 9) MQTT (optional)

MQTT is controlled by:
```cpp
#define ENABLE_MQTT 1
```

### Configuration
MQTT is configured only through the web API:

- `GET /api/settings/mqtt`
- `POST /api/settings/mqtt` (form fields):
  - `enabled` = `1` or `0`
  - `host`
  - `port`
  - `user`
  - `pass`
  - `topic` (base topic, default `waterlevel`)
  - `retain` = `1` or `0`
  - `period_ms` (publish period)

> Note: password is **never returned** by GET (always blank).

### Topics published
The firmware builds a per-device topic:

```
<base_topic>/<deviceName>/
```

And publishes:
- `telemetry` (JSON)
- `level` (simple value, percent if available)
- `voltage` (sensor voltage)

Example:
```
waterlevel/waterlevel-ABCD/telemetry
waterlevel/waterlevel-ABCD/level
waterlevel/waterlevel-ABCD/voltage
```

### Home Assistant examples (manual MQTT)

#### A) MQTT Sensor: percent level
```yaml
mqtt:
  sensor:
    - name: "Water Level (%)"
      state_topic: "waterlevel/waterlevel-ABCD/level"
      unit_of_measurement: "%"
      value_template: "{{ value | float }}"
```

#### B) MQTT Sensor: telemetry JSON (voltage)
```yaml
mqtt:
  sensor:
    - name: "Water Level Voltage"
      state_topic: "waterlevel/waterlevel-ABCD/telemetry"
      unit_of_measurement: "V"
      value_template: "{{ value_json.voltage_sensor | float }}"
```

> Replace `waterlevel-ABCD` with your actual device name shown on LCD Info page or `/api/status`.

---

## 10) Building with PlatformIO (recommended)

### Suggested `platformio.ini`
Use something like:

```ini
[env:esp32c3]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino

monitor_speed = 115200
upload_speed = 460800

board_build.filesystem = littlefs

lib_deps =
  bblanchon/ArduinoJson@^6.21.3
  knolleary/PubSubClient@^2.8
  https://github.com/esphome/ESPAsyncWebServer.git
  https://github.com/esphome/AsyncTCP.git
  marcoschwartz/LiquidCrystal_I2C@^1.1.4

lib_ldf_mode = deep+
build_flags =
  -D ARDUINO_USB_CDC_ON_BOOT=1
  -D ARDUINO_USB_MODE=1
```

### Upload firmware
- PlatformIO: **Upload** (Ctrl+Alt+U)

### Upload LittleFS UI
- PlatformIO: **Upload Filesystem Image**
  - In VS Code, open PlatformIO sidebar → *Project Tasks* → *Upload Filesystem Image*

---

## 11) Project folder structure

Typical layout:

```
WaterLevel-Node/
├─ platformio.ini
├─ src/
│  └─ main.cpp
└─ data/
   └─ www/
      ├─ index.html
      ├─ ap.html
      ├─ app.js
      ├─ style.css
      └─ (images...)
```

PlatformIO will pack `data/` into LittleFS.

> In this firmware, the runtime expects files in `/www/` on LittleFS.

---

## 12) Security notes (important)

- AP password is currently hardcoded as `12345678`.
- File manager endpoints are enabled **only in AP mode**.
- Firmware uploads (`.bin`) are blocked via `/upload`.

For deployments, consider:
- Changing AP password to a per-device value
- Adding an admin PIN/token for `/files` endpoints
- Running MQTT with username/password and a private broker (e.g., Mosquitto)

---

## 13) Troubleshooting

### Device always stays in AP mode
- It cannot connect to saved Wi‑Fi credentials.
- Go to `http://192.168.4.1/setup` and re-enter SSID/password.
- Ensure 2.4GHz Wi‑Fi is used (ESP32‑C3 does not support 5GHz).

### Weird ADC readings
- Confirm your sensor output range.
- If sensor is 0–5V, confirm divider and `VOLTAGE_DIVIDER_RATIO`.
- Confirm sensor ground is shared with ESP32 GND.

### MQTT shows OFF on LCD
- Ensure you enabled MQTT via `/api/settings/mqtt` (`enabled=1`)
- Ensure Wi‑Fi is connected and MQTT host is reachable.
- Check `/api/status` → `mqtt.last_error`
