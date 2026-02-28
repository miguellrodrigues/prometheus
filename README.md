<div align="center">

# 🔥 Prometheus

**ESP32 closed-loop temperature controller with MQTT telemetry**

![Language](https://img.shields.io/badge/language-C-blue?style=flat-square&logo=c)
![Framework](https://img.shields.io/badge/framework-ESP--IDF%20≥5.0-red?style=flat-square&logo=espressif)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green?style=flat-square)
![Build](https://img.shields.io/badge/build-CMake-blue?style=flat-square&logo=cmake)
![License](https://img.shields.io/github/license/miguellrodrigues/prometheus?style=flat-square)

</div>

---

## 📖 Overview

**Prometheus** is an ESP32 firmware that implements a real-time, closed-loop temperature control system. It reads temperature from a DS18B20 sensor, computes a 2nd-order IIR control signal, and drives a PWM actuator. Sensor data and control signals are streamed over MQTT, and the system can be commanded remotely to change the setpoint or toggle between open-loop and closed-loop modes.

---

## ✨ Features

- 🌡️ **DS18B20** 1-Wire temperature sensing (configurable 9–12-bit resolution)
- 🔁 **2nd-order IIR pre-filter + PID controller** — starts in open-loop warm-up, auto-switches to closed-loop when within 5 °C of the setpoint
- ⚡ **MCPWM actuation** via GPIO 2 (65535-tick resolution, 80 MHz clock, ~1.2 kHz PWM)
- 📡 **WiFi** — STA mode with automatic fallback to AP mode on connection failure
- 🛰️ **MQTT** telemetry — publishes `{timestamp, temperature, control_signal, set_point}`; subscribes to remote config and setpoint update commands
- 💾 **SPIFFS** filesystem with `bdc_config` for persistent controller configuration (PID gains, filter coefficients, calibration, setpoint)
- 🧩 Modular HAL: separate abstraction layers for I2C, SPI, and 1-Wire peripherals
- 🔀 **Dual-core FreeRTOS** — control loop and sampling pinned to core 0, MQTT task pinned to core 1

---

## 🏗️ Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                          ESP32 Firmware                              │
│                                                                      │
│  ┌─────────────┐  periodic timer  ┌──────────────────────────────┐  │
│  │  DS18B20    │ ───notify──────▶ │   sampling_task  (Core 0)    │  │
│  │  (1-Wire)   │                  │   → temperature_queue        │  │
│  └─────────────┘                  └──────────────┬───────────────┘  │
│                                                  │                   │
│                             ┌────────────────────▼───────────────┐  │
│                             │   control_loop_task  (Core 0)      │  │
│                             │                                    │  │
│                             │  OPEN_LOOP  ──► fixed u₀           │  │
│                             │      │ |e| < 5°C ?                │  │
│                             │      ▼                             │  │
│                             │  CLOSED_LOOP                       │  │
│                             │    IIR pre-filter → PID            │  │
│                             │                   ↓                │  │
│                             │             actuate()              │  │
│                             │         MCPWM GPIO 2               │  │
│                             │                   ↓                │  │
│                             │          → mqtt_queue              │  │
│                             └────────────────────────────────────┘  │
│                                                  │                   │
│                             ┌────────────────────▼───────────────┐  │
│                             │    mqtt_task  (Core 1)             │  │
│                             │                                    │  │
│                             │  Publish: /streaming/data          │  │
│                             │  Subscribe: /control/update_config │  │
│                             │             /control/update_setpoint│  │
│                             └──────────────┬───────────────────┐ │  │
│                                            │  WiFi STA→AP      │ │  │
│                                            └───────────────────┘ │  │
│                                                                      │
│  ┌──────────────────────┐   SPIFFS                                  │
│  │  bdc_config.bin      │ ── PID gains, filter coeffs,              │
│  │                      │    calibration, setpoint, Ts              │
│  └──────────────────────┘                                           │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 🛠️ Hardware Components

| Component | Interface | Pin / Address | Notes |
|---|---|---|---|
| DS18B20 temperature sensor | 1-Wire | GPIO 47 | 10-bit resolution by default |
| PWM actuator | MCPWM | GPIO 2 | 80 MHz clock, 65535 ticks, ~1.2 kHz |
| Motor direction (IN1 / IN2) | GPIO | GPIO 13 / GPIO 1 | Pull-down, output |
| Motor enable (EN) | GPIO | GPIO 38 | Pull-down, output |

---

## 🧩 Software Modules

| Module | Header | Description |
|---|---|---|
| `bdc_config` | `include/bdc_config.h` | Persistent controller configuration (PID gains, filter coefficients, calibration, setpoint, sampling interval) stored to SPIFFS |
| `pid` | `include/pid.h` | Discrete PID controller with derivative filter, anti-windup (back-calculation), and output saturation |
| `entityx` | `include/entityx.h` | Generic entity manager with SPIFFS persistence; supports typed entries with custom compare and free functions |
| `file_tools` | `include/file_tools.h` | SPIFFS file utilities — create, open, close, hex-dump |
| `ow_device` | `include/ow_device.h` | 1-Wire device abstraction — bus init, reset, read, write |
| `i2c_device` | `include/i2c_device.h` | I2C device abstraction — read, write, write+receive |
| `spi_device` | `include/spi_device.h` | SPI device abstraction — read register, write |
| `keypad` | `include/keypad.h` | 4×3 keypad driver over PCF8574 I2C expander |
| `lcd` | `include/lcd.h` | LCD 16×2 driver — commands, data, strings, cursor, custom chars |

---

## 📁 Project Structure

```
prometheus/
├── CMakeLists.txt          ← top-level ESP-IDF CMake project
├── dependencies.lock       ← ESP-IDF component manager lock file
├── partitions.csv          ← custom partition table (includes SPIFFS)
├── sdkconfig               ← ESP-IDF SDK configuration (generated)
├── LICENSE
├── README.md
└── main/
    ├── CMakeLists.txt      ← component registration
    ├── prometheus.c        ← application entry point
    ├── include/            ← public header files
    │   ├── bdc_config.h
    │   ├── entityx.h
    │   ├── file_tools.h
    │   ├── i2c_device.h
    │   ├── keypad.h
    │   ├── lcd.h
    │   ├── ow_device.h
    │   ├── pid.h
    │   └── spi_device.h
    └── src/                ← module implementations
        ├── bdc_config.c
        ├── entityx.c
        ├── file_tools.c
        ├── i2c_device.c
        ├── keypad.c
        ├── lcd.c
        ├── ow_device.c
        ├── pid.c
        └── spi_device.c
```

---

## 🚀 Getting Started

### Prerequisites

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/) **≥ v5.0** (uses the new I2C master API and `mcpwm_prelude.h`)
- CMake ≥ 3.16
- Python 3.x (used by ESP-IDF tooling)
- An ESP32 board with a USB-to-serial adapter

### Build & Flash

```bash
# Source the ESP-IDF environment (adjust path as needed)
. $HOME/esp/esp-idf/export.sh

# Clone the repository
git clone https://github.com/miguellrodrigues/prometheus.git
cd prometheus

# Configure (optional — opens menuconfig)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor (replace /dev/ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## ⚙️ Configuration

Key compile-time parameters are defined at the top of `main/prometheus.c`. Runtime parameters are loaded from `bdc_config.bin` on SPIFFS and can be updated over MQTT without reflashing.

| `#define` | Default | Description |
|---|---|---|
| `DS18B20_RESOLUTION` | `10` | Sensor resolution in bits (9–12) |
| `ACTUATE_GPIO` | `GPIO_NUM_2` | PWM output pin |
| `IN1_GPIO` / `IN2_GPIO` | `GPIO_NUM_13` / `GPIO_NUM_1` | Motor direction pins |
| `EN_GPIO` | `GPIO_NUM_38` | Motor enable pin |
| `ST_WIFI_SSID` / `ST_WIFI_PASS` | `"HW"` / `"..."` | STA-mode WiFi credentials (**set before flashing**) |
| `AP_WIFI_SSID` / `AP_WIFI_PASS` | `GREA_HEAT_PUMP` / `"..."` | Fallback AP-mode credentials |
| `MQTT_URI` | `mqtt://greamqtt.broker:1883` | MQTT broker address (auto-switches to `192.168.4.2` on AP fallback) |

Runtime-configurable values (stored in `bdc_config.bin`, updated via `/control/update_config`):

| Parameter | Description |
|---|---|
| `setPoint` | Temperature setpoint in °C |
| `openLoopControlSignal` | Fixed control signal during open-loop warm-up |
| `calibrationAngularTerm` / `calibrationLinearTerm` | Linear calibration curve coefficients (control unit → voltage) |
| `kp`, `ki`, `kd`, `tf`, `ksi` | PID gains and derivative filter / anti-windup parameters |
| `satUp` / `satDown` | PID output saturation limits |
| `filterNum[]` / `filterDen[]` | IIR pre-filter numerator / denominator coefficients |
| `filterOrder` | IIR pre-filter order |
| `samplingIntervalMs` | Temperature sampling period in ms |

---

## 📜 License

This project is licensed under the terms of the [LICENSE](LICENSE) file in this repository.

---

## 👤 Author

**Miguel L. Rodrigues**  
📧 [miguellukas52@gmail.com](mailto:miguellukas52@gmail.com)  
🐙 [github.com/miguellrodrigues](https://github.com/miguellrodrigues)
