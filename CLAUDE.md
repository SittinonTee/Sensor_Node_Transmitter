# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an **Arduino/ESP32 firmware project** for a DPV (Diver Propulsion Vehicle). The ESP32 board acts as a **Sensor Slave Node** that:
- Reads orientation/heading from a BNO055 IMU (I2C 0x28)
- Reads depth/temperature from a BMP280 barometer (I2C 0x76)
- Monitors battery voltage and current from an INA226 (I2C 0x40)
- Reads external water pressure via analog pin GPIO 34
- Controls a Hobbywing Skywalker 60A V2 ESC on GPIO 27 via 50Hz PWM
- Transmits all sensor data to a Main Board via **ESP-NOW** (Wi-Fi channel 1, unicast to a hardcoded MAC)

## Uploading / Building

This project uses the **Arduino IDE** (not a Makefile or cmake build). There is no terminal build command. To compile and flash:

1. Open `Sensor_Node_Transmitter.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module** (or equivalent ESP32 board)
3. Select the correct COM port
4. Click Upload (Ctrl+U)

For ESC calibration only: open and upload `ESC_Calibration/ESC_Calibration.ino` instead.

## Required Libraries (Arduino Library Manager)

- `Adafruit BNO055`
- `Adafruit BMP280`
- `Adafruit Unified Sensor`
- `ESP32Servo`
- `esp_now` / `WiFi` / `esp_wifi` — bundled with the ESP32 Arduino core

## Architecture

### Data Flow

```
Sensors (I2C) ──► loop() reads & packs ──► SensorData struct ──► esp_now_send() every 200ms
GPIO 34 (analog) ──► EMA filtered ──► pressure/depth ──/
ESC (GPIO 27) ──► handleMotorRamping() every 20ms ──► esc.writeMicroseconds()
Buttons (GPIO 4, 5, 15) / Serial ──► updateMotorState() ──► targetPWM
```

### Key Timing Constants

| Constant | Value | Purpose |
|---|---|---|
| `SENSOR_INTERVAL` | 200 ms | ESP-NOW transmission rate |
| `MOTOR_RAMP_INTERVAL` | 20 ms | ESC PWM update rate |
| `RAMP_STEP` | 5 µs | PWM change per ramp tick (smooths throttle) |

### SensorData Struct

Defined with `__attribute__((packed))` — byte layout must match exactly on the receiving Main Board. Do not reorder or change field types without updating the receiver.

### ESP-NOW Configuration

- Wi-Fi mode: `WIFI_STA`, modem sleep disabled (`WIFI_PS_NONE`)
- Channel locked to **1** via promiscuous mode trick (required for stable unicast)
- I2C timeout: 150 ms (prevents I2C hang from blocking ESP-NOW transmission)
- Receiver MAC hardcoded in `broadcastAddress[]` — must match the Main Board's MAC

### Motor Control Logic

- `updateMotorState()` — sets `targetPWM` based on gear (1330/1660/2000 µs) and run state
- `handleMotorRamping()` — runs every 20 ms in `loop()`, moves `currentPWM` toward `targetPWM` by `RAMP_STEP` to prevent motor jerk
- Gear cycles 1→2→3→1; two run buttons (GPIO 4 & 5) are OR'd — either held LOW runs the motor

### ESC Calibration

`ESC_Calibration/ESC_Calibration.ino` is a standalone sketch (not included in main firmware). Flash it separately when the ESC needs to relearn throttle range. Follow the Serial Monitor prompts — it requires physically connecting the battery during the sequence.

## Serial Monitor

Baud rate: **115200**. Each transmission cycle prints a formatted status block including sensor readings, battery stats, gear state, and ESP-NOW send result. Serial commands during runtime: `1`/`2`/`3` sets gear, `R` toggles run, `+` cycles gear.

## Hardware Pin Summary

| Pin | Function |
|---|---|
| GPIO 27 | ESC PWM signal |
| GPIO 4 | Run button 1 (INPUT_PULLUP, active LOW) |
| GPIO 5 | Run button 2 (INPUT_PULLUP, active LOW) |
| GPIO 15 | Gear cycle button (INPUT_PULLUP, active LOW) |
| GPIO 34 | External pressure sensor (0–3.3V analog, 10k/20k voltage divider) |
| SDA/SCL | BNO055, BMP280, INA226 on shared I2C bus |
