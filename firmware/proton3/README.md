# PROTON-3 Flight Computer Firmware (Validated)

PROTON-3 is a modular, ESP32-S3–based flight computer platform designed for reliable **canard/servo control**, **high-rate sensor logging**, and **clear, safe operating states** (SAFE → ARMED → FLIGHT → RECOVERY → POSTFLIGHT). 

This repository contains **validated firmware only**. Broader explanations, rationale, and implementation notes are typically written **directly inside the source files as comments** (that’s intentional: the code is the documentation). 

---

## What PROTON-3 Is

A robust, serviceable flight stack with a focus on:

* reproducible bring-up and testing,
* stable servo output (minimizing jitter / timing interference),
* reliable logging under real SD-card latency,
* safe recovery logic via explicit arming states and checks. 

---

## Key Features

* **Core MCU:** ESP32-S3 module with native USB (flash + serial logging) and Wi-Fi (U.FL capable, depending on module variant). 
* **Actuation:** Up to **8 servos** for canards via **PCA9685** (I2C PWM, 16 channels available). 
* **Sensors:** IMU + barometer as the minimum set; optional GNSS/telemetry expansion path. 
* **Logging:** microSD as primary storage, with an optional **PSRAM ring buffer** concept to absorb SD stalls (batch writes for fewer dropouts). 
* **Safety / State Machine:** explicit operating states, preflight checks, and hardware-minded “default safe” behavior (e.g., PWM output gating concept). 

---

## System Architecture (High Level)

### Hardware (Conceptual)

* ESP32-S3 as central controller (native USB for flashing/debug).
* PCA9685 generates servo PWM so the control loop and logging are not disturbed by PWM timing load.
* microSD for logs; PSRAM buffer for burst-safe logging.
* Separate power domains are intended (logic 3.3 V vs servo rail) to prevent servo current spikes from browning out the MCU. 

### Software (Conceptual Task Split)

Typical division (often mapped to FreeRTOS tasks):

* Sensor acquisition (timed/interrupt-assisted where possible)
* Estimation/control
* Servo command output (I2C to PCA9685, rate-limited/smoothed)
* Logger task (batching + block writes)
* Comms (USB/Wi-Fi preflight/config/download)
* Safety task (state machine + checks + watchdog integration) 

---

## Logging Philosophy

The design goal is **correlated, timestamped** sensor data that stays usable even under resets/partial writes. CSV is convenient but inefficient; a framed binary log with checks (e.g., CRC) is a common direction, with optional conversion tooling later. 

---

## Safety Notes

PROTON-3 is built around the idea that “armed” must be an explicit, verifiable condition—not an accident of software state. The intended architecture separates *safe* behavior from *armed/flight* behavior and encourages hardware-level gating where appropriate. 

---

## How to Use This Repo

* Open the firmware in your preferred ESP32 toolchain (commonly Arduino-ESP32 or PlatformIO).
* Library requirements, pin mapping assumptions, and calibration notes are documented **in-code** (header comments and inline notes).
* Follow the bring-up flow: USB flash/CDC → I2C scan → SD read/write → servo neutral output → sensor sanity checks. 

---

## Project Status

The underlying platform is designed from a tested, debug-friendly workflow mindset: clear boot paths, reproducible verification steps, and field-serviceability first. Hardware architecture details and exact component values are treated as review items and evolve with mission needs. 

---

## Contributing

If you contribute:

* keep changes small and testable,
* preserve safe defaults (SAFE must stay safe),
* document behavior changes in code comments where they matter most (right next to the logic).

---

## Credits

PROTON-3 is developed as a personal engineering project with a strong focus on validation, repeatable testing, and clean failure modes. 

