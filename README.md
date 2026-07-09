# EVA-Rover_2025 — Autonomous Exploration and Manual Control with LoRa

A complete hardware and software system that controls the differential rover **EVA** through an exploration cycle: autonomous navigation with obstacle avoidance, manual control, real-time telemetry transmission, and image capture. All operated from a remote ground station via long-range links.

The rover integrates sensor fusion for health monitoring (IMU, ToF, Voltage/Current), an **A*** path planner for grid navigation, and a **Computer Vision (YOLOv8 and OpenCV)** system at the base station for object and color detection. The entire system is orchestrated by an adaptive dynamic communication protocol over **LoRa**.

> Developed for the Space Technologies concentration at Tecnológico de Monterrey. 

![C++](https://img.shields.io/badge/C++-Arduino-00599C?logo=c%2B%2B)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![LoRa](https://img.shields.io/badge/Comms-LoRa_433MHz-FF6C37)
![OpenCV](https://img.shields.io/badge/OpenCV-YOLOv8-5C3EE8?logo=opencv&logoColor=white)
![Hardware](https://img.shields.io/badge/Hardware-ESP32-black?logo=espressif)
![License](https://img.shields.io/badge/License-MIT-blue)

---

## Demo

https://github.com/rafael-sp-debug/EVA-Semiautonomous-Exploration-Rover/blob/main/docs/demo_eva.mp4

| | |
|---|---|
| **Mission** | Explore the terrain manually or autonomously, avoid obstacles, capture images, and send real-time telemetry. |
| **Mechanical Design** | Modular PLA chassis < 20x20 cm, TPU tires for high traction, NEMA17 actuators. |
| **Sensors** | MPU6050 (IMU), VL53L0X x3 (ToF Distance), INA219 (Power), SHT31/AHT10 (Temp/Hum). |
| **Navigation** | A* Planner (Manhattan Heuristic) with reactive Evasion and Escape modes. |
| **Communications** | LoRa link (433MHz) with dynamic adaptation of SF and Bandwidth (Short/Mid/Long range). |
| **Computer Vision** | Base64 capture via ESP32-CAM; YOLOv8 (Faces/Rovers) and OpenCV (Colors) detection at the ground base. |

---

## How it Works

The system is divided into independent subsystems running on three main microcontrollers: an ESP32 and an ESP32-CAM on the rover, and an ESP32 at the ground station.

```text
                  [ ESP32-CAM (Payload) ]
                             │ Base64 Image Chunks
                             ▼
 ┌─────────────────────────────────────────────────────┐      LoRa (433MHz)       ┌───────────────────────┐
 │                      EVA ROVER                      │ ◄── (Commands / ACKs) ──►│    GROUND STATION     │
 │                                                     │ ◄───── (Telemetry) ─────►│                       │
 │  [ ESP32U Central ]                                 │                          │  [ ESP32U ]           │
 │     ├─ Navigation (MotorTask, A*)                   │                          │      │ (UART-USB)     │
 │     ├─ Telemetry (I2C: IMU, ToF, INA219, SHT31)     │                          │      ▼                │
 │     └─ Actuators (DRV8825 -> NEMA17)                │                          │  [ Computer (PC) ]    │
 └─────────────────────────────────────────────────────┘                          │  Web UI (WebSockets)  │
                                                                                  │  Digital Twin         │
                                                                                  │  Vision (YOLO/OpenCV) │
                                                                                  └───────────────────────┘
```
---
## Subsystems (Modules)

| Subsystem | Description | Key Components |
|---|---|---|
| **Communications** | Pseudo-full duplex bidirectional link. Dynamically adapts its Spreading Factor (SF7 to SF11) based on average RSSI to secure the link. | Ra-02 Modules (LoRa 433MHz), UART |
| **Sensors** | Collects 24 scaled sensory variables in a 65-byte CSV packet sent to the base. Manages visual rover states via a Neopixel matrix. | MPU6050, VL53L0X, INA219, AHT10 |
| **Navigation** | Discretizes the environment into 33 cm cells. Uses A* to plan routes towards targets (`GOAL X Y`). If a ToF sensor detects a blockage, it triggers bypass (evasion) or 180° turn (escape) modes. | ESP32U, DRV8825, NEMA17 |
| **Payload (Capture)** | Captures images, encodes them in Base64, and splits them into chunks (e.g., 64-200 bytes). Sends them with integrity checks, re-requesting lost parts. | ESP32-CAM, OV2640 Camera |
| **Ground Station** | Generates a WiFi Access Point. Deploys a Web Dashboard (HTML/JS/CSS) that graphs telemetry, draws the obstacle map, and sends commands via WebSockets. | Embedded Web Server, Python |

---

## Commands and Control

The system processes the following commands sent from the serial interface or the web dashboard to the rover:

| Command | Action | Command | Action |
|---|---|---|---|
| `W`, `S` | Move Forward / Backward one cell | `AUTO` / `STOPAUTO` | Starts / Stops autonomous navigation mode |
| `A`, `D` | Turn 90° Left / Right | `GOAL X Y` | Sets a target at X, Y coordinates |
| `IMAGE` | Capture and request an image | `REVERSE` | Returns the rover to the origin (0,0) |
| `START` / `STOP` | Start / Stop telemetry flow | `OBJ X Y` | Manually defines a logical obstacle on the map |
| `FORCE###` | Forces LoRa mode (`SHORT`, `MID`, `LONG`) | `CLEARALL` | Resets the map and robot position |

---

## Technical Approach

*   **Dynamic LoRa Adaptation:** To prevent packet loss over long distances, the station evaluates the average RSSI. If it drops below specific thresholds (-65 dBm, -95 dBm), the system automatically negotiates a mode change (increasing the Spreading Factor from SF7 to SF9 or SF11 and the preamble), sacrificing speed for robustness.
*   **Secure Image Transmission:** Since LoRa easily loses packets, images are encoded in Base64 and sent in small "Chunks". The station requests the pieces one by one, verifying their length and ASCII content. If a timeout or error occurs, it specifically re-requests that chunk.
*   **Safe Navigation:** All movement relies on a central non-blocking task (`motorTask()`). This prevents race conditions between telemetry and motor control, ensuring that odometry pose updates remain accurate.
*   **Digital Twin & Vision:** The project features a digital twin in Webots that replicates the real robot's movements in 3D using received telemetry. Additionally, images arriving at the PC go through YOLOv8 (trained on 10,000 augmented images) to detect faces and rovers, and OpenCV for color segmentation (Red, Blue, Green, Yellow).

---

## Installation and Materials

To replicate this project, you need to manufacture the custom PCBs and assemble the 3D design.

**Main Materials:**
*   4x 18650 Li-ion Batteries and MP1584 Regulator
*   2x NEMA17 Motors with DRV8825 drivers
*   2x ESP32-DevKitC-32U and 1x ESP32-CAM
*   2x Ra-02 LoRa Modules (433MHz)
*   PLA (Chassis) and TPU (Tires) filaments

*(All technical information, STL files, and detailed BOM can be found in the `/Hardware` folder of the repository).*

---

## Authors

*   Rafael Soto Padilla
*   José Eduardo Castellanos Avila
*   Nicolas De Bruijn Prieto
*   Diana Marisol Hernández Jiménez

## References

1. **Macenski, S., Foote, T., Gerkey, B., et al.** (2022). *Robot Operating System 2: Design, architecture, and uses in the wild*. Science Robotics, 7(66). [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
2. **Jocher, G., Chaurasia, A., & Qiu, J.** (2023). *Ultralytics YOLOv8* (Version 8.0.0) [Computer software]. [https://github.com/ultralytics/ultralytics](https://github.com/ultralytics/ultralytics)
3. **Hart, P. E., Nilsson, N. J., & Raphael, B.** (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Transactions on Systems Science and Cybernetics, 4(2), 100-107.
4. **Espressif Systems.** (2023). *ESP32 Series Datasheet*. [https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
5. **Semtech Corporation.** (2020). *LoRa Modulation Basics* (AN1200.22). [https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/2R0000001Rbr/6EfVZUorrpoKFfvaF_Fkpgp5kzjiNyiAbqcpqh9qSjE](https://semtech.my.salesforce.com/sfc/p/E0000000JelG/a/2R0000001Rbr/6EfVZUorrpoKFfvaF_Fkpgp5kzjiNyiAbqcpqh9qSjE)
6. **Bradski, G.** (2000). *The OpenCV Library*. Dr. Dobb's Journal of Software Tools. [https://opencv.org/](https://opencv.org/)
