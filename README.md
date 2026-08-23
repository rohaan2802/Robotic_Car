# Robotic_Car

**BatMobile** - a Wi-Fi-controlled mobile robot on **NodeMCU (ESP8266)** with ultrasonic ranging and **MPU6050** orientation, plus a **Python** client that sends drive commands and builds a live occupancy-grid map.

---

## Overview

Firmware (`NodeMCU_Car.ino`) exposes an ESP8266 SoftAP web API for motion, distance, and IMU data. The host-side `app.py` talks to the car over HTTP, fuses ultrasonic hits into an occupancy grid (`map.npy`), and visualizes pose with Matplotlib.

Supporting materials: `CarMeasures.xlsx`, final robotics report PDF, and a demo video.

---

## Features

### Firmware (ESP8266)

- SoftAP: SSID `ESP8266_Car` (change password in sketch)
- Motor driver pins: `IN1..IN4` on D8/D7/D4/D3
- HC-SR04 style ultrasonic on `TRIG=D6`, `ECHO=D5`
- MPU6050 DMP over I2C (`SDA=D2`, `SCL=D1`) with yaw moving-average filter
- HTTP routes:
  - `GET /` - root UI/handler
  - `GET /command?cmd=F|B|L|R|S` - drive / stop
  - `GET /distance` - latest range (cm)
  - `GET /mpu` - estimated pitch/roll/yaw

### Python client

- Commands `F/B/L/R/S` via `requests`
- Occupancy grid mapping (occupied / free / unknown probabilities)
- Car footprint overlay using measured width/length from inches→cm
- Interactive Matplotlib map; can persist grid as `map.npy`

---

## Repository structure

```text
Robotic_Car/
├── BatMobile/BatMobile/
│   ├── NodeMCU_Car/NodeMCU_Car.ino
│   ├── app.py
│   └── map.npy
├── CarMeasures.xlsx
├── ROBOTICS_REPORT_FINALIZED.pdf
└── WhatsApp Video ... .mp4
```

---

## Build / run

### 1. Flash firmware

1. Install Arduino IDE (or PlatformIO) with **ESP8266** board support.
2. Libraries: `ESP8266WiFi`, `ESP8266WebServer`, `Wire`, `MPU6050_6Axis_MotionApps20` (i2cdevlib).
3. Open `NodeMCU_Car.ino`, select NodeMCU board, flash.
4. Connect to AP `ESP8266_Car` - default IP typically `192.168.4.1`.

### 2. Run Python mapper

```bash
cd BatMobile/BatMobile
python -m venv .venv
# Windows: .venv\Scripts\activate
pip install requests numpy matplotlib
python app.py
```

Confirm `CAR_IP = 'http://192.168.4.1'` matches your SoftAP address.

---

## Usage

1. Power the robot; join its Wi-Fi AP from the laptop.
2. Start `app.py` - send teleop commands and watch the occupancy grid update.
3. Tune car dimensions / map size in the configuration block at the top of `app.py`.
4. Consult `CarMeasures.xlsx` and the finalized PDF for mechanical notes and results.
5. Change SoftAP credentials in the `.ino` before demos on shared networks.

---

## Extending

- Replace SoftAP with station mode + mDNS for lab Wi-Fi.
- Stream odometry from yaw + commanded motion for better SLAM-lite.
- Add obstacle stop when `/distance` falls below a threshold (firmware or client).
- Expose a small Flask/FastAPI wrapper if you need multi-client control.

---

## License

Academic robotics project - verify library licenses (MPU6050 / ESP8266 cores) for redistribution.
