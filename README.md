# Robotic Car (BatMobile)

Wi-Fi **teleoperated robot** on **NodeMCU (ESP8266)** with ultrasonic ranging and **MPU6050 DMP** orientation, plus a **Python** host that sends drive commands and maintains a live **occupancy-grid** map.

**Author:** Mohammad Rohaan · **GitHub:** [rohaan2802](https://github.com/rohaan2802)

---

## Table of contents

1. [System overview](#system-overview)
2. [Hardware map](#hardware-map)
3. [Firmware HTTP API](#firmware-http-api)
4. [Python mapper (`app.py`)](#python-mapper-apppy)
5. [Occupancy grid](#occupancy-grid)
6. [Repository layout](#repository-layout)
7. [Flash and run](#flash-and-run)
8. [Usage](#usage)
9. [Known limitations](#known-limitations)
10. [Extending](#extending)

---

## System overview

```text
Laptop (app.py) --HTTP GET--> ESP8266 SoftAP 192.168.4.1
                    |              |
                    |         motor driver IN1..IN4
                    |         HC-SR04 TRIG/ECHO
                    |         MPU6050 I2C DMP (yaw/pitch/roll)
                    v
              occupancy_grid (MAP_SIZE×MAP_SIZE)
              matplotlib live view + optional map.npy
```

Supporting files on GitHub: `CarMeasures.xlsx`, `ROBOTICS_REPORT_FINALIZED.pdf`, demo MP4.

---

## Hardware map

| Function | Pin / note |
|----------|------------|
| Right motor forward / back | `IN1=D8`, `IN2=D7` |
| Left motor forward / back | `IN3=D4`, `IN4=D3` |
| Ultrasonic | `TRIG=D6`, `ECHO=D5` |
| MPU6050 I2C | `SDA=D2`, `SCL=D1` |
| Wi-Fi | SoftAP SSID **`ESP8266_Car`**, password **`password`** (change before a public demo) |
| Web server | port **80** |
| UART | `Serial.begin(9600)` |

MPU DMP calibration offsets in firmware: accel `(-2291, -1602, 1228)`, gyro `(87, -72, 8)`.  
DMP rate: `setRate(4)` → **200 / (1+4) = 40 Hz**.  
Yaw smoothing: moving average buffer size **10**.  
Ultrasonic refresh: every **500 ms** in `loop()`.

Libraries: `ESP8266WiFi`, `ESP8266WebServer`, `Wire`, `MPU6050_6Axis_MotionApps20` (i2cdevlib).

---

## Firmware HTTP API

| Route | Role |
|-------|------|
| `GET /` | `handleRoot` — simple control / status page |
| `GET /command?cmd=F\|B\|L\|R\|S` | Forward, Back, Left, Right, **Stop** |
| `GET /distance` | Latest range in **cm** (or `-1`) |
| `GET /mpu` | JSON: `estimated_pitch`, `estimated_roll`, `estimated_yaw` |

Motors are stopped in `setup()`. Always send `S` after a timed move from Python.

---

## Python mapper (`app.py`)

Dependencies: `requests`, `numpy`, `matplotlib`.

| Constant | Value | Meaning |
|----------|-------|---------|
| `CAR_IP` | `http://192.168.4.1` | SoftAP default |
| `MAP_SIZE` | 500 | Grid edge (cells) |
| `RESOLUTION` | 1 | **1 cm per cell** |
| `PROB_OCCUPIED` | 0.9 | Hit cell |
| `PROB_FREE` | 0.1 | Ray free space |
| `PROB_UNKNOWN` | 0.5 | Prior |
| `CAR_WIDTH` | `5.9 * 2.54` cm ≈ 15.0 | From inches in `CarMeasures` |
| `CAR_LENGTH` | `10.1 * 2.54` cm ≈ 25.7 | |

Helpers:

- `send_command(cmd)` — GET `/command`
- `get_distance()` — text body
- `get_mpu_data()` — JSON yaw/pitch/roll
- `move_and_update` — optional duration; updates pose from yaw; `rotate_to_angle` for L/R with target
- Footprint overlay via `Rectangle` + `Affine2D`
- Interactive `plt.ion()` colormap: white / lightgrey / grey / black

Join the car’s AP **before** starting Python or requests time out (5 s).

---

## Occupancy grid

- Array `occupancy_grid` shape `(500, 500)` filled with 0.5.
- Pose: `position {x,y}` in cm, `orientation` degrees (0 = +X).
- Ultrasonic hits along the facing ray mark occupied; intervening cells trend free.
- Persist with `numpy.save("map.npy")` when the script does so.

This is **teleop mapping**, not full SLAM (no loop closure).

---

## Repository layout

```text
Robotic_Car/
├── BatMobile/BatMobile/
│   ├── NodeMCU_Car/NodeMCU_Car.ino
│   ├── app.py
│   └── map.npy
├── CarMeasures.xlsx
├── ROBOTICS_REPORT_FINALIZED.pdf
└── *.mp4
```

---

## Flash and run

1. Arduino IDE + ESP8266 board package.  
2. Install MPU6050 DMP library.  
3. Flash `NodeMCU_Car.ino` (NodeMCU 1.0).  
4. Connect laptop Wi-Fi to `ESP8266_Car`.  
5. Confirm IP (usually `192.168.4.1`).

```bash
pip install requests numpy matplotlib
python app.py
```

---

## Usage

1. Power the robot; join SoftAP.  
2. Run `app.py`; send `F/B/L/R/S`.  
3. Watch the grid and yaw.  
4. Use the PDF + spreadsheet for mechanical dimensions and viva numbers.  
5. Change SoftAP credentials in the `.ino` for shared labs.

---

## Known limitations

- Default Wi-Fi password is weak and committed.  
- Mapping quality depends on yaw drift; DMP offsets are board-specific — recalibrate.  
- No automatic emergency stop on small `/distance` in the snippets reviewed (easy to add).  
- SoftAP = one laptop; no station-mode lab Wi-Fi.

---

## Extending

- Station mode + mDNS.  
- Stop if distance &lt; threshold in firmware.  
- Fuse commanded motion + yaw for better odometry.  
- Recalibrate MPU with the official calibration sketch.

---

## Author

**Mohammad Rohaan** · [rohaan2802](https://github.com/rohaan2802)
