# Hardware setup

Use a NodeMCU 1.0 ESP-12E board with the ESP8266 Arduino core and the I2Cdevlib Arduino I2Cdev and MPU6050 libraries. Open [NodeMCU_Car.ino](../firmware/NodeMCU_Car/NodeMCU_Car.ino) in Arduino IDE. Set the access point name and password, select the NodeMCU board and serial port, and verify the sketch before uploading.

| Subsystem | NodeMCU pins | Connection |
| --- | --- | --- |
| Right motor | D8 / D7 | L298N IN1 / IN2 |
| Left motor | D4 / D3 | L298N IN3 / IN4 |
| Ultrasonic | D6 / D5 | Trigger / echo |
| MPU6050 | D2 / D1 | SDA / SCL |

Use a shared ground, an appropriate motor supply, and a level shifter or divider if your ultrasonic module drives echo at 5 V. D3, D4, and D8 affect boot on this board, so confirm the driver and wiring preserve the required boot levels. Check motor polarity and mounting against the supplied chassis and calibrate the MPU6050 offsets on your unit. Test direction and stop with the wheels clear of the floor.

Connect to the board’s configured SoftAP and open `http://192.168.4.1/`. Each tap sends one 250 ms drive pulse. Stop cancels motion on receipt. The firmware checks expiry during its loop, so the configured pulse length is a software timeout and can be affected by loop latency. Keep this interface on an isolated access point because the HTTP endpoints have no application-level authentication.

| Request | Response |
| --- | --- |
| GET /health | 200 and robotic-car/1 |
| GET /command?cmd=F&ms=250 | 204 for a valid command. 400 and stop on invalid motion input |
| GET /command?cmd=S | 204 and stop |
| GET /distance | Distance in cm, or −1 when there is no echo |
| GET /mpu | Yaw, pitch and roll in degrees, or 503 before a fresh sample is available |

The firmware uses the motor patterns from the original sketch. The Python host can query telemetry and send pulses. For physical mapping, measured range and a calibrated pose must be supplied to the map model. The bundled map and synthetic demo can be reviewed without hardware.
