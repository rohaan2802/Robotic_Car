#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>  // Use the DMP library

// Motor Control Pins
#define IN1 D8  // Right Motor Forward
#define IN2 D7  // Right Motor Backward
#define IN3 D4  // Left Motor Forward
#define IN4 D3  // Left Motor Backward

// Ultrasonic Sensor Pins
#define TRIG_PIN D6  // Trigger Pin
#define ECHO_PIN D5  // Echo Pin

ESP8266WebServer server(80);  // Initialize web server on port 80
MPU6050 mpu;  // Create MPU6050 object

// Variables
constexpr unsigned long DEFAULT_PULSE_MS = 250;
constexpr unsigned long MAX_PULSE_MS = 1000;
unsigned long motionStarted = 0;
unsigned long motionDuration = 0;
bool moving = false;
bool dmpReady = false;
bool hasAttitude = false;
unsigned long lastAttitude = 0;
long distance = -1;  // Distance measured
unsigned long lastMeasureTime = 0;

// MPU6050 DMP variables
uint8_t fifoBuffer[64];  // FIFO storage buffer
Quaternion q;            // [w, x, y, z]         quaternion container
VectorFloat gravity;     // [x, y, z]            gravity vector
float ypr[3];            // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float estimated_yaw = 0.0;
float estimated_pitch = 0.0;
float estimated_roll = 0.0;

void setup() {
  Serial.begin(9600);
  Serial.println("ESP8266 Car Initialized.");

  // Initialize Motor Control Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ensure motors are stopped at startup
  stopCar();

  // Initialize Ultrasonic Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup Wi-Fi as Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP8266_Car", "password");  // Change SSID and Password as needed
  Serial.print("Access Point IP: ");
  Serial.println(WiFi.softAPIP());

  // Define Routes
  server.on("/", handleRoot);
  server.on("/command", HTTP_GET, handleCommand);
  server.on("/health", HTTP_GET, []() {
    server.send(200, "text/plain", "robotic-car/1");
  });
  server.on("/distance", handleDistance);
  server.on("/mpu", handleMPUData);

  // Start the server
  server.begin();
  Serial.println("Web server started.");

  // Initialize MPU6050 with DMP
  Wire.begin(D2, D1);  // SDA to D2, SCL to D1
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Initialize DMP
  uint8_t devStatus = mpu.dmpInitialize();

  // Supply your own offsets here, e.g., mpu.setXAccelOffset(yourOffset);
  // You can find offsets by running the MPU6050_calibration sketch
  mpu.setXAccelOffset(-2291);
  mpu.setYAccelOffset(-1602);
  mpu.setZAccelOffset(1228);
  mpu.setXGyroOffset(87);
  mpu.setYGyroOffset(-72);
  mpu.setZGyroOffset(8);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("DMP ready");

    // Set DMP output rate (e.g., 50 Hz)
    mpu.setRate(4);  // Set the rate to 200 / (1 + 4) = 40 Hz
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

}

void enforceMotionLimit() {
  if (moving && millis() - motionStarted >= motionDuration) {
    stopCar();
  }
}

void loop() {
  enforceMotionLimit();
  server.handleClient();
  enforceMotionLimit();

  if (millis() - lastMeasureTime >= 500) {
    lastMeasureTime = millis();
    distance = measureDistance();
  }

  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    const float degrees = 180.0f / PI;
    estimated_yaw = fmodf(ypr[0] * degrees + 360.0f, 360.0f);
    estimated_pitch = ypr[1] * degrees;
    estimated_roll = ypr[2] * degrees;
    hasAttitude = true;
    lastAttitude = millis();
  }
}

// Stop all motors
void stopCar() {
  moving = false;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Move forward
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Moving Forward.");
}

// Move backward
void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Moving Backward.");
}

// Turn left
void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning Left.");
}

// Turn right
void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning Right.");
}

// Rotate around left wheels (only right wheels move forward)
void rotateAroundLeft() {
  digitalWrite(IN1, HIGH);  // Right Motor Forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Left Motor Stopped
  digitalWrite(IN4, LOW);
  Serial.println("Rotating Around Left Wheels.");
}

// Rotate around right wheels (only left wheels move forward)
void rotateAroundRight() {
  digitalWrite(IN1, LOW);   // Right Motor Stopped
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Left Motor Forward
  digitalWrite(IN4, HIGH);
  Serial.println("Rotating Around Right Wheels.");
}

// Measure distance using ultrasonic sensor
long measureDistance() {
  long duration, measured_distance;

  // Ensure TRIG_PIN is low
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send 10µs pulse to TRIG_PIN
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read ECHO_PIN
  duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Timeout after 30ms

  // Calculate distance in cm
  if (duration == 0) {
    Serial.println("No echo received.");
    measured_distance = -1;
  } else {
    measured_distance = (duration / 2) / 29.1;
    Serial.print("Measured Distance: ");
    Serial.print(measured_distance);
    Serial.println(" cm.");
  }
  return measured_distance;
}

// Handle MPU6050 data request with DMP data
void handleMPUData() {
  if (!hasAttitude || millis() - lastAttitude > 1000) {
    server.send(503, "text/plain", "IMU sample unavailable");
    return;
  }
  // Prepare JSON response with DMP data
  String jsonResponse = "{";
  jsonResponse += "\"estimated_pitch\": " + String(estimated_pitch, 2) + ",";
  jsonResponse += "\"estimated_roll\": " + String(estimated_roll, 2) + ",";
  jsonResponse += "\"estimated_yaw\": " + String(estimated_yaw, 2) + "}";
  server.send(200, "application/json", jsonResponse);
}

const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robotic Car Controls</title>
<style>
body{font:1rem system-ui;color:#e9f7fa;background:#142838;max-width:34rem;margin:2rem auto;padding:1rem}
.controls{display:grid;grid-template-columns:repeat(3,1fr);gap:.65rem}
button{font:inherit;font-weight:bold;min-height:54px;border:0;border-radius:.5rem;background:#a7e0e0;color:#142838}
button[data-cmd=S]{background:#f6c46d} pre{white-space:pre-wrap}
</style>
<h1>Robotic Car Controls</h1><p>Each tap sends a short drive pulse.</p>
<div class="controls">
<button data-cmd="RL">Pivot L</button><button data-cmd="F">Forward</button><button data-cmd="RR">Pivot R</button>
<button data-cmd="L">Left</button><button data-cmd="S">Stop</button><button data-cmd="R">Right</button>
<button data-cmd="B">Reverse</button>
</div>
<p id="status" role="status"></p><pre id="telemetry">Waiting for telemetry</pre>
<script>
const statusLabel = document.querySelector('#status');
const telemetry = document.querySelector('#telemetry');
document.querySelector('.controls').addEventListener('click', async event => {
  const button = event.target.closest('button');
  if (!button) return;
  try {
    const response = await fetch('/command?cmd=' + button.dataset.cmd, {cache:'no-store'});
    statusLabel.textContent = response.ok ? 'Accepted: ' + button.textContent : 'Command rejected';
  } catch {
    statusLabel.textContent = 'Connection lost';
  }
});
async function read(path, json) {
  const response = await fetch(path, {cache:'no-store', signal:AbortSignal.timeout(1500)});
  if (!response.ok) throw Error('Telemetry unavailable');
  return json ? response.json() : response.text();
}
async function refresh() {
  const [range, imu] = await Promise.allSettled([read('/distance'), read('/mpu', true)]);
  const distance = range.status === 'fulfilled' && Number(range.value) > 0 ? range.value + ' cm' : 'unavailable';
  const orientation = imu.status === 'fulfilled' ? 'Yaw ' + imu.value.estimated_yaw + '° | Pitch ' + imu.value.estimated_pitch + '° | Roll ' + imu.value.estimated_roll + '°' : 'IMU unavailable';
  telemetry.textContent = 'Distance: ' + distance + '\n' + orientation;
  setTimeout(refresh, 500);
}
refresh();
</script></html>
)HTML";

// Handle root path "/"
void handleRoot() {
  server.send_P(200, "text/html", CONTROL_PAGE);
}

// Handle command path "/command?cmd=..."
void handleCommand() {
  server.sendHeader("Cache-Control", "no-store");
  const String command = server.arg("cmd");
  if (command == "S") {
    stopCar();
    server.send(204);
    return;
  }

  unsigned long duration = DEFAULT_PULSE_MS;
  if (server.hasArg("ms")) {
    const String raw = server.arg("ms");
    bool valid = raw.length() > 0 && raw.length() <= 4;
    for (unsigned int i = 0; i < raw.length() && valid; i++) {
      valid = raw[i] >= '0' && raw[i] <= '9';
    }
    duration = raw.toInt();
    if (!valid || duration < 50 || duration > MAX_PULSE_MS) {
      stopCar();
      server.send(400, "text/plain", "Expected duration of 50 to 1000 ms");
      return;
    }
  }

  stopCar();
  if (command == "F") moveForward();
  else if (command == "B") moveBackward();
  else if (command == "L") turnLeft();
  else if (command == "R") turnRight();
  else if (command == "RL") rotateAroundLeft();
  else if (command == "RR") rotateAroundRight();
  else {
    server.send(400, "text/plain", "Unknown command");
    return;
  }
  motionStarted = millis();
  motionDuration = duration;
  moving = true;
  server.send(204);
}

// Handle distance update
void handleDistance() {
  server.send(200, "text/plain", String(distance));
}
