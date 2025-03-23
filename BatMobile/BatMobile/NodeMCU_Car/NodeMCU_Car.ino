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

// Moving Average Filter for yaw
#define YAW_BUFFER_SIZE 10
float yawBuffer[YAW_BUFFER_SIZE];
int yawBufferIndex = 0;
float yawSum = 0.0;

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
  server.on("/command", handleCommand);
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
    Serial.println("DMP ready");

    // Set DMP output rate (e.g., 50 Hz)
    mpu.setRate(4);  // Set the rate to 200 / (1 + 4) = 40 Hz
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  // Initialize yaw buffer
  for (int i = 0; i < YAW_BUFFER_SIZE; i++) {
    yawBuffer[i] = 0.0;
  }
}

void loop() {
  server.handleClient();

  // Update distance every 500 ms
  if (millis() - lastMeasureTime >= 500) {
    lastMeasureTime = millis();
    distance = measureDistance();
  }

  // Read MPU6050 data when available
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Get quaternion and gravity vectors
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convert yaw, pitch, roll from radians to degrees
    float raw_yaw = ypr[0] * 180 / PI;
    estimated_pitch = ypr[1] * 180 / PI;
    estimated_roll = ypr[2] * 180 / PI;

    // Map yaw angle from -180° to 180° to 0° to 360°
    if (raw_yaw < 0) {
      raw_yaw += 360.0;
    }
    estimated_yaw = raw_yaw;

    // Apply moving average filter to yaw
    yawSum -= yawBuffer[yawBufferIndex];
    yawBuffer[yawBufferIndex] = estimated_yaw;
    yawSum += estimated_yaw;
    yawBufferIndex = (yawBufferIndex + 1) % YAW_BUFFER_SIZE;
    float yawAverage = yawSum / YAW_BUFFER_SIZE;
    estimated_yaw = yawAverage;

    // Print readings to Serial Monitor
    Serial.print("Yaw: ");
    Serial.print(estimated_yaw, 2);
    Serial.print("°, Pitch: ");
    Serial.print(estimated_pitch, 2);
    Serial.print("°, Roll: ");
    Serial.print(estimated_roll, 2);
    Serial.println("°");
  }
}

// Stop all motors
void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Car Stopped.");
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
  // Prepare JSON response with DMP data
  String jsonResponse = "{";
  jsonResponse += "\"estimated_pitch\": " + String(estimated_pitch, 2) + ",";
  jsonResponse += "\"estimated_roll\": " + String(estimated_roll, 2) + ",";
  jsonResponse += "\"estimated_yaw\": " + String(estimated_yaw, 2) + "}";
  server.send(200, "application/json", jsonResponse);
}

// Create the HTML web page
String createWebPage() {
  String page = "<!DOCTYPE html><html><head><title>ESP8266 Car Control</title>";
  page += "<style>";
  page += "body { text-align: center; font-family: Arial; }";
  page += ".button { width: 80px; height: 80px; font-size: 30px; margin: 10px; }";
  page += "</style></head><body>";
  page += "<h1>ESP8266 Car Control</h1>";
  page += "<div>";
  page += "<button class='button' id='forward'>Forward</button><br>";
  page += "<button class='button' id='left'>Left</button>";
  page += "<button class='button' id='stop'>Stop</button>";
  page += "<button class='button' id='right'>Right</button><br>";
  page += "<button class='button' id='backward'>Back</button><br>";
  page += "<button class='button' id='rotateLeft'>Rotate L</button>";
  page += "<button class='button' id='rotateRight'>Rotate R</button>";
  page += "</div>";
  page += "<p>Distance: <span id='distance'>-</span> cm</p>";
  page += "<p>MPU6050 Data:</p>";
  page += "<p>Pitch: <span id='pitch'>-</span>°, Roll: <span id='roll'>-</span>°, Yaw: <span id='yaw'>-</span>°</p>";
  page += "<script>";
  page += "function sendCommand(cmd){ fetch('/command?cmd=' + cmd); }";
  page += "function addButtonEvent(id, cmd){";
  page += "  var button = document.getElementById(id);";
  page += "  button.addEventListener('touchstart', function(e){ e.preventDefault(); sendCommand(cmd); });";
  page += "  button.addEventListener('touchend', function(e){ e.preventDefault(); sendCommand('S'); });";
  page += "  button.addEventListener('mousedown', function(){ sendCommand(cmd); });";
  page += "  button.addEventListener('mouseup', function(){ sendCommand('S'); });";
  page += "}";
  page += "addButtonEvent('forward', 'F');";
  page += "addButtonEvent('backward', 'B');";
  page += "addButtonEvent('left', 'L');";
  page += "addButtonEvent('right', 'R');";
  page += "addButtonEvent('rotateLeft', 'RL');";
  page += "addButtonEvent('rotateRight', 'RR');";
  page += "function updateDistance(){";
  page += "  fetch('/distance').then(function(response){ return response.text(); }).then(function(data){";
  page += "    document.getElementById('distance').innerHTML = data;";
  page += "  });";
  page += "}";
  page += "function updateMPUData(){";
  page += "  fetch('/mpu').then(function(response){ return response.json(); }).then(function(data){";
  page += "    document.getElementById('pitch').innerHTML = data.estimated_pitch.toFixed(2);";
  page += "    document.getElementById('roll').innerHTML = data.estimated_roll.toFixed(2);";
  page += "    document.getElementById('yaw').innerHTML = data.estimated_yaw.toFixed(2);";
  page += "  });";
  page += "}";
  page += "setInterval(updateDistance, 500);";  // Update every 500ms
  page += "setInterval(updateMPUData, 500);";   // Update every 500ms
  page += "</script></body></html>";
  return page;
}

// Handle root path "/"
void handleRoot() {
  server.send(200, "text/html", createWebPage());
}

// Handle command path "/command?cmd=..."
void handleCommand() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    Serial.print("Received Command: ");
    Serial.println(cmd);

    if (cmd == "F") {
      moveForward();
    } else if (cmd == "B") {
      moveBackward();
    } else if (cmd == "L") {
      turnLeft();
    } else if (cmd == "R") {
      turnRight();
    } else if (cmd == "S") {
      stopCar();
    } else if (cmd == "RL") {
      rotateAroundLeft();
    } else if (cmd == "RR") {
      rotateAroundRight();
    } else {
      Serial.println("Unknown Command.");
    }
  } else {
    stopCar();
  }
  server.send(204, "text/plain", "");  // Send empty response
}

// Handle distance update
void handleDistance() {
  server.send(200, "text/plain", String(distance));
}
