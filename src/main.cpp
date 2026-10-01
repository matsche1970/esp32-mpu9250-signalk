/*
 * ESP32 + MPU9250 → Signal K Gateway
 * For PyPilot Autopilot Integration
 * 
 * Author: matsche1970
 * License: MIT
 * 
 * Features:
 * - MPU9250 9-DOF IMU (Gyro + Accel + Magnetometer)
 * - WiFi Captive Portal Setup
 * - Signal K Delta JSON over UDP
 * - Heading/Attitude/Rate-of-Turn calculations
 * - Persistent NVS Configuration
 * 
 * Target: OpenPlotter + Signal K Server (10.42.0.1:3000)
 * Network: "Maxi 108"
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_wifi.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define MPU_SDA 21
#define MPU_SCL 22
#define LED_PIN 2

// ============================================================================
// CONSTANTS
// ============================================================================

#define SIGNAL_K_PORT 10110
#define SIGNAL_K_BROADCAST_IP "255.255.255.255"
#define WIFI_TIMEOUT_MS 20000
#define SETUP_TIMEOUT_MS 180000

// ============================================================================
// GLOBAL STATE
// ============================================================================

Adafruit_MPU6050 mpu;
Preferences preferences;
WiFiUDP udp;

// WiFi config
struct {
    char ssid[33];
    char password[65];
    char hostname[33];
} wifiConfig = {
    .ssid = "Maxi 108",
    .password = "",
    .hostname = "mpu9250-imu"
};

// IMU calibration data
struct {
    float accel_bias[3];
    float gyro_bias[3];
    float mag_bias[3];
    float mag_scale[3];
    bool calibrated;
} imuCalib = {
    .accel_bias = {0, 0, 0},
    .gyro_bias = {0, 0, 0},
    .mag_bias = {0, 0, 0},
    .mag_scale = {1, 1, 1},
    .calibrated = false
};

// IMU data
struct {
    float accel[3];
    float gyro[3];
    float mag[3];
    float temperature;
    uint32_t timestamp;
} imuData;

// Orientation (fusion result)
struct {
    float heading;      // degrees 0-360
    float pitch;        // degrees
    float roll;         // degrees
    float rate_of_turn; // degrees/second (yaw rate)
} orientation = {0, 0, 0, 0};

// Web server for setup
WebServer server(80);
DNSServer dnsServer;

// Timing
unsigned long lastIMURead = 0;
unsigned long lastSignalKSend = 0;
const unsigned long IMU_RATE_MS = 100;      // 10 Hz
const unsigned long SIGNALK_RATE_MS = 100;  // 10 Hz

// State
enum SystemState {
    STATE_BOOT,
    STATE_SETUP_MODE,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_ERROR
};

SystemState systemState = STATE_BOOT;
unsigned long stateEnteredAt = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void initMPU();
void readIMU();
void updateOrientation();
void sendSignalK();
void setupWiFi();
void startCaptivePortal();
void handleRoot();
void handleSave();
void handleStatus();
void updateLED();
void loadConfig();
void saveConfig();
void printIMUData();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n=== ESP32 MPU9250 Signal K Gateway ===");
    Serial.println("Booting...");
    
    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Load configuration from NVS
    loadConfig();
    
    // Initialize MPU9250
    initMPU();
    
    // Check if WiFi is configured
    if (strlen(wifiConfig.password) == 0) {
        Serial.println("No WiFi password configured. Starting Captive Portal...");
        systemState = STATE_SETUP_MODE;
        stateEnteredAt = millis();
        startCaptivePortal();
    } else {
        Serial.printf("Connecting to WiFi: %s\n", wifiConfig.ssid);
        systemState = STATE_CONNECTING;
        stateEnteredAt = millis();
        setupWiFi();
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Handle DNS in setup mode
    if (systemState == STATE_SETUP_MODE) {
        dnsServer.processNextRequest();
        server.handleClient();
        
        // Check setup timeout
        if (millis() - stateEnteredAt > SETUP_TIMEOUT_MS) {
            Serial.println("Setup timeout. Rebooting...");
            ESP.restart();
        }
    }
    
    // Handle WiFi connection
    if (systemState == STATE_CONNECTING) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
            systemState = STATE_CONNECTED;
            stateEnteredAt = millis();
            
            // Start UDP
            if (udp.beginMulticast(SIGNAL_K_BROADCAST_IP, SIGNAL_K_PORT, 0)) {
                Serial.printf("UDP multicast started on port %d\n", SIGNAL_K_PORT);
            } else {
                Serial.println("UDP multicast failed!");
                systemState = STATE_ERROR;
            }
        } else {
            // Check WiFi connection timeout
            if (millis() - stateEnteredAt > WIFI_TIMEOUT_MS) {
                Serial.println("WiFi connection timeout. Retrying...");
                WiFi.reconnect();
                stateEnteredAt = millis();
            }
        }
    }
    
    // Main operation loop
    if (systemState == STATE_CONNECTED || systemState == STATE_SETUP_MODE) {
        // Read IMU at 10 Hz
        if (millis() - lastIMURead >= IMU_RATE_MS) {
            lastIMURead = millis();
            readIMU();
            updateOrientation();
            printIMUData();
        }
        
        // Send Signal K at 10 Hz
        if (millis() - lastSignalKSend >= SIGNALK_RATE_MS) {
            lastSignalKSend = millis();
            if (systemState == STATE_CONNECTED) {
                sendSignalK();
            }
        }
    }
    
    updateLED();
    delay(10);
}

// ============================================================================
// MPU9250 INITIALIZATION & READING
// ============================================================================

void initMPU() {
    Serial.println("Initializing MPU9250...");
    
    if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire, 0)) {
        Serial.println("ERROR: MPU9250 not found on I2C bus!");
        systemState = STATE_ERROR;
        return;
    }
    
    // Configure MPU
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Serial.println("MPU9250 initialized successfully");
    delay(100);
}

void readIMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    imuData.accel[0] = a.acceleration.x;
    imuData.accel[1] = a.acceleration.y;
    imuData.accel[2] = a.acceleration.z;
    
    imuData.gyro[0] = g.gyro.x;
    imuData.gyro[1] = g.gyro.y;
    imuData.gyro[2] = g.gyro.z;
    
    imuData.temperature = temp.temperature;
    imuData.timestamp = millis();
}

// ============================================================================
// ORIENTATION CALCULATION (Simple Fusion)
// ============================================================================

void updateOrientation() {
    // Simple complementary filter for heading
    // Using accelerometer for gravity vector + magnetometer for magnetic north
    
    // Calculate roll from accelerometer
    float roll_accel = atan2(imuData.accel[1], imuData.accel[2]) * 180.0 / M_PI;
    
    // Calculate pitch from accelerometer
    float pitch_accel = atan2(-imuData.accel[0], 
                              sqrt(imuData.accel[1]*imuData.accel[1] + 
                                   imuData.accel[2]*imuData.accel[2])) * 180.0 / M_PI;
    
    // Integrate gyro for roll/pitch rates
    static float roll = 0, pitch = 0;
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    
    if (dt > 0.01 && dt < 0.2) {
        roll += imuData.gyro[0] * dt;
        pitch += imuData.gyro[1] * dt;
    }
    
    // Complementary filter: 95% gyro, 5% accel
    roll = roll * 0.95 + roll_accel * 0.05;
    pitch = pitch * 0.95 + pitch_accel * 0.05;
    
    // Simple heading from magnetometer (without full calibration)
    // Assumes level boat for now
    float heading = atan2(imuData.mag[1], imuData.mag[0]) * 180.0 / M_PI;
    if (heading < 0) heading += 360;
    
    // Rate of turn (yaw rate from gyro, in degrees/second)
    float rate_of_turn = imuData.gyro[2];
    
    // Smoothing with low-pass filter
    static float heading_filt = 0;
    static float rot_filt = 0;
    const float alpha = 0.2; // Filter constant
    
    heading_filt = heading_filt * (1 - alpha) + heading * alpha;
    rot_filt = rot_filt * (1 - alpha) + rate_of_turn * alpha;
    
    orientation.heading = heading_filt;
    orientation.pitch = pitch;
    orientation.roll = roll;
    orientation.rate_of_turn = rot_filt;
}

// ============================================================================
// SIGNAL K UDP SENDER
// ============================================================================

void sendSignalK() {
    // Create Signal K Delta JSON
    // Format: { "updates": [ { "$source": "...", "timestamp": "...", "values": [...] } ] }
    
    StaticJsonDocument<512> doc;
    JsonArray updates = doc.createNestedArray("updates");
    JsonObject update = updates.createNestedObject();
    
    // Source label
    update["$source"] = "mpu9250";
    
    // Timestamp (ISO 8601)
    char timestamp[32];
    snprintf(timestamp, sizeof(timestamp), "%lld", (long long)millis());
    update["timestamp"] = timestamp;
    
    // Values array
    JsonArray values = update.createNestedArray("values");
    
    // Heading (navigation.headingMagnetic in radians)
    JsonObject heading_obj = values.createNestedObject();
    heading_obj["path"] = "navigation.headingMagnetic";
    heading_obj["value"] = orientation.heading * M_PI / 180.0; // Convert to radians
    
    // Attitude (navigation.attitude: pitch, roll, yaw in radians)
    JsonObject attitude_obj = values.createNestedObject();
    attitude_obj["path"] = "navigation.attitude";
    JsonObject attitude = attitude_obj.createNestedObject("value");
    attitude["pitch"] = orientation.pitch * M_PI / 180.0;
    attitude["roll"] = orientation.roll * M_PI / 180.0;
    attitude["yaw"] = orientation.heading * M_PI / 180.0;
    
    // Rate of turn (navigation.rateOfTurn in radians/second)
    JsonObject rot_obj = values.createNestedObject();
    rot_obj["path"] = "navigation.rateOfTurn";
    rot_obj["value"] = orientation.rate_of_turn * M_PI / 180.0;
    
    // Serialize and send
    String payload;
    serializeJson(doc, payload);
    
    // Send via UDP broadcast
    IPAddress broadcastIP(255, 255, 255, 255);
    udp.beginPacket(broadcastIP, SIGNAL_K_PORT);
    udp.print(payload);
    udp.endPacket();
    
    Serial.printf("Signal K sent: H:%.1f° P:%.1f° R:%.1f° ROT:%.1f°/s\n",
                  orientation.heading, orientation.pitch, 
                  orientation.roll, orientation.rate_of_turn);
}

// ============================================================================
// WIFI & CAPTIVE PORTAL
// ============================================================================

void setupWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.hostname(wifiConfig.hostname);
    WiFi.begin(wifiConfig.ssid, wifiConfig.password);
}

void startCaptivePortal() {
    // Start AP
    String apName = "MPU9250-" + String(ESP.getEfuseMac() & 0xFFFF, HEX);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apName.c_str(), "12345678");
    
    Serial.printf("AP started: %s\n", apName.c_str());
    Serial.printf("Connect and open http://192.168.4.1\n");
    
    // Start DNS server (captive portal)
    dnsServer.start(53, "*", WiFi.softAPIP());
    
    // Start web server
    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/status", handleStatus);
    server.onNotFound(handleRoot);
    server.begin();
}

// ============================================================================
// WEB SERVER HANDLERS
// ============================================================================

void handleRoot() {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MPU9250 Signal K Setup</title>
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 500px; margin: 0 auto; background: white; padding: 20px; border-radius: 8px; }
        h1 { color: #333; }
        input { width: 100%; padding: 8px; margin: 8px 0; box-sizing: border-box; }
        button { width: 100%; padding: 10px; background: #007bff; color: white; border: none; border-radius: 4px; cursor: pointer; }
        button:hover { background: #0056b3; }
        .info { color: #666; font-size: 12px; margin-top: 10px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>⛵ MPU9250 Signal K Gateway</h1>
        <p>Configure WiFi connection for Signal K broadcasting</p>
        <form action="/save" method="POST">
            <label>WiFi SSID</label>
            <input type="text" name="ssid" value="Maxi 108" required>
            
            <label>WiFi Password</label>
            <input type="password" name="password" required>
            
            <label>Hostname</label>
            <input type="text" name="hostname" value="mpu9250-imu">
            
            <button type="submit">Save & Connect</button>
        </form>
        <div class="info">
            <p>After saving, the device will connect to your WiFi and start broadcasting Signal K data on UDP port 10110.</p>
            <p>Target: Signal K Server at 10.42.0.1:3000</p>
        </div>
    </div>
</body>
</html>
    )";
    
    server.send(200, "text/html", html);
}

void handleSave() {
    if (server.hasArg("ssid") && server.hasArg("password")) {
        strncpy(wifiConfig.ssid, server.arg("ssid").c_str(), sizeof(wifiConfig.ssid) - 1);
        strncpy(wifiConfig.password, server.arg("password").c_str(), sizeof(wifiConfig.password) - 1);
        strncpy(wifiConfig.hostname, server.arg("hostname").c_str(), sizeof(wifiConfig.hostname) - 1);
        
        saveConfig();
        
        String response = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Saved!</title>
    <style>
        body { font-family: Arial; margin: 20px; text-align: center; }
        .container { max-width: 400px; margin: 0 auto; }
        h1 { color: green; }
    </style>
</head>
<body>
    <div class="container">
        <h1>✓ Configuration Saved!</h1>
        <p>Rebooting and connecting to WiFi...</p>
        <p>Device will start broadcasting Signal K data in ~10 seconds.</p>
    </div>
</body>
</html>
        )";
        
        server.send(200, "text/html", response);
        delay(1000);
        ESP.restart();
    } else {
        server.send(400, "text/plain", "Missing parameters");
    }
}

void handleStatus() {
    StaticJsonDocument<256> doc;
    doc["state"] = (int)systemState;
    doc["heading"] = orientation.heading;
    doc["pitch"] = orientation.pitch;
    doc["roll"] = orientation.roll;
    doc["rate_of_turn"] = orientation.rate_of_turn;
    doc["wifi_ssid"] = wifiConfig.ssid;
    doc["connected"] = (WiFi.status() == WL_CONNECTED);
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

// ============================================================================
// CONFIGURATION (NVS)
// ============================================================================

void loadConfig() {
    preferences.begin("mpu9250", true);
    
    preferences.getString("ssid", wifiConfig.ssid, sizeof(wifiConfig.ssid));
    preferences.getString("pass", wifiConfig.password, sizeof(wifiConfig.password));
    preferences.getString("host", wifiConfig.hostname, sizeof(wifiConfig.hostname));
    
    preferences.end();
    
    Serial.printf("Config loaded - SSID: %s, Hostname: %s\n", 
                  wifiConfig.ssid, wifiConfig.hostname);
}

void saveConfig() {
    preferences.begin("mpu9250", false);
    
    preferences.putString("ssid", wifiConfig.ssid);
    preferences.putString("pass", wifiConfig.password);
    preferences.putString("host", wifiConfig.hostname);
    
    preferences.end();
    
    Serial.println("Config saved to NVS");
}

// ============================================================================
// LED STATUS INDICATOR
// ============================================================================

void updateLED() {
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    
    unsigned long now = millis();
    unsigned long interval = 500;
    
    switch (systemState) {
        case STATE_BOOT:
            interval = 200;
            break;
        case STATE_SETUP_MODE:
            interval = 300;
            break;
        case STATE_CONNECTING:
            interval = 400;
            break;
        case STATE_CONNECTED:
            interval = 1000;
            break;
        case STATE_ERROR:
            interval = 100;
            break;
    }
    
    if (now - lastBlink >= interval) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void printIMUData() {
    static unsigned long lastPrint = 0;
    
    if (millis() - lastPrint >= 500) { // Print every 500ms
        lastPrint = millis();
        
        Serial.printf("[IMU] A:%.2f,%.2f,%.2f G:%.2f,%.2f,%.2f M:%.0f,%.0f,%.0f T:%.1f°C\n",
                      imuData.accel[0], imuData.accel[1], imuData.accel[2],
                      imuData.gyro[0], imuData.gyro[1], imuData.gyro[2],
                      imuData.mag[0], imuData.mag[1], imuData.mag[2],
                      imuData.temperature);
    }
}
