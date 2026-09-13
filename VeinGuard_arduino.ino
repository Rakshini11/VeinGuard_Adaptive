/* ============================================================
   VEINGUARD ADAPTIVE — ESP32-S3 FIRMWARE (FINAL)
   Sensors: MAX30102 (HR + SpO2), MPU6050 (posture + activity)
   ============================================================ */

#include <WiFi.h>
#include <WiFiServer.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ================== Wi-Fi Credentials ==================
const char* WIFI_SSID     = "Trickster_2.4G";
const char* WIFI_PASSWORD = "prabha1122";

// ================== Pin Definitions ==================
#define I2C_SDA      8
#define I2C_SCL      9

// ================== Objects ==================
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;
WiFiServer server(80);

// ================== State Variables ==================
float heartRate     = 0;
float spo2          = 0;
float activityLevel = 0;
String posture      = "Sitting";

// Heart rate
const byte RATE_SIZE = 4;
byte  rates[RATE_SIZE];
byte  rateSpot = 0;
long  lastBeat = 0;
float beatAvg  = 0;

// Finger detection
bool fingerPresent = false;

// MPU6050 filtering
float smoothAx = 0, smoothAy = 0, smoothAz = 0;
const float ALPHA = 0.85;

// Posture timing
unsigned long lastMovementTime = 0;
unsigned long sessionStart = 0;

// ================== SETUP ==================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n============================================");
    Serial.println("   VeinGuard Adaptive — ESP32-S3");
    Serial.println("============================================");

    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("I2C OK");

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found!");
        while (1) delay(1000);
    }
    particleSensor.setup(0x7F, 4, 2, 100, 411, 4096);
    particleSensor.setPulseAmplitudeRed(0x7F);
    particleSensor.setPulseAmplitudeIR(0x7F);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("MAX30102 OK");

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1) delay(1000);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 OK");

    Serial.printf("\nConnecting to %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500); Serial.print("."); attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWi-Fi failed!");
        while (1) delay(1000);
    }

    Serial.println("\nWi-Fi OK");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.println("HTTP server on port 80");

    sessionStart = millis();
    lastMovementTime = millis();
}

// ================== MAIN LOOP ==================
void loop() {
    readMAX30102();
    readMPU6050();
    computeActivity();
    handleHTTP();
}

// ================== MAX30102 ==================
void readMAX30102() {
    long irValue  = particleSensor.getIR();
    long redValue = particleSensor.getRed();

    // ===== Finger detection =====
    // IR must be above 50000 to consider finger present
    if (irValue < 50000) {
        // No finger — reset EVERYTHING
        fingerPresent = false;
        heartRate = 0;
        spo2 = 0;
        beatAvg = 0;
        rateSpot = 0;
        for (byte i = 0; i < RATE_SIZE; i++) rates[i] = 0;
        lastBeat = 0;
        return;
    }

    // Finger detected
    fingerPresent = true;

    // ===== Heart Rate =====
    if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        if (delta > 300 && delta < 2000) {
            float bpm = 60.0 / (delta / 1000.0);

            if (bpm > 45 && bpm < 150) {
                rates[rateSpot++] = (byte)bpm;
                rateSpot %= RATE_SIZE;

                float sum = 0;
                for (byte x = 0; x < RATE_SIZE; x++) sum += rates[x];
                float avg = sum / RATE_SIZE;

                bool consistent = true;
                for (byte x = 0; x < RATE_SIZE; x++) {
                    if (abs((int)rates[x] - (int)avg) > 30) consistent = false;
                }

                if (consistent) beatAvg = avg;
            }
        }
    }

    if (beatAvg > 0) heartRate = beatAvg;

    // ===== SpO2 =====
    static float redMin = 100000, redMax = 0;
    static float irMin  = 100000, irMax  = 0;
    static unsigned long lastSpO2Calc = 0;

    if (redValue < redMin) redMin = redValue;
    if (redValue > redMax) redMax = redValue;
    if (irValue  < irMin)  irMin  = irValue;
    if (irValue  > irMax)  irMax  = irValue;

    if (millis() - lastSpO2Calc > 2000) {
        float redAC = redMax - redMin;
        float irAC  = irMax  - irMin;
        float redDC = (redMax + redMin) / 2.0;
        float irDC  = (irMax  + irMin)  / 2.0;

        if (redDC > 0 && irDC > 0 && irAC > 0) {
            float R = (redAC / redDC) / (irAC / irDC);
            float s = 110.0 - 25.0 * R;
            if (s > 100) s = 100;
            if (s < 0)   s = 0;
            spo2 = s;
        }

        redMin = 100000; redMax = 0;
        irMin  = 100000; irMax  = 0;
        lastSpO2Calc = millis();
    }
}

// ================== MPU6050 ==================
void readMPU6050() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    smoothAx = ALPHA * smoothAx + (1 - ALPHA) * a.acceleration.x;
    smoothAy = ALPHA * smoothAy + (1 - ALPHA) * a.acceleration.y;
    smoothAz = ALPHA * smoothAz + (1 - ALPHA) * a.acceleration.z;
}

// ================== POSTURE ==================
void computeActivity() {
    float mag = sqrt(smoothAx*smoothAx + smoothAy*smoothAy + smoothAz*smoothAz);
    float dev = fabs(mag - 9.81);
    activityLevel = constrain(dev * 25.0, 0, 100);

    unsigned long now = millis();

    if (activityLevel > 25) lastMovementTime = now;

    String newPosture;

    if (now - lastMovementTime < 1500) {
        newPosture = "Walking";
    }
    else if (now - lastMovementTime > 10000) {
        newPosture = "Idle";
    }
    else if (fabs(smoothAz) > 6.5) {
        newPosture = "Standing";
    }
    else {
        newPosture = "Sitting";
    }

    posture = newPosture;
}

// ================== HTTP ==================
void handleHTTP() {
    WiFiClient client = server.available();
    if (!client) return;

    unsigned long timeout = millis();
    while (client.available() == 0) {
        if (millis() - timeout > 200) {
            client.stop();
            return;
        }
    }

    String request = client.readStringUntil('\r');
    client.flush();

    String json = buildJSON();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Access-Control-Allow-Methods: GET, OPTIONS");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(json.length());
    client.println();
    client.print(json);

    delay(1);
    client.stop();
}

// ================== JSON ==================
String buildJSON() {
    unsigned long up = (millis() - sessionStart) / 1000;
    float wearHours = up / 3600.0;

    String json = "{";
    json += "\"heartRate\":"   + String((int)heartRate) + ",";
    json += "\"spo2\":"        + String((int)spo2) + ",";
    json += "\"finger\":"      + String(fingerPresent ? "true" : "false") + ",";
    json += "\"posture\":\""   + posture + "\",";
    json += "\"activity\":"    + String((int)activityLevel) + ",";
    json += "\"wearTime\":"    + String(wearHours, 2) + ",";
    json += "\"rssi\":"        + String(WiFi.RSSI()) + ",";
    json += "\"uptime\":"      + String(up);
    json += "}";
    return json;
}