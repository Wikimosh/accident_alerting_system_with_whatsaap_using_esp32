/*
 * Integrated Impact Detection, GPS Location Reporting,
 * WhatsApp Alerts via CallMeBot, MPU6050 Roll/Tilt Display on OLED
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <Callmebot_ESP32.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// WiFi credentials
const char* ssid = "Nickson";
const char* password = "12345678";

// CallMeBot credentials
String phoneNumber = "+254702482457";
String apiKey = "5987481";

// Car Details
String carPlate = "KBW 512C";

// GPS
TinyGPSPlus gps;
HardwareSerial GPSserial(1); // Using Serial1 for GPS

// MPU6050
Adafruit_MPU6050 mpu;

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Impact and roll thresholds
float impactThreshold = 9.0; // G-force threshold
float rollThreshold = 1.0;   // Gyro X-axis threshold for roll (rad/s)
unsigned long lastAlertTime = 0;
const unsigned long alertDelay = 10000; // 10 seconds between alerts

// Suppression Button
const int buttonPin = 12;
bool suppressAlert = false;

void setup() {
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED allocation failed");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize GPS
  GPSserial.begin(9600, SERIAL_8N1, 16, 17); // RX = 16, TX = 17

  // Connect WiFi
  WiFi.begin(ssid, password);
  displayMessage("Connecting WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  displayMessage("WiFi Connected!");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float totalAccel = sqrt(ax * ax + ay * ay + az * az);
  float rollSpeed = g.gyro.x;

  suppressAlert = digitalRead(buttonPin) == LOW;

  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  if (!suppressAlert && millis() - lastAlertTime > alertDelay) {
    if (gps.location.isValid()) {
      String latitude = String(gps.location.lat(), 6);
      String longitude = String(gps.location.lng(), 6);

      if (totalAccel > impactThreshold) {
        String message = "Impact Detected!\nCar: " + carPlate +
                         "\nLat: " + latitude +
                         "\nLng: " + longitude;
        Callmebot.whatsappMessage(phoneNumber, apiKey, message);
        lastAlertTime = millis();
      }

      if (abs(rollSpeed) > rollThreshold) {
        String message = "Roll Detected!\nCar: " + carPlate +
                         "\nLat: " + latitude +
                         "\nLng: " + longitude;
        Callmebot.whatsappMessage(phoneNumber, apiKey, message);
        lastAlertTime = millis();
      }
    }
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Roll: ");
  display.println(rollSpeed);
  display.print("Pitch: ");
  display.println(g.gyro.y);
  display.print("Impact G: ");
  display.println(totalAccel);

  if (suppressAlert) {
    display.println("API Suppressed");
  } else {
    display.println("API Active");
  }

  if (gps.location.isValid()) {
    display.print("Lat: ");
    display.println(gps.location.lat(), 4);
    display.print("Lng: ");
    display.println(gps.location.lng(), 4);
  } else {
    display.println("GPS Waiting...");
  }

  display.display();
  delay(500);
}

void displayMessage(String msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
}
