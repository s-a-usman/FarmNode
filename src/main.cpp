#include <Arduino.h>
#include <WiFi.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize a 20x4 LCD with I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Smart Agricultural Node for ESP32
// This code reads environmental and soil data using various sensors and sends it to a server
// for monitoring and control purposes.


// DHT22 Configuration
#define DHT_PIN 4
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// NPK Sensor Configuration (RS485/UART)
#define NPK_RX_PIN 32 //RO
#define NPK_TX_PIN 33 //DI

#define NPK_DE_PIN 25  // Direction control pin for RS485
#define NPK_RE_PIN 26  // Receive Enable pin for RS485

SoftwareSerial npkSerial(NPK_RX_PIN, NPK_TX_PIN); // RX=22, TX=23

// GPS Module Configuration (NEO-6M)
#define GPS_RX_PIN 18
#define GPS_TX_PIN 19
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

// Soil Moisture Sensor Configuration
#define SOIL_MOISTURE_PIN 35  // Analog pin
#define SOIL_MOISTURE_POWER_PIN 5  // Power control pin

// WiFi Configuration
const char* ssid = "Home-Wifi";   //Office router
const char* password = "homePass"; //Password

// Sensor data structure
struct SensorData {
  float temperature;
  float humidity;
  float nitrogen;
  float phosphorus;
  float potassium;
  float ph;
  float ec;
  // float NPKMoisture;
  // float NPKHumidity;
  int soilMoisture;
  int soilMoisturePercent;
  double latitude;
  double longitude;
  double altitude;
  int satellites;
  bool gpsValid;
  String gpsTime;
  String gpsDate;
};

// NPK Sensor Commands (typical for soil NPK sensors)
byte npkCommand[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
byte npkCommandSize = 8;

void readDHT22(SensorData &data) {
  Serial.println("Reading DHT22...");
  
  data.temperature = dht.readTemperature();
  data.humidity = dht.readHumidity();
  
  if (isnan(data.temperature) || isnan(data.humidity)) {
    Serial.println("Failed to read from DHT22 sensor!");
    data.temperature = -999;
    data.humidity = -999;
  }
}

void readGPS(SensorData &data) {
  Serial.println("Reading GPS...");
  
  unsigned long startTime = millis();
  bool newData = false;
  
  // Read GPS data for up to 2 seconds
  while (millis() - startTime < 2000) {
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read())) {
        newData = true;
      }
    }
  }
  
  if (newData && gps.location.isValid()) {
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.kilometers();
    data.satellites = gps.satellites.value();
    data.gpsValid = true;
    
    // Format time and date
    if (gps.time.isValid()) {
      char timeStr[10];
      sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      data.gpsTime = String(timeStr);
    } else {
      data.gpsTime = "N/A";
    }
    
    if (gps.date.isValid()) {
      char dateStr[12];
      sprintf(dateStr, "%02d/%02d/%04d", gps.date.month(), gps.date.day(), gps.date.year());
      data.gpsDate = String(dateStr);
    } else {
      data.gpsDate = "N/A";
    }
    
  } else {
    Serial.println("GPS data not available or invalid!");
    data.latitude = -999;
    data.longitude = -999;
    data.altitude = -999;
    data.satellites = 0;
    data.gpsValid = false;
    data.gpsTime = "N/A";
    data.gpsDate = "N/A";
  }
}

void readNPKSensor(SensorData &data) {
  Serial.println("Reading NPK Sensor...");
  
  // Set to transmit mode
  digitalWrite(NPK_DE_PIN, HIGH);
  digitalWrite(NPK_RE_PIN, HIGH);
  delay(10);
  
  // Send command to NPK sensor
  npkSerial.write(npkCommand, npkCommandSize);
  npkSerial.flush();
  
  // Set to receive mode
  digitalWrite(NPK_DE_PIN, LOW);
  digitalWrite(NPK_RE_PIN, LOW);
  delay(100);
  
  // Read response
  byte response[19]; // Typical response size for NPK sensor
  int responseIndex = 0;
  unsigned long startTime = millis();
  
  while (npkSerial.available() && responseIndex < 19 && (millis() - startTime) < 1000) {
    response[responseIndex] = npkSerial.read();
    responseIndex++;
    delay(10);
  }
  
  if (responseIndex >= 19) {
    // Parsing response (Susceptible to change based on NPK sensor calibration)
    data.nitrogen = ((response[3] << 8) | response[4]) / 10.0;
    data.phosphorus = ((response[5] << 8) | response[6]) / 10.0;
    data.potassium = ((response[7] << 8) | response[8]) / 10.0;
    data.ph = ((response[9] << 8) | response[10]) / 100.0;
    data.ec = ((response[11] << 8) | response[12]) / 100.0;
    // data.NPKMoisture = ((response[13] << 8) | response[14]) / 10.0;
    // data.NPKHumidity = ((response[15] << 8) | response[16]) / 10.0;
  } else {
    Serial.println("Failed to read from NPK sensor!");
    data.nitrogen = -999;
    data.phosphorus = -999;
    data.potassium = -999;
    data.ph = -999;
    data.ec = -999;
    // data.NPKMoisture = -999;
    // data.NPKHumidity = -999;
  }
}

void readSoilMoisture(SensorData &data) {
  Serial.println("Reading Soil Moisture...");
  
  // Power on the sensor
  digitalWrite(SOIL_MOISTURE_POWER_PIN, HIGH);
  delay(100); // Wait for sensor to stabilize
  
  // Read analog value
  int rawValue = analogRead(SOIL_MOISTURE_PIN);
  
  // Power off the sensor to prevent corrosion
  digitalWrite(SOIL_MOISTURE_POWER_PIN, LOW);
  
  data.soilMoisture = rawValue;
  
  // Convert to percentage (calibrate these values based on your sensor)
  // Typical values: 0-300 (water), 300-700 (moist), 700-4095 (dry)
  data.soilMoisturePercent = map(rawValue, 0, 4095, 100, 0);
  data.soilMoisturePercent = constrain(data.soilMoisturePercent, 0, 100);
}

void displaySensorData(const SensorData &data) {
  Serial.println("\n=== SENSOR READINGS ===");
  
  Serial.println("GPS Location:");
  if (data.gpsValid) {
    Serial.printf("  Latitude: %.6f°\n", data.latitude);
    Serial.printf("  Longitude: %.6f°\n", data.longitude);
    Serial.printf("  Altitude: %.2f m\n", data.altitude);
    Serial.printf("  Satellites: %d\n", data.satellites);
    Serial.printf("  GPS Time: %s\n", data.gpsTime.c_str());
    Serial.printf("  GPS Date: %s\n", data.gpsDate.c_str());
  } else {
    Serial.println("  GPS: No valid fix");
  }
  
  Serial.println("\nEnvironmental Data:");
  Serial.printf("  Temperature: %.2f°C\n", data.temperature);
  Serial.printf("  Humidity: %.2f%%\n", data.humidity);
  
  Serial.println("\nSoil Nutrients:");
  Serial.printf("  Nitrogen (N): %.1f mg/kg\n", data.nitrogen);
  Serial.printf("  Phosphorus (P): %.1f mg/kg\n", data.phosphorus);
  Serial.printf("  Potassium (K): %.1f mg/kg\n", data.potassium);
  
  Serial.println("\nSoil Conditions:");
  Serial.printf("  pH: %.2f\n", data.ph);
  Serial.printf("  EC: %.2f mS/cm\n", data.ec);
  // Serial.printf("  NPK Moisture: %.1f%%\n", data.NPKMoisture);
  // Serial.printf("  NPK Humidity: %.1f%%\n", data.NPKHumidity);
  Serial.printf("  Soil Moisture: %d (Raw: %d)\n", data.soilMoisturePercent, data.soilMoisture);
  
  // Soil moisture interpretation
  if (data.soilMoisturePercent > 70) {
    Serial.println("  Status: Wet");
  } else if (data.soilMoisturePercent > 30) {
    Serial.println("  Status: Moist");
  } else {
    Serial.println("  Status: Dry");
  }
  
  Serial.println("========================\n");
}


// Create a new function to display sensor data on the LCD
void displayOnLCD(const SensorData &data) {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(data.temperature, 1);
  lcd.print("C     H:");
  lcd.print(data.humidity, 0);

  lcd.setCursor(0, 1);
  lcd.print("N:");
  lcd.print(data.nitrogen, 0);
  lcd.print("     EC:");
  lcd.print(data.ec, 2);

  lcd.setCursor(0, 2);
  lcd.print("P:");
  lcd.print(data.phosphorus, 0);
  lcd.print("     pH:");
  lcd.print(data.ph, 1);

  lcd.setCursor(0, 3);
  lcd.print("K:");
  lcd.print(data.potassium, 0);
  lcd.print("     SM:");
  lcd.print(data.soilMoisturePercent);
  lcd.print("%");
}

void sendDataToServer(const SensorData &data) {
  // Create JSON payload
  StaticJsonDocument<768> doc;
  doc["device_id"] = "agri_node_001";
  doc["timestamp"] = millis();
  
  JsonObject location = doc.createNestedObject("location");
  location["latitude"] = data.latitude;
  location["longitude"] = data.longitude;
  location["altitude"] = data.altitude;
  location["satellites"] = data.satellites;
  location["gps_valid"] = data.gpsValid;
  location["gps_time"] = data.gpsTime;
  location["gps_date"] = data.gpsDate;
  
  JsonObject environmental = doc.createNestedObject("environmental");
  environmental["temperature"] = data.temperature;
  environmental["humidity"] = data.humidity;
  
  JsonObject soil_nutrients = doc.createNestedObject("soil_nutrients");
  soil_nutrients["nitrogen"] = data.nitrogen;
  soil_nutrients["phosphorus"] = data.phosphorus;
  soil_nutrients["potassium"] = data.potassium;
  
  JsonObject soil_conditions = doc.createNestedObject("soil_conditions");
  soil_conditions["ph"] = data.ph;
  soil_conditions["ec"] = data.ec;
  soil_conditions["moisture_percent"] = data.soilMoisturePercent;
  soil_conditions["moisture_raw"] = data.soilMoisture;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Serial.println("JSON Data:");
  // Serial.println(jsonString);
  
  // Here you would typically send the data to your server
  // Examples: HTTP POST, MQTT publish, LoRaWAN, etc.
  // sendHttpPost(jsonString);
  // publishMQTT(jsonString);
}

// Optional: HTTP POST function
/*
void sendHttpPost(String data) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("http://your-server.com/api/sensor-data");
    http.addHeader("Content-Type", "application/json");
    
    int httpResponseCode = http.POST(data);
    
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("HTTP Response: " + response);
    } else {
      Serial.println("HTTP POST failed");
    }
    
    http.end();
  }
}
*/

// Optional: MQTT publish function
/*
#include <PubSubClient.h>
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void publishMQTT(String data) {
  if (mqttClient.connected()) {
    mqttClient.publish("agriculture/sensor/data", data.c_str());
    Serial.println("Data published to MQTT");
  }
}
*/


void setup() {
  Serial.begin(115200);
  

  lcd.init();
  lcd.backlight();
  lcd.clear();
  // Initialize DHT sensor
  dht.begin();
  
  // Initialize NPK sensor
  npkSerial.begin(4800);
  pinMode(NPK_DE_PIN, OUTPUT);
  pinMode(NPK_RE_PIN, OUTPUT);

  digitalWrite(NPK_DE_PIN, LOW); // Set to receive mode
  digitalWrite(NPK_RE_PIN, LOW); // Set to receive mode
  
  // Initialize GPS module
  gpsSerial.begin(9600);
  Serial.println("Waiting for GPS fix...");
  
  // Initialize soil moisture sensor
  pinMode(SOIL_MOISTURE_POWER_PIN, OUTPUT);
  digitalWrite(SOIL_MOISTURE_POWER_PIN, LOW); // Turn off initially
  
  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println("Smart Agricultural Node Initialized");
  Serial.println("=====================================");
}

void loop() {
  SensorData data;
  
  // Read all sensors
  readDHT22(data);
  readGPS(data);
  readNPKSensor(data);
  readSoilMoisture(data);
  
  // Display sensor data
  displaySensorData(data);

  // Display on LCD
  displayOnLCD(data);
  
  // Send data to server/cloud (optional)
  sendDataToServer(data);
  
  // Wait before next reading
  delay(3000); // 3 seconds
}