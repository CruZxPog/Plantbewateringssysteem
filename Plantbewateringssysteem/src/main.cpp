#include "config.h"
#include "security.h"

#include "Arduino.h"
#include "ArduinoTrace.h"

#include "OneWire.h"
#include "DallasTemperature.h"
#include "WiFi.h"
#include "HTTPClient.h"
#include "time.h"

#include "HardwareSerial.h"
#include "TinyGPS++.h"

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// NTP (Network Time Protocol) configuration
const char *NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 0;
const int DAYLIGHT_OFFSET_SEC = 0;

const String GOOGLE_APPS_SCRIPT_URL = "https://script.google.com/macros/s/";

// Enable ArduinoTrace debugging
#define ARDUINOTRACE_ENABLE 1

RTC_DATA_ATTR int bootCount = 0;

// Pin definitions for sensors and actuators
#define OVERRIDE_BUTTON_PIN 25
#define BVHR_PIN A3
#define BVHC_PIN A1
#define TEMPSENS_PIN D7
#define PUMP_PIN D12
#define RX_PIN 16
#define TX_PIN 17
#define SENSOR_POWER D6  // GPIO pin controlling 2N3906 transistor for sensor power

OneWire oneWire(TEMPSENS_PIN);
DallasTemperature sensors(&oneWire);

// Timing and state variables
unsigned long lastReadTime = 0;
unsigned long startWateringTime = 0;
int wateringDuration = 0;
byte pumpState = OFF;  // Pump state (ON/OFF)

// Deep sleep wake-up reason
esp_sleep_wakeup_cause_t wakeup_reason;
void print_wakeup_reason() {
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}
int temperatuur = 0;  // Temperature variable
// Sensor reading functions
int leesTemperatuur() {
  // Request temperature from the sensor and return it in °C
  sensors.requestTemperatures();
  temperatuur = sensors.getTempCByIndex(0);
  return sensors.getTempCByIndex(0);
}

Categorie capBVH = ONBEKEND;
Categorie berekenCategorieCapactieveBVH(int sensorwaarde) {
  // Determine category for capacitieve sensor reading
  if (sensorwaarde >= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX) {
    capBVH = DROOG;
    return DROOG;
  } else if (sensorwaarde >= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
    capBVH = VOCHTIG;
    return VOCHTIG;
  } else if (sensorwaarde >= CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX) {
    capBVH = NAT;
    return NAT;
  } else {
    return ONBEKEND;
  }
}

Categorie resBVH = ONBEKEND;
Categorie berekenCategorieResistieveBVH(int sensorwaarde) {
  // Determine category for resistieve sensor reading
  if (sensorwaarde >= RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX) {
    resBVH = DROOG;
    return DROOG;
  } else if (sensorwaarde >= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
    resBVH = VOCHTIG;
    return VOCHTIG;
  } else if (sensorwaarde >= RESISTIEVE_SENSOR_NAT_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_NAT_INTERVAL_MAX) {
    resBVH = NAT;
    return NAT;
  } else {
    return ONBEKEND;
  }
}

Categorie berekenSamengesteldeCategorie(Categorie categorieResistieveBVH, Categorie categorieCapacitieveBVH, int temperatuur) {
  // Determine combined category based on both sensors and temperature
  if (temperatuur > TEMP_HIGH) {
    if (categorieResistieveBVH == DROOG && categorieCapacitieveBVH == DROOG) {
      return DROOG;
    } else if (categorieResistieveBVH == DROOG && categorieCapacitieveBVH == VOCHTIG) {
      return DROOG;
    } else if (categorieResistieveBVH == VOCHTIG && categorieCapacitieveBVH == DROOG) {
      return DROOG;
    } else if (categorieResistieveBVH == DROOG && categorieCapacitieveBVH == NAT) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == NAT && categorieCapacitieveBVH == DROOG) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == VOCHTIG && categorieCapacitieveBVH == VOCHTIG) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == VOCHTIG && categorieCapacitieveBVH == NAT) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == NAT && categorieCapacitieveBVH == VOCHTIG) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == NAT && categorieCapacitieveBVH == NAT) {
      return NAT;
    } else {
      return ONBEKEND;
    }
  } else {
    if (categorieResistieveBVH == DROOG && categorieCapacitieveBVH == DROOG) {
      return DROOG;
    } else if (categorieResistieveBVH == DROOG && categorieCapacitieveBVH == VOCHTIG) {
      return DROOG;
    } else if (categorieResistieveBVH == VOCHTIG && categorieCapacitieveBVH == DROOG) {
      return DROOG;
    } else if (categorieResistieveBVH == DROOG && categorieCapacitieveBVH == NAT) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == NAT && categorieCapacitieveBVH == DROOG) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == VOCHTIG && categorieCapacitieveBVH == VOCHTIG) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == VOCHTIG && categorieCapacitieveBVH == NAT) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == NAT && categorieCapacitieveBVH == VOCHTIG) {
      return VOCHTIG;
    } else if (categorieResistieveBVH == NAT && categorieCapacitieveBVH == NAT) {
      return NAT;
    } else {
      return ONBEKEND;
    }
  }
}

// Pump control functions
void zetWaterpompAan(int duurtijd) {
  TRACE();  // entering pump ON function
  pumpState = ON;
  digitalWrite(PUMP_PIN, ON);
  // Initialize timing for watering
  startWateringTime = millis();
  wateringDuration = duurtijd;
  DUMP(wateringDuration);    // log how long the pump will run (ms)
  DUMP(startWateringTime);   // log when the pump started (timestamp)
}

void zetWaterpompUit() {
  TRACE();  // entering pump OFF function
  pumpState = OFF;
  digitalWrite(PUMP_PIN, OFF);
  // Reset watering timing variables
  startWateringTime = 0;
  wateringDuration = 0;
  DUMP(pumpState);  // log pump state to confirm it's OFF
}

// Read sensors and water if needed
void leesSensorenEnGeefWaterIndienNodig() {
  TRACE();  // entering sensor reading routine
  // Power up sensors and read values
  int capacitieve_bvh_waarde = analogRead(BVHC_PIN);
  DUMP(capacitieve_bvh_waarde);   // log capacitieve sensor raw value
  int resistieve_bvh_waarde = analogRead(BVHR_PIN);
  DUMP(resistieve_bvh_waarde);    // log resistieve sensor raw value
  int temperatuur = leesTemperatuur();
  DUMP(temperatuur);              // log temperature reading

  // Determine categories from sensor values
  Categorie categorieCapacitieveBVH = berekenCategorieCapactieveBVH(capacitieve_bvh_waarde);
  DUMP(categorieCapacitieveBVH);  // log category from capacitieve sensor
  Categorie categorieResistieveBVH = berekenCategorieResistieveBVH(resistieve_bvh_waarde);
  DUMP(categorieResistieveBVH);   // log category from resistieve sensor
  Categorie categorie = berekenSamengesteldeCategorie(categorieResistieveBVH, categorieCapacitieveBVH, temperatuur);
  DUMP(categorie);                // log combined category

  // Print human-readable category result
  if (categorie == DROOG) {
    Serial.println("categorie = DROOG");
  } else if (categorie == VOCHTIG) {
    Serial.println("categorie = VOCHTIG");
  } else if (categorie == NAT) {
    Serial.println("categorie = NAT");
  } else {
    Serial.println("categorie = ONBEKEND");
  }

  // Decide on watering based on category and temperature thresholds
  if (temperatuur > TEMP_HIGH && categorie == DROOG) {
    TRACE();  // dry soil and high temp branch
    zetWaterpompAan(PUMP_TIMER_LONG);
    Serial.println("Water geven met lange duur");
  } else if (temperatuur >= TEMP_LOW && temperatuur <= TEMP_HIGH && categorie == DROOG) {
    TRACE();  // dry soil and moderate temp branch
    zetWaterpompAan(PUMP_TIMER_SHORT);
    Serial.println("Water geven met korte duur");
  }
}

// WiFi and GPS initialization
void initWifiAndGps() {
  TRACE();  // entering WiFi/GPS init
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  Serial.flush();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi!");

  Serial.print("Search for GPS signal...");
  bool gpsState = false;
  int trys = 0;
  while (!gpsState && trys <= 5) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        Serial.print(',');
        Serial.print(gps.location.lng(), 6);
        gpsState = true;
        Serial.println("GPS signal found!");
      } else {
        Serial.println("INVALID");
      }
    }
    trys++;
    if (trys == 5) {
      Serial.println("GPS signal not found!");
      break;
    }
    delay(500);
  }
}

void setTime() {
  TRACE();  // entering NTP time sync
  Serial.println("Synchronizing time with NTP...");
  configTime(3600, 3600, NTP_SERVER);  // Set timezone offset (UTC+1 standard, +2 DST)

  // Wait for a valid time
  struct tm timeinfo;
  int retry = 0;
  while (!getLocalTime(&timeinfo) && retry < 10) {
    Serial.println("Failed to obtain time, retrying...");
    delay(1000);
    retry++;
  }

  if (retry >= 10) {
    Serial.println("Could not get time from NTP! Using default time.");
  } else {
    Serial.println("Time synchronized successfully.");
  }
}

// Helper to get current date/time as string
String getCurrentDateAndTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "";
  }
  char timeStringBuff[50];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  return String(timeStringBuff);
}

struct SensorData {
  String date;
  int temp;
  Categorie bvh_cap;
  Categorie bvh_res;
  Categorie bvh_samen;
  int pomp_sec;
  float latitude;
  float longitude;
};

SensorData compileSensorData() {
  SensorData data;
  data.date = getCurrentDateAndTime();
  if (temperatuur == 0){
    temperatuur = leesTemperatuur();  // Read temperature if not already done
  }
  data.temp = temperatuur;  // Use the temperature read from the sensor
  data.bvh_cap = capBVH;
  data.bvh_res = resBVH;
  data.bvh_samen = berekenSamengesteldeCategorie(data.bvh_res, data.bvh_cap, data.temp);
  if (wateringDuration == 0) {
    data.pomp_sec = wateringDuration; // Avoid division by zero
  } else {
    data.pomp_sec = wateringDuration / 1000;  // milliseconds to seconds
  }
  data.latitude = gps.location.lat();
  data.longitude = gps.location.lng();
  return data;
}

String httpPayload() {
  SensorData sData = compileSensorData();
  // Construct the URL query with sensor data
  String urlFinal = GOOGLE_APPS_SCRIPT_URL + GOOGLE_SCRIPT_DEPLOYMENT_ID + "/exec?" +
                    "date=" + sData.date +
                    "&temp=" + sData.temp +
                    "&bvh_cap=" + sData.bvh_cap +
                    "&bvh_res=" + sData.bvh_res +
                    "&bvh_samen=" + sData.bvh_samen +
                    "&sec=" + sData.pomp_sec +
                    "&long=" + sData.longitude +
                    "&lat=" + sData.latitude;
  return urlFinal;
}

void gotoSleep() {
  TRACE();  // entering deep sleep routine
  Serial.println("Going to sleep now for " + String(WAKEUP_SECONDS) + " seconds");
  Serial.println("turning off the sensors");
  digitalWrite(SENSOR_POWER, HIGH);  // cut power to sensors
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  TRACE();  // mark setup start
  pinMode(SENSOR_POWER, OUTPUT);
  digitalWrite(SENSOR_POWER, LOW);  // turn on sensors power
  Serial.println("turning on the sensors");

  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  initWifiAndGps();
  setTime();

  // Initialize I/O pins
  pinMode(OVERRIDE_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BVHR_PIN, INPUT);
  pinMode(BVHC_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, OFF);
  sensors.begin();

  // Configure wake-up sources for deep sleep
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);           // External wakeup on button (GPIO 25)
  esp_sleep_enable_timer_wakeup(WAKEUP_TIMER);            // Timer wakeup after WAKEUP_TIMER microseconds

  // Print wakeup reason or go to sleep on first boot
  if (bootCount == 0) {
    bootCount++;
    Serial.print("First boot - ");
    gotoSleep();
  } else {
    print_wakeup_reason();
  }
}

bool runOnce = false;
bool payloadSent = false;

void loop() {
  unsigned long huidigeMillis = millis();

  // Check if pump should be turned off (duration elapsed)
  if (pumpState == ON && (huidigeMillis - startWateringTime >= wateringDuration)) {
    TRACE();  // pump-off condition met
    Serial.println("Water geven is afgelopen, pomp uit");
    zetWaterpompUit();
  }

  // Check if panic button (override) was pressed to start pump
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 && runOnce == false) {
    TRACE();  // panic button branch
    runOnce = true;
    Serial.println("Panic button pressed");
    zetWaterpompAan(PUMP_TIMER_PANIC);
    Serial.println("Water geven met paniek duur");
  }

  // Check if timer wake-up to read sensors and possibly water
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER && runOnce == false) {
    TRACE();  // scheduled sensor-read branch
    runOnce = true;
    leesSensorenEnGeefWaterIndienNodig();
    lastReadTime = huidigeMillis;
  }

  // If WiFi is connected and pump is not running, send data (once)
  if (WiFi.status() == WL_CONNECTED && payloadSent == false && pumpState == OFF) {
    TRACE();  // entering data-send branch
    String urlFinal = httpPayload();
    Serial.print("POST data to spreadsheet: ");
    Serial.println(urlFinal);
    HTTPClient http;
    http.begin(urlFinal.c_str());
    int httpCode = http.GET();
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    // Get response (if any) and report result
    String payload;
    if (httpCode == 200) {
      Serial.println("Data sent successfully!");
      payload = http.getString();
      Serial.println("Payload: " + payload);
      payloadSent = true;
    } else {
      Serial.println("Failed to send data.");
      payload = http.getString();
      Serial.println("Payload: " + payload);
    }
    Serial.flush();
    http.end();
  }

  // If data is sent and pump is off, put ESP32 into deep sleep
  if (pumpState == OFF && payloadSent == true) {
    gotoSleep();
  }
}
