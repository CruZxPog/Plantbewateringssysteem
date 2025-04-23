#include "config.h"
#include "security.h"

#include "Arduino.h"

#include "OneWire.h"
#include "DallasTemperature.h"

#include "ArduinoTrace.h"

#include "WiFi.h"
#include "HTTPClient.h"
#include "time.h"

#include "HardwareSerial.h"
#include "TinyGPS++.h"

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// NTP = Network Time Protocol
const char *NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 0; // 19800;
const int DAYLIGHT_OFFSET_SEC = 0;

const String GOOGLE_APPS_SCRIPT_URL = "https://script.google.com/macros/s/";

#define ARDUINOTRACE_ENABLE 1

RTC_DATA_ATTR int bootCount = 0;

// DONE: Definieer juiste pinnummers voor sensoren
#define OVERRIDE_BUTTON_PIN 25
#define BVHR_PIN A3
#define BVHC_PIN A1
#define TEMPSENS_PIN D7
#define PUMP_PIN D12
#define RX_PIN 16
#define TX_PIN 17
#define SENSOR_POWER  D6  // GPIO pin controlling 2N3906

OneWire oneWire(TEMPSENS_PIN);
DallasTemperature sensors(&oneWire);

// DONE: Variabelen om wachttijd tussen inlezen sensoren te kunnen regelen
unsigned long lastReadTime = 0;

// DONE: Variabelen om duurtijd van water geven te kunnen regelen
unsigned long startWateringTime = 0;
int wateringDuration = 0;

// DONE: Variabele om status van de waterpomp aan te geven, dit is nodig om te kunnen controlleren of de waterpomp gestopt moet worden
byte pumpState = OFF;

//functions for deep sleep
/*
Method to print the reason by which ESP32
has been awaken from sleep
code from: https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
*/
esp_sleep_wakeup_cause_t wakeup_reason;
void print_wakeup_reason(){

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}
//functions for deep sleep

//functions for sensor reading
/**
 * Bepaal de temperatuur, op basis van de gekozen temperatuursensor.
 * Voor een digitale sensor zal dit anders zijn dan voor een analoge.
 * Geeft de temperatuur in °C terug.
 */
int leesTemperatuur() {
  // DONE: Implementeer zodat de temperatuur op de juiste manier wordt ingelezen
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}
/**
 * Bepaal de categorie van de capacitieve bodemCategoriessensor voor de gemeten sensorwaarde.
 * We gebruiken hierbij de configuratie uit onze calibratie.  Per categorie checken we of de waarde
 * tussen de MIN en de MAX valt.
 * Opgelet!!  Gebruik enkel de categoriën uit je configuratiebestand!
 */
Categorie berekenCategorieCapactieveBVH(int sensorwaarde) {
  // DUMP(sensorwaarde);
  // DONE: Implementeer zodat de categorie voor de capacitieve BVH sensor wordt berekend.
  if (sensorwaarde >= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX) {
      return DROOG;
  } else if (sensorwaarde >= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
      return VOCHTIG;
  } else if (sensorwaarde >= CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN && sensorwaarde <= CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX) {
      return NAT;
  } else {
      return ONBEKEND;
  }
}
/**
 * Bepaal de categorie van de resistieve bodemCategoriessensor voor de gemeten sensorwaarde.
 * We gebruiken hierbij de configuratie uit onze calibratie.  Per categorie checken we of de waarde
 * tussen de MIN en de MAX valt.
 * Opgelet!!  Gebruik enkel de categoriën uit je configuratiebestand!
 */
Categorie berekenCategorieResistieveBVH(int sensorwaarde) {
  // DUMP(sensorwaarde);
  if (sensorwaarde >= RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX) {
      return DROOG;
  } else if (sensorwaarde >= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
      return VOCHTIG;
  } else if (sensorwaarde >= RESISTIEVE_SENSOR_NAT_INTERVAL_MIN && sensorwaarde <= RESISTIEVE_SENSOR_NAT_INTERVAL_MAX) {
      return NAT;
  } else {
      return ONBEKEND;
  }
}
/**
 * Bereken de samengestelde categorie voor beide bodemCategoriessensoren.
 * Mogelijke strategiën: droogste wint altijd / één wint altijd / geen mogelijke categorie bij verschil
 * Opgelet!!  Gebruik enkel de categoriën uit je configuratiebestand!
 */
Categorie berekenSamengesteldeCategorie(Categorie  categorieResistieveBVH, Categorie  categorieCapacitieveBVH, int temperatuur) {
  if(temperatuur > TEMP_HIGH){
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
/**
 * Zet de waterpomp aan voor een bepaalde tijd.   
 * Opgelet!!  Deze functie mag GEEN DELAY bevatten.  De duurtijd zal dus via een variabele moeten bijgehouden worden.
 *            Het hoofdprogramma moet telkens controlleren of de duurtijd reeds verstreken is, via millis().
 *            Gebruik een status om aan te geven dat de waterpomp aan het water geven is.
*/
void zetWaterpompAan(int duurtijd) {
  TRACE();
  // DONE: Implementeer code om de pomp aan te zetten
  pumpState = ON;
  digitalWrite(PUMP_PIN, ON);
  // DONE: Initialiseer de variabelen om de starttijd en duurtijd van het water geven te regelen
  startWateringTime = millis();
  wateringDuration = duurtijd;
  DUMP(startWateringTime);
}
/**
 * Zet de waterpomp uit. 
 * Opgelet!! Aangezien de zetWaterpompAan() functie geen delay bevat, zullen de variabelen die daar gebruikt worden
 *           opnieuw geïnitialiseerd moeten worden bij het uitzetten van de pomp.
 */
void zetWaterpompUit() {
  TRACE();
  // DONE: Implementeer code om de pomp uit te zetten
  pumpState = OFF;
  digitalWrite(PUMP_PIN, OFF);
  // DONE: Initialiseer de variabelen om de starrtijd en duurtijd van het water geven te regelen
  startWateringTime = 0;
  wateringDuration = 0;

}
/**
 * Deze functie bevat alle code voor het uitlezen van de sensoren en om de waterpomp indien nodig aan te zetten.
 * Het uitzetten van de waterpomp gebeurt niet hier maar in de loop() functie na controle of er voldoende tijd verstreken is.
 */
void leesSensorenEnGeefWaterIndienNodig() {
  // DONE: Implementeer inlezen met correcte pinnen
  int capacitieve_bvh_waarde = analogRead(BVHC_PIN);
  DUMP(capacitieve_bvh_waarde);
  int resistieve_bvh_waarde = analogRead(BVHR_PIN);
  DUMP(resistieve_bvh_waarde);
  int temperatuur = leesTemperatuur();
  DUMP(temperatuur);
  // Bepaal individuele categoriën en samengestelde categorie
  Categorie categorieCapacitieveBVH = berekenCategorieCapactieveBVH(capacitieve_bvh_waarde);
  DUMP(categorieCapacitieveBVH);
  Categorie categorieResistieveBVH = berekenCategorieResistieveBVH(resistieve_bvh_waarde);
  DUMP(categorieResistieveBVH);
  Categorie categorie = berekenSamengesteldeCategorie(categorieCapacitieveBVH, categorieResistieveBVH, temperatuur);
  DUMP(categorie);
  if(categorie==DROOG){
    Serial.println("categorie = DROOG");
  } else if(categorie==VOCHTIG){
    Serial.println("categorie = VOCHTIG");
  } else if(categorie==NAT){
    Serial.println("categorie = NAT");
  } else {
    Serial.println("categorie = ONBEKEND");
  }

  // DONE: Beslis over water geven en pas de controles toe uit de flowchart.  
  // !! Gebruik enkel de constanten uit de configuratie om met een categorie te vergelijken!
  // !! Gebruik enkel de constanten uit de configuratie om de duurtijd van het water geven mee te geven
  // !! Gebruik verder enkel de functies zetWaterpompAan() en zetWaterpompUit() om de waterpomp aan/uit te zetten
  if (temperatuur > TEMP_HIGH && categorie == DROOG){
    zetWaterpompAan(PUMP_TIMER_LONG);
    Serial.println("Water geven met lange duur");
  } else if (temperatuur >= TEMP_LOW && temperatuur <= TEMP_HIGH && categorie == DROOG){
    zetWaterpompAan(PUMP_TIMER_SHORT);
    Serial.println("Water geven met korte duur");
  }
}
//functions for sensor reading

//functions for http payload
void initWifiAndGps() {
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
  // bool gpsState = false;
  // while(gpsState == false){
  //   while (gpsSerial.available() > 0) {
  //     if (gps.encode(gpsSerial.read())) {
  //       if (gps.location.isValid()) {
  //         Serial.print(gps.location.lat(), 6);
  //         Serial.print(F(","));
  //         Serial.print(gps.location.lng(), 6);
  //         gpsState = true;
  //         Serial.println("GPS signal found!");
  //       } else {
  //         Serial.println(F("INVALID"));
  //       }
  //     }
  //   }
  }
//}

void setTime() {
  Serial.println("Synchronizing time with NTP...");
  configTime(3600, 3600, NTP_SERVER);  // UTC+1 wintertijd, +2 zomertijd (wordt automatisch geregeld)
  
  // Wacht op een geldige tijd
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
String getCurrentDateAndTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))   {
    Serial.println("Failed to obtain time");
    return "";
  }

  char timeStringBuff[50]; // 50 chars should be enough
  //strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%dT%H:%M:%S", &timeinfo);


  String asString(timeStringBuff);
  asString.replace(" ", "-");

  return asString;
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
SensorData compileSensorData(){
  SensorData data;
  data.date = getCurrentDateAndTime();
  data.temp = leesTemperatuur();
  data.bvh_cap = berekenCategorieCapactieveBVH(analogRead(BVHC_PIN));
  data.bvh_res = berekenCategorieResistieveBVH(analogRead(BVHR_PIN));
  data.bvh_samen = berekenSamengesteldeCategorie(data.bvh_res, data.bvh_cap, data.temp);
  data.pomp_sec = wateringDuration / 1000; // Convert milliseconds to seconds
  data.latitude = gps.location.lat();
  data.longitude = gps.location.lng();
  return data;
}
String httpPayload() {
  SensorData sData = compileSensorData();
  String urlFinal = GOOGLE_APPS_SCRIPT_URL + GOOGLE_SCRIPT_DEPLOYMENT_ID + "/exec?" + 
    "date=" + sData.date  + 
    "&temp=" + sData.temp +
    "&bvh_cap=" + sData.bvh_cap +
    "&bvh_res=" + sData.bvh_res +
    "&bvh_samen=" + sData.bvh_samen +
    "&sec=" + sData.pomp_sec +
    "&long=" + sData.longitude +
    "&lat=" + sData.latitude;

  return urlFinal;
}

void gotoSleep(){
  Serial.println("Going to sleep now for " + String(WAKEUP_SECONDS) + " seconds");
  Serial.println("turning off the sensors");
  digitalWrite(SENSOR_POWER, HIGH); // Zet de VCC van sensors uit
  esp_deep_sleep_start();
}

void setup() {
  // DONE: Implementeer de nodig code voor lezen sensoren (indien nodig)
  Serial.begin(115200);
  pinMode(SENSOR_POWER, OUTPUT);
  digitalWrite(SENSOR_POWER, LOW); // Zet de VCC van sensors aan
  Serial.println("turning on the sensors");

  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  
  initWifiAndGps();
  setTime();
  
  pinMode(OVERRIDE_BUTTON_PIN, INPUT_PULLDOWN);
  pinMode(BVHR_PIN,INPUT);
  pinMode(BVHC_PIN,INPUT);
  pinMode(PUMP_PIN,OUTPUT);
  digitalWrite(PUMP_PIN,OFF);
  sensors.begin();

  // DONE: Implementeer wake-up sources
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);
  esp_sleep_enable_timer_wakeup(WAKEUP_TIMER);

  //Print the wakeup reason for ESP32
  if (bootCount == 0) {
    bootCount++;
    Serial.print("First boot - ");
    gotoSleep();
  } else {
    print_wakeup_reason();
  }
}
//functions for http payload

// this is used because otherwise the code will go in an infitnite loop
bool runOnce = false;

void loop() {
  Serial.println("waiting");
 delay(2000);
 Serial.println("blabhlbhlkkjb");
  //this only runs once after a reset
  //we do this to prevent the code from running on first start up
  

  // We hebben huidige millis nodig om de verschillende processen te controleren (water geven / stoppen)
  unsigned long huidigeMillis = millis();
  //we place the code for the http payload before all the checks beause we only want to send the data once after the sensors are read
  if (WiFi.status() == WL_CONNECTED && runOnce == false) {
    // Create URL with parameters to call Google Apps Script
    String urlFinal = httpPayload();
    
    Serial.print("POST data to spreadsheet: ");
    Serial.println(urlFinal);

    // Send HTTP request and get status code
    HTTPClient http;
    http.begin(urlFinal.c_str());
    // http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);

    // Get response from HTTP request
    String payload;
    if (httpCode > 0) {
      payload = http.getString();
      Serial.println("Payload: " + payload);
    }
    http.end();
  }
  // DONE: Controleer of de waterpomp uitgezet moet worden en roep functie zetWaterpompUit() aan indien nodig
  if (pumpState == ON && (huidigeMillis - startWateringTime >= wateringDuration)) {
    zetWaterpompUit();    
  }
  // DONE: Controleer of de waterpomp aangezet moet worden en roep functie zetWaterpompAan() aan indien nodig
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 && runOnce == false) {
    runOnce = true;
    Serial.println("Panic button pressed");
    zetWaterpompAan(PUMP_TIMER_PANIC);
    Serial.println("Water geven met paniek duur");
  }
  // DONE: Controleer of sensoren ingelezen moeten worden en roep functie leesSensorenEnGeefWaterIndienNodig() aan indien nodig
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER && runOnce == false) {
    runOnce = true;
    leesSensorenEnGeefWaterIndienNodig();
    lastReadTime = huidigeMillis;
  }

  if(pumpState == OFF){
    //put esp to sleep
    gotoSleep();
  }
}