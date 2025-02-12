#include <Arduino.h>
//libs for temp sens 
#include <OneWire.h>
#include <DallasTemperature.h>
//lib for servo
#include <ESP32Servo.h>
//lib for built in rgb led
#include <FastLED.h>

#define SEN0114_PIN A4
#define SEN0193_PIN A3

#define TEMPSENS_PIN D3
OneWire oneWire(TEMPSENS_PIN);
DallasTemperature sensors(&oneWire);

#define SERVO_PIN D5
Servo myservo;
const int ServoOpenVal = 300;
const int ServoCloseVal = 10;
void OpenServo(){
  myservo.write(ServoOpenVal);
  
}
void CloseServo(){
  myservo.write(ServoCloseVal);
}

#define NUM_LEDS 1     //Number of RGB LED beads
#define DATA_PIN D8    //The pin for controlling RGB LED
#define LED_TYPE NEOPIXEL    //RGB LED strip type
CRGB leds[NUM_LEDS];    //Instantiate RGB LED


int calculateMoisturePercentage(int sensorValue, int sensorMinValue, int sensorMaxValue) {
    return map(sensorValue, sensorMinValue, sensorMaxValue, 0, 100);
}

void outputSerialSoilValues(String soilSensorType, int rawValue, int percentage) {
  Serial.print(soilSensorType + "soil sensor value: ");
  Serial.print(rawValue);
  Serial.print(" - Soil moisture percentage: ");
  Serial.print(percentage);
  Serial.println("%");
}

void setup() {
  Serial.begin(9600);
  sensors.begin();
  myservo.attach(SERVO_PIN);
   FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, NUM_LEDS); 
   
}  

void loop() {
  int resSensorValue = analogRead(SEN0114_PIN);
  int resMoisturePercentage = calculateMoisturePercentage(resSensorValue, RESISTANCE_MIN, RESISTANCE_MAX); 

  int capSensorValue = analogRead(SEN0193_PIN);
  int capMoisturePercentage = calculateMoisturePercentage(capSensorValue, CAPACITANCE_MIN, CAPACITANCE_MAX);
  
  sensors.requestTemperatures();
  int temperaturVal = sensors.getTempCByIndex(0);

  outputSerialSoilValues("Resistance type ", resSensorValue, resMoisturePercentage);
  outputSerialSoilValues("Capacitance type ", capSensorValue, capMoisturePercentage);
  Serial.print("Temperature: ");
  Serial.print(temperaturVal);
  Serial.println("--------");
  if(resMoisturePercentage < 50 || capMoisturePercentage < 50){
    OpenServo;
    leds[0] = CRGB::Red;
  } else {
    CloseServo;
    leds[0] = CRGB::Green;
  }
  FastLED.show();
  delay(500);
}