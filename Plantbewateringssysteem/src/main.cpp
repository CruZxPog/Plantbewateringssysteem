#include "Arduino.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "config.h"

#include "ArduinoTrace.h"
#define ARDUINOTRACE_ENABLE 1

// DONE: Definieer juiste pinnummers voor sensoren
#define OVERRIDE_BUTTON_PIN D4
#define BVHR_PIN A4
#define BVHC_PIN A3
#define TEMPSENS_PIN D7
#define PUMP_PIN D12

OneWire oneWire(TEMPSENS_PIN);
DallasTemperature sensors(&oneWire);

// DONE: Variabelen om wachttijd tussen inlezen sensoren te kunnen regelen
unsigned long lastReadTime = 0;

// DONE: Variabelen om duurtijd van water geven te kunnen regelen
unsigned long startWateringTime = 0;
int wateringDuration = 0;

// DONE: Variabele om status van de waterpomp aan te geven, dit is nodig om te kunnen controlleren of de waterpomp gestopt moet worden
byte pumpState = OFF;

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

void setup() {
  // DONE: Implementeer de nodig code voor lezen sensoren (indien nodig)
  Serial.begin(460800);
  sensors.begin();
  pinMode(OVERRIDE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BVHR_PIN,INPUT);
  pinMode(BVHC_PIN,INPUT);
  pinMode(PUMP_PIN,OUTPUT);
  digitalWrite(PUMP_PIN,OFF);
}

void loop() {
  // We hebben huidige millis nodig om de verschillende processen te controleren (water geven / stoppen)
  unsigned long huidigeMillis = millis();
  
  // DONE: Controleer of de waterpomp uitgezet moet worden en roep functie zetWaterpompUit() aan indien nodig
  if (pumpState == ON && (huidigeMillis - startWateringTime >= wateringDuration)) {
    zetWaterpompUit();
  }
  // DONE: Controleer of de waterpomp aangezet moet worden en roep functie zetWaterpompAan() aan indien nodig
  if (pumpState != ON && digitalRead(OVERRIDE_BUTTON_PIN) == LOW) {
    Serial.println("Panic button pressed");
    zetWaterpompAan(PUMP_TIMER_PANIC);
    Serial.println("Water geven met paniek duur");
  }
  // DONE: Controleer of sensoren ingelezen moeten worden en roep functie leesSensorenEnGeefWaterIndienNodig() aan indien nodig
  if (huidigeMillis - lastReadTime >= READ_TIMER) {
    leesSensorenEnGeefWaterIndienNodig();
    lastReadTime = huidigeMillis;
  }
}