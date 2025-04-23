#include "Arduino.h"

// DONE: Wachttijd tussen 2 opeenvolgende inlezingen van sensoren
const int uS_TO_S_FACTOR = 1000000;
const int WAKEUP_SECONDS = 10;
const int WAKEUP_TIMER = WAKEUP_SECONDS * uS_TO_S_FACTOR;

// DONE: Temperatuur schakelwaarden
const byte TEMP_LOW = 5;
const byte TEMP_HIGH = 25;
// DONE: Categoriën vochtigheid
enum Categorie {
  DROOG,
  VOCHTIG,
  NAT,
  ONBEKEND  // Handig als foutwaarde
};

// DONE: Statussen water geven (geen water, wél water)
enum PompStatus {
  OFF = LOW,
  ON = HIGH
};

// DONE: Duurtijden water geven (in milliseconden)
const int PUMP_TIMER_SHORT = 1000;
const int PUMP_TIMER_LONG = 2000;
const int PUMP_TIMER_PANIC = 3000;
/*
  Opgelet, we definiëren de intervallen als gesloten: [min, max]
*/

/*
    RESISTIEVE_SENSOR
*/

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN = 0;
const int RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX = 1792;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = 2219;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MIN = RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MAX = 2538;

/*
    CAPACITIEVE_SENSOR
*/

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN = 2514;
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX = 9999;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = 1853;
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN - 1;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN = 1441;
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX = CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN - 1;
