/*
  Opgelet, we definiëren de intervallen als gesloten: [min, max]
*/

/*
    RESISTIEVE_SENSOR
*/

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN = 0;
const int RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX = 1535;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = 1800;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MIN = RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MAX = 2581;

/*
    CAPACITIEVE_SENSOR
*/

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN = 2881;
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX = 3003;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = 2467;
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN - 1;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN = 1666;
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX = CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN - 1;