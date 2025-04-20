#ifndef POWER_H
#define POWER_H

#define POWER_OFF_THRESHOLD_MS  2000    // button hold time to trigger power-off sequence

#define BATT_CONNECTED_MV       1000    // no battery threshold
#define BATT_MIN_VOLTS_MV       3400    // battery low turn off threshold
#define BATT_MAX_VOLTS_MV       4500    // battery high turn off threshold
#define BATT_VOLTAGE_SAMPLES    10      // number of battery votlage samples
#define BATT_SCALE_FACTOR       2       // inverse of voltage divider ratio
#define BATT_CHECK_INTERVAL_US  1000000 // 1 second


#define BATT_MIN_VOLTS_MV       3400
#define BATT_MAX_VOLTS_MV       4500
#define BATT_VOLTAGE_SAMPLES    10
#define BATT_SCALE_FACTOR       2
#define BATT_CHECK_INTERVAL_US  1000000   // 1 second

// #define DEBUG_POWER
#ifdef DEBUG_POWER
#define DEBUG_POWER_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_POWER_PRINT(...) do {} while (0)
#endif // DEBUG_POWER


void initPower();
void powerCheckButton();
void powerDownNow();

uint16_t batteryReadVolatge();
void batteryCheckVolatge();

#endif // POWER_H