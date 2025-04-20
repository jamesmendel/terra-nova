#ifndef POWER_H
#define POWER_H

#define POWER_OFF_THRESHOLD_MS  2000

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
#endif


void IRAM_ATTR powerButtonISR();
void ARDUINO_ISR_ATTR battCheckISR();

void initPower();
void powerDownNow();
uint16_t readBatteryVolatge();
void checkBatteryVolatge();

#endif // POWER_H