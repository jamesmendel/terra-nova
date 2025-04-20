// Power management

#include <Arduino.h>
#include "terra.h"
#include "power.h"
#include "haptics.h"

// Private members
void IRAM_ATTR powerButtonISR();
void IRAM_ATTR battCheckISR();


unsigned long buttonPressStart = 0;
volatile bool buttonPressedFlag  = false;
volatile bool buttonReleasedFlag = false;

hw_timer_t *battCheckTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool batteryPendingCheck = false;


// Respond to power button
void IRAM_ATTR powerButtonISR() {
    if(digitalRead(PIN_PWR_SW)) {
        buttonPressedFlag = true;
    }
    else {
        buttonReleasedFlag = true;
    }
}

// Check battery
void IRAM_ATTR battCheckISR() {
    batteryPendingCheck = true;
}

void initPower() {
    // Power switch
    pinMode(PIN_PWR_SW, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PWR_SW), powerButtonISR, CHANGE);
    
    // Power off signal
    pinMode(PIN_PWROFF, INPUT); // configure as an input so the pin is high impedance (allow SW4 to function)
    
    // Battery monitor adc
    pinMode(PIN_BAT_VOLTAGE, INPUT);
    adcAttachPin(PIN_BAT_VOLTAGE);
    analogReadResolution(12u);
    analogSetPinAttenuation(PIN_BAT_VOLTAGE, ADC_11db);

    // Battery monitor timer
    battCheckTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(battCheckTimer, &battCheckISR, true);
    timerAlarmWrite(battCheckTimer, BATT_CHECK_INTERVAL_US, true);
    timerAlarmEnable(battCheckTimer); 
}

void powerDownNow() {
    Serial.println("Powering down now!");

    playEffect(HAP_EFFECT_PWRDOWN);
    while(isEffectPlaying()) usleep(1000);

    pinMode(PIN_PWROFF, OUTPUT);

    while(1) {
        digitalWrite(PIN_PWROFF, HIGH);
    }
}

uint16_t batteryReadVolatge() {
    uint32_t millivolts = 0;

    for(uint8_t i = 0; i < BATT_VOLTAGE_SAMPLES; i++) {
        millivolts += analogReadMilliVolts(PIN_BAT_VOLTAGE) * BATT_SCALE_FACTOR;
    }
    millivolts /= BATT_VOLTAGE_SAMPLES;

    return (uint16_t)millivolts;
}

void batteryCheckVolatge() {
    if(batteryPendingCheck) {
        uint16_t millivolts = batteryReadVolatge();

        if(millivolts <= BATT_CONNECTED_MV) {
            // Battery is disconnected, running on USB only
            return;
        }

        if(millivolts <= BATT_MIN_VOLTS_MV || millivolts >= BATT_MAX_VOLTS_MV) {
            printf("Battery in unsafe condition: %d mV\n", millivolts);
            powerDownNow();
        } 

        DEBUG_POWER_PRINT("Battery voltage: %d\n", millivolts);
    }

    batteryPendingCheck = false;
}

void powerCheckButton() {
    if (buttonPressedFlag) {
        buttonPressedFlag = false;
        buttonPressStart = millis();
        DEBUG_POWER_PRINT("Button pressed at %lu ms\n", buttonPressStart);
    }

    if (buttonReleasedFlag) {
        buttonReleasedFlag = false;
        unsigned long buttonPressDuration = millis() - buttonPressStart;

        #ifdef DEBUG_POWER
        if (buttonPressDuration > 50) {
            DEBUG_POWER_PRINT("Button released after %lu ms\n", buttonPressDuration);
        }
        #endif // DEBUG_POWER

        if (buttonPressDuration >= POWER_OFF_THRESHOLD_MS) {
            powerDownNow();
        }
    }
}