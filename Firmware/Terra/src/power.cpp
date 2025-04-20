// Power management

#include <Arduino.h>
#include "terra.h"
#include "power.h"
#include "haptics.h"

volatile unsigned long buttonPressStart = 0;
volatile bool buttonPressed = false;

hw_timer_t *battCheckTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Respond to power button
void IRAM_ATTR powerButtonISR() {
    bool currentState = digitalRead(PIN_PWR_SW);

    if(currentState) {
        // Pressed
        buttonPressStart = millis();
        buttonPressed = true;
        DEBUG_POWER_PRINT("Button pressed at %d ms", buttonPressStart);
    }
    else {
        // Released
        unsigned long buttonPressDuration = millis() - buttonPressStart;    // duration in ms
        
        #ifdef DEBUG_POWER
        if(buttonPressDuration > 50) {  // debounce button press for logging
            DEBUG_POWER_PRINT("Button released after %d ms", buttonPressDuration);
        }
        #endif  // DEBUG_POWER
        
        if(buttonPressDuration >= POWER_OFF_THRESHOLD_MS) {
            powerDownNow();
        }

        buttonPressed = false;
    }
}

// Check battery
void IRAM_ATTR battCheckISR() {
    // atomic lock in case main needs any variables from this file
    portENTER_CRITICAL_ISR(&timerMux);
    
    DEBUG_POWER_PRINT("Check battery now!\n");
    checkBatteryVolatge();
    
    portEXIT_CRITICAL_ISR(&timerMux);
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

uint16_t readBatteryVolatge() {
    uint32_t millivolts = 0;

    for(uint8_t i = 0; i < BATT_VOLTAGE_SAMPLES; i++) {
        millivolts += analogReadMilliVolts(PIN_BAT_VOLTAGE) * BATT_SCALE_FACTOR;
    }
    millivolts /= BATT_VOLTAGE_SAMPLES;

    return (uint16_t)millivolts;
}

void checkBatteryVolatge() {
    uint16_t millivolts = readBatteryVolatge();

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