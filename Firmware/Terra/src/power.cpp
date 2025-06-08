// Power management

#include <Arduino.h>
#include "power.h"
#include "haptics.h"
#include "display.h"
#include "terra.h"

#include "esp_log.h"
static const char* LOGTAG = "Power";

// Private members
void IRAM_ATTR powerButtonISR();
void IRAM_ATTR battCheckISR();


unsigned long button1PressStart = 0;
volatile bool button1PressedFlag  = false;
volatile bool button1ReleasedFlag = false;

unsigned long button2PressStart = 0;
volatile bool button2PressedFlag  = false;
volatile bool button2ReleasedFlag = false;

hw_timer_t *battCheckTimer = NULL;
static volatile bool batteryPendingCheck = false;


// Check battery
void IRAM_ATTR battCheckISR() {
    batteryPendingCheck = true;
}

void initPower() {
    // Power switch
    pinMode(PIN_PWR_SW, INPUT);
    // attachInterrupt(digitalPinToInterrupt(PIN_PWR_SW), powerButtonISR, CHANGE);
    
    // Power off signal
    pinMode(PIN_PWROFF, INPUT); // configure as an input so the pin is high impedance (allow SW4 to function)
    pinMode(0, INPUT_PULLUP);  // boot0 as input

    // Battery monitor adc
    pinMode(PIN_BAT_VOLTAGE, INPUT);
    adcAttachPin(PIN_BAT_VOLTAGE);
    analogReadResolution(12u);
    analogSetPinAttenuation(PIN_BAT_VOLTAGE, ADC_11db);
}

void powerDownNow() {
    printf("Powering down now!\n");

    playEffect(HAP_EFFECT_PWRDOWN);
    while(isEffectPlaying()) usleep(1000);
    
    displaySetImage(I_NONE);
    while(displayGetBrightness() >= DISPLAY_BRIGHTNESS_OFF) {
        displayUpdate();
        if(displayGetBrightness() <= DISPLAY_BRIGHTNESS_OFF)
            break;
    }
    
    pinMode(PIN_PWROFF, OUTPUT);
    while(1) {
        digitalWrite(PIN_PWROFF, HIGH);
    }
}

uint16_t batteryReadVolatge() {
    uint32_t millivolts = 0;

    for(uint8_t i = 0; i < BATT_VOLTAGE_SAMPLES; i++) {
        millivolts += analogReadMilliVolts(PIN_BAT_VOLTAGE) * BATT_SCALE_FACTOR;
        terraSleep(5);
    }
    millivolts /= BATT_VOLTAGE_SAMPLES;

    return (uint16_t)millivolts;
}

void batteryCheckVolatgeTask() {
    while(1) {      
        batteryLastMillivolts = batteryReadVolatge();

        if(batteryLastMillivolts <= BATT_CONNECTED_MV) {
            // Battery is disconnected, running on USB only
            return;
        }

        if(batteryLastMillivolts <= BATT_MIN_VOLTS_MV || batteryLastMillivolts >= BATT_MAX_VOLTS_MV) {
            ESP_LOGW(LOGTAG, "Battery in unsafe condition: %d mV", batteryLastMillivolts);
            powerDownNow();
        } 

        ESP_LOGD(LOGTAG, "Battery voltage: %d", batteryLastMillivolts);

        char dbg3[16];
        snprintf(dbg3, 16, "BAT: %04d", batteryLastMillivolts);
        tftDrawDebugOverlay(dbg3, 3, 1);

        vTaskDelay(BATT_CHECK_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

void powerCheckButton1() {
    static volatile bool button1Pressed = false;
    button1Pressed = digitalRead(PIN_PWR_SW);

    if (!button1Pressed) {
        if(button1PressedFlag == true) {
            // Button was previously pressed and now released
            button1PressedFlag = false;
            unsigned long button1PressDuration = millis() - button1PressStart;
            DEBUG_POWER_PRINT("Button 1 released after %lu ms\n", button1PressDuration);
            if (button1PressDuration >= POWER_OFF_THRESHOLD_MS) {
                changeHaptics(true);
            }
            return;
        }        
    }
    
    if (button1Pressed && !button1PressedFlag) {
        button1PressedFlag = true;
        button1PressStart = millis();
        DEBUG_POWER_PRINT("Button 1 pressed at %lu ms\n", button1PressStart);
        return;
    }
}

void powerCheckButton2() {
    static volatile bool button2Pressed = false;
    button2Pressed = !digitalRead(0);

    if (!button2Pressed) {
        if(button2PressedFlag == true) {
            // Button was previously pressed and now released
            button2PressedFlag = false;
            unsigned long button2PressDuration = millis() - button2PressStart;
            DEBUG_POWER_PRINT("Button 2 released after %lu ms\n", button2PressDuration);
            if (button2PressDuration >= POWER_OFF_THRESHOLD_MS) {
                changeHaptics(false);
            }
            return;
        }        
    }
    
    if (button2Pressed && !button2PressedFlag) {
        button2PressedFlag = true;
        button2PressStart = millis();
        DEBUG_POWER_PRINT("Button 2 pressed at %lu ms\n", button2PressStart);
        return;
    }
}