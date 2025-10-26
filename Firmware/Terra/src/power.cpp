// Power management

#include <Arduino.h>
#include "power.h"
#include "haptics.h"
#include "display.h"

#include "esp_log.h"
static const char* LOGTAG = "Power";

// Private members
void IRAM_ATTR powerButtonISR();
void IRAM_ATTR battCheckISR();


unsigned long buttonPressStart = 0;
volatile bool buttonPressedFlag  = false;
volatile bool buttonReleasedFlag = false;

hw_timer_t *battCheckTimer = NULL;
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
    // attachInterrupt(digitalPinToInterrupt(PIN_PWR_SW), powerButtonISR, CHANGE);
    
    // Power off signal
    pinMode(PIN_PWROFF, INPUT); // configure as an input so the pin is high impedance (allow SW4 to function)
    
    // Battery monitor adc
    pinMode(PIN_BAT_VOLTAGE, INPUT);
    adcAttachPin(PIN_BAT_VOLTAGE);
    analogReadResolution(12u);
    analogSetPinAttenuation(PIN_BAT_VOLTAGE, ADC_11db);
    
    // Charger status
    pinMode(PIN_BAT_CHARGEn, INPUT);
    pinMode(PIN_USB_CONN, INPUT);
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
        delay(2);
    }
    millivolts /= BATT_VOLTAGE_SAMPLES;

    return (uint16_t)millivolts;
}

void batteryCheckVolatgeTask() {
    while(1) {      
        batteryLastMillivolts = batteryReadVolatge();

        if(batteryLastMillivolts <= BATT_MIN_VOLTS_MV || batteryLastMillivolts >= BATT_MAX_VOLTS_MV) {
            ESP_LOGE(LOGTAG, "Battery in unsafe condition: %d mV", batteryLastMillivolts);
            powerDownNow();
        } 

        ESP_LOGI(LOGTAG, "Battery voltage: %d", batteryLastMillivolts);

        char dbg3[16];
        snprintf(dbg3, 16, "BAT: %04d", batteryLastMillivolts);
        tftDrawDebugOverlay(dbg3, 3, 1);

        vTaskDelay(BATT_CHECK_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

void powerCheckButton() {
    static volatile bool buttonPressed = false;
    buttonPressed = digitalRead(PIN_PWR_SW);

    if (!buttonPressed) {
        if(buttonPressedFlag == true) {
            // Button was previously pressed and now released
            buttonPressedFlag = false;
            unsigned long buttonPressDuration = millis() - buttonPressStart;
            DEBUG_POWER_PRINT("Button released after %lu ms\n", buttonPressDuration);
            if (buttonPressDuration >= POWER_OFF_THRESHOLD_MS) {
                powerDownNow();
            }
            return;
        }        
    }
    
    if (buttonPressed && !buttonPressedFlag) {
        buttonPressedFlag = true;
        buttonPressStart = millis();
        DEBUG_POWER_PRINT("Button pressed at %lu ms\n", buttonPressStart);
        return;
    }

    // if (buttonReleasedFlag) {
    // // if (digitalRead(PIN_PWR_SW)) {
    //     buttonReleasedFlag = false;
    //     unsigned long buttonPressDuration = millis() - buttonPressStart;

    //     #ifdef DEBUG_POWER
    //     // if (buttonPressDuration > 50) {
    //         DEBUG_POWER_PRINT("Button released after %lu ms\n", buttonPressDuration);
    //     // }
    //     #endif // DEBUG_POWER

    //     if (buttonPressDuration >= POWER_OFF_THRESHOLD_MS) {
    //         powerDownNow();
    //     }
    // }
}

bool batteryIsCharging() {
    return !digitalRead(PIN_BAT_CHARGEn);
}

bool usbIsConnected() {
    return digitalRead(PIN_USB_CONN);
}