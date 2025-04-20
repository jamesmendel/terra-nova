#include <Arduino.h>
#include "haptics.h"

// Private prototypes
void _triggerPlayback();

void initHaptics() {
    pinMode(PIN_HAP_IN_TRIG, OUTPUT);
    digitalWrite(PIN_HAP_IN_TRIG, LOW);

    if (!drv.begin()) {
        Serial.println("Could not find DRV2605");
        while (1) delay(10);
    }
    drv.selectLibrary(1);
    // I2C trigger by sending 'go' command
    drv.setMode(DRV2605_MODE_EXTTRIGEDGE);
}

void playEffect(uint8_t effect) {
    if(effect > HAP_MAX_EFFECT_NUM) return;
    
    drv.setWaveform(0, effect);
    drv.setWaveform(1, HAP_EFFECT_STOP);

    _triggerPlayback();
}

void playEffectSequence(uint8_t *effects, uint8_t len) {
    if(len == 0 || len > HAP_MAX_SEQUENCE_NUM) return;
    
    for(uint8_t i = 0; i < len; i++) {
        drv.setWaveform(i, effects[i]);
    }

    if(len < HAP_MAX_SEQUENCE_NUM) {
        drv.setWaveform(len-1, HAP_EFFECT_STOP);
    }

    _triggerPlayback();
}

bool isEffectPlaying() {
    return (drv.readRegister8(DRV2605_REG_GO) & 0x01);
}

void _triggerPlayback() {
    // 7.3.5.6.2: The pulse width should be at least 1 μs to ensure detection.
    digitalWrite(PIN_HAP_IN_TRIG, HIGH);
    usleep(2);
    digitalWrite(PIN_HAP_IN_TRIG, LOW);
}