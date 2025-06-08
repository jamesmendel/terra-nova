#include <Arduino.h>
#include "haptics.h"
#include "display.h"
#include "terra.h"

// Private prototypes
void _triggerPlayback();
static int currentEffect = 0;

/**
 * @brief Initializes the haptic hardware and driver.
 * 
 */
void initHaptics() {
    pinMode(PIN_HAP_IN_TRIG, OUTPUT);
    digitalWrite(PIN_HAP_IN_TRIG, LOW);

    if (!drv.begin()) {
        printf("Could not find DRV2605.\n");
        return;
    }
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_EXTTRIGEDGE);
    printf("Initialized DRV2605!\n");
}

/**
 * @brief Loads one effect and plays it.
 * 
 * @param effect waveform number [0 to 123]
 */
void playEffect(uint8_t effect) {
    if(effect > HAP_MAX_EFFECT_NUM) return;
    
    drv.setWaveform(0, effect);
    drv.setWaveform(1, HAP_EFFECT_STOP);

    _triggerPlayback();
}

/**
 * @brief Loads up to 8 effects and plays them in sequence.
 * 
 * @param effects Array of effect indicies [0 to 123]
 * @param len Length of array (max 8)
 */
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

/**
 * @brief Gets state of haptic driver.
 * 
 * @return true haptics are playing
 * @return false haptics are finished
 */
bool isEffectPlaying() {
    return (drv.readRegister8(DRV2605_REG_GO) & 0x01);
}

/**
 * @brief Triggers haptic playback.
 * 
 */
void _triggerPlayback() {
    // 7.3.5.6.2: The pulse width should be at least 1 μs to ensure detection.
    digitalWrite(PIN_HAP_IN_TRIG, HIGH);
    usleep(2);
    digitalWrite(PIN_HAP_IN_TRIG, LOW);
}

void changeHaptics(bool up) {
    if(up) {
        currentEffect++;
    } else {
        currentEffect--;
    }

    if(currentEffect > HAP_MAX_EFFECT_NUM) {
        currentEffect = 0;
    }
    else if(currentEffect < 0) {
        currentEffect = HAP_MAX_EFFECT_NUM;
    }

    static char effStr[4];
    snprintf(effStr, 4, "%03d", currentEffect);
    tftDrawDebugOverlay(effStr, 0, 4);
}

void hapticsPlayTask() {
    static char effState[5];
    static long startTime = 0;
    while(1) {
        ESP_LOGI("Haptic", "Playing effect: %d...", currentEffect);
        snprintf(effState, 5, "PLAY", currentEffect);
        tftDrawDebugOverlay(effState, -1, 1);
        
        startTime = millis();
        playEffect(currentEffect);
        while(isEffectPlaying() && (millis() - startTime < 2000)) {terraSleep(5);}
        playEffect(HAP_EFFECT_STOP);
        
        snprintf(effState, 5, "WAIT", currentEffect);
        tftDrawDebugOverlay(effState, -1, 1);
        
        ESP_LOGD("Haptic", "Done!");
        
        terraSleep(1000);
    }
}