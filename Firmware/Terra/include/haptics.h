#ifndef HAPTICS_H
#define HAPTICS_H

#include <Adafruit_DRV2605.h>

#define HAP_EFFECT_STOP     0   // end playback sequence
#define HAP_EFFECT_PROX     58  // effect to notify user of proximity to checkpoint
#define HAP_EFFECT_PWRON    86  // notify user of power on
#define HAP_EFFECT_PWRDOWN  96  // notify user of power down

#define HAP_MAX_EFFECT_NUM      123
#define HAP_MAX_SEQUENCE_NUM    8


static Adafruit_DRV2605 drv;   // global haptics object

// Public prototypes
void initHaptics();
void playEffect(uint8_t effect);
void playEffectSequence(uint8_t *effect, uint8_t len);
bool isEffectPlaying();

#endif // HAPTICS_H