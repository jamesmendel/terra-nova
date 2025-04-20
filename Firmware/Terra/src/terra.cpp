// Include Libraries
#include <Arduino.h>
#include "terra.h"
#include "power.h"
#include "haptics.h"
#include "compass.h"
#include "navigation.h"
#include "display.h"

#include <map>

// Setup Function
void setup() {
  Serial.begin(115200);

  initPower();
  initHaptics();

  // Initialize Screen
  displayInit();

  // Initialize GPS
  initNav();

  // Initialize compass
  initCompass();
  initCompassNoMotionDetection();

  Wire.begin();

  playEffect(HAP_EFFECT_PWRON); // power on sequence finsihed!
}

// Main Loop
void loop() {
  navUpdate();
  
  batteryCheckVolatge();
  powerCheckButton();
  compassServiceInterrupts();
}