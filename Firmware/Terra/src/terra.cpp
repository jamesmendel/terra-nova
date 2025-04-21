/**
 * @file terra.cpp
 * @brief Entrypoint for Terra
 * @date 2025-04-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Arduino.h>
#include "terra.h"
#include "power.h"
#include "haptics.h"
#include "compass.h"
#include "navigation.h"
#include "display.h"


void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Power management
  initPower();

  // Haptic driver
  initHaptics();

  // Screen
  displayInit();

  // GPS
  initNav();

  // Compass
  initCompass();
  initCompassNoMotionDetection(TERRA_IDLE_SHUTDOWN_SEC);

  playEffect(HAP_EFFECT_PWRON); // power on sequence finsihed!
}

// Main Loop
void loop() {
  navUpdate();
  displayUpdate();
  
  batteryCheckVolatge();
  powerCheckButton();
  compassServiceInterrupts();
}