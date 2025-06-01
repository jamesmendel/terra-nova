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

// Private task definitions
void _InitPeripheralTask(void *pvParam);


void setup() {
  esp_log_level_set("*", ESP_LOG_DEBUG);  // or ESP_LOGV for verbose
  
  pinMode(PIN_DISPLAY_PWM_BL, OUTPUT);
  analogWrite(PIN_DISPLAY_PWM_BL, displayBrightness);  // 0 at fisrt init

  Serial.begin(115200);
  printf("========= TERRA =========\n");
  printf("log filter: %d color: %s\n", CORE_DEBUG_LEVEL, LOG_COLOR_BLACK);
  printf("sha:  %s\nat:   %s %s\n", GIT_COMMIT_ID, BUILD_DATE, BUILD_TIME);
  printf("=========================\n");
  
  Wire.begin(SDA, SCL);

  // Power management
  initPower();

  // Haptic driver
  initHaptics();

  // Screen
  displayInit();

  // GPS
  initNav();
  
  // Compass
  // initCompass();
  xTaskCreatePinnedToCore(_InitPeripheralTask, "CompassInit", 4096, (void*)initCompass, 1, NULL, 0);
  // initCompassNoMotionDetection(TERRA_IDLE_SHUTDOWN_SEC);

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

void _InitPeripheralTask(void *pvParam) {
  void (*initFn)() = (void (*)())pvParam;
  initFn();
  vTaskDelete(NULL);
}