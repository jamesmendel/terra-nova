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
void _InitMainTask(void *pvParam);
void _heartbeatTask();


void setup() {
  esp_log_level_set("*", ESP_LOG_DEBUG);  // or ESP_LOGV for verbose
  
  pinMode(PIN_DISPLAY_PWM_BL, OUTPUT);
  analogWrite(PIN_DISPLAY_PWM_BL, displayBrightness);  // 0 at fisrt init

  Serial.begin(115200);
  printf("========= TERRA =========\n");
  printf("sha:  %s\nat:   %s %s\n", GIT_COMMIT_ID, BUILD_DATE, BUILD_TIME);
  printf("log filter: %d\n", CORE_DEBUG_LEVEL);
  printf("=========================\n");
  
  Wire.begin(SDA, SCL);

  // Power management
  initPower();
  xTaskCreatePinnedToCore(_InitMainTask, "CheckBatteryTask", 4096, (void*)batteryCheckVolatgeTask, 1, NULL, 1);

  // Haptic driver
  initHaptics();

  // Screen
  displayInit();

  // GPS
  initNav();
  
  // Compass
  // initCompass();
  xTaskCreatePinnedToCore(_InitMainTask, "CompassInit", 4096, (void*)initCompass, 1, NULL, 0);
  // initCompassNoMotionDetection(TERRA_IDLE_SHUTDOWN_SEC);

  playEffect(HAP_EFFECT_PWRON); // power on sequence finsihed!

  // Start the navigation task
  xTaskCreatePinnedToCore(_InitMainTask, "CompassTask", 4096, (void*)compassUpdateTask, 1, NULL, 1);
  
  xTaskCreatePinnedToCore(_InitMainTask, "NavigationTask", 4096, (void*)navUpdateTask, 1, NULL, 1);
  
  xTaskCreatePinnedToCore(_InitMainTask, "Heartbeat", 4096, (void*)_heartbeatTask, 1, NULL, 1);
}

// Main Loop
void loop() {
  displayUpdate();
  navServiceHaptics();
  powerCheckButton();
  compassServiceInterrupts();
}

void _InitMainTask(void *pvParam) {
  void (*initFn)() = (void (*)())pvParam;
  initFn();
  vTaskDelete(NULL);
}

void _heartbeatTask() {
  bool beat = true;
  while(1) {
    digitalWrite(LED_BUILTIN, beat);
    beat = !beat;
    // GPIO.out1.val ^= (1 << (LED_BUILTIN - 32));
    terraSleep(1000);
  }
}