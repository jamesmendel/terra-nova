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
#include "display.h"

// Private task definitions
void _InitMainTask(void *pvParam);
void _heartbeatTask();


void setup() {
  esp_log_level_set("*", ESP_LOG_DEBUG);  // or ESP_LOGV for verbose
  
  pinMode(PIN_DISPLAY_PWM_BL, OUTPUT);
  analogWrite(PIN_DISPLAY_PWM_BL, displayBrightness);  // 0 at fisrt init
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  printf("========= TERRA =========\n");
  printf("from: %s\n", GIT_CURRENT_BRANCH);
  printf("sha:  %s\nat:   %s %s\n", GIT_COMMIT_ID, BUILD_DATE, BUILD_TIME);
  printf("log filter: %d\n", CORE_DEBUG_LEVEL);
  printf("=========================\n");
  
  Wire.begin(SDA, SCL);

  // Power management
  initPower();
  
  // Haptic driver
  initHaptics();
  
  // Screen
  displayInit();
  
  xTaskCreatePinnedToCore(_InitMainTask, "CheckBatteryTask", 4096, (void*)batteryCheckVolatgeTask, 1, NULL, 0);
  // xTaskCreatePinnedToCore(_InitMainTask, "Heartbeat", 4096, (void*)_heartbeatTask, 1, NULL, 0);
  xTaskCreatePinnedToCore(_InitMainTask, "HapticsTask", 4096, (void*)hapticsPlayTask, 10, NULL, 1);
  
  changeHaptics(true);
}

// Main Loop
void loop() {
  displayUpdate();
  powerCheckButton1();
  powerCheckButton2();
}

void _InitMainTask(void *pvParam) {
  void (*initFn)() = (void (*)())pvParam;
  initFn();
  vTaskDelete(NULL);
}

void _heartbeatTask() {
  pinMode(LED_BUILTIN, OUTPUT);
  bool beat = true;
  while(1) {
    digitalWrite(LED_BUILTIN, beat);
    beat = !beat;
    // GPIO.out1.val ^= (1 << (LED_BUILTIN - 32));
    terraSleep(1000);
  }
}