// Include Libraries
#include <Arduino.h>
#include "terra.h"
#include "power.h"
#include "haptics.h"
#include "compass.h"
#include "navigation.h"

#include <TinyGPSPlus.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <map>

// Include Configurations and Images
#include "config.h"     // Assumes this contains GPS waypoints and number of stops
#include "imagelist.h"  // Assumes this includes declarations for images

// TFT Display Settings
// Note: TFT_eSPI configuration is managed in platformio.ini build_flags
TFT_eSPI tft = TFT_eSPI();

ImageType currentDisplayedImage = E_NONE;

// Initialize your navigation state

// Global Variables for GPS Data and State

bool screenOn = false;



// Setup Function
void setup() {
  Serial.begin(115200);

  initPower();
  initHaptics();

  // Initialize Screen
  tft.init();
  tft.setRotation(1);
  pinMode(PIN_DISPLAY_PWM_BL, OUTPUT);
  digitalWrite(PIN_DISPLAY_PWM_BL, LOW);

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

// Display image function - fades images out then in again
void displayImage(ImageType image) {
  if (currentDisplayedImage != image) {
    if (screenOn) {
      fadeOut();
    }
    tft.fillScreen(TFT_BLACK);  // This line is common to all cases
    switch (image) {
      case E_PENDING:
        drawBitmap(pending);
        Serial.println("Drawn pending.h");
        break;
      case E_GOTOSTART:
        drawBitmap(gotostart);
        Serial.println("Drawn gotostart.h");
        break;
      case E_ARROW_N:
        drawBitmap(arrow_N);
        Serial.println("Drawn arrow_N.h");
        break;
      case E_ARROW_NNE:
        drawBitmap(arrow_NNE);
        Serial.println("Drawn arrow_NNE.h");
        break;
      case E_ARROW_NE:
        drawBitmap(arrow_NE);
        Serial.println("Drawn arrow_NE.h");
        break;
      case E_ARROW_ENE:
        drawBitmap(arrow_ENE);
        Serial.println("Drawn arrow_ENE.h");
        break;
      case E_ARROW_E:
        drawBitmap(arrow_E);
        Serial.println("Drawn arrow_E.h");
        break;
      case E_ARROW_ESE:
        drawBitmap(arrow_ESE);
        Serial.println("Drawn arrow_ESE.h");
        break;
      case E_ARROW_SE:
        drawBitmap(arrow_SE);
        Serial.println("Drawn arrow_SE.h");
        break;
      case E_ARROW_SSE:
        drawBitmap(arrow_SSE);
        Serial.println("Drawn arrow_SSE.h");
        break;
      case E_ARROW_S:
        drawBitmap(arrow_S);
        Serial.println("Drawn arrow_S.h");
        break;
      case E_ARROW_SSW:
        drawBitmap(arrow_SSW);
        Serial.println("Drawn arrow_SSW.h");
        break;
      case E_ARROW_SW:
        drawBitmap(arrow_SW);
        Serial.println("Drawn arrow_SW.h");
        break;
      case E_ARROW_WSW:
        drawBitmap(arrow_WSW);
        Serial.println("Drawn arrow_WSW.h");
        break;
      case E_ARROW_W:
        drawBitmap(arrow_W);
        Serial.println("Drawn arrow_W.h");
        break;
      case E_ARROW_WNW:
        drawBitmap(arrow_WNW);
        Serial.println("Drawn arrow_WNW.h");
        break;
      case E_ARROW_NW:
        drawBitmap(arrow_NW);
        Serial.println("Drawn arrow_NW.h");
        break;
      case E_ARROW_NNW:
        drawBitmap(arrow_NNW);
        Serial.println("Drawn arrow_NNW.h");
        break;
      case E_CHECKPOINT_1:
        drawBitmap(checkpoint_1);
        Serial.println("Drawn checkpoint_1.h");
        break;
      case E_CHECKPOINT_2:
        drawBitmap(checkpoint_2);
        Serial.println("Drawn checkpoint_2.h");
        break;
      case E_CHECKPOINT_3:
        drawBitmap(checkpoint_3);
        Serial.println("Drawn checkpoint_3.h");
        break;
      case E_CHECKPOINT_4:
        drawBitmap(checkpoint_4);
        Serial.println("Drawn checkpoint_4.h");
        break;
      case E_CHECKPOINT_5:
        drawBitmap(checkpoint_5);
        Serial.println("Drawn checkpoint_5.h");
        break;
      case E_CHECKPOINT_6:
        drawBitmap(checkpoint_6);
        Serial.println("Drawn checkpoint_6.h");
        break;
      case E_CHECKPOINT_7:
        drawBitmap(checkpoint_7);
        Serial.println("Drawn checkpoint_7.h");
        break;
      case E_CHECKPOINT_8:
        drawBitmap(checkpoint_8);
        Serial.println("Drawn checkpoint_8.h");
        break;
      case E_CHECKPOINT_9:
        drawBitmap(checkpoint_9);
        Serial.println("Drawn checkpoint_9.h");
        break;
      case E_CHECKPOINT_10:
        drawBitmap(checkpoint_10);
        Serial.println("Drawn checkpoint_10.h");
        break;
      default:
        tft.fillScreen(TFT_BLACK);
        Serial.println("No image to display");
    }
    fadeIn();
    currentDisplayedImage = image;
  }
}

void drawBitmap(const unsigned char* bitmap) {
  tft.drawXBitmap(0, 0, bitmap, BITMAP_WIDTH, BITMAP_HEIGHT, TFT_BLACK, TFT_WHITE);
}

// Example function to calculate relative direction based on angles
int calculateRelativeDirection(int currentAngle, int targetAngle) {
  int difference = targetAngle - currentAngle;
  if (difference < 0) {
    difference += 360;  // Adjust for negative differences
  }
  return difference % 360;  // Ensure the result is within 0-359 degreess
}


// Select an arrow image based on relative direction
ImageType selectArrowImage(int relativeDirection) {
  if (relativeDirection >= 348.75 || relativeDirection < 11.25) return E_ARROW_N;
  else if (relativeDirection >= 11.25 && relativeDirection < 33.75) return E_ARROW_NNE;
  else if (relativeDirection >= 33.75 && relativeDirection < 56.25) return E_ARROW_NE;
  else if (relativeDirection >= 56.25 && relativeDirection < 78.75) return E_ARROW_ENE;
  else if (relativeDirection >= 78.75 && relativeDirection < 101.25) return E_ARROW_E;
  else if (relativeDirection >= 101.25 && relativeDirection < 123.75) return E_ARROW_ESE;
  else if (relativeDirection >= 123.75 && relativeDirection < 146.25) return E_ARROW_SE;
  else if (relativeDirection >= 146.25 && relativeDirection < 168.75) return E_ARROW_SSE;
  else if (relativeDirection >= 168.75 && relativeDirection < 191.25) return E_ARROW_S;
  else if (relativeDirection >= 191.25 && relativeDirection < 213.75) return E_ARROW_SSW;
  else if (relativeDirection >= 213.75 && relativeDirection < 236.25) return E_ARROW_SW;
  else if (relativeDirection >= 236.25 && relativeDirection < 258.75) return E_ARROW_WSW;
  else if (relativeDirection >= 258.75 && relativeDirection < 281.25) return E_ARROW_W;
  else if (relativeDirection >= 281.25 && relativeDirection < 303.75) return E_ARROW_WNW;
  else if (relativeDirection >= 303.75 && relativeDirection < 326.25) return E_ARROW_NW;
  else if (relativeDirection >= 326.25 && relativeDirection < 348.75) return E_ARROW_NNW;
  else return E_ARROW_N;  // Default case, although logically unnecessary due to the first condition
}

// Non blocking delay to read sensors
bool nonBlockingDelay(unsigned long ms) {
  static unsigned long lastCheck = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - lastCheck >= ms) {
    lastCheck = currentMillis;
    return true;
  }
  return false;
}

void fadeOut() {
  for (int i = 255; i >= 0; i -= 5) {
    analogWrite(PIN_DISPLAY_PWM_BL, i);
    delay(10);
  }
  screenOn = false;
}

void fadeIn() {
  for (int i = 0; i <= 255; i += 5) {
    analogWrite(PIN_DISPLAY_PWM_BL, i);
    delay(10);
  }
  screenOn = true;
}
