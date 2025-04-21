/**
 * @file display.cpp
 * @author James Mendel (jrm.mendel@gmail.com)
 * @brief Implements the display driver
 * @date 2025-04-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <Arduino.h>

#include "display.h"
#include "imagelist.h"

void _displayDrawImage();
void _displayFadeOut();
void _displayFadeIn();
void _drawBitmap(const unsigned char* bitmap);

/**
 * @brief Initializes tft and backlight
 * 
 */
void displayInit()
{
    pinMode(PIN_DISPLAY_PWM_BL, OUTPUT);
    analogWrite(PIN_DISPLAY_PWM_BL, displayBrightness);

    tft.init();
    tft.setRotation(1);
}

/**
 * @brief Converts between heading and visual representation
 * 
 * @param relativeDirection compass heading (degrees)
 * @return ImageType corresponding image to display
 */
ImageType selectArrowImage(int relativeDirection)
{
    if (relativeDirection >= 348.75 || relativeDirection < 11.25)
        return E_ARROW_N;
    else if (relativeDirection >= 11.25 && relativeDirection < 33.75)
        return E_ARROW_NNE;
    else if (relativeDirection >= 33.75 && relativeDirection < 56.25)
        return E_ARROW_NE;
    else if (relativeDirection >= 56.25 && relativeDirection < 78.75)
        return E_ARROW_ENE;
    else if (relativeDirection >= 78.75 && relativeDirection < 101.25)
        return E_ARROW_E;
    else if (relativeDirection >= 101.25 && relativeDirection < 123.75)
        return E_ARROW_ESE;
    else if (relativeDirection >= 123.75 && relativeDirection < 146.25)
        return E_ARROW_SE;
    else if (relativeDirection >= 146.25 && relativeDirection < 168.75)
        return E_ARROW_SSE;
    else if (relativeDirection >= 168.75 && relativeDirection < 191.25)
        return E_ARROW_S;
    else if (relativeDirection >= 191.25 && relativeDirection < 213.75)
        return E_ARROW_SSW;
    else if (relativeDirection >= 213.75 && relativeDirection < 236.25)
        return E_ARROW_SW;
    else if (relativeDirection >= 236.25 && relativeDirection < 258.75)
        return E_ARROW_WSW;
    else if (relativeDirection >= 258.75 && relativeDirection < 281.25)
        return E_ARROW_W;
    else if (relativeDirection >= 281.25 && relativeDirection < 303.75)
        return E_ARROW_WNW;
    else if (relativeDirection >= 303.75 && relativeDirection < 326.25)
        return E_ARROW_NW;
    else if (relativeDirection >= 326.25 && relativeDirection < 348.75)
        return E_ARROW_NNW;
    else
        return E_ARROW_N;
}

/**
 * @brief Sets target image and updates state machine if necessary.
 * 
 * @param image 
 */
void displaySetImage(ImageType image)
{
    if (displayImage != image)
    {
        displayImage = image;

        if(displayNextState == E_DISPLAY_OFF) 
            displayNextState = E_DISPLAY_UPDATING;
        else
            displayNextState = E_DISPLAY_FADEOUT;
        return;
    }
    displayNextState = E_DISPLAY_STATIC;
}

/**
 * @brief Non-blocking fade-out effect for transitioning between states.
 * 
 */
void _displayFadeOut()
{
    static int lastStep = 0;
    if (millis() - lastStep >= DISPLAY_FADE_DELAY_MS)
    {
        lastStep = millis();
        displayBrightness -= 5;
        if (displayBrightness <= DISPLAY_BRIGHTNESS_OFF)
        {
            displayBrightness = DISPLAY_BRIGHTNESS_OFF;
            displayNextState = E_DISPLAY_UPDATING;
        }
        analogWrite(PIN_DISPLAY_PWM_BL, (uint8_t)displayBrightness);
    }
}

/**
 * @brief Non-blocking fade-in effect for transitioning between states.
 * 
 */
void _displayFadeIn()
{
    static int lastStep = 0;
    if (millis() - lastStep >= DISPLAY_FADE_DELAY_MS)
    {
        lastStep = millis();
        displayBrightness += 5;
        if (displayBrightness >= DISPLAY_BRIGHTNESS_ON)
        {
            displayBrightness = DISPLAY_BRIGHTNESS_ON;
            displayNextState = E_DISPLAY_STATIC;
        }
        analogWrite(PIN_DISPLAY_PWM_BL, (uint8_t)displayBrightness);
    }
}

/**
 * @brief Display state machine controls transition logic between display states.
 *        All bitmap transitions are fade-out, update, then fade-in.
 */
void displayUpdate()
{
    switch (displayNextState)
    {
    case (E_DISPLAY_OFF):
        tft.fillScreen(TFT_BLACK);
        displayBrightness = 0;
        displayNextState = E_DISPLAY_STATIC;
    case (E_DISPLAY_FADEOUT):
        _displayFadeOut();
        break;
    case (E_DISPLAY_UPDATING):
        _displayDrawImage();
        break;
    case (E_DISPLAY_FADEIN):
        _displayFadeIn();
        break;
    case (E_DISPLAY_STATIC):
    default:
        break;
    };
}

/**
 * @brief Sends target image to display and starts fade-in effect.
 * 
 */
void _displayDrawImage()
{
    displayNextState = E_DISPLAY_FADEIN;

    tft.fillScreen(TFT_BLACK);  // clear first
    switch (displayImage)
    {
    case E_PENDING:
        _drawBitmap(pending);
        Serial.println("Drawn pending.h");
        break;
    case E_GOTOSTART:
        _drawBitmap(gotostart);
        Serial.println("Drawn gotostart.h");
        break;
    case E_ARROW_N:
        _drawBitmap(arrow_N);
        Serial.println("Drawn arrow_N.h");
        break;
    case E_ARROW_NNE:
        _drawBitmap(arrow_NNE);
        Serial.println("Drawn arrow_NNE.h");
        break;
    case E_ARROW_NE:
        _drawBitmap(arrow_NE);
        Serial.println("Drawn arrow_NE.h");
        break;
    case E_ARROW_ENE:
        _drawBitmap(arrow_ENE);
        Serial.println("Drawn arrow_ENE.h");
        break;
    case E_ARROW_E:
        _drawBitmap(arrow_E);
        Serial.println("Drawn arrow_E.h");
        break;
    case E_ARROW_ESE:
        _drawBitmap(arrow_ESE);
        Serial.println("Drawn arrow_ESE.h");
        break;
    case E_ARROW_SE:
        _drawBitmap(arrow_SE);
        Serial.println("Drawn arrow_SE.h");
        break;
    case E_ARROW_SSE:
        _drawBitmap(arrow_SSE);
        Serial.println("Drawn arrow_SSE.h");
        break;
    case E_ARROW_S:
        _drawBitmap(arrow_S);
        Serial.println("Drawn arrow_S.h");
        break;
    case E_ARROW_SSW:
        _drawBitmap(arrow_SSW);
        Serial.println("Drawn arrow_SSW.h");
        break;
    case E_ARROW_SW:
        _drawBitmap(arrow_SW);
        Serial.println("Drawn arrow_SW.h");
        break;
    case E_ARROW_WSW:
        _drawBitmap(arrow_WSW);
        Serial.println("Drawn arrow_WSW.h");
        break;
    case E_ARROW_W:
        _drawBitmap(arrow_W);
        Serial.println("Drawn arrow_W.h");
        break;
    case E_ARROW_WNW:
        _drawBitmap(arrow_WNW);
        Serial.println("Drawn arrow_WNW.h");
        break;
    case E_ARROW_NW:
        _drawBitmap(arrow_NW);
        Serial.println("Drawn arrow_NW.h");
        break;
    case E_ARROW_NNW:
        _drawBitmap(arrow_NNW);
        Serial.println("Drawn arrow_NNW.h");
        break;
    case E_CHECKPOINT_1:
        _drawBitmap(checkpoint_1);
        Serial.println("Drawn checkpoint_1.h");
        break;
    case E_CHECKPOINT_2:
        _drawBitmap(checkpoint_2);
        Serial.println("Drawn checkpoint_2.h");
        break;
    case E_CHECKPOINT_3:
        _drawBitmap(checkpoint_3);
        Serial.println("Drawn checkpoint_3.h");
        break;
    case E_CHECKPOINT_4:
        _drawBitmap(checkpoint_4);
        Serial.println("Drawn checkpoint_4.h");
        break;
    case E_CHECKPOINT_5:
        _drawBitmap(checkpoint_5);
        Serial.println("Drawn checkpoint_5.h");
        break;
    case E_CHECKPOINT_6:
        _drawBitmap(checkpoint_6);
        Serial.println("Drawn checkpoint_6.h");
        break;
    case E_CHECKPOINT_7:
        _drawBitmap(checkpoint_7);
        Serial.println("Drawn checkpoint_7.h");
        break;
    case E_CHECKPOINT_8:
        _drawBitmap(checkpoint_8);
        Serial.println("Drawn checkpoint_8.h");
        break;
    case E_CHECKPOINT_9:
        _drawBitmap(checkpoint_9);
        Serial.println("Drawn checkpoint_9.h");
        break;
    case E_CHECKPOINT_10:
        _drawBitmap(checkpoint_10);
        Serial.println("Drawn checkpoint_10.h");
        break;
    default:
        tft.fillScreen(TFT_BLACK);
        Serial.println("No image to display");
    };
}

void _drawBitmap(const unsigned char *bitmap)
{
    tft.drawXBitmap(0, 0, bitmap, BITMAP_WIDTH, BITMAP_HEIGHT, TFT_BLACK, TFT_WHITE);
}