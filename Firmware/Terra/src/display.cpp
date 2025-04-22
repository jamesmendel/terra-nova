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
    analogWrite(PIN_DISPLAY_PWM_BL, displayBrightness);  // 0 at fisrt init

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
        return I_ARROW_N;
    else if (relativeDirection >= 11.25 && relativeDirection < 33.75)
        return I_ARROW_NNE;
    else if (relativeDirection >= 33.75 && relativeDirection < 56.25)
        return I_ARROW_NE;
    else if (relativeDirection >= 56.25 && relativeDirection < 78.75)
        return I_ARROW_ENE;
    else if (relativeDirection >= 78.75 && relativeDirection < 101.25)
        return I_ARROW_E;
    else if (relativeDirection >= 101.25 && relativeDirection < 123.75)
        return I_ARROW_ESE;
    else if (relativeDirection >= 123.75 && relativeDirection < 146.25)
        return I_ARROW_SE;
    else if (relativeDirection >= 146.25 && relativeDirection < 168.75)
        return I_ARROW_SSE;
    else if (relativeDirection >= 168.75 && relativeDirection < 191.25)
        return I_ARROW_S;
    else if (relativeDirection >= 191.25 && relativeDirection < 213.75)
        return I_ARROW_SSW;
    else if (relativeDirection >= 213.75 && relativeDirection < 236.25)
        return I_ARROW_SW;
    else if (relativeDirection >= 236.25 && relativeDirection < 258.75)
        return I_ARROW_WSW;
    else if (relativeDirection >= 258.75 && relativeDirection < 281.25)
        return I_ARROW_W;
    else if (relativeDirection >= 281.25 && relativeDirection < 303.75)
        return I_ARROW_WNW;
    else if (relativeDirection >= 303.75 && relativeDirection < 326.25)
        return I_ARROW_NW;
    else if (relativeDirection >= 326.25 && relativeDirection < 348.75)
        return I_ARROW_NNW;
    else
        return I_ARROW_N;
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

        if(displayCurrentState == DISPLAY_OFF) 
            // display is not showing anything, transition to updating bitmap without fade
            displayCurrentState = DISPLAY_UPDATING;
        else
            // display has an image showing, fade out first.
            displayCurrentState = DISPLAY_FADEOUT;
        return;
    }

    // no change was made, display is not changing.
    displayCurrentState = DISPLAY_STATIC;
    return;
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
            displayCurrentState = DISPLAY_UPDATING;
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
            displayCurrentState = DISPLAY_STATIC;
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
    switch (displayCurrentState)
    {
    case (DISPLAY_OFF):
        tft.fillScreen(TFT_BLACK);
        displayBrightness = 0;
        displayCurrentState = DISPLAY_STATIC;
    case (DISPLAY_FADEOUT):
        _displayFadeOut();
        break;
    case (DISPLAY_UPDATING):
        _displayDrawImage();
        break;
    case (DISPLAY_FADEIN):
        _displayFadeIn();
        break;
    case (DISPLAY_STATIC):
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
    displayCurrentState = DISPLAY_FADEIN;

    tft.fillScreen(TFT_BLACK);  // clear first
    switch (displayImage)
    {
    case I_PENDING:
        _drawBitmap(pending);
        Serial.println("Drawn pending.h");
        break;
    case I_GOTOSTART:
        _drawBitmap(gotostart);
        Serial.println("Drawn gotostart.h");
        break;
    case I_ARROW_N:
        _drawBitmap(arrow_N);
        Serial.println("Drawn arrow_N.h");
        break;
    case I_ARROW_NNE:
        _drawBitmap(arrow_NNE);
        Serial.println("Drawn arrow_NNE.h");
        break;
    case I_ARROW_NE:
        _drawBitmap(arrow_NE);
        Serial.println("Drawn arrow_NE.h");
        break;
    case I_ARROW_ENE:
        _drawBitmap(arrow_ENE);
        Serial.println("Drawn arrow_ENE.h");
        break;
    case I_ARROW_E:
        _drawBitmap(arrow_E);
        Serial.println("Drawn arrow_E.h");
        break;
    case I_ARROW_ESE:
        _drawBitmap(arrow_ESE);
        Serial.println("Drawn arrow_ESE.h");
        break;
    case I_ARROW_SE:
        _drawBitmap(arrow_SE);
        Serial.println("Drawn arrow_SE.h");
        break;
    case I_ARROW_SSE:
        _drawBitmap(arrow_SSE);
        Serial.println("Drawn arrow_SSE.h");
        break;
    case I_ARROW_S:
        _drawBitmap(arrow_S);
        Serial.println("Drawn arrow_S.h");
        break;
    case I_ARROW_SSW:
        _drawBitmap(arrow_SSW);
        Serial.println("Drawn arrow_SSW.h");
        break;
    case I_ARROW_SW:
        _drawBitmap(arrow_SW);
        Serial.println("Drawn arrow_SW.h");
        break;
    case I_ARROW_WSW:
        _drawBitmap(arrow_WSW);
        Serial.println("Drawn arrow_WSW.h");
        break;
    case I_ARROW_W:
        _drawBitmap(arrow_W);
        Serial.println("Drawn arrow_W.h");
        break;
    case I_ARROW_WNW:
        _drawBitmap(arrow_WNW);
        Serial.println("Drawn arrow_WNW.h");
        break;
    case I_ARROW_NW:
        _drawBitmap(arrow_NW);
        Serial.println("Drawn arrow_NW.h");
        break;
    case I_ARROW_NNW:
        _drawBitmap(arrow_NNW);
        Serial.println("Drawn arrow_NNW.h");
        break;
    case I_CHECKPOINT_1:
        _drawBitmap(checkpoint_1);
        Serial.println("Drawn checkpoint_1.h");
        break;
    case I_CHECKPOINT_2:
        _drawBitmap(checkpoint_2);
        Serial.println("Drawn checkpoint_2.h");
        break;
    case I_CHECKPOINT_3:
        _drawBitmap(checkpoint_3);
        Serial.println("Drawn checkpoint_3.h");
        break;
    case I_CHECKPOINT_4:
        _drawBitmap(checkpoint_4);
        Serial.println("Drawn checkpoint_4.h");
        break;
    case I_CHECKPOINT_5:
        _drawBitmap(checkpoint_5);
        Serial.println("Drawn checkpoint_5.h");
        break;
    case I_CHECKPOINT_6:
        _drawBitmap(checkpoint_6);
        Serial.println("Drawn checkpoint_6.h");
        break;
    case I_CHECKPOINT_7:
        _drawBitmap(checkpoint_7);
        Serial.println("Drawn checkpoint_7.h");
        break;
    case I_CHECKPOINT_8:
        _drawBitmap(checkpoint_8);
        Serial.println("Drawn checkpoint_8.h");
        break;
    case I_CHECKPOINT_9:
        _drawBitmap(checkpoint_9);
        Serial.println("Drawn checkpoint_9.h");
        break;
    case I_CHECKPOINT_10:
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