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

#include "esp_log.h"
static const char* LOGTAG = "Display";

void _displayDrawImage();
void _displayFadeOut();
void _displayFadeIn();
void _drawBitmap(const unsigned char* bitmap);
static SemaphoreHandle_t _displayMutex;
class DisplayLock {
public:
    DisplayLock(SemaphoreHandle_t mtx) : _mtx(mtx) { xSemaphoreTake(_mtx, portMAX_DELAY); }
    ~DisplayLock() { xSemaphoreGive(_mtx); }
private:
    SemaphoreHandle_t _mtx;
};


/**
 * @brief Initializes tft and backlight
 * 
 */
void displayInit()
{
    _displayMutex = xSemaphoreCreateMutex();
    DisplayLock lock(_displayMutex);
    pinMode(PIN_DISPLAY_PWM_BL, OUTPUT);
    analogWrite(PIN_DISPLAY_PWM_BL, displayBrightness);  // 0 at fisrt init
    
    tft.init();
    tft.setRotation(1);
    
    tft.fillScreen(TFT_BLACK);
    //analogWrite(PIN_DISPLAY_PWM_BL, 100);
    
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(2);
    tft.drawString("Hello, TERRA!", 10, 100);
    ESP_LOGI(LOGTAG, "Finished display init!");
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

        #ifdef DISPLAY_NO_TRANSITION
            displayCurrentState = DISPLAY_UPDATING;
        #else
        if(displayCurrentState == DISPLAY_OFF) 
            // display is not showing anything, transition to updating bitmap without fade
            displayCurrentState = DISPLAY_UPDATING;
            else if(displayCurrentState != DISPLAY_FADEIN && displayCurrentState != DISPLAY_FADEOUT)
            // display has an image showing, fade out first.
            displayCurrentState = DISPLAY_FADEOUT;
            return;
            #endif
        }
}

/**
 * @brief Non-blocking fade-out effect for transitioning between states.
 * 
 */
void _displayFadeOut()
{
    static unsigned long lastStep = 0;
    int dt = millis() - lastStep;
    if(dt > DISPLAY_FADE_UPDATE_MS*4) dt = DISPLAY_FADE_UPDATE_MS;
    
    if (dt >= DISPLAY_FADE_UPDATE_MS)
    {
        lastStep = millis();
        displayBrightness -= (int)(5 * (float)dt/DISPLAY_FADE_UPDATE_MS);
        if (displayBrightness <= DISPLAY_BRIGHTNESS_OFF)
        {
            displayBrightness = DISPLAY_BRIGHTNESS_OFF;
            displayCurrentState = DISPLAY_UPDATING;
        }
        ESP_LOGD(LOGTAG, "out: %d dt: %d", displayBrightness, dt);
        analogWrite(PIN_DISPLAY_PWM_BL, (uint8_t)displayBrightness);
    }
}

/**
 * @brief Non-blocking fade-in effect for transitioning between states.
 * 
 */
void _displayFadeIn()
{
    static unsigned long lastStep = 0;
    int dt = millis() - lastStep;
    if(dt > DISPLAY_FADE_UPDATE_MS*4) dt = DISPLAY_FADE_UPDATE_MS;
    if (dt >= DISPLAY_FADE_UPDATE_MS)
    {
        lastStep = millis();
        displayBrightness += (int)(5 * (float)dt/DISPLAY_FADE_UPDATE_MS);
        if (displayBrightness >= DISPLAY_BRIGHTNESS_ON)
        {
            displayBrightness = DISPLAY_BRIGHTNESS_ON;
            displayCurrentState = DISPLAY_STATIC;
        }
        ESP_LOGD(LOGTAG, "in : %d dt %d", displayBrightness, dt);
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
    if (displayImage == I_NONE) {
        displayCurrentState = DISPLAY_FADEOUT;
        return;
    }
    
    #ifdef DISPLAY_NO_TRANSITION
    displayCurrentState = DISPLAY_STATIC;
    #else
    displayCurrentState = DISPLAY_FADEIN;
    #endif
    {
        DisplayLock lock(_displayMutex);
        tft.fillScreen(TFT_BLACK);  // clear first
    }
    switch (displayImage)
    {
    case I_PENDING:
        _drawBitmap(pending);
        ESP_LOGI(LOGTAG, "Draw pending.h");
        break;
    case I_GOTOSTART:
        _drawBitmap(gotostart);
        ESP_LOGI(LOGTAG, "Draw gotostart.h");
        break;
    case I_ARROW_N:
        _drawBitmap(arrow_N);
        ESP_LOGI(LOGTAG, "Draw arrow_N.h");
        break;
    case I_ARROW_NNE:
        _drawBitmap(arrow_NNE);
        ESP_LOGI(LOGTAG, "Draw arrow_NNE.h");
        break;
    case I_ARROW_NE:
        _drawBitmap(arrow_NE);
        ESP_LOGI(LOGTAG, "Draw arrow_NE.h");
        break;
    case I_ARROW_ENE:
        _drawBitmap(arrow_ENE);
        ESP_LOGI(LOGTAG, "Draw arrow_ENE.h");
        break;
    case I_ARROW_E:
        _drawBitmap(arrow_E);
        ESP_LOGI(LOGTAG, "Draw arrow_E.h");
        break;
    case I_ARROW_ESE:
        _drawBitmap(arrow_ESE);
        ESP_LOGI(LOGTAG, "Draw arrow_ESE.h");
        break;
    case I_ARROW_SE:
        _drawBitmap(arrow_SE);
        ESP_LOGI(LOGTAG, "Draw arrow_SE.h");
        break;
    case I_ARROW_SSE:
        _drawBitmap(arrow_SSE);
        ESP_LOGI(LOGTAG, "Draw arrow_SSE.h");
        break;
    case I_ARROW_S:
        _drawBitmap(arrow_S);
        ESP_LOGI(LOGTAG, "Draw arrow_S.h");
        break;
    case I_ARROW_SSW:
        _drawBitmap(arrow_SSW);
        ESP_LOGI(LOGTAG, "Draw arrow_SSW.h");
        break;
    case I_ARROW_SW:
        _drawBitmap(arrow_SW);
        ESP_LOGI(LOGTAG, "Draw arrow_SW.h");
        break;
    case I_ARROW_WSW:
        _drawBitmap(arrow_WSW);
        ESP_LOGI(LOGTAG, "Draw arrow_WSW.h");
        break;
    case I_ARROW_W:
        _drawBitmap(arrow_W);
        ESP_LOGI(LOGTAG, "Draw arrow_W.h");
        break;
    case I_ARROW_WNW:
        _drawBitmap(arrow_WNW);
        ESP_LOGI(LOGTAG, "Draw arrow_WNW.h");
        break;
    case I_ARROW_NW:
        _drawBitmap(arrow_NW);
        ESP_LOGI(LOGTAG, "Draw arrow_NW.h");
        break;
    case I_ARROW_NNW:
        _drawBitmap(arrow_NNW);
        ESP_LOGI(LOGTAG, "Draw arrow_NNW.h");
        break;
    case I_CHECKPOINT_1:
        _drawBitmap(checkpoint_1);
        ESP_LOGI(LOGTAG, "Draw checkpoint_1.h");
        break;
    case I_CHECKPOINT_2:
        _drawBitmap(checkpoint_2);
        ESP_LOGI(LOGTAG, "Draw checkpoint_2.h");
        break;
    case I_CHECKPOINT_3:
        _drawBitmap(checkpoint_3);
        ESP_LOGI(LOGTAG, "Draw checkpoint_3.h");
        break;
    case I_CHECKPOINT_4:
        _drawBitmap(checkpoint_4);
        ESP_LOGI(LOGTAG, "Draw checkpoint_4.h");
        break;
    case I_CHECKPOINT_5:
        _drawBitmap(checkpoint_5);
        ESP_LOGI(LOGTAG, "Draw checkpoint_5.h");
        break;
    case I_CHECKPOINT_6:
        _drawBitmap(checkpoint_6);
        ESP_LOGI(LOGTAG, "Draw checkpoint_6.h");
        break;
    case I_CHECKPOINT_7:
        _drawBitmap(checkpoint_7);
        ESP_LOGI(LOGTAG, "Draw checkpoint_7.h");
        break;
    case I_CHECKPOINT_8:
        _drawBitmap(checkpoint_8);
        ESP_LOGI(LOGTAG, "Draw checkpoint_8.h");
        break;
    case I_CHECKPOINT_9:
        _drawBitmap(checkpoint_9);
        ESP_LOGI(LOGTAG, "Draw checkpoint_9.h");
        break;
    case I_CHECKPOINT_10:
        _drawBitmap(checkpoint_10);
        ESP_LOGI(LOGTAG, "Draw checkpoint_10.h");
        break;
    default:
        ESP_LOGW(LOGTAG, "Image type unknown!");
    };
}

void _drawBitmap(const unsigned char *bitmap)
{
    DisplayLock lock(_displayMutex);
    tft.drawXBitmap(0, 0, bitmap, BITMAP_WIDTH, BITMAP_HEIGHT, TFT_BLACK, TFT_WHITE);
}

int16_t displayGetBrightness() {
    return displayBrightness;
}

void tftDrawDebugOverlay(const char* str, float line, uint8_t size) {
    DisplayLock lock(_displayMutex);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(size);
    tft.drawString(str, 10, 100 + (uint8_t)(line*16));
}