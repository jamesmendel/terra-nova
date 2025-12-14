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
void _bitmapToRGB565(const uint8_t* src, uint16_t* dst);
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
    tft.initDMA(true);  
    ESP_LOGI(LOGTAG, "TFT_eSPI DMA: %s", tft.DMA_Enabled ? "ENABLED" : "DISABLED");
    
    tft.pushImageDMA(0, 0, 240, 240, fb_dram);  // TFT_BLACK at Init
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
        // ESP_LOGD(LOGTAG, "out: %d dt: %d", displayBrightness, dt);
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
        // ESP_LOGD(LOGTAG, "in : %d dt %d", displayBrightness, dt);
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
    switch (displayImage)
    {
    case I_PENDING:
        ESP_LOGI(LOGTAG, "Draw pending.h");
        _drawBitmap(pending);
        break;
    case I_GOTOSTART:
        ESP_LOGI(LOGTAG, "Draw gotostart.h");
        _drawBitmap(gotostart);
        break;
    case I_ARROW_N:
        ESP_LOGI(LOGTAG, "Draw arrow_N.h");
        _drawBitmap(arrow_N);
        break;
    case I_ARROW_NNE:
        ESP_LOGI(LOGTAG, "Draw arrow_NNE.h");
        _drawBitmap(arrow_NNE);
        break;
    case I_ARROW_NE:
        ESP_LOGI(LOGTAG, "Draw arrow_NE.h");
        _drawBitmap(arrow_NE);
        break;
    case I_ARROW_ENE:
        ESP_LOGI(LOGTAG, "Draw arrow_ENE.h");
        _drawBitmap(arrow_ENE);
        break;
    case I_ARROW_E:
        ESP_LOGI(LOGTAG, "Draw arrow_E.h");
        _drawBitmap(arrow_E);
        break;
    case I_ARROW_ESE:
        ESP_LOGI(LOGTAG, "Draw arrow_ESE.h");
        _drawBitmap(arrow_ESE);
        break;
    case I_ARROW_SE:
        ESP_LOGI(LOGTAG, "Draw arrow_SE.h");
        _drawBitmap(arrow_SE);
        break;
    case I_ARROW_SSE:
        ESP_LOGI(LOGTAG, "Draw arrow_SSE.h");
        _drawBitmap(arrow_SSE);
        break;
    case I_ARROW_S:
        ESP_LOGI(LOGTAG, "Draw arrow_S.h");
        _drawBitmap(arrow_S);
        break;
    case I_ARROW_SSW:
        ESP_LOGI(LOGTAG, "Draw arrow_SSW.h");
        _drawBitmap(arrow_SSW);
        break;
    case I_ARROW_SW:
        ESP_LOGI(LOGTAG, "Draw arrow_SW.h");
        _drawBitmap(arrow_SW);
        break;
    case I_ARROW_WSW:
        ESP_LOGI(LOGTAG, "Draw arrow_WSW.h");
        _drawBitmap(arrow_WSW);
        break;
    case I_ARROW_W:
        ESP_LOGI(LOGTAG, "Draw arrow_W.h");
        _drawBitmap(arrow_W);
        break;
    case I_ARROW_WNW:
        ESP_LOGI(LOGTAG, "Draw arrow_WNW.h");
        _drawBitmap(arrow_WNW);
        break;
    case I_ARROW_NW:
        ESP_LOGI(LOGTAG, "Draw arrow_NW.h");
        _drawBitmap(arrow_NW);
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
    
    unsigned long mark0 = micros();
    
    _bitmapToRGB565(bitmap, fb_dram);
    unsigned long mark1 = micros();
       
    tft.pushImageDMA(0, 0, BITMAP_WIDTH, BITMAP_HEIGHT, fb_dram);
    tft.dmaWait();
    unsigned long mark2 = micros();
    
    ESP_LOGD(LOGTAG, "DMA push done!");
    
    ESP_LOGD(LOGTAG, "BENCHMARK:\nmark1\t%d\nmark2\t%d", mark1-mark0, mark2-mark0);
    // tft.drawXBitmap(0, 0, bitmap, BITMAP_WIDTH, BITMAP_HEIGHT, TFT_BLACK, TFT_WHITE);
    // tft.pushPixelsDMA
}

void _bitmapToRGB565(const uint8_t* src, uint16_t* dst)
{
    const int stride = (BITMAP_WIDTH + 7) / 8;

    for (int y = 0; y < BITMAP_HEIGHT; y++) {
        for (int x = 0; x < BITMAP_WIDTH; x++) {
            uint8_t byte = src[y * stride + (x >> 3)];
            bool bit = byte & (1 << (x & 7));  // LSB-first
            dst[y * BITMAP_WIDTH + x] = bit ? TFT_BLACK : TFT_WHITE;
        }
    }
}

int16_t displayGetBrightness() {
    return displayBrightness;
}

void tftDrawDebugOverlay(const char* str, float line, uint8_t size) {
    #if 0
    DisplayLock lock(_displayMutex);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(size);
    tft.drawString(str, 10, 100 + (uint8_t)(line*16));
    #endif
}