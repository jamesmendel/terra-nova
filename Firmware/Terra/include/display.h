/**
 * @file display.h
 * @author James Mendel (jrm.mendel@gmail.com)
 * @brief Implements the display driver
 * @date 2025-04-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#define BITMAP_WIDTH 240
#define BITMAP_HEIGHT 240

#define DISPLAY_FADE_DELAY_MS   10
#define DISPLAY_BRIGHTNESS_ON   (uint8_t)255
#define DISPLAY_BRIGHTNESS_OFF  (uint8_t)0

#include <TFT_eSPI.h>

// Image Types
enum ImageType
{
    I_NONE,
    I_PENDING,
    I_GOTOSTART,
    I_ARROW_N,
    I_ARROW_NNE,
    I_ARROW_NE,
    I_ARROW_ENE,
    I_ARROW_E,
    I_ARROW_ESE,
    I_ARROW_SE,
    I_ARROW_SSE,
    I_ARROW_S,
    I_ARROW_SSW,
    I_ARROW_SW,
    I_ARROW_WSW,
    I_ARROW_W,
    I_ARROW_WNW,
    I_ARROW_NW,
    I_ARROW_NNW,
    I_CHECKPOINT_1,
    I_CHECKPOINT_2,
    I_CHECKPOINT_3,
    I_CHECKPOINT_4,
    I_CHECKPOINT_5,
    I_CHECKPOINT_6,
    I_CHECKPOINT_7,
    I_CHECKPOINT_8,
    I_CHECKPOINT_9,
    I_CHECKPOINT_10
};

enum DisplayState
{
    DISPLAY_OFF = 0x00,
    DISPLAY_FADEOUT,
    DISPLAY_UPDATING,
    DISPLAY_FADEIN,
    DISPLAY_STATIC,
};

static TFT_eSPI tft = TFT_eSPI();
static DisplayState displayCurrentState = DISPLAY_OFF;
static ImageType displayImage = I_NONE;
static bool displayUpdating = false;
static int16_t displayBrightness = 0;

void displayInit();
void displayUpdate();
void displaySetImage(ImageType image);
ImageType selectArrowImage(int relativeDirection);

#endif // DISPLAY_H