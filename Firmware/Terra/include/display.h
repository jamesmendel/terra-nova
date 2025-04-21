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
    E_NONE,
    E_PENDING,
    E_GOTOSTART,
    E_ARROW_N,
    E_ARROW_NNE,
    E_ARROW_NE,
    E_ARROW_ENE,
    E_ARROW_E,
    E_ARROW_ESE,
    E_ARROW_SE,
    E_ARROW_SSE,
    E_ARROW_S,
    E_ARROW_SSW,
    E_ARROW_SW,
    E_ARROW_WSW,
    E_ARROW_W,
    E_ARROW_WNW,
    E_ARROW_NW,
    E_ARROW_NNW,
    E_CHECKPOINT_1,
    E_CHECKPOINT_2,
    E_CHECKPOINT_3,
    E_CHECKPOINT_4,
    E_CHECKPOINT_5,
    E_CHECKPOINT_6,
    E_CHECKPOINT_7,
    E_CHECKPOINT_8,
    E_CHECKPOINT_9,
    E_CHECKPOINT_10
};

enum DisplayState
{
    E_DISPLAY_OFF = 0x00,
    E_DISPLAY_FADEOUT,
    E_DISPLAY_UPDATING,
    E_DISPLAY_FADEIN,
    E_DISPLAY_STATIC,
};

static TFT_eSPI tft = TFT_eSPI();
static DisplayState displayNextState = E_DISPLAY_OFF;
static ImageType displayImage = E_NONE;
static bool displayUpdating = false;
static int16_t displayBrightness = 0;

void displayInit();
void displayUpdate();
void displaySetImage(ImageType image);
ImageType selectArrowImage(int relativeDirection);

#endif // DISPLAY_H