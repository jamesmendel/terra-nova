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

static TFT_eSPI tft = TFT_eSPI();
static ImageType currentDisplayedImage = E_NONE;
static bool displayIsOn = false;

void displayInit();
void displayShowImage(ImageType image);
void drawBitmap(const unsigned char* bitmap);
ImageType selectArrowImage(int relativeDirection);
void displayFadeOut();
void displayFadeIn();

#endif // DISPLAY_H