#ifndef TFT_ESPI_CONFIG_H
#define TFT_ESPI_CONFIG_H

#include <Arduino.h>

#define DISABLE_ALL_LIBRARY_WARNINGS

#define USER_SETUP_LOADED 1

#define GC9A01_DRIVER             // TFT Driver IC type
#define TFT_MISO MISO             // TFT Data out
#define TFT_MOSI MOSI             // TFT Data in
#define TFT_SCLK SCK              // TFT Clock
#define TFT_CS SS                 // Chip select control pin
#define TFT_DC PIN_DISP_DC        // Data Command control pin
#define TFT_RST PIN_DISPLAY_RST   // Reset pin (could connect to RST pin)
#define TFT_BL PIN_DISPLAY_PWM_BL // Backlight pin
#define TFT_BACKLIGHT_ON HIGH     // Backlight polarity
// #define LOAD_GLCD    // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
// #define LOAD_FONT2   // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
// #define LOAD_FONT4   // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
// #define LOAD_FONT6   // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
// #define LOAD_FONT7   // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
// #define LOAD_FONT8   // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
// #define LOAD_GFXFF   // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
// #define SMOOTH_FONT
#define TFT_WIDTH 240
#define TFT_HEIGHT 240
#define SPI_FREQUENCY 40000000
#define SPI_READ_FREQUENCY 20000000
#define SPI_TOUCH_FREQUENCY 2500000

#endif