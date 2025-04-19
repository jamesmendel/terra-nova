#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>
#include "soc/soc_caps.h"

#define USB_VID 0x303a
#define USB_PID 0x1AA1

// BUILTIN_LED can be used in new Arduino API digitalWrite() like in Blink.ino
static const uint8_t LED_BUILTIN = 48;
#define BUILTIN_LED  LED_BUILTIN // backward compatibility
#define LED_BUILTIN LED_BUILTIN  // allow testing #ifdef LED_BUILTIN


// ================================
// UART:    GPS
// ================================
static const uint8_t TX = 17;
static const uint8_t RX = 18;


// ================================
// I2C:     IMU, Compass, Haptics
// ================================
static const uint8_t SDA = 7;
static const uint8_t SCL = 9;


// ================================
// SPI:     Display
// ================================
static const uint8_t SS    = 10;
static const uint8_t MOSI  = 11;
static const uint8_t MISO  = 13;
static const uint8_t SCK   = 12;


// ================================
// GPIO:    Peripherals, etc.
// ================================
//  Battery
static const uint8_t PIN_BAT_VOLTAGE    = 5;

//  Display
static const uint8_t PIN_DISPLAY_RST    = 8;    // active low display reset
static const uint8_t PIN_DISPLAY_PWM_BL = 9;    // display pwm backlight
static const uint8_t PIN_DISP_DC        = 33;   // display DC

//  Power Management
static const uint8_t PIN_PWR_SW         = 34;   // power switch input
static const uint8_t PIN_PWROFF         = 35;   // power off signal

//  Compass
static const uint8_t PIN_CMP_INT        = 37;   // compass interrupt
static const uint8_t PIN_CMP_RST        = 38;   // active low compass reset

//  GPS
static const uint8_t PIN_GPS_RST        = 39;   // active low GPS reset
static const uint8_t PIN_GPS_STBY       = 39;   // GPS standby mode
static const uint8_t PIN_GPS_FIX        = 40;   // GPS fix indicator

//  Haptics
static const uint8_t PIN_HAP_IN_TRIG    = 42;   // haptics in or trigger



// ================================
// GPIO:    UNUSED -- Legacy for
//          dev boards.
// ================================
// static const uint8_t A0 = 1;
// static const uint8_t A1 = 2;
// static const uint8_t A2 = 3;
// static const uint8_t A3 = 4;
// static const uint8_t A4 = 5;
// static const uint8_t A5 = 6;
// static const uint8_t A6 = 7;
// static const uint8_t A7 = 8;
// static const uint8_t A8 = 9;
// static const uint8_t A9 = 10;
// static const uint8_t A10 = 11;
// static const uint8_t A11 = 12;
// static const uint8_t A12 = 13;
// static const uint8_t A13 = 14;
// static const uint8_t A14 = 15;
// static const uint8_t A15 = 16;
// static const uint8_t A16 = 17;
// static const uint8_t A17 = 18;
// static const uint8_t A18 = 19;
// static const uint8_t A19 = 20;

// static const uint8_t T1 = 1;
// static const uint8_t T2 = 2;
// static const uint8_t T3 = 3;
// static const uint8_t T4 = 4;
// static const uint8_t T5 = 5;
// static const uint8_t T6 = 6;
// static const uint8_t T7 = 7;
// static const uint8_t T8 = 8;
// static const uint8_t T9 = 9;
// static const uint8_t T10 = 10;
// static const uint8_t T11 = 11;
// static const uint8_t T12 = 12;
// static const uint8_t T13 = 13;
// static const uint8_t T14 = 14;

#endif /* Pins_Arduino_h */
