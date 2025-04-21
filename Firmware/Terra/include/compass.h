#ifndef COMPASS_H
#define COMPASS_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define CMP_REG_INT_STA     0x37    // section 4.3.56
#define CMP_REG_INT_MSK     0x0F    // section 4.4.8
#define CMP_REG_INT_EN      0x10    // section 4.4.9
#define CMP_REG_ACC_NM_THRESH   0x15    // section 4.4.14
#define CMP_REG_ACC_NM_SET      0x16    // section 4.4.15

// CMP_REG_INT_STA bitmasks:
#define CMP_MASK_ACC_NM         (0x01 << 7)
#define CMP_MASK_ACC_AM         (0x01 << 6)
#define CMP_MASK_ACC_HIGH_G     (0x01 << 5)
#define CMP_MASK_GYR_DRDY       (0x01 << 4)
#define CMP_MASK_GYR_HIGH_RATE  (0x01 << 3)
#define CMP_MASK_GRYO_AM        (0x01 << 2)
#define CMP_MASK_MAG_DRDY       (0x01 << 1)
#define CMP_MASK_ACC_BSX_DRDY   (0x01 << 0)

// Global compass
static Adafruit_BNO055 cmp = Adafruit_BNO055(55, BNO055_ADDRESS_A, &Wire);

void initCompass();
void initCompassNoMotionDetection(uint16_t timeout);
void compassServiceInterrupts();

int compassReadHeading();

#endif // COMPASS_H