#include <Arduino.h>
#include "compass.h"
#include "power.h"

static volatile bool compassPendingInterrupt = false;


void IRAM_ATTR compassIntISR() {
    compassPendingInterrupt = true;
}

void initCompass() {
    pinMode(PIN_CMP_RST, OUTPUT);
    pinMode(PIN_CMP_INT, INPUT);
    digitalWrite(PIN_CMP_RST, !LOW);

    attachInterrupt(PIN_CMP_INT, compassIntISR, RISING);

    if (!cmp.begin(OPERATION_MODE_NDOF)) {
        Serial.print("Could not find BNO055");
        while (1) delay(10);
    }
    cmp.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);  // TODO: tune this value! (see 3.1 of BNO055 datasheet)
    cmp.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);   // TODO: tune this value! (see 3.1 of BNO055 datasheet)
}

void initCompassNoMotionDetection() {
    uint8_t buf[2];
    // 3.8.2.2 Accelerometer Slow/No Motion Interrupt
    
    // Configure no-motion threshold
    buf[0] = CMP_REG_ACC_NM_THRESH;
    buf[1] = 0x0A; // default, NDOF fusion mode controls ACC_Config sensitivity
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write(buf, 2);
    Wire.endTransmission();

    // Configure no-motion timer and mode
    buf[0] = CMP_REG_ACC_NM_SET;
    buf[1] =  (0x3D << 1) | 0b1; // 360 seconds, no-motion 
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write(buf, 2);
    Wire.endTransmission();
    
    // Configure interrupt state
    buf[0] = CMP_REG_INT_EN;
    buf[1] =  0b1 << 7; // enable ACC_NM
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write(buf, 2);
    Wire.endTransmission();

    // Configure interrupt mask
    buf[0] = CMP_REG_INT_MSK;
    buf[1] =  0b1 << 7; // enable ACC_NM
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write(buf, 2);
    Wire.endTransmission();

}

void compassServiceInterrupts() {
    if(compassPendingInterrupt){
        static volatile uint8_t status = 0;

        Wire.beginTransmission(BNO055_ADDRESS_A);
        Wire.write(CMP_REG_INT_STA);
        status = Wire.read();
        Wire.endTransmission();

        if(status & CMP_MASK_ACC_NM) {
            Serial.printf("BNO055 says device is idle!\n");
            powerDownNow();
        }
    }
    compassPendingInterrupt = false;
}


/**
 * @brief reads rompass absolute orientation
 * 
 * @return int heading in degress
 */
int compassReadHeading() {
  sensors_event_t orientationData;
  int heading;

  cmp.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  heading = (int)orientationData.orientation.x;  // x=heading, y=roll, z=pitch

  return heading; // TODO: Verify the accuracy of this output. Is degress the correct unit to return?
}