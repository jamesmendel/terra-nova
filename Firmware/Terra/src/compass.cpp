#include <Arduino.h>
#include <cmath>
#include "compass.h"
#include "power.h"

inline uint8_t _encodeNoMotionDuration(uint16_t target);
void IRAM_ATTR _compassIntISR();

static volatile bool compassPendingInterrupt = false;


/**
 * @brief Configure compass pins and construct the device
 * 
 */
void initCompass() {
    pinMode(PIN_CMP_RST, OUTPUT);
    pinMode(PIN_CMP_INT, INPUT);
    digitalWrite(PIN_CMP_RST, !LOW);

    attachInterrupt(PIN_CMP_INT, _compassIntISR, RISING);

    if (!cmp.begin(OPERATION_MODE_NDOF)) {
        Serial.print("Could not find BNO055");
        while (1) delay(10);
    }
    cmp.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);  // TODO: tune this value! (see 3.1 of BNO055 datasheet)
    cmp.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);   // TODO: tune this value! (see 3.1 of BNO055 datasheet)
}

/**
 * @brief Initializes compass no-motion detection feature interrupts and sets timeout
 * 
 * @param timeout desired timeout in seconds
 */
void initCompassNoMotionDetection(uint16_t timeout) {
    if(timeout)
    {
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
        buf[1] =  (_encodeNoMotionDuration(timeout) << 1) | 0b1; // 360 seconds, no-motion 
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
}

/**
 * @brief Service pending compass interrupts.
 * 
 */
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

/**
 * @brief Set pending interrupt when available
 * 
 */
void IRAM_ATTR _compassIntISR() {
    compassPendingInterrupt = true;
}

/**
 * @brief Helper to encode a desired no-motion timeout to its register setting.
 *        Valid timeouts are (1:1:16), (20:4:88), (96:8:336)
 * @param target desired timeout (seconds)
 * @return uint8_t 6-bit NoMotion timeout register value
 */
inline uint8_t _encodeNoMotionDuration(uint16_t target)
{
    
    const uint16_t Region2_thresh = 18;  // midpoint of {16,20}
    const uint16_t Region3_thresh = 84;  // midpoint of {80,88}

    if (target <= Region2_thresh) {
        // Region 1: val[5:4]=00, duration = (low4 + 1)
        int v = int(std::round(double(target) - 1.0));
        if (v < 0)   v = 0;
        if (v > 15)  v = 15;
        return uint8_t(v);
    }
    else if (target <= Region3_thresh) {
        // Region 2: val[5:4]=01, duration = (low4 * 4 + 20)
        double x = (double(target) - 20.0) / 4.0;
        int v = int(std::round(x));
        if (v < 0)   v = 0;
        if (v > 15)  v = 15;
        return uint8_t(0x10 | v);
    }
    else {
        // Region 3: val[5]=1, duration = (low5 * 8 + 88)
        double x = (double(target) - 88.0) / 8.0;
        int v = int(std::round(x));
        if (v < 0)   v = 0;
        if (v > 31)  v = 31;
        return uint8_t(0x20 | v);
    }
}