// ******************
// Accelerometer Library Definitions
// ******************
//
// Comment these braces out when you want to compile; they're just here for viewing ease
//  (they're collapsable)

//{

//#include "Arduino.h"
// lol

#ifndef FIMU_ADXL345_h
#define FIMU_ADXL345_h

/* -- ADXL345 addresses --*/
#define ADXL345_ADDR_ALT_HIGH 0x1D // ADXL345 address when ALT is connected to HIGH
#define ADXL345_ADDR_ALT_LOW 0x53 // ADXL345 address when ALT is connected to LOW

/* ------- Register names ------- */
#define ADXL345_DEVID 0x00
#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
#define ADXL345_OFSX 0x1e
#define ADXL345_OFSY 0x1f
#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
#define ADXL345_THRESH_INACT 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INACT_CTL 0x27
#define ADXL345_THRESH_FF 0x28
#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STATUS 0x2b
#define ADXL345_BW_RATE 0x2c
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_INT_SOURCE 0x30
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
#define ADXL345_FIFO_CTL 0x38
#define ADXL345_FIFO_STATUS 0x39

#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800 0xE // 1110
#define ADXL345_BW_400 0xD // 1101
#define ADXL345_BW_200 0xC // 1100
#define ADXL345_BW_100 0xB // 1011
#define ADXL345_BW_50 0xA // 1010
#define ADXL345_BW_25 0x9 // 1001
#define ADXL345_BW_12 0x8 // 1000
#define ADXL345_BW_6 0x7 // 0111
#define ADXL345_BW_3 0x6 // 0110


/*
Interrupt PINs
INT1: 0
INT2: 1
*/
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

/*
Interrupt bit position
*/
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT 0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT 0x02
#define ADXL345_INT_WATERMARK_BIT 0x01
#define ADXL345_INT_OVERRUNY_BIT 0x00

#define ADXL345_OK 1 // no error
#define ADXL345_ERROR 0 // indicates error is predent

#define ADXL345_NO_ERROR 0 // initial state
#define ADXL345_READ_ERROR 1 // problem reading accel
#define ADXL345_BAD_ARG 2 // bad method argument


/*
Used in Members:
*/
#define TO_READ (6)      // num of bytes we are going to read each time (two bytes for each axis)

#include <stdint.h>
#include <unistd.h>
#include <math.h>

typedef uint8_t byte;

class ADXL345
{
public:
    bool status; // set when error occurs
    // see error code for details
    byte error_code; // Initial state
    float gains[3]; // counts to Gs

    ADXL345();
    void init(int16_t address);
    void powerOn();
    void readAccel(int16_t* xyx);
    void readAccel(int16_t* x, int16_t* y, int16_t* z);
    void get_Gxyz(float *xyz);

    void setTapThreshold(int16_t tapThreshold);
    int16_t getTapThreshold();
    void setAxisGains(float *_gains);
    void getAxisGains(float *_gains);
    void setAxisOffset(int16_t x, int16_t y, int16_t z);
    void getAxisOffset(int16_t* x, int16_t* y, int16_t*z);
    void setTapDuration(int16_t tapDuration);
    int16_t getTapDuration();
    void setDoubleTapLatency(int16_t floatTapLatency);
    int16_t getDoubleTapLatency();
    void setDoubleTapWindow(int16_t floatTapWindow);
    int16_t getDoubleTapWindow();
    void setActivityThreshold(int16_t activityThreshold);
    int16_t getActivityThreshold();
    void setInactivityThreshold(int16_t inactivityThreshold);
    int16_t getInactivityThreshold();
    void setTimeInactivity(int16_t timeInactivity);
    int16_t getTimeInactivity();
    void setFreeFallThreshold(int16_t freeFallthreshold);
    int16_t getFreeFallThreshold();
    void setFreeFallDuration(int16_t freeFallDuration);
    int16_t getFreeFallDuration();

    bool isActivityXEnabled();
    bool isActivityYEnabled();
    bool isActivityZEnabled();
    bool isInactivityXEnabled();
    bool isInactivityYEnabled();
    bool isInactivityZEnabled();
    bool isActivityAc();
    bool isInactivityAc();
    void setActivityAc(bool state);
    void setInactivityAc(bool state);

    bool getSuppressBit();
    void setSuppressBit(bool state);
    bool isTapDetectionOnX();
    void setTapDetectionOnX(bool state);
    bool isTapDetectionOnY();
    void setTapDetectionOnY(bool state);
    bool isTapDetectionOnZ();
    void setTapDetectionOnZ(bool state);

    void setActivityX(bool state);
    void setActivityY(bool state);
    void setActivityZ(bool state);
    void setInactivityX(bool state);
    void setInactivityY(bool state);
    void setInactivityZ(bool state);

    bool isActivitySourceOnX();
    bool isActivitySourceOnY();
    bool isActivitySourceOnZ();
    bool isTapSourceOnX();
    bool isTapSourceOnY();
    bool isTapSourceOnZ();
    bool isAsleep();

    bool isLowPower();
    void setLowPower(bool state);
    float getRate();
    void setRate(float rate);
    void set_bw(byte bw_code);
    byte get_bw_code();

    byte getInterruptSource();
    bool getInterruptSource(byte interruptBit);
    bool getInterruptMapping(byte interruptBit);
    void setInterruptMapping(byte interruptBit, bool interruptPin);
    bool isInterruptEnabled(byte interruptBit);
    void setInterrupt(byte interruptBit, bool state);

    void getRangeSetting(byte* rangeSetting);
    void setRangeSetting(int16_t val);
    bool getSelfTestBit();
    void setSelfTestBit(bool selfTestBit);
    bool getSpiBit();
    void setSpiBit(bool spiBit);
    bool getInterruptLevelBit();
    void setInterruptLevelBit(bool interruptLevelBit);
    bool getFullResBit();
    void setFullResBit(bool fullResBit);
    bool getJustifyBit();
    void setJustifyBit(bool justifyBit);
    void printAllRegister();
    void writeTo(byte address, byte val);

private:
    void readFrom(byte address, int16_t num, byte buff[]);
    void setRegisterBit(byte regAdress, int16_t bitPos, bool state);
    bool getRegisterBit(byte regAdress, int16_t bitPos);
    byte _buff[6] ; //6 bytes buffer for saving data read from the device
    byte _dev_address;
};
void print_byte(byte val);
#endif  // #ifndef FIMU_ADXL345_h
//}
