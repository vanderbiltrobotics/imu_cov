// ******************
// IMU Class Definitions (Forward)
// ******************
// This is the class we'll access directly with our program. Do not use any method
// in the ADXL345 or the ITG3200 classes!!! They are buffered by this class and
// are not designed for direct use.
// Use of this class assumes that the first (1) bus of the I2C port on the BBB has been
// initialized. If it hasn't, the RoboticsCape library will not work properly. Run the i2c_init
// method as defined above before use.



#ifndef FreeSixIMU_h
#define FreeSixIMU_h

#define FIMU_ACC_ADDR ADXL345_ADDR_ALT_LOW // SDO connected to GND

#define FIMU_BMA180_DEF_ADDR BMA180_ADDRESS_SDO_LOW
#define FIMU_ITG3200_DEF_ADDR ITG3200_ADDR_AD0_LOW // AD0 connected to GND
// HMC5843 address is fixed so don't bother to define it


#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

// Clear special register function (not used in C++/RoboticsCape)
/*#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif*/

#include "imu/ADXL345.h"
#include "imu/ITG3200.h"

typedef uint8_t byte;

class FreeSixIMU
{
  public:
    FreeSixIMU();
    void init();
    void init(bool fastmode);
    void init(int16_t acc_addr, int16_t gyro_addr, bool fastmode);
    void getRawValues(int16_t * raw_values);
    void getValues(float * values);
    void getQ(float * q);
    void getEuler(float * angles);
    void getYawPitchRoll(float * ypr);
    void getAngles(float * angles);


    ADXL345 acc;
    ITG3200 gyro;

    int16_t* raw_acc, raw_gyro, raw_magn;

  private:
    void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    //float q0, q1, q2, q3; // quaternion elements representing the estimated orientation
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;  // scaled integral error
    volatile float twoKp;      // 2 * proportional gain (Kp)
    volatile float twoKi;      // 2 * integral gain (Ki)
    volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx,  integralFBy, integralFBz;
    unsigned long long lastUpdate, now; // sample period expressed in milliseconds
    float sampleFreq; // half the sample period expressed in seconds
    int16_t startLoopTime;
};

float invSqrt(float number);

#endif // FreeSixIMU_h
