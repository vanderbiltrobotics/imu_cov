
extern "C" {
    #include <roboticscape.h>
    #include <rc_usefulincludes.h>
}

#include "FreeSixIMU.h"
#include <iostream>
#include <fstream>
#include <unistd.h>

int main() {
    // start roboticscape
    rc_initialize();
    // start i2c with the target register pointed to nothing
    // (this is important because FreeSixIMU assumes that the i2c port has 
    //  already been initialized)
    rc_i2c_init(1, 0x00);

    FreeSixIMU imu;
//    std::cout << "TEST" << std::endl;
    imu.init();
   // std::cout << "TEST" << std::endl;
    float accel_gyro_vals[6];
    std::ofstream data_file;
    data_file.open("data.txt");
    for (size_t i = 0; i < 10000; i++)
    {
        imu.getValues(accel_gyro_vals);
        data_file << accel_gyro_vals[0] << "," << accel_gyro_vals[1] << "," << accel_gyro_vals[2] << "\n";
        usleep(10000);
    }
    data_file.close();

    return 0;
}
