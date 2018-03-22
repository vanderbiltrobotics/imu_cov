extern "C" {
    #include <roboticscape.h>
    #include <rc_usefulincludes.h>
}
#include <covariance-tracker/covariance-tracker.h>
// other includes
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "FreeSixIMU.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
typedef double covtype;
// How big you want the covariance matrix (e.g. how many variables you keep track of)
#define DIMENSION   3

int main() {
    // start roboticscape
    rc_initialize();
    // start i2c with the target register pointed to nothing
    // (this is important because FreeSixIMU assumes that the i2c port has
    //  already been initialized)
    rc_i2c_init(1, 0x00);

    FreeSixIMU imu;
    imu.init();

    int length = 50;
    CovarianceTracker<covtype, DIMENSION> cov3(length);

    float accel_gyro_vals[6];
    double point[3];
    std::ofstream matDataFile;
    std::ofstream meanDataFile;
    std::ofstream dataFile;
    matDataFile.open("matData.txt");
    meanDataFile.open("meanData.txt");
    dataFile.open("data.txt");
    double time = 0.0;

    for (size_t i = 0; i < 10000; i++)
    {
        imu.getValues(accel_gyro_vals);
        point[0] = (double)(accel_gyro_vals[0]);
        point[1] = (double)(accel_gyro_vals[1]);
        point[2] = (double)(accel_gyro_vals[2]);
        cov3.addData(point);
	usleep(10000);
	time+=(10000.0/1000000.0);
	dataFile << accel_gyro_vals[0] << "," << accel_gyro_vals[1] << "," << accel_gyro_vals[2] << "," << time << "\n";
        if (i%25 == 0 && i != 0 && i != 25)
        {
          Eigen::Matrix<double, 3, 3> covMat = cov3.getCovariance();
          Eigen::Matrix<double, 3, 1> mean = cov3.getMean();
          matDataFile << covMat << "\n\n";
          meanDataFile << mean << "\n\n";
        }
    }
    matDataFile.close();
    meanDataFile.close();

    return 0;
}
