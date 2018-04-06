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
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
typedef double covtype;
// How big you want the covariance matrix (e.g. how many variables you keep track of)
#define DIMENSION   3

void covCalculator(double inputMat[3][9]) {
    // start roboticscape
    rc_initialize();
    // start i2c with the target register pointed to nothing
    // (this is important because FreeSixIMU assumes that the i2c port has
    //  already been initialized)
    rc_i2c_init(1, 0x00);

    FreeSixIMU imu;
    imu.init();

    int length = 100;
    CovarianceTracker<covtype, DIMENSION> orient_cov(length);
    CovarianceTracker<covtype, DIMENSION> lin_accel_cov(length);
    CovarianceTracker<covtype, DIMENSION> ang_vel_cov(length);

    float gyro_vals[4];
    float accel_vals[6];

    double gyro_double[3];
    double lin_accel_double[3];
    double ang_vel_double[3];

    for (size_t i = 0; i < 100; i++)
    {
        imu.getQ(gyro_vals);
        imu.getValues(accel_vals);

        gyro_double[0] = (double)(gyro_vals[0]);
        gyro_double[1] = (double)(gyro_vals[1]);
        gyro_double[2] = (double)(gyro_vals[2]);

        lin_accel_double[0] = (double)(accel_vals[0]);
        lin_accel_double[1] = (double)(accel_vals[1]);
        lin_accel_double[2] = (double)(accel_vals[2]);

        ang_vel_double[0] = (double)(accel_vals[3]);
        ang_vel_double[1] = (double)(accel_vals[4]);
        ang_vel_double[5] = (double)(accel_vals[6]);

        orient_cov.addData(gyro_double);
        lin_accel_cov.addData(lin_accel_double);
        ang_vel_cov.addData(lin_accel_double);

	      usleep(10000);
    }
    Eigen::Matrix<double, 3, 3> orient_cov_mat = orient_cov.getCovariance();
    Eigen::Matrix<double, 3, 3> lin_accel_cov_mat = lin_accel_cov.getCovariance();
    Eigen::Matrix<double, 3, 3> ang_vel_cov_mat = ang_vel_cov.getCovariance();

    //double covMats[3][9];

    double covMats0[] = {orient_cov_mat(0),orient_cov_mat(1),orient_cov_mat(2),
                  orient_cov_mat(3),orient_cov_mat(4),orient_cov_mat(5),
                  orient_cov_mat(6),orient_cov_mat(7),orient_cov_mat(8)};

    double covMats1[] = {lin_accel_cov_mat(0),lin_accel_cov_mat(1),lin_accel_cov_mat(2),
                  lin_accel_cov_mat(3),lin_accel_cov_mat(4),lin_accel_cov_mat(5),
                  lin_accel_cov_mat(6),lin_accel_cov_mat(7),lin_accel_cov_mat(8)};

    double covMats2[] = {ang_vel_cov_mat(0),ang_vel_cov_mat(1),ang_vel_cov_mat(2),
                  ang_vel_cov_mat(3),ang_vel_cov_mat(4),ang_vel_cov_mat(5),
                  ang_vel_cov_mat(6),ang_vel_cov_mat(7),ang_vel_cov_mat(8)};
    double covMats[3][9] = {covMats0, covMats1, covMats2};
    //covMats[0] = covMats0;
    //covMats[1] = covMats1;
    //covMats[2] = covMats2;
    inputMat = covMats;
}

int main(){
  double covMats[3][9];
  covCalculator(covMats);
  std::cout << covMats[0] << std::endl;
  std::cout << covMats[1] << std::endl;
  std::cout << covMats[2] << std::endl;
}
