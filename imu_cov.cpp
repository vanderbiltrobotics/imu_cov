extern "C" {
	#include <roboticscape.h>
	#include <rc_usefulincludes.h>
}
#include "./src/imu/src/imu/FreeSixIMU.cpp"
#include <iostream>
#include <fstream>
#include <unistd.h>

int main() {

    rc_initialize();
    FreeSixIMU imu;
    std::cout << "TEST" << std::endl;
    imu.init();
    std::cout << "TEST" << std::endl;
    float orient_arr[4];
    std::ofstream data_file;
    data_file.open("orient_data.txt");
    for (size_t i = 0; i < 10000; i++)
    {
        imu.getQ(orient_arr);
        data_file << orient_arr[0] << "," << orient_arr[1] << "," << orient_arr[2] << "," << orient_arr[3] << std::endl;
        usleep(10);
    }
    data_file.close();

    return 0;
}

