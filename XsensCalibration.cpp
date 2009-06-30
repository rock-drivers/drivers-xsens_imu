#include"XsensDriver.hpp"
#include<iostream>

int main(int argc, char* argv[]) {
    if(argc < 2)
    {
        std::cout << "usage: xsensCalibration <device>" << std::endl;
        exit(1);
    }

    std::string dev(argv[1]);

    xsens_imu::XsensDriver driver;
    if (!driver.open(dev))
        return 1;
    if (!driver.setCalibrationMode())
        return 1;


    while(true)
    {
        int ret = driver.getReading();
        //if(ret != 0)
        //{
        //    std::cerr << "error while reading data" << std::endl;
        //    return 1;
        //}
    }

}
