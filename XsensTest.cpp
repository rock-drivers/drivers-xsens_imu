#include"XsensDriver.hpp"
#include<iostream>

int main(int argc, char* argv[]) {
    if(argc < 2) {  
        std::cout << "usage xsensTest <device>" << std::endl;
        exit(1);
    }

    std::string dev(argv[1]);

    xsens_imu::XsensDriver driver;

    if (! driver.open(dev))
    {
        std::cerr << "failed to open IMU" << std::endl;
        return 1;
    }
    if (! driver.setReadingMode(xsens_imu::CAL_AND_ORI_DATA))
    {
        std::cerr << "failed to go into sample reading mode" << std::endl;
        return 1;
    }

    while(true) {
        int ret = driver.getReading();


        std::cout << "Got Data From IMU" << std::endl;
        std::cout << "Acc  " << driver.getCalibratedAccData() << std::endl;

        std::cout << "Gyro " << driver.getCalibratedGyroData() << std::endl;
        std::cout << "Magnetometer " << driver.getCalibratedMagData() << std::endl;

        std::cout << "Orientation " << driver.getOrientation().x() << " " << driver.getOrientation().y() << " " << driver.getOrientation().z() << " " << driver.getOrientation().w() << std::endl;

        std::cout << "Packet counter " << driver.getPacketCounter() << std::endl;

        if(ret < -1)
            exit(1);

        if(ret < 0)
            std::cout << "Timout" << std::endl;
    }
  
}
