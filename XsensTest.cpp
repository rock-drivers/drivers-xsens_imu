#include "XsensDriver.hpp"
#include <iostream>
#include <iomanip>

using namespace std;
int main(int argc, char* argv[]) {
    if(argc < 2) {  
        std::cout << "usage xsensTest <device>" << std::endl;
        exit(1);
    }

    std::string dev(argv[1]);

    xsens_imu::XsensDriver driver;

    if (! driver.open(dev))
        return 1;
    if (! driver.setReadingMode(xsens_imu::CAL_AND_ORI_DATA))
        return 1;

    std::cout << setw(10) << "packet_id"
        << " " << setw(10) << "acc.x"
        << " " << setw(10) << "acc.y"
        << " " << setw(10) << "acc.z"
        << " " << setw(10) << "gyro.x"
        << " " << setw(10) << "gyro.y"
        << " " << setw(10) << "gyro.z"
        << " " << setw(10) << "mag.x"
        << " " << setw(10) << "mag.y"
        << " " << setw(10) << "mag.z"
        << " " << setw(10) << "q.x"
        << " " << setw(10) << "q.y"
        << " " << setw(10) << "q.z"
        << " " << setw(10) << "q.w" << std::endl;

    while(true) {
        int ret = driver.getReading();
        if( ret == xsens_imu::NO_ERROR ) {
            std::cout << setw(10) << driver.getPacketCounter()
                << " " << setw(10) << driver.getCalibratedAccData().x()
                << " " << setw(10) << driver.getCalibratedAccData().y()
                << " " << setw(10) << driver.getCalibratedAccData().z()
                << " " << setw(10) << driver.getCalibratedGyroData().x()
                << " " << setw(10) << driver.getCalibratedGyroData().y()
                << " " << setw(10) << driver.getCalibratedGyroData().z()
                << " " << setw(10) << driver.getCalibratedMagData().x()
                << " " << setw(10) << driver.getCalibratedMagData().y()
                << " " << setw(10) << driver.getCalibratedMagData().z()
                << " " << setw(10) << driver.getOrientation().x()
                << " " << setw(10) << driver.getOrientation().y()
                << " " << setw(10) << driver.getOrientation().z()
                << " " << setw(10) << driver.getOrientation().w() << std::endl;
        }

        if( ret == xsens_imu::ERROR_TIMEOUT ) {
            std::cout << "timeout" << std::endl;
        }

        if( ret == xsens_imu::ERROR_OTHER ) {
            std::cout << "No idea what, but an error occured." << std::endl;
            exit(1);
        }
   }
}
