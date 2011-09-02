#include "XsensDriver.hpp"
#include <iostream>
#include <iomanip>
#include <boost/lexical_cast.hpp>

using namespace std;
int main(int argc, char* argv[]) {
    if(argc < 2) {  
        std::cout << "usage: xsensTest <device> [count]" << std::endl;
        exit(1);
    }

    std::string dev(argv[1]);

    int count = 100;
    if (argc == 3)
        count = boost::lexical_cast<int>(argv[2]);


    xsens_imu::XsensDriver driver;

    if (! driver.open(dev))
        return 1;

    // example: uncomment the following line to set frequency of 50Hz
    //driver.setFrequency(50);

    if (! driver.setReadingMode(xsens_imu::CAL_AND_ORI_DATA))
        return 1;

    // use timeout of 100ms
    if (! driver.setTimeout(100))
        return 1;

    std::cout << "available scenarios: " << std::endl;
    std::list<std::string> scenarios = driver.getAvailableScenarios();
    for (std::list<std::string>::const_iterator it = scenarios.begin(); it != scenarios.end(); ++it)
    	std::cout << "  '" << *it << "'" << std::endl;

    std::cout << "file handle: " << driver.getFileHandle() << std::endl << std::endl;

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
        << " " << setw(10) << "q.w" 
        << " " << setw(10) << "euler.z" 
        << " " << setw(10) << "euler.y"
        << " " << setw(10) << "euler.x" 
	<< std::endl;

    while(count--) {
        int ret = driver.getReading();
        if( ret == xsens_imu::NO_ERROR ) {
	    Eigen::Vector3d euler = driver.getOrientation().toRotationMatrix().eulerAngles(2,1,0) / M_PI * 180.0;

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
                << " " << setw(10) << driver.getOrientation().w() 
                << " " << setw(10) << euler[0]
                << " " << setw(10) << euler[1]
                << " " << setw(10) << euler[2]
		<< std::endl;
        }

        if( ret == xsens_imu::ERROR_TIMEOUT ) {
            std::cout << "timeout" << std::endl;
        }

        if( ret == xsens_imu::ERROR_OTHER ) {
            std::cout << "No idea what, but an error occured." << std::endl;
            exit(1);
        }
   }

    std::cout << "closing driver: " << driver.close() << std::endl;
}
