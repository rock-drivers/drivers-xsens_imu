#include"XsensDriver.hpp"
#include<iostream>

int main(int argc, char* argv[]) {
  if(argc < 2) {  
    std::cout << "usage xsensTest <device>" << std::endl;
    exit(1);
  }
  
  std::string dev(argv[1]);
  
  xsens_imu::XsensDriver driver;
  
  int ret = driver.connectDevice(dev);
  if(ret < 0)
    exit(1);
  
  while(true) {
    ret = driver.getReading();

    
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
