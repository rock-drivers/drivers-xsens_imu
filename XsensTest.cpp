#include"XsensDriver.hpp"
#include<iostream>

int main(int argc, char* argv[]) {
  if(argc < 2) {  
    std::cout << "usage xsensTest <device>" << std::endl;
    exit(1);
  }
  
  std::string dev(argv[1]);
  
  XsensDriver driver;
  
  int ret = driver.connectDevice(dev);
  if(ret < 0)
    exit(1);
  
  while(true) {
    ret = driver.getReading();
    if(ret < 0)
      exit(1);
  }
  
}
