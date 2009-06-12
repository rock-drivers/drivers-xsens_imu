#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"
#include<string>

class XsensDriver {
  public:
  XsensDriver();
  int connectDevice(std::string dev);
  int getReading();
  
  private:
    xsens::Cmt3 cmt3;
    xsens::Packet* packet;
};

