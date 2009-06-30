#include<stdlib.h>
#include<iostream>
#include"XsensDriver.hpp"
#include <limits.h>

namespace xsens_imu {

XsensDriver::XsensDriver() {
  packet = 0;
}


bool XsensDriver::open(std::string const& dev) {
  CmtDeviceId deviceIds[256];

  //hardcoded Baudrate for now
  int baudrate = B115200;
  
  XsensResultValue ret = cmt3.openPort(dev.c_str(), baudrate);
  if(ret != XRV_OK)
      return false;
  return true;
}

bool XsensDriver::setCalibrationMode()
{
    int cmt_output  = CMT_OUTPUTMODE_RAW;
    int cmt_options = CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
    int sample_freq = CMT_DEFAULT_SAMPLE_FREQUENCY;

    // Set sensor to config state
    int ret = cmt3.gotoConfig();
    if(ret != XRV_OK)
    {
        std::cerr << "failed to go into config mode" << std::endl;
        return false;
    }

    CmtDeviceMode deviceMode(cmt_output, cmt_options, sample_freq);
    deviceMode.m_outputMode &= 0xFF0F;
    ret = cmt3.setDeviceMode(deviceMode, true);
    if(ret != XRV_OK)
    {
        std::cerr << "failed to set device mode" << std::endl;
        return false;
    }

    // Set logging
    ret = cmt3.createLogFile("xsens-imu-calib.bin", true);
    if (ret != XRV_OK)
    {
        std::cerr << "failed to enable logging" << std::endl;
        return false;
    }

    char logfile_name[PATH_MAX];
    cmt3.getLogFileName(logfile_name);
    std::cout << "Xsens driver is now in calibration mode\n";
    std::cout << "  output file: " << logfile_name << std::endl;

    // Go into measurement mode
    ret = cmt3.gotoMeasurement();
    if (ret != XRV_OK)
    {
        std::cerr << "failed to go into measurement mode" << std::endl;
        return false;
    }

    unsigned int mtCount = cmt3.getMtCount();
    delete packet;
    packet = new xsens::Packet(mtCount, cmt3.isXm());
    return true;
}

bool XsensDriver::setReadingMode(imuMode output_mode)
{
    CmtOutputMode     mode = 0;
    CmtOutputSettings settings = 0;

    switch (output_mode) {
        case ONLY_CAL_DATA:
            //configure IMU in calibrated data mode
            mode = CMT_OUTPUTMODE_CALIB;
            break;
        case ONLY_ORI_DATA:
            //configure IMU in orientation mode
            mode = CMT_OUTPUTMODE_ORIENT;

            //tell IMU to give us orientation as quaternion
            settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
            break;
        case CAL_AND_ORI_DATA:
            //configure IMU in calibrated data and orientation mode
            mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;

            //tell IMU to give us orientation as quaternion
            settings = CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
            break;
    }

    //add timestamps to data
    settings |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;

    unsigned int mtCount = cmt3.getMtCount();

    // set sensor to config sate
    int ret = cmt3.gotoConfig();
    if(ret != XRV_OK) {
        std::cerr << "Failed to set Xsens MTi into Config mode" << std::endl;
        return false;
    }

    unsigned short sampleFreq = cmt3.getSampleFrequency();

    CmtDeviceMode deviceMode(mode, settings, sampleFreq);
    deviceMode.m_outputMode &= 0xFF0F;
    ret = cmt3.setDeviceMode(deviceMode, true);
    if(ret != XRV_OK) {
        std::cerr << "Failed to set Xsens MTi into devicemode" << std::endl;
        return false;
    }

    delete packet;
    packet = new xsens::Packet((unsigned short) mtCount, cmt3.isXm());

    return true;
}

bool XsensDriver::setTimeout(const uint32_t timeout) {
  return !cmt3.setTimeoutMeasurement(timeout);
}



enum xsens_imu::errorCodes XsensDriver::getReading() {

  XsensResultValue ret = cmt3.waitForDataMessage(packet);
  
  switch(ret) {
    case XRV_TIMEOUT:
    case XRV_TIMEOUTNODATA:
      //timeout, no data available
      return ERROR_TIMEOUT;
      break;

    default:
      //error occured
      return ERROR_OTHER;
      break;
      
    case XRV_OK:
      caldata  = packet->getCalData(0);
      qat_data = packet->getOriQuat(0);

      return NO_ERROR;
      break;
  }
  
  return ERROR_OTHER;
}

int XsensDriver::getPacketCounter() {
  return packet->getSampleCounter();
}

Eigen::Quaternion<double> XsensDriver::getOrientation() const {
  return Eigen::Quaternion<double>(qat_data.m_data[0], qat_data.m_data[1], qat_data.m_data[2], qat_data.m_data[3]);
}

Eigen::Vector3d XsensDriver::getCalibratedAccData() const {
  return Eigen::Vector3d(caldata.m_acc.m_data[0], caldata.m_acc.m_data[1], caldata.m_acc.m_data[2]);
}

Eigen::Vector3d XsensDriver::getCalibratedGyroData() const {
  return Eigen::Vector3d(caldata.m_gyr.m_data[0], caldata.m_gyr.m_data[1], caldata.m_gyr.m_data[2]);
}

Eigen::Vector3d XsensDriver::getCalibratedMagData() const {
  return Eigen::Vector3d(caldata.m_mag.m_data[0], caldata.m_mag.m_data[1], caldata.m_mag.m_data[2]);
}

}

